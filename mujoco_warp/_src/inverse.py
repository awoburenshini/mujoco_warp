# Copyright 2025 The Newton Developers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import warp as wp

from mujoco_warp._src import derivative
from mujoco_warp._src import forward
from mujoco_warp._src import sensor
from mujoco_warp._src import smooth
from mujoco_warp._src import solver
from mujoco_warp._src import support
from mujoco_warp._src.support import mul_m
from mujoco_warp._src.types import Data
from mujoco_warp._src.types import DisableBit
from mujoco_warp._src.types import EnableBit
from mujoco_warp._src.types import IntegratorType
from mujoco_warp._src.types import Model

wp.set_module_options({"enable_backward": False})


@wp.kernel
def _qfrc_eulerdamp(
  # Model:
  opt_timestep: wp.array[float],
  dof_damping: wp.array2d[float],
  # Data in:
  qacc_in: wp.array2d[float],
  # Out:
  qfrc_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()
  timestep = opt_timestep[worldid % opt_timestep.shape[0]]
  qfrc_out[worldid, dofid] += timestep * dof_damping[worldid % dof_damping.shape[0], dofid] * qacc_in[worldid, dofid]


@wp.kernel
def _qfrc_inverse(
  # Data in:
  qfrc_bias_in: wp.array2d[float],
  qfrc_passive_in: wp.array2d[float],
  qfrc_constraint_in: wp.array2d[float],
  # In:
  Ma: wp.array2d[float],
  # Data out:
  qfrc_inverse_out: wp.array2d[float],
):
  worldid, dofid = wp.tid()

  qfrc_inverse = qfrc_bias_in[worldid, dofid]
  qfrc_inverse += Ma[worldid, dofid]
  qfrc_inverse -= qfrc_passive_in[worldid, dofid]
  qfrc_inverse -= qfrc_constraint_in[worldid, dofid]

  qfrc_inverse_out[worldid, dofid] = qfrc_inverse


def discrete_acc(m: Model, d: Data, qacc: wp.array2d[float]):
  """Convert discrete-time qacc to continuous-time qacc.

  Args:
    m: The model containing kinematic and dynamic information.
    d: The data object containing the current state and output arrays.
    qacc: Acceleration.
  """
  qfrc = wp.empty((d.nworld, m.nv), dtype=float)

  if m.opt.integrator == IntegratorType.RK4:
    raise NotImplementedError("discrete inverse dynamics is not supported by RK4 integrator")
  elif m.opt.integrator == IntegratorType.EULER:
    if m.opt.disableflags & DisableBit.EULERDAMP:
      wp.copy(qacc, d.qacc)
      return

    # TODO(team): qacc = d.qacc if (m.dof_damping == 0.0).all()

    # set qfrc = (d.qM + m.opt.timestep * diag(m.dof_damping)) * d.qacc

    # d.qM @ d.qacc
    # <md>
    # $$\texttt{qfrc} \;=\; M(q)\,\ddot q\quad\text{(mass-matrix product; first term of the Euler-damped discrete force)}$$
    # </md>
    support.mul_m(m, d, qfrc, d.qacc)

    # qfrc += m.opt.timestep * m.dof_damping * d.qacc
    # <md>
    # $$\texttt{qfrc} \;\mathrel{+}=\; h\,\operatorname{diag}(b)\,\ddot q\quad\text{(add explicit Euler damping; }h=\texttt{timestep},\ b=\texttt{dof\_damping)}$$
    # </md>
    wp.launch(
      _qfrc_eulerdamp,
      dim=(d.nworld, m.nv),
      inputs=[m.opt.timestep, m.dof_damping, d.qacc],
      outputs=[qfrc],
    )
  elif m.opt.integrator == IntegratorType.IMPLICITFAST:
    if m.is_sparse:
      qDeriv = wp.empty((d.nworld, 1, m.nM), dtype=float)
    else:
      qDeriv = wp.empty((d.nworld, m.nv, m.nv), dtype=float)
    # <md>
    # $$D \;=\; \frac{\partial \dot q_{\text{smooth}}}{\partial \dot q}\quad\text{(velocity Jacobian of smooth dynamics, used to form the implicit-fast discrete operator)}$$
    # </md>
    derivative.deriv_smooth_vel(m, d, qDeriv)
    # <md>
    # $$\texttt{qfrc} \;=\; \bigl(M(q) - h\,D\bigr)\,\ddot q\quad\text{(IMPLICITFAST discrete force; }M\text{ replaced by the implicit operator via }\texttt{M=qDeriv)}$$
    # </md>
    mul_m(m, d, qfrc, d.qacc, M=qDeriv)
    # <md>
    # $$\bigl(M(q) - h\,D\bigr)\,\ddot q_{\text{cont}} \;=\; \texttt{qfrc}\quad\text{(solve the implicit operator for the continuous-time acceleration)}$$
    # </md>
    smooth.factor_solve_i(m, d, d.qM, d.qLD, d.qLDiagInv, qacc, qfrc)
  else:
    raise NotImplementedError(f"integrator {m.opt.integrator} not implemented.")

  # solve for qacc: qfrc = d.qM @ d.qacc
  # <md>
  # $$M(q)\,\ddot q_{\text{cont}} \;=\; \texttt{qfrc}\;\Longrightarrow\; \ddot q_{\text{cont}} = M^{-1}\texttt{qfrc}\quad\text{(recover continuous-time acceleration from the discrete force)}$$
  # </md>
  smooth.solve_m(m, d, qacc, qfrc)


def inv_constraint(m: Model, d: Data):
  """Inverse constraint solver."""
  # no constraints
  if d.njmax == 0:
    d.qfrc_constraint.zero_()
    return

  ctx = solver.create_inverse_context(m, d)
  solver.init_context(m, d, ctx, grad=False)


def inverse(m: Model, d: Data):
  """Inverse dynamics."""
  # <md>
  # $$\bigl(G_i(q),\,\mathcal{I}^{\text{world}}_b,\,M(q)\bigr)\quad\text{(reuse forward position stage: kinematics, CoM/inertia, CRB mass matrix — all functions of }q\text{ only)}$$
  # </md>
  forward.fwd_position(m, d)
  sensor.sensor_pos(m, d)
  # <md>
  # $$\bigl(v_b,\,c_b\bigr) = f(q,\dot q)\quad\text{(reuse forward velocity stage: spatial velocities and Coriolis-velocity products from }(q,\dot q))$$
  # </md>
  forward.fwd_velocity(m, d)
  sensor.sensor_vel(m, d)

  invdiscrete = m.opt.enableflags & EnableBit.INVDISCRETE
  if invdiscrete:
    # save discrete-time qacc and compute continuous-time qacc
    qacc_discrete = wp.clone(d.qacc)
    discrete_acc(m, d, d.qacc)

  # <md>
  # $$\texttt{qfrc\_constraint} \;=\; J^{\top}\lambda\quad\text{(inverse constraint solve: constraint force consistent with the prescribed state }(q,\dot q,\ddot q))$$
  # </md>
  inv_constraint(m, d)
  # <md>
  # $$\texttt{qfrc\_bias} \;=\; C(q,\dot q) \;=\; J^{I,\top}\!\bigl(\mathcal{I}\,a_g + v\times^{*}\mathcal{I}\,v\bigr)\quad\text{(recursive Newton-Euler bias: Coriolis, centrifugal and gravity, with }\ddot q=0)$$
  # </md>
  smooth.rne(m, d)
  # <md>
  # $$\texttt{qfrc\_bias} \;\mathrel{+}=\; J_t^{\top} f_t\quad\text{(add tendon bias forces)}$$
  # </md>
  smooth.tendon_bias(m, d, d.qfrc_bias)
  sensor.sensor_acc(m, d)

  # <md>
  # $$\texttt{qfrc\_inverse} \;=\; M(q)\,\ddot q\quad\text{(inertial term; }\texttt{Ma}\text{ argument to the final assembly)}$$
  # </md>
  support.mul_m(m, d, d.qfrc_inverse, d.qacc)

  # <md>
  # $$\tau \;=\; \underbrace{M(q)\,\ddot q}_{\texttt{Ma}} \;+\; \underbrace{C(q,\dot q)}_{\texttt{qfrc\_bias}} \;-\; \texttt{qfrc\_passive} \;-\; \underbrace{J^{\top}\lambda}_{\texttt{qfrc\_constraint}}\quad\text{(applied force recovered by inverse dynamics }\to\texttt{qfrc\_inverse})$$
  # </md>
  wp.launch(
    _qfrc_inverse,
    dim=(d.nworld, m.nv),
    inputs=[
      d.qfrc_bias,
      d.qfrc_passive,
      d.qfrc_constraint,
      d.qfrc_inverse,
    ],
    outputs=[d.qfrc_inverse],
  )

  if invdiscrete:
    # restore discrete-time qacc
    wp.copy(d.qacc, qacc_discrete)
