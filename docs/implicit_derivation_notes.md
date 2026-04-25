# Forward Simulation and Implicit Derivation Notes

## Goal

This note only focuses on forward simulation.

The current derivation plan is:

1. Start from the geometric map in world coordinates
2. Define velocity, kinetic energy, density, and collision in world coordinates
3. Pull everything back to generalized coordinates $q$
4. Start from a global implicit optimization problem and derive a one-step discrete update
5. Align the result with `IMPLICITFAST` in this repository

---

## Current Conclusions

- This repository implements `EULER`, `RK4`, and `IMPLICITFAST`
- For robot forward simulation, the most useful integrator to align first is `IMPLICITFAST`
- `IMPLICITFAST` does not drop the collision / constraint force itself
- `IMPLICITFAST` drops the derivative of the constraint force with respect to velocity
- `IMPLICITFAST` can be interpreted as one quasi-Newton step for a full implicit residual

---

## Notation

We only keep the symbols that are actually used later in the rigid-body
contact derivation.

### Time-Stepping Variables

- $q$: generalized coordinates
- $v = \dot q$: generalized velocity
- $a = \dot v$: generalized acceleration
- $h$: time step

### Rigid-Body Geometry

- $b$: a rigid body
- $\xi$: a point fixed on body $b$ in the body-local frame
- $x_b(q,\xi) \in \mathbb{R}^3$: the world position of that point

Equivalently, for each body $b$ and configuration $q$, we can view
$x_b(q,\cdot): \mathbb{R}^3 \to \mathbb{R}^3$ as the body-to-world kinematic
map. In the rigid-body case, if $\xi$ denotes body-local coordinates, then
$$
x_b(q,\xi) = R_b(q)\,\xi + t_b(q),
$$
where $R_b(q) \in SO(3)$ and $t_b(q) \in \mathbb{R}^3$ are the orientation and
translation of body $b$ in world coordinates.

We will use $x_b(q,\xi)$ only for rigid-body kinematics and contact points. We
do not start from a continuum description in the basic notation.

### Contact Data at Frame $k$

- $\mathcal{A}_k$: the set of contacts that are active in the solver at frame
  $k$
- $c \in \mathcal{A}_k$: one active contact
- $b_c^1, b_c^2$: the two bodies participating in contact $c$
- $d_c \in \mathbb{R}$: the signed nearest-point distance, with $d_c < 0$
  meaning penetration
- $x_c \in \mathbb{R}^3$: the world-space contact point
- $R_c =
  \begin{bmatrix}
  r_{c,0}^T \\
  r_{c,1}^T \\
  r_{c,2}^T
  \end{bmatrix}
  \in SO(3)$: the contact frame, with first row $r_{c,0} = n_c$ the contact
  normal
- $\epsilon_c = (\xi_{c,1}, \xi_{c,2})$: the frozen witness pair attached to
  the two contacting bodies
- $\mu_c$: the tangential friction data associated with contact $c$; when
  needed, we write $\mu_{c,1}, \mu_{c,2}$ for the two tangential coefficients
- $m_c = 3$: in this note we restrict attention to ordinary rigid contact with
  one normal and two tangential directions

The batched implementation also stores a `worldid`, but we suppress that index
in the single-world notation of this note. In code, contact generation is split
into two stages: `broadphase` finds candidate pairs, and `narrowphase`
constructs the active contacts $\mathcal{A}_k$ and their associated contact
data. Additional implementation details are introduced only when needed later.

---


## Exact Frozen-Force Step

At frame $k$, once $(q_k, v_k)$ are known, we first freeze the contact data
$\mathcal{A}_k$ and solve an auxiliary contact problem on that frame-local
model. This produces one contact force for each active contact,
$$
f_k := (f_{c,k})_{c \in \mathcal{A}_k},
\qquad
f_{c,k} \in \mathbb{R}^3.
$$
If $\lambda_{c,k} \in \mathbb{R}^3$ denotes the same force in the local contact
frame, then
$$
f_{c,k} = R_c^T \lambda_{c,k}.
$$

We keep that auxiliary contact solve abstract. The only property used below is
that it is evaluated on the frozen frame-$k$ contact model and returns a force
$f_{c,k}$ before the subsequent implicit correction of the configuration. In
the code, this corresponds to building the frame-$k$ contact model in
`fwd_position()` and then solving for the contact force in `solver.solve()`.

With that frozen contact force in hand, introduce the explicit predictor
$$
q_k^{\mathrm{pr}} := q_k + h v_k.
$$

For a material point $\bar{x}$ in the reference configuration, write
$$
x_k(\bar{x}) := x(q_k, \bar{x}),
\qquad
\dot{x}_k(\bar{x}) := J_x(q_k, \bar{x})\, v_k,
$$
and define the predictor residual
$$
r_k(q, \bar{x})
:=
x(q, \bar{x}) - x_k(\bar{x}) - h\,\dot{x}_k(\bar{x}).
$$

For each active contact, define the relative witness displacement
$$
\Delta x_c(q,\epsilon_c)
:=
x_{b_c^2}(q,\xi_{c,2}) - x_{b_c^1}(q,\xi_{c,1}).
$$

With the frame-$k$ contact forces frozen, the one-step configuration update is
written as
$$
\widehat{\mathcal{J}}_k(q; f_k)
:=
\frac{1}{2h^2}
\int_{\Omega_0}
\rho(\bar{x})
\left\|
r_k(q, \bar{x})
\right\|^2
\mathrm d\bar{x}
+
\Phi^x(x(q, \cdot))
+
\Phi^q(q)
-
\sum_{c \in \mathcal{A}_k}
f_{c,k}^T \Delta x_c(q,\epsilon_c),
$$
and
$$
q_{k+1} \in \arg\min_q \widehat{\mathcal{J}}_k(q; f_k).
$$

Here $\Phi^x$ and $\Phi^q$ are the smooth potential energies, and the last term
is the virtual work of the frozen contact forces.

Define the witness Jacobian
$$
\Delta J_c(q) := \nabla_q \Delta x_c(q,\epsilon_c).
$$

Then the gradient of the frozen-force objective is
$$
\nabla_q \widehat{\mathcal{J}}_k(q; f_k)
=
\frac{1}{h^2}
\int_{\Omega_0}
\rho(\bar{x})\,
J_x(q,\bar{x})^T r_k(q,\bar{x})
\mathrm d\bar{x}
+
\nabla_q\!\left[\Phi^x(x(q,\cdot)) + \Phi^q(q)\right]
-
\sum_{c \in \mathcal{A}_k}
\Delta J_c(q)^T f_{c,k}.
$$

Evaluated at the predictor, this gives the frame-$k$ gradient
$$
g_k := \nabla_q \widehat{\mathcal{J}}_k(q_k^{\mathrm{pr}}; f_k).
$$

To write the exact Hessian compactly, define
$$
\mathcal{H}_x(q,\bar{x})[u]
:=
\sum_{i=1}^3 u_i \nabla_{qq}^2 x_i(q,\bar{x}),
$$
and
$$
\mathcal{H}_{\Delta x_c}(q,\epsilon_c)[f_{c,k}]
:=
\sum_{i=1}^3 (f_{c,k})_i\, \nabla_{qq}^2 (\Delta x_c)_i(q,\epsilon_c).
$$

Then the exact Hessian at the predictor is
$$
\begin{aligned}
H_k
:=
\nabla_{qq}^2 \widehat{\mathcal{J}}_k(q_k^{\mathrm{pr}}; f_k)
=\;&
\frac{1}{h^2}
\int_{\Omega_0}
\rho(\bar{x})
\Bigl(
J_x(q_k^{\mathrm{pr}}, \bar{x})^T J_x(q_k^{\mathrm{pr}}, \bar{x})
+
\mathcal{H}_x(q_k^{\mathrm{pr}}, \bar{x})
\bigl[r_k(q_k^{\mathrm{pr}}, \bar{x})\bigr]
\Bigr)
\mathrm d\bar{x} \\
&+
\nabla_{qq}^2\!\left[
\Phi^x(x(q_k^{\mathrm{pr}},\cdot)) + \Phi^q(q_k^{\mathrm{pr}})
\right]
-
\sum_{c \in \mathcal{A}_k}
\mathcal{H}_{\Delta x_c}(q_k^{\mathrm{pr}},\epsilon_c)[f_{c,k}].
\end{aligned}
$$

The exact one-step correction from the predictor is obtained from
$$
H_k\,\delta q_k = -g_k,
\qquad
q_{k+1} = q_k^{\mathrm{pr}} + \delta q_k.
$$

## Approximations Toward `IMPLICITFAST`
In mujoco-warp, the right-hand side term:
$$-g_k = 
\underbrace{- \frac{1}{h^2} \int_{\Omega_0} \rho(\bar{x})\, J_x(q_k^{\text{pr}})^T r_k(q_k^{\text{pr}}) \mathrm d\bar{x}}_{\substack{\text{Kinematic Residual (Coriolis)} \\ \text{MuJoCo: } \texttt{-qfrc\_bias}}}
+
\underbrace{\left( -\nabla_q \Phi_{\text{total}} \right)}_{\substack{\text{Smooth Forces (Applied + Passive)} \\ \text{MuJoCo: } \texttt{qfrc\_applied} + \texttt{qfrc\_passive}}}
+
\underbrace{\sum_{c \in \mathcal{A}_k} \Delta J_c(q_k^{\text{pr}})^T f_{c,k}}_{\substack{\text{Constraint Forces (Frozen)} \\ \text{MuJoCo: } \texttt{qfrc\_constraint}}}$$