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
\widehat{\mathcal{J}}_k(q;\, f_k,\, \tau_k,\, q_k)
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
\Phi^q(q;\, q_k,\, \tau_k)
-
\sum_{c \in \mathcal{A}_k}
f_{c,k}^T \Delta x_c(q,\epsilon_c),
$$
and
$$
q_{k+1} \in \arg\min_q \widehat{\mathcal{J}}_k(q;\, f_k,\, \tau_k,\, q_k).
$$

Here $\Phi^x(x(q,\cdot))$ collects all spatial-domain smooth potential
energies (gravity, world-frame elastic potentials), pulled back to $q$
through $x(q,\cdot)$. The configuration-space term $\Phi^q$ is no longer a
pure potential of $q$: it carries the frame-$k$ frozen parameters
$(q_k,\tau_k)$ and is defined as
$$
\Phi^q(q;\, q_k,\, \tau_k)
\;:=\;
V_{\mathrm{spring}}(q)
\;+\;
h\cdot R\!\left(\frac{q-q_k}{h}\right)
\;-\;
\tau_k^\top q,
\qquad
\tau_k \,:=\, \tau_{k}^{\mathrm{act}} + \tau_{k}^{\mathrm{ext}}.
$$
Each piece plays a different role:

- $V_{\mathrm{spring}}(q)$ is the genuine configuration-space potential
  (joint and tendon stiffness).
- $h\cdot R\!\left(\tfrac{q-q_k}{h}\right)$ is the discrete Rayleigh
  dissipation potential (joint and tendon damping). The substitution
  $v \equiv (q-q_k)/h$ promotes this $v$-dependent force into a
  $q$-function with $q_k$ frozen. The factor of $h$ makes the gradient
  reproduce the damping force $\partial R/\partial v$ exactly, and makes
  the Hessian contribute $\partial^2 R/\partial v^2 / h$, which is
  precisely the velocity Jacobian that `IMPLICITFAST` keeps on the
  left-hand side.
- $-\tau_k^\top q$ is the D'Alembert virtual-work term for the frozen
  external generalized force, where
  $\tau_k^{\mathrm{act}}$ is the (frozen) actuator force and
  $\tau_k^{\mathrm{ext}}$ is the (frozen) user-applied force. This is
  not a true potential — it is the same freezing trick used for the
  contact force $f_{c,k}$ — and is needed because most actuators
  (motors, velocity-actuators with $\mathrm{ctrl}_k$ input) are not
  derivable from a potential of $q$ alone. The last term in
  $\widehat{\mathcal{J}}_k$ is the virtual work of the frozen contact
  forces, structurally identical to the $-\tau_k^\top q$ block.

A practical rule for which forces to freeze versus to keep
$v$-dependent: anything that contributes a non-zero entry to the
velocity Jacobian computed in `derivative.deriv_smooth_vel`
(`derivative.py:321`) must enter through $R$ (so its $\partial/\partial v$
survives in the Hessian); everything else can be frozen as a virtual-work
linear term in $\tau_k$. In mujoco-warp the only non-zero blocks of
$\partial_v\,\texttt{qfrc\_smooth}$ are joint damping (`m.dof_damping`),
tendon damping (`m.tendon_damping`), and the AFFINE actuator gain/bias
contributions; fluid drag is excluded by the
`NotImplementedError` gate at `io.py:126`. **Caveat for AFFINE actuators:**
when an actuator has an AFFINE gain or bias, $\partial \tau^{\mathrm{act}}/\partial v$
is non-zero and that block belongs in $R$ rather than the frozen
$\tau_k^{\mathrm{act}}$ — otherwise the corresponding velocity Hessian
entry assembled by `_qderiv_actuator_passive_vel`
(`derivative.py:32`) is lost. The cleanest accounting is to write
$\tau_k^{\mathrm{act}} = \tau_k^{\mathrm{act,frozen}} + \tau^{\mathrm{act,AFFINE}}(v)$ and absorb the
AFFINE part into a Rayleigh-type dissipation term $R^{\mathrm{act}}(v)$
parallel to the damping term.

Define the witness Jacobian
$$
\Delta J_c(q) := \nabla_q \Delta x_c(q,\epsilon_c).
$$

Then the gradient of the frozen-force objective is
$$
\nabla_q \widehat{\mathcal{J}}_k(q;\, f_k,\, \tau_k,\, q_k)
=
\frac{1}{h^2}
\int_{\Omega_0}
\rho(\bar{x})\,
J_x(q,\bar{x})^T r_k(q,\bar{x})
\mathrm d\bar{x}
+
\nabla_q\!\left[\Phi^x(x(q,\cdot)) + \Phi^q(q;\, q_k,\, \tau_k)\right]
-
\sum_{c \in \mathcal{A}_k}
\Delta J_c(q)^T f_{c,k}.
$$

Evaluated at the predictor, this gives the frame-$k$ gradient
$$
g_k := \nabla_q \widehat{\mathcal{J}}_k(q_k^{\mathrm{pr}};\, f_k,\, \tau_k,\, q_k).
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

Then the exact Hessian at the predictor, with $\Phi^q$ expanded into
its three pieces $V_{\mathrm{spring}}(q) + h\,R((q-q_k)/h) - \tau_k^\top q$,
is
$$
\begin{aligned}
H_k
\;:=\;&
\nabla_{qq}^2 \widehat{\mathcal{J}}_k(q_k^{\mathrm{pr}};\, f_k,\, \tau_k,\, q_k) \\[6pt]
=\;&
\frac{1}{h^2}\!
\int_{\Omega_0}\!\rho(\bar{x})
\Bigl(
\underbrace{J_x(q_k^{\mathrm{pr}}, \bar{x})^\top J_x(q_k^{\mathrm{pr}}, \bar{x})}_{\substack{\text{(a) integrates to }M(q_k^{\mathrm{pr}}) \\ \text{kinetic / mass block}}}
\;+\;
\underbrace{\mathcal{H}_x(q_k^{\mathrm{pr}}, \bar{x})\bigl[r_k(q_k^{\mathrm{pr}}, \bar{x})\bigr]}_{\substack{\text{(b) kinematic curvature} \\ \text{higher-order in }h}}
\Bigr)\,
\mathrm d\bar{x} \\[6pt]
&+\;
\underbrace{\nabla_{qq}^2\,\Phi^x\bigl(x(q_k^{\mathrm{pr}},\cdot)\bigr)}_{\substack{\text{(c) spatial-potential Hessian} \\ \text{(gravity, world-frame elastic)}}}
\;+\;
\underbrace{\nabla_{qq}^2\,V_{\mathrm{spring}}(q_k^{\mathrm{pr}})}_{\substack{\text{(d) joint/tendon spring stiffness}}}
\\[6pt]
&+\;
\underbrace{\frac{1}{h}\,\frac{\partial^2 R}{\partial v^2}(v_k)}_{\substack{\text{(e) Rayleigh velocity Hessian} \\ =\; \nabla_{qq}^2\bigl[h\,R((q-q_k)/h)\bigr]_{q=q_k^{\mathrm{pr}}}}}
\;+\;
\underbrace{0}_{\substack{\text{(f) }\nabla_{qq}^2(-\tau_k^\top q) \\ \text{linear in }q\text{, drops out}}}
\\[6pt]
&-\;
\underbrace{\sum_{c \in \mathcal{A}_k}
\mathcal{H}_{\Delta x_c}(q_k^{\mathrm{pr}},\epsilon_c)\bigl[f_{c,k}\bigr]}_{\substack{\text{(g) contact-witness curvature}}}.
\end{aligned}
$$
Notes on the labels:

- Block **(a)** evaluates as $\frac{1}{h^2}\int\rho\, J_x^\top J_x\,\mathrm d\bar x = \frac{1}{h^2} M(q_k^{\mathrm{pr}})$ — this is the only block that survives untouched in `IMPLICITFAST`'s left-hand side and gets named $M/h^2$ below.
- Block **(e)** uses the chain rule on $v(q)=(q-q_k)/h$: each $\partial/\partial q$ produces a $1/h$, the prefactor $h$ cancels one of them, leaving $(1/h)\,\partial^2 R/\partial v^2$ evaluated at $v=v_k$. This is the Rayleigh velocity Hessian that `IMPLICITFAST` keeps.
- Block **(f)** is identically zero because $-\tau_k^\top q$ is linear in $q$; the frozen actuator/applied force shows up only on the right-hand side, never on the left.
- Blocks **(b)(c)(d)(g)** are all position-second-derivative blocks; they are exactly the four that `IMPLICITFAST` discards (see next section).

The exact one-step correction from the predictor is obtained from
$$
H_k\,\delta q_k = -g_k,
\qquad
q_{k+1} = q_k^{\mathrm{pr}} + \delta q_k.
$$

## Approximations Toward `IMPLICITFAST`

With the expanded $\Phi^q(q; q_k, \tau_k)$ above, the negative gradient at
the predictor decomposes block-by-block into the right-hand side of the
linear system that `IMPLICITFAST` actually solves in mujoco-warp,
$$
\bigl(M - h\,\partial_v\,\texttt{qfrc\_smooth}\bigr)\, q_{\mathrm{acc}}
\;=\;
\texttt{efc.Ma}
\;=\;
\texttt{qfrc\_smooth} + \texttt{qfrc\_constraint},
$$
(see `forward.py:493-505` for the linear solve and `forward.py:940-945`
for the assembly of `qfrc_smooth`). Reading $-g_k$ from left to right:

$$
\begin{aligned}
-g_k \;=\;
&\underbrace{
- \frac{1}{h^2}\!\int_{\Omega_0}\!\rho(\bar{x})\, J_x(q_k^{\mathrm{pr}})^\top r_k(q_k^{\mathrm{pr}})\,\mathrm d\bar{x}
\;-\;\nabla_q \Phi^x(q_k^{\mathrm{pr}})
}_{\substack{\text{Coriolis + centrifugal + gravity}\\ \text{mujoco-warp: }-\texttt{d.qfrc\_bias} \\ \text{built in }\texttt{smooth.rne}\text{ (}\texttt{smooth.py}\text{)}}} \\[4pt]
&+\;\underbrace{
-\nabla_q V_{\mathrm{spring}}(q_k^{\mathrm{pr}})
\;-\;\nabla_q\!\left[h\,R\!\left(\tfrac{q-q_k}{h}\right)\right]_{q=q_k^{\mathrm{pr}}}
}_{\substack{\text{joint/tendon springs + damping}\\ \text{mujoco-warp: }\texttt{d.qfrc\_passive} \\ \text{built in }\texttt{passive.passive}\text{ (}\texttt{passive.py}\text{)}}} \\[4pt]
&+\;\underbrace{\tau_{k}^{\mathrm{act}}}_{\substack{\text{actuator force (frozen)}\\ \text{mujoco-warp: }\texttt{d.qfrc\_actuator} \\ \text{built in }\texttt{fwd\_actuation}\text{ (}\texttt{forward.py}\text{)}}}
\;+\;\underbrace{\tau_{k}^{\mathrm{ext}}}_{\substack{\text{user-applied force (frozen)}\\ \text{mujoco-warp: }\texttt{d.qfrc\_applied}\\ \text{(set by caller; xfrc accumulated} \\ \text{in }\texttt{xfrc\_accumulate}\text{)}}} \\[4pt]
&+\;\underbrace{\sum_{c\in\mathcal{A}_k} \Delta J_c(q_k^{\mathrm{pr}})^\top f_{c,k}}_{\substack{\text{frozen constraint forces}\\ \text{mujoco-warp: }\texttt{d.qfrc\_constraint} \\ \text{built in }\texttt{solver.solve}\text{ (}\texttt{solver.py}\text{)}}}
\end{aligned}
$$

The first four blocks together reconstruct
$\texttt{qfrc\_smooth} = \texttt{qfrc\_passive} - \texttt{qfrc\_bias} + \texttt{qfrc\_actuator} + \texttt{qfrc\_applied}$,
and adding the constraint block yields exactly `efc.Ma`.

The Hessian is approximated by **dropping every position-derivative
block and keeping only the kinetic + Rayleigh velocity blocks**. Reusing
the labels (a)–(g) introduced for the exact $H_k$ above and crossing
out the discarded ones:

$$
\begin{aligned}
H_k
\;=\;&
\underbrace{\frac{M(q_k^{\mathrm{pr}})}{h^2}}_{\text{(a) kept}}
\;+\;
\underbrace{\frac{1}{h}\,\frac{\partial^2 R}{\partial v^2}(v_k)}_{\text{(e) kept}}
\;+\;
\underbrace{0}_{\text{(f) trivially zero}} \\[4pt]
&+\;
\underbrace{\cancel{\frac{1}{h^2}\!\int_{\Omega_0}\!\rho(\bar{x})\,\mathcal{H}_x(q_k^{\mathrm{pr}},\bar{x})[r_k]\,\mathrm d\bar{x}}}_{\text{(b) dropped}}
\;+\;
\underbrace{\cancel{\nabla^2_{qq}\,\Phi^x(q_k^{\mathrm{pr}})}}_{\text{(c) dropped}} \\[4pt]
&+\;
\underbrace{\cancel{\nabla^2_{qq}\,V_{\mathrm{spring}}(q_k^{\mathrm{pr}})}}_{\text{(d) dropped}}
\;+\;
\underbrace{\cancel{\sum_{c\in\mathcal{A}_k}\!\mathcal{H}_{\Delta x_c}(q_k^{\mathrm{pr}},\epsilon_c)[f_{c,k}]}}_{\text{(g) dropped}}.
\end{aligned}
$$

Collecting only the surviving blocks (a) and (e) gives `IMPLICITFAST`'s
left-hand side matrix:

$$
\widehat{H}_k
\;=\;
\frac{M}{h^2}
\;+\;
\frac{1}{h}\,\frac{\partial^2 R}{\partial v^2}
\;=\;
\frac{1}{h^2}\Bigl(M \;-\; h\,\partial_v\,\texttt{qfrc\_smooth}\Bigr),
$$

where $\partial_v\,\texttt{qfrc\_smooth}$ is exactly what
`derivative.deriv_smooth_vel` (`derivative.py:321`) assembles: joint
damping (`m.dof_damping`), tendon damping (`m.tendon_damping`), and the
AFFINE actuator gain/bias blocks are its only nonzero contributions.
This is the full content of the "fast" approximation — every dropped
term above is a position-derivative block evaluated at the predictor.

## Collision Force Calculation

### What is still abstract

Take the previous section's Newton step $\widehat{H}_k\,\delta q_k = -g_k$ and
re-parameterize via $\delta q_k = h^2 q_{\mathrm{acc}}$,
$\widetilde{M} := h^2 \widehat{H}_k$. Split $-g_k$ into the four smooth blocks
(gravity + bias, passive springs + damping, frozen actuator, frozen applied)
and the contact block, and call the smooth sum $F^{\mathrm{sm}}_k$:

$$
\boxed{\;\;
\widetilde{M}\, q_{\mathrm{acc}}
\;=\;
F^{\mathrm{sm}}_k \;+\; \sum_{c \in \mathcal{A}_k} \Delta J_c(q_k^{\mathrm{pr}})^\top f_{c,k}.
\;\;}
$$

Inherited from the previous section:

- $M = M(q_k^{\mathrm{pr}})$ — inertia matrix from block (a) of the exact
  Hessian.
- $\widetilde{M} = M - h\,\partial F^{\mathrm{sm}}_k / \partial v$ — the (a) +
  (e) approximation's LHS.
- $F^{\mathrm{sm}}_k$ — total smooth-side generalized force at frame $k$ (the
  first four terms of the previous gradient decomposition).
- $\Delta J_c = \nabla_q \Delta x_c$ — witness Jacobian.
- $q_{\mathrm{acc}}$ — one-step acceleration of the predictor,
  $\delta q_k = h^2 q_{\mathrm{acc}}$.

Everything except $f_{c,k}$ is now defined. This section closes the story.

### Target problem

Decompose $f_{c,k} = R_c^\top \lambda_c$ in the local contact frame, with
$\lambda_c = (\lambda_{c,n}, \lambda_{c,t}) \in \mathbb{R} \times \mathbb{R}^{m_c-1}$.
Stack $\lambda := (\lambda_c)_c$, $J := (R_c \Delta J_c)_c$. The exact
rigid-body frictional contact problem at the **acceleration level** asks for
$(q_{\mathrm{acc}}, \lambda)$ with

$$
\widetilde{M}\, q_{\mathrm{acc}} = F^{\mathrm{sm}}_k + J^\top \lambda,
$$

and per contact $c$:

$$
\underbrace{\lambda_{c,n} \ge 0,\quad J_{c,n} q_{\mathrm{acc}} \ge 0,\quad \lambda_{c,n}\,(J_{c,n} q_{\mathrm{acc}}) = 0}_{\text{Signorini at acceleration level}},
\qquad
\underbrace{\lambda_{c,t} \in \arg\min_{\|\xi\|\le \mu_c \lambda_{c,n}} \xi^\top (J_{c,t} q_{\mathrm{acc}})}_{\text{Coulomb max-dissipation}}.
$$

With the friction cone $K_c^* = \{(\lambda_n,\lambda_t) : \|\lambda_t\|\le\mu_c\lambda_n\}$
and $K^* = \prod_c K_c^*$, this is the cone-complementarity problem (CCP)

$$
\widetilde{M}\, q_{\mathrm{acc}} = F^{\mathrm{sm}}_k + J^\top \lambda,
\qquad
\lambda \in K^*,
\qquad
-J q_{\mathrm{acc}} \in N_{K^*}(\lambda),
$$

equivalently the KKT condition of the **constrained** convex QP

$$
q_{\mathrm{acc}}^* = \arg\min_{q_{\mathrm{acc}}}\;
\tfrac{1}{2}\bigl(q_{\mathrm{acc}} - q_{\mathrm{acc}}^{\mathrm{free}}\bigr)^\top
\widetilde{M}
\bigl(q_{\mathrm{acc}} - q_{\mathrm{acc}}^{\mathrm{free}}\bigr)
\;+\;\sum_i \mathbb{1}_{K_i}\!\bigl(J_i q_{\mathrm{acc}}\bigr),
$$

where $\widetilde{M}\, q_{\mathrm{acc}}^{\mathrm{free}} = F^{\mathrm{sm}}_k$;
$K_i$ is the per-row admissible cone (the normal row uses $K_i = \mathbb{R}_{\ge 0}$,
the tangential rows of one contact are jointly constrained by the friction
cone); $\mathbb{1}_{K_i}$ is the convex indicator (zero on $K_i$, $+\infty$
off); and the optimal multiplier
$\lambda_i^* \in -\partial \mathbb{1}_{K_i}(J_i q_{\mathrm{acc}}^*)$
recovers the cone complementarity. In short: **hard contact = indicator-function
form**.

### From CCP to the convex program — two relaxations

Two ingredients turn the indicator-function form into the smooth convex program
mujoco-warp solves; both have a clear physical meaning.

**(A) Constraint-stabilization shift $a_{\mathrm{ref}}$.** Replace
$J_i q_{\mathrm{acc}} \in K_i$ by $J_i q_{\mathrm{acc}} - a_{\mathrm{ref},i} \in K_i$.
The shift $a_{\mathrm{ref},i}$ is a **target rate of change in constraint
space**: positive ⇒ contact should push apart faster, negative ⇒ existing
penetration / closing velocity that should be undone. The hard form would only
prevent further closing of an already-violated constraint; the shifted form
actively pulls it back to feasibility (Baumgarte / spring-damper trick).
Define the constraint-space residual

$$
r := J\, q_{\mathrm{acc}} - a_{\mathrm{ref}}.
$$

**(B) Smooth barrier $s_i$.** Replace the indicator $\mathbb{1}_{K_i}$ by a
finite-valued convex penalty that grows quadratically with the violation:

$$
s_i(r) = \tfrac{1}{2}\,D_i\,\mathrm{dist}\bigl(r,\,K_i\bigr)^2,
\qquad
\lambda_i := -s_i'(r_i)\;\;(\text{now finite}),
$$

with row-specific stiffness $D_i$. For unilateral $K_i = \mathbb{R}_{\ge 0}$:

$$
s_i^{\mathrm{uni}}(r) = \tfrac{1}{2}\,D_i\,\min(r,0)^2
\;\;\Longrightarrow\;\;
\lambda_i = -D_i\,\min(r_i,0) \ge 0
$$

— a finite spring force whose stiffness is $D_i$. The hard CCP is recovered as
$D_i \to \infty$ and $a_{\mathrm{ref}} \to 0$.

### What the solver actually solves

After both relaxations:

$$
\boxed{\;\;
q_{\mathrm{acc}}^* \;=\; \arg\min_{q_{\mathrm{acc}}}\;
\tfrac{1}{2}\bigl(q_{\mathrm{acc}} - q_{\mathrm{acc}}^{\mathrm{free}}\bigr)^\top
\widetilde{M}
\bigl(q_{\mathrm{acc}} - q_{\mathrm{acc}}^{\mathrm{free}}\bigr)
\;+\;\sum_i s_i(r_i),
\qquad
r_i = J_i q_{\mathrm{acc}} - a_{\mathrm{ref},i}.
\;\;}
$$

First-order optimality

$$
\widetilde{M}\bigl(q_{\mathrm{acc}}^* - q_{\mathrm{acc}}^{\mathrm{free}}\bigr) - J^\top \lambda^* = 0,
\qquad
\lambda_i^* = -s_i'(r_i^*),
$$

reproduces the IMPLICITFAST balance, and the contact force we owed is

$$
f_{c,k} \;=\; R_c^\top \lambda_c^*.
$$

### Concrete choices in mujoco-warp

The two relaxations leave $a_{\mathrm{ref}}$ and $s_i$ as **modeling choices**.
mujoco-warp picks them as follows.

**(i) Spring-damper $a_{\mathrm{ref}}$ + per-row stiffness $D$.** Each
contact carries time constant $T$, damping ratio $\zeta$, impedance shape
$(d_{\min}, d_{\max}, w, m, p)$, body-pair inverse-mass weight $w_M$. With
gap $\text{pos} = d_c$, $v = J\dot q$,

$$
k = \frac{1}{d_{\max}^2 T^2 \zeta^2},
\qquad
b = \frac{2}{d_{\max} T},
$$

$$
\mathrm{imp}(\text{pos}) = d_{\min} + (d_{\max}-d_{\min}) \cdot
\begin{cases}
\dfrac{1}{m^{p-1}}\!\left(\dfrac{|\text{pos}|}{w}\right)^{\!p}, & |\text{pos}|/w < m,\\[6pt]
1 - \dfrac{1}{(1-m)^{p-1}}\!\left(1-\dfrac{|\text{pos}|}{w}\right)^{\!p}, & \text{else,}
\end{cases}
$$

$$
a_{\mathrm{ref}} = \underbrace{-k\,\mathrm{imp}(\text{pos})\,\text{pos}}_{\text{spring on the gap}}
\;\underbrace{- b\,v}_{\text{damper}},
\qquad
D = \frac{\mathrm{imp}}{w_M\,(1-\mathrm{imp})}.
$$

So $a_{\mathrm{ref}}$ is a **PD controller in constraint space** that pulls the
gap to zero, and $D$ is a gap-dependent stiffness that modulates the soft
barrier of relaxation (B).

**(ii) Soft Coulomb cone (elliptic).** For a frictional contact, $K_i$ is the
friction cone $K_c^*$ jointly across the rows of one contact, and the per-contact
penalty is $\tfrac{1}{2} D_0 \cdot \mathrm{dist}(r, K_c^*)^2$. Closed form:
with cone-aspect parameter $\eta$, $\mu := \mu_{c,1}/\sqrt{\eta}$,
$N := \mu r_0$,
$\sigma := \sqrt{\sum_{j\ge 1}(\mu_{c,j} r_j)^2}$,
$d_m := D_0/(\mu^2(1+\mu^2))$,

$$
\lambda_i =
\begin{cases}
0, & N \ge \mu \sigma \;\text{ (top: cone interior)},\\[2pt]
-D_i\,r_i, & \mu N + \sigma \le 0 \;\text{ (bottom: deep penetration)},\\[2pt]
-d_m(N - \mu \sigma)\,\mu, & i = 0,\;\text{middle (cone surface, normal)},\\[2pt]
\dfrac{d_m(N - \mu \sigma)\,\mu}{\sigma}\,\mu_{c,i}^2 r_i, & i \ge 1,\;\text{middle (cone surface, tangent)}.
\end{cases}
$$

The middle case is the metric projection of $r$ onto $K_c^*$.

**(iii) Pyramidal LP relaxation.** Approximate each Lorentz cone by its
inscribed pyramid $\{f : |f_{tj}| \le \mu_j f_n\}$. Each pyramid edge becomes
one independent unilateral row of type (i), so the per-contact coupling in (ii)
disappears and a frictional contact ($m_c = 3$) contributes
$2(m_c - 1) = 4$ decoupled rows.

**(iv) Frozen contact data.** $\mathcal{A}_k, R_c, d_c, \mu_c$ are evaluated
once at frame $k$ — the same freezing trick used for $f_{c,k}$ and $\tau_k$
in the previous section, extended to the contact set itself.

### Code mapping

| object                                                                | code                                       | location                                       |
| --------------------------------------------------------------------- | ------------------------------------------ | ---------------------------------------------- |
| $\mathcal{A}_k, R_c, d_c, \mu_c, (T,\zeta), (d_{\min},\dots,p), \eta$ | `Contact` / `write_contact` / `collision`  | `types.py:1617`, `collision_core.py:160`, `collision_driver.py:752` |
| $J$, $a_{\mathrm{ref}}$, $D$ assembly                                 | `make_constraint` / `_efc_row`             | `constraint.py:2207` / `:51`                   |
| Pyramidal $J^{(c,i)}$                                                 | `_contact_pyramidal`                       | `constraint.py:2661`, `:1864`                  |
| Elliptic $J^{(c,i)}$                                                  | `_contact_elliptic`                        | `constraint.py:2720`, `:2117`                  |
| Per-row $s_i$ and $\lambda_i$                                         | `update_constraint_efc`                    | `solver.py:1800`, `:1857`, `:1880`             |
| $F^{\mathrm{sm}}_k$, $\widetilde{M}$, $q_{\mathrm{acc}}^{\mathrm{free}}$ | `qfrc_smooth`, IMPLICITFAST LHS, `qacc_smooth` | `forward.py` (smooth assembly + integrator)    |
| Newton: $H = \widetilde{M} + J^\top D J$                              | `_JTDAJ_sparse` / `_dense_tiled`           | `solver.py:2929` / `:2944`                     |
| CG: $\widetilde{M}^{-1}$ preconditioner                               | `smooth.solve_m`                           | `solver.py:2923`                               |
| Line search                                                           | `_linesearch_parallel` / `_iterative`      | `solver.py:480` / `:1342`                      |
| $\sum_c \Delta J_c^\top f_{c,k} = J^\top \lambda^*$ scatter to dofs   | `update_constraint_init_qfrc_constraint_*` | `solver.py:1948` / `:1981`                     |

