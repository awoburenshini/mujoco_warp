# Contact

Convex-vs-convex contact between two rigid bodies $b_1, b_2$. Narrowphase (GJK/EPA) emits per contact $c$

$$
(p_c,\; F_c = [n_c;\, t^1_c;\, t^2_c],\; g_c,\; \mu_c,\; \text{condim}_c),
$$

with $g_c$ = signed gap, $F_c$ = orthonormal contact frame (rows = normal + tangents). Below: physical model → acceleration-level complementarity → Jacobian construction → solver QP.

---

## 1. Physical constraint model

Let $v^{\text{rel}}_c := \dot p_{b_2}(p_c) - \dot p_{b_1}(p_c) \in \mathbb{R}^3$ (relative velocity at the contact point), $\lambda_c = (\lambda_n, \lambda^1_t, \lambda^2_t) \in \mathbb{R}^3$ (contact wrench in frame $F_c$).

**Signorini (non-penetration, unilateral):**

$$
g_c \geq 0, \qquad \lambda_n \geq 0, \qquad g_c \cdot \lambda_n = 0.
$$

**Coulomb friction cone (SOC):**

$$
\big(\lambda^1_t,\lambda^2_t\big) \in \mathcal{K}_{\mu_c}(\lambda_n) := \big\{ \|(\lambda^1_t,\lambda^2_t)\| \leq \mu_s\,\lambda_n \big\}.
$$

**Slip law:**

$$
v_t := (t_1^\top v^{\text{rel}}_c,\; t_2^\top v^{\text{rel}}_c), \qquad
\begin{cases} v_t = 0: & \lambda_t \in \text{int}\,\mathcal{K}_{\mu}\;\text{(sticking)} \\ v_t \neq 0: & \lambda_t = -\mu_s\,\lambda_n\, \dfrac{v_t}{\|v_t\|}\;\text{(sliding, on cone boundary)} \end{cases}
$$

**This is NCP + SOC, not equality.** $\lambda_n$ has a sign constraint, $\lambda_t$ is cone-bounded.

---

## 2. Acceleration-level complementarity

The integrator updates $q$ via $\ddot q$, but $g_c$ is at position level. Differentiate twice:

$$
\dot g_c = n_c^\top v^{\text{rel}}_c =: J^c_n\, \dot q, \qquad \ddot g_c = J^c_n\, \ddot q + \dot J^c_n\, \dot q.
$$

Baumgarte stabilization with $(\tau, d) = $ `solref`, softened gap $\widetilde g_c := g_c - \text{includemargin}$:

$$
a^{\text{ref}}_n := -\frac{d^2}{\tau^2}\, \widetilde g_c - \frac{2d}{\tau}\, \dot g_c, \qquad a^{\text{ref}}_t := 0.
$$

The §1 laws lifted to acceleration level:

$$
\boxed{\;\begin{aligned}
\text{normal:}\quad & J^c_n\, \ddot q - a^{\text{ref}}_n \;\geq\; 0,\quad \lambda_n \geq 0,\quad \lambda_n\, (J^c_n\, \ddot q - a^{\text{ref}}_n) = 0 \\[2pt]
\text{tangent:}\quad & J^c_t\, \ddot q - a^{\text{ref}}_t \;=\; 0,\quad \|(\lambda_t^1, \lambda_t^2)\| \leq \mu_s\, \lambda_n
\end{aligned}\;}
$$

- **Normal: unilateral.** Contact only pushes ($\lambda_n \geq 0$). When the bodies are separating, $\lambda_n = 0$.
- **Tangent: bilateral target with cone-bounded reaction.** Looks like an equality, but $\lambda_t$ is bounded; saturated $\Rightarrow$ slipping, in which case the equality breaks ($J^c_t \ddot q \neq 0$).

Contrast with `eq_connect / eq_weld / eq_joint / eq_tendon`: those are true equalities with unbounded $\lambda \in \mathbb{R}$. Contact is **never** an equality constraint.

---

## 3. Constructing $J^c$ from kinematics

Need: $J^c$ such that $J^c\, \dot q = E_c (V^{p_c}_{b_2} - V^{p_c}_{b_1})$ — relative spatial velocity at $p_c$ projected to $F_c$, where $E_c$ selects condim directions:

$$
E_c = \begin{bmatrix} 0 & n^\top \\ 0 & t_1^\top \\ 0 & t_2^\top \end{bmatrix}\;(\text{rows up to condim}).
$$

### 3a. Spatial velocity at $p_c$ for body $b$

Reuse the dof Jacobian decomposition from [docs/inertia.md](docs/inertia.md), with reference point shifted from $\tilde x_{r(b)}$ to $p_c$:

$$
J^{p_c}_{b,k} = T^{p_c}_b\, S_k, \qquad T^{p_c}_b = \begin{bmatrix} I & 0 \\ -[p_c - \tilde x_{r(b)}]_\times & I \end{bmatrix}, \qquad S_k = \begin{bmatrix} \bar\omega_k \\ \bar\omega_k \times (\tilde x_{r(k)} - a_k) + \bar v_k \end{bmatrix} = \texttt{d.cdof}[k].
$$

$$
V^{p_c}_b = \begin{bmatrix} \omega_b \\ \dot p_b(p_c) \end{bmatrix} = \sum_{k \in \text{ancestors}(b)} J^{p_c}_{b,k}\, \dot q_k.
$$

### 3b. The shift is computed on-the-fly in `support.jac_dof`

The factor $T^{p_c}_b\, S_k$ is **not stored** — recomputed per dof inside [support.jac_dof](mujoco_warp/_src/support.py#L394):

$$
\begin{aligned}
\text{offset} &:= p_c - \tilde x_{r(b)}                                  &&\texttt{= point - subtree\_com[r(b)]} \\
\bar\omega_k  &:= \texttt{spatial\_top(cdof[k])}                          && = S_k[\text{ang}] \\
S_k[\text{lin}] &:= \texttt{spatial\_bottom(cdof[k])}                     && = \bar\omega_k\times(\tilde x_r - a_k) + \bar v_k \\
\texttt{jacp} &= S_k[\text{lin}] + \bar\omega_k \times \text{offset}      && = \bar\omega_k\times(p_c - a_k) + \bar v_k = J^{p_c}_{b,k}[\text{lin}] \\
\texttt{jacr} &= \bar\omega_k                                              && = J^{p_c}_{b,k}[\text{ang}].
\end{aligned}
$$

`d.contact.pos[c]` enters as the `point` argument that builds `offset`.

### 3c. Assembling one row of $J^c$

Relative motion: $J^{p_c}_{\text{rel},k} := J^{p_c}_{b_2,k} - J^{p_c}_{b_1,k}$. Common ancestors with $r(b_1) = r(b_2)$ cancel automatically ($T^{p_c}_{b_1} = T^{p_c}_{b_2}$).

Contact-frame axis $a$ (normal: $a=0$; tangent: $a=1,2$), column $k$, in [constraint.py:1864](mujoco_warp/_src/constraint.py#L1864):

$$
J^c_{a,k} \;\mathrel{+}=\; F_{a,:} \cdot (\texttt{jac2p}_k - \texttt{jac1p}_k) \quad\text{(linear axes 0–2)}.
$$

Torsion or rolling axes ($a = 3,4,5$ for $\text{condim} \in \{4,6\}$) use `jacr` (angular part) instead. Here $a$ is the axis index within the contact frame; do not confuse with $r(b) = \texttt{body\_rootid}[b]$ from smooth.py.

---

## 4. Soft-constraint convex QP

The §2 NCP+SOC is nonsmooth. MuJoCo softens to a smooth convex QP:

$$
\boxed{\;\min_{\ddot q}\; \tfrac{1}{2} (\ddot q - \ddot q_{\text{s}})^\top M\, (\ddot q - \ddot q_{\text{s}}) \;+\; \sum_i s_i\big(J_i\,\ddot q - a^{\text{ref}}_i\big),\;}
$$

per-row barrier dispatched by `d.efc.type`:

$$
s_i(r) = \begin{cases}
\tfrac{1}{2} D_i\, \min(r, 0)^2 & \text{normal (one-sided, } \lambda_n \geq 0 \text{ via } \lambda_n = -D \min(r,0)\text{)} \\
\tfrac{1}{2} D_i\, r^2 + B_{\mu}(\lambda) & \text{tangent (two-sided + cone barrier)} \\
\tfrac{1}{2} D_i\, r^2 & \text{equality (two-sided, unbounded }\lambda\text{)}
\end{cases}
$$

$D_i \in \mathbb{R}_+$ from `solimp` (larger $D$ → harder constraint). First-order optimality:

$$
M\,(\ddot q - \ddot q_{\text{s}}) + J^\top \lambda = 0, \qquad \lambda_i = -s_i'\big(J_i \ddot q - a^{\text{ref}}_i\big).
$$

For the normal row, $\lambda_n = -D_n \min(r, 0) \geq 0$ — Signorini sign automatically. As $D \to \infty$ and $B_\mu$ steepens, §1 hard conditions recovered.

---

## 5. Code map

| symbol | code |
|---|---|
| $p_c, F_c, g_c, \mu_c, \text{condim}_c$ | `d.contact.{pos, frame, dist, friction, dim}` |
| $\widetilde g_c$ | `d.contact.dist - d.contact.includemargin` |
| $(\tau, d)$, solimp | `d.contact.{solref, solimp}` |
| $S_k$ | `d.cdof[k]` |
| $\tilde x_{r(b)}$ | `d.subtree_com[r(b)]` |
| $J^{p_c}_{b,k}$ | computed on the fly by `support.jac_dof(con_pos, b, k)` |
| $J^c_c, a^{\text{ref}}_c, D_c$ | `d.efc.{J, aref, D}[\rho : \rho + \text{condim}]` |
| efc row offset $\rho$ of contact $c$ | `d.contact.efc_address[c]` |
| barrier type per row | `d.efc.type[\rho : \rho + \text{condim}]` $\in$ `{CONTACT_FRICTIONLESS, CONTACT_PYRAMIDAL, CONTACT_ELLIPTIC_*}` |
| $\lambda_c$ (soft multiplier) | recovered by `solver.solve`, written to `d.efc.force` |
