# Contact

## Geometry

Bodies $b_1, b_2$ with poses $G_{b_i}(q) \in SE(3)$. Each carries a body-local contact representative $X_i \in \mathbb R^3$. World-frame contact representatives:

$$
p_i(q) \;=\; G_{b_i}(q)\, X_i, \qquad i = 1, 2.
$$

Contact frame, orthonormal:

$$
F_c \;=\; [\,n_c;\; \mathbf{t}^1_c;\; \mathbf{t}^2_c\,] \;\in\; SO(3).
$$

Signed normal gap:

$$
g_n(q) \;:=\; n_c^\top\big(p_2(q) - p_1(q)\big), \qquad g_n > 0 \text{ separated},\; g_n < 0 \text{ penetrating}.
$$

## Contact configuration

Position-level configuration ($F_c$ from narrowphase, frozen):

$$
\Phi_c(q) \;:=\; F_c^\top\big(p_2(q) - p_1(q)\big) \;=\; \begin{bmatrix} g_n \\ g_{\mathbf{t}}^1 \\ g_{\mathbf{t}}^2 \end{bmatrix}_{[0:\text{condim}_c]}, \qquad g_n := n_c^\top(p_2 - p_1), \quad g_{\mathbf{t}}^a := (\mathbf{t}^a_c)^\top(p_2 - p_1).
$$

condim=1: only $g_n$ (frictionless). condim=3: $(g_n, g_{\mathbf{t}}^1, g_{\mathbf{t}}^2)$. Higher (torsion/rolling) recovered by multi-point contact + condim=3, ignored here.

Velocity (with $F_c$ constant in time):

$$
v^{\text{rel}}_c \;:=\; \dot p_2 - \dot p_1 \;\in\; \mathbb R^3, \qquad \dot\Phi_c \;=\; F_c^\top\, v^{\text{rel}}_c \;=\; \begin{bmatrix} \dot g_n \\ \dot g_{\mathbf{t}}^1 \\ \dot g_{\mathbf{t}}^2 \end{bmatrix}_{[0:\text{condim}_c]}.
$$


## Jacobian construction

Decompose into each body's linear velocity at $p_c$:

$$
v^{\text{rel}}_c \;=\; \dot p_2(p_c) - \dot p_1(p_c).
$$

Each $\dot p_i(p_c)$ is **linear in $\dot q$** (rigid body kinematics), so it factors as

$$
\dot p_b(p_c) \;=\; J^{p_c}_b\, \dot q, \qquad J^{p_c}_b \;\in\; \mathbb R^{3 \times n_v}.
$$

Matrix expansion (column $k$ = effect of dof $k$ on body $b$'s linear velocity at $p_c$):

$$
\dot p_b(p_c)
\;=\;
\underbrace{
\begin{bmatrix}
\vert & \vert & & \vert \\
J^{p_c}_{b,0} & J^{p_c}_{b,1} & \cdots & J^{p_c}_{b,n_v - 1} \\
\vert & \vert & & \vert
\end{bmatrix}
}_{J^{p_c}_b}
\begin{bmatrix}
\dot q_0 \\
\dot q_1 \\
\vdots \\
\dot q_{n_v - 1}
\end{bmatrix}
\;=\; \sum_{k = 0}^{n_v - 1} J^{p_c}_{b,k}\, \dot q_k.
$$

Tree-structured sparsity:

$$
J^{p_c}_{b,k} \;=\; 0 \quad \text{for } k \notin \text{ancestors}(b).
$$

Subtract to get the relative version, factoring $\dot q$ out:

$$
v^{\text{rel}}_c \;=\; \big(J^{p_c}_{b_2} - J^{p_c}_{b_1}\big)\, \dot q \;=\; \sum_{k = 0}^{n_v - 1} \big(J^{p_c}_{b_2,k} - J^{p_c}_{b_1,k}\big)\, \dot q_k.
$$

Project onto the contact frame:

$$
\dot\Phi_c \;=\; F_c^\top\, v^{\text{rel}}_c \;=\; \underbrace{F_c^\top\,\big(J^{p_c}_{b_2} - J^{p_c}_{b_1}\big)}_{=:\;J^c}\, \dot q, \qquad J^c \;\in\; \mathbb R^{\text{condim}_c \times n_v}.
$$

### Column formula — derivation from $G_b(q)\,X$

Homogeneous coordinates: $\hat p_c = (p_c, 1)^\top$, $\hat X = (X, 1)^\top$ (body-fixed). Then

$$
\hat p_c \;=\; G_b(q)\, \hat X.
$$

Differentiate in time ($\dot{\hat X} = 0$):

$$
\dot{\hat p}_c \;=\; \dot G_b(q)\, \hat X \;=\; \sum_{k\in\text{ancestors}(b)} \frac{\partial G_b}{\partial q_k}\, \dot q_k\, \hat X.
$$

Chain split at joint $k$ (see [inertia.md](inertia.md)):

$$
G_b(q) \;=\; P_k^I(q)\, G_k(q_k)\, D^I_{k^+\leftarrow b}(q).
$$

$P_k^I$ uses dofs upstream of $k$, $G_k(q_k)$ is joint $k$'s local transform, $D^I_{k^+\leftarrow b}$ uses dofs downstream. Only $G_k$ depends on $q_k$, so

$$
\frac{\partial G_b}{\partial q_k} \;=\; P_k^I\, G_k'(q_k)\, D^I_{k^+\leftarrow b} \;=\; \underbrace{P_k^I\, G_k'(q_k)\, G_k^{-1}(q_k)\, P_k^{I,-1}}_{=:\;\widehat\xi_k^{\text{world}}}\, G_b(q).
$$

$\widehat\xi_k^{\text{world}}$ is joint $k$'s infinitesimal twist in world frame (a $4\times 4$ Lie-algebra element):

$$
\widehat\xi_k^{\text{world}} \;=\; \begin{cases}
\begin{bmatrix} [\bar\omega^{\text{world}}_k]_\times & -[\bar\omega^{\text{world}}_k]_\times\, a_k \\ 0 & 0 \end{bmatrix} & \text{hinge} \\[8pt]
\begin{bmatrix} 0 & \bar v^{\text{world}}_k \\ 0 & 0 \end{bmatrix} & \text{slide}
\end{cases}
\;=\;
\begin{bmatrix} [\bar\omega^{\text{world}}_k]_\times & -[\bar\omega^{\text{world}}_k]_\times\, a_k + \bar v^{\text{world}}_k \\ 0 & 0 \end{bmatrix}.
$$

Unified form: $\bar v = 0$ for hinge, $\bar\omega = 0$ for slide. $a_k$ is the world-frame joint anchor.

Substitute, noting $G_b(q)\,\hat X = \hat p_c$:

$$
\dot{\hat p}_c \;=\; \sum_{k} \dot q_k\, \widehat\xi_k^{\text{world}}\, \hat p_c \;=\; \sum_k \dot q_k \begin{bmatrix} [\bar\omega^{\text{world}}_k]_\times\, p_c - [\bar\omega^{\text{world}}_k]_\times\, a_k + \bar v^{\text{world}}_k \\ 0 \end{bmatrix}.
$$

Use $[\omega]_\times r = \omega \times r$ to read off the top block:

$$
\dot p_c \;=\; \sum_k \dot q_k \big(\bar\omega^{\text{world}}_k \times (p_c - a_k) + \bar v^{\text{world}}_k\big).
$$

Identify the column of $J^{p_c}_b$:

$$
\boxed{\;J^{p_c}_{b,k} \;=\; \bar\omega^{\text{world}}_k \times (p_c - a_k) + \bar v^{\text{world}}_k \;\in\; \mathbb R^3,\qquad k \in \text{ancestors}(b).\;}
$$

Only $p_c$-dependent term is the lever arm $\bar\omega \times (p_c - a_k)$ — changing reference point only changes this piece (cheap shift, see code: `support.jac_dof`).

## Physical laws

Wrench in contact frame: $\lambda_c = (\lambda_n, \lambda_{\mathbf{t}}^1, \lambda_{\mathbf{t}}^2)$.

Signorini:

$$
g_n \geq 0, \qquad \lambda_n \geq 0, \qquad g_n\, \lambda_n = 0.
$$

Coulomb cone:

$$
\big\|(\lambda_{\mathbf{t}}^1, \lambda_{\mathbf{t}}^2)\big\| \;\leq\; \mu_s\, \lambda_n.
$$

Slip law (with $v_{\mathbf{t}} := (\dot g_{\mathbf{t}}^1, \dot g_{\mathbf{t}}^2)$):

$$
v_{\mathbf{t}} = 0 \;\Longrightarrow\; \lambda_{\mathbf{t}} \in \operatorname{int}\mathcal K_{\mu_s} \quad(\text{stick}),
$$

$$
v_{\mathbf{t}} \neq 0 \;\Longrightarrow\; \lambda_{\mathbf{t}} = -\mu_s\, \lambda_n\, \frac{v_{\mathbf{t}}}{\|v_{\mathbf{t}}\|} \quad(\text{slide}).
$$

## Time-stepping & Linearization

$$
\begin{aligned}
\begin{cases}
v^{t + 1} = v^t + \ddot{q} \Delta t \\
q^{t + 1} = q^t + v^{t + 1} \Delta t
\end{cases}
\end{aligned}
$$
So
$$
q^{t + 1} = q^t + v^t \Delta t + \ddot{q} \Delta t^2.
$$

### Linearize $g_n$ around $q^t$ (Signorini):

Put it into the $g_n$, now
$$
\begin{aligned}
0 &\leq g_n(q^{t + 1}) \\
&\approx g_n(q^t) + \nabla g_n(q^t)^\top (v^t \Delta t + \ddot{q} \Delta t^2)\\
&= g_n(q^t) + J_n^c (v^t \Delta t + \ddot{q} \Delta t^2)
\end{aligned}
$$
So
$$
\begin{aligned}
J_n^c \ddot{q}  &\geq -\frac{1}{\Delta t^2} g_n(q^t) - \frac{1}{\Delta t} J_n^c v^t. \\
\end{aligned}
$$

Rewrite as $J_n^c\, \ddot q \geq a^{\text{ref}}_n$ with

$$
a^{\text{ref}}_n \;:=\; -K_p\, g_n \;-\; K_d\, J_n^c v^t.
$$

Discrete derivation pinned the coefficients to $\Delta t$:

$$
K_p^{\text{discrete}} \;=\; \frac{1}{\Delta t^2}, \qquad K_d^{\text{discrete}} \;=\; \frac{1}{\Delta t}.
$$

Code (`solref = (\tau, d)`, see [_efc_row](../mujoco_warp/_src/constraint.py#L52)) replaces $\Delta t$ by user-settable $\tau$ and adds damping ratio $d$:

$$
K_p^{\text{code}} \;=\; \frac{1}{\tau^2\, d^2}, \qquad K_d^{\text{code}} \;=\; \frac{2}{\tau}.
$$

Discrete case $\equiv$ $\tau = 2\Delta t,\; d = \tfrac{1}{2}$. Default `solref = (0.02, 1)` → $\tau \gg \Delta t$ + critical damping (soft contact, residual penetration decays over several steps).

### Discretize elliptic Coulomb cone at $q^t$

$$
J^c =
\begin{bmatrix}
J_n^c \\
J_{\mathbf t}^c
\end{bmatrix},
\qquad
J_n^c \in \mathbb R^{1\times n_v},
\qquad
J_{\mathbf t}^c \in \mathbb R^{2\times n_v}.
$$


$$
\boxed{
v_{\mathbf t}^{t+1}
=
J_{\mathbf t}^c v^t
+
\Delta t\, J_{\mathbf t}^c \ddot q.
}
$$

Stick target $v_{\mathbf t}^{t+1} = 0$ gives $J_{\mathbf t}^c \ddot q = -\tfrac{1}{\Delta t}\, J_{\mathbf t}^c v^t$. Rewrite as $J_{\mathbf t}^c \ddot q = a^{\text{ref}}_{\mathbf t}$ with

$$
a^{\text{ref}}_{\mathbf t} \;:=\; -K_d \cdot J_{\mathbf t}^c v^t, \qquad K_p = 0\ \text{(no position drive for tangent)}.
$$

(Code: `pos_aref = 0`, `Jqvel = J_t^c v^t`, then `aref = -K_d · Jqvel`.)

Discrete: $K_d^{\text{discrete}} = \tfrac{1}{\Delta t}$. Code (`ref = solref` or `solreffriction $= (\tau, d)$`): $K_d^{\text{code}} = \tfrac{2}{d_{\max} \tau}$ — same form as normal's $K_d$; only $K_p$ differs (0 vs nonzero).

The contact wrench in the contact frame is

$$
\lambda_c =
\begin{bmatrix}
\lambda_n \\
\lambda_{\mathbf t}
\end{bmatrix}
=
\begin{bmatrix}
\lambda_n \\
\lambda_{\mathbf t}^1 \\
\lambda_{\mathbf t}^2
\end{bmatrix},
\qquad
\lambda_{\mathbf t}
=
\begin{bmatrix}
\lambda_{\mathbf t}^1 \\
\lambda_{\mathbf t}^2
\end{bmatrix}.
$$

The elliptic Coulomb cone is kept exactly as

$$
\boxed{
\|\lambda_{\mathbf t}\|_2
\leq
\mu_s \lambda_n,
\qquad
\lambda_n \geq 0.
}
$$

Equivalently,

$$
\boxed{
(\lambda_n,\lambda_{\mathbf t})
\in
\mathcal K_{\mu_s}
:=
\left\{
(\lambda_n,\lambda_{\mathbf t})
\;\middle|\;
\lambda_n \geq 0,\;
\lambda_{\mathbf t}^\top \lambda_{\mathbf t}
\leq
\mu_s^2 \lambda_n^2
\right\}.
}
$$

This is a second-order cone constraint in the contact force variables.

## Optimization problem

$$
\phi(\ddot q) \;=\; \underbrace{\tfrac12 (\ddot q - \ddot q_{\text{smooth}})^{\top} M(q^t)\,(\ddot q - \ddot q_{\text{smooth}})}_{\text{Gauss term}} \;+\; \sum_{c}\, s_{\mathcal{K}_c}\!\big(\underbrace{J_c\ddot q - a^{\text{ref}}_c}_{=:\,a_c}\big),
$$

The per-contact penalty $s_{\mathcal{K}_c}$ has three forms of increasing generality: frictionless, isotropic Coulomb, anisotropic. Each case adds one more piece of geometry on top of the previous.

Common notation across all three cases:

1) Per-row split of the residual:
$$
a_c \;=\; (a_n,\; a_t^{1},\; a_t^{2})_{[0:\text{condim}_c]}, \qquad a_n \in \mathbb R,\;\; a_t \in \mathbb R^{\text{condim}_c - 1}.
$$

2) Per-row regularization (from $(\texttt{solref}, \texttt{solimp})$, stored at `d.efc.D[i]`):
$$
D_i \;=\; \text{row }i\text{'s }D, \qquad D_n \;:=\; D_{i_n(c)} \quad\text{($i_n(c)$ = normal row of contact $c$)}.
$$

3) Physical Coulomb friction (cone constraint $\|\lambda_t\|\leq\mu_s\lambda_n$):
$$
\mu_s \;=\; \texttt{friction[0]}.
$$

---

### Case 1 — Frictionless ($\mu_s = 0$)

In MJWarp this is $\text{condim}_c = 1$: only the normal row exists, $a_c = (a_n)$, and the constraint set is the half-space $\mathcal{H} = \{a_n \geq 0\}$. The soft penalty is the Moreau envelope of $\delta_\mathcal{H}$:

$$
\boxed{\;
s_{\mathcal{K}_c}(a_n)
\;=\; \tfrac12\, D_n\, \min(a_n,\, 0)^{2}
\;=\; \begin{cases} 0 & a_n \geq 0 \\[4pt] \tfrac12\, D_n\, a_n^{2} & a_n < 0 \end{cases}
\;}
$$

Geometry: $\mathcal{H}$ is flat (no apex), so projection has only **two** regions — inside ($a_n \geq 0$, dist $= 0$) and outside ($a_n < 0$, dist $= |a_n|$).

Contact force:
$$
\lambda_n \;=\; -\,D_n\, \min(a_n,\, 0) \;=\; D_n\, \max(0,\, -a_n) \;\geq\; 0.
$$

In MJWarp the kernel uses the generic "limit / frictionless / pyramidal" branch ([solver.py:1871-1879](../mujoco_warp/_src/solver.py#L1871-L1879)), not the elliptic-cone block.

---

### Case 2 — Isotropic Coulomb friction ($\mu_s > 0$, all tangent scales $= \mu_s$, $\texttt{impratio} = 1$)

$\text{condim}_c \geq 3$. The constraint set in force space is the tipped second-order cone

$$
\mathcal{K} \;=\; \{(\lambda_n, \lambda_t) : \lambda_n \geq 0,\; \|\lambda_t\|_2 \leq \mu_s \lambda_n\}.
$$

The cone has an apex at the origin, so projection onto $\mathcal{K}$ splits the $(a_n, \|a_t\|_2)$-plane into **three** regions (Top = inside, Middle = nearest point on slant surface, Deep = apex):

$$
\boxed{\;
s_{\mathcal{K}_c}(a_c) \;=\; \begin{cases}
    0 & a_n \geq \mu_s\,\|a_t\|_2 \\[4pt]
    \dfrac{D_c}{2(1+\mu_s^{2})}\,\big(a_n - \mu_s\,\|a_t\|_2\big)^{2} & \text{otherwise} \\[4pt]
    \dfrac{D_c}{2}\,\big(a_n^{2} + \|a_t\|_2^{2}\big) \;=\; \dfrac{D_c}{2}\,\|a_c\|_2^{2} & \mu_s\, a_n + \|a_t\|_2 \leq 0
\end{cases}
\;}
$$

**Per-row $D$ in Case 2.** The 3 rows $i \in \{n, t_1, t_2\}$ of one contact share the same `solref/solimp` and (under impratio = 1, isotropic friction) the same invweight, so by [`_efc_row`](../mujoco_warp/_src/constraint.py#L113)

$$
D_n = D_{t,1} = D_{t,2} \;=:\; D_c \;=\; \frac{\text{imp}}{\text{invweight}\,(1-\text{imp})} \;>\; 0,
$$

where $\text{invweight} \approx (JM^{-1}J^{\top})_{ii}|_{q_{\text{ref}}}$ is the inverse effective mass at the contact and $\text{imp} \in [\text{dmin}, \text{dmax}]$ is the impedance from `solimp`. So Deep's $\sum_{i \in c} D_i a_i^2$ collapses to a single $D_c \|a_c\|_2^2$ (one regularization constant times the squared Euclidean norm — that's just "distance² from $a_c$ to the apex").

Top and Deep are dual cones with reciprocal slopes ($\mu_s$ vs $1/\mu_s$); $(a_n - \mu_s\|a_t\|_2)/\sqrt{1+\mu_s^2}$ is the signed Euclidean distance to the slant surface, squared and weighted gives Middle.

Force per zone:

| Zone | $\lambda_n$ | $\lambda_t$ |
|---|---|---|
| Top | $0$ | $0$ |
| Middle | $\dfrac{D_c}{1+\mu_s^{2}}\big(\mu_s\|a_t\|_2 - a_n\big) > 0$ | $\|\lambda_t\| = \mu_s\lambda_n$ (classic stick/slip) |
| Deep | $-D_c\, a_n$ | $-D_c\, a_t^{j}$ (no cone coupling) |

Taking $\mu_s \to 0$ collapses the cone to a half-space, Middle and Deep merge into $\tfrac12 D_c a_n^2$, recovering Case 1.

---

### Case 3 — Anisotropic friction (general)

Two extras over Case 2:

4) Per-tangent scales (different friction in each tangent direction):
$$
\mu_{t,j} \;:=\; \texttt{friction}[j-1], \qquad j = 1, \dots, \text{condim}_c - 1.
$$

5) Impedance-ratio rescaling (user-set normal/tangent stiffness ratio):
$$
\mu_c \;:=\; \mu_s\,/\,\sqrt{\texttt{impratio}}, \qquad \texttt{impratio} = 1 \;\Longrightarrow\; \mu_c = \mu_s.
$$

6) Anisotropy-weighted tangent norm replaces the ordinary 2-norm:
$$
\|a_t\|_\mu \;:=\; \sqrt{\,\sum_j (\mu_{t,j}/\mu_s)^{2}\,(a_t^{j})^{2}\,}, \qquad \mu_{t,j} \equiv \mu_s \;\Longrightarrow\; \|a_t\|_\mu = \|a_t\|_2.
$$

The three-zone formula is **structurally identical** to Case 2 — just replace $\|a_t\|_2$ by $\|a_t\|_\mu$ and the normalizing $\mu_s^2$ by $\mu_c^2$:

$$
\boxed{\;
s_{\mathcal{K}_c}(a_c) \;=\; \begin{cases}
    0 & a_n \geq \mu_s\,\|a_t\|_\mu \quad\text{(Top)} \\[4pt]
    \dfrac{D_n}{2(1+\mu_c^{2})}\,\big(a_n - \mu_s\,\|a_t\|_\mu\big)^{2} & \text{otherwise (Middle)} \\[4pt]
    \tfrac12 \sum_{i \in c} D_i\, a_i^{2} & \mu_c^{2}\,a_n + \mu_s\,\|a_t\|_\mu \leq 0 \quad\text{(Deep)}
\end{cases}
\;}
$$

This matches the kernel implementation ([solver.py:1880-1937](../mujoco_warp/_src/solver.py#L1880-L1937)), which computes $N = \mu_c a_n$, $u_j = \mu_{t,j} a_t^j$, $T = \|u\|_2$ as scaled intermediates and uses $N \geq \mu_c T$, $\mu_c N + T \leq 0$ as zone tests — those are exactly the boxed conditions above after substituting $T = \mu_s \|a_t\|_\mu$.

Setting $\mu_{t,j} \equiv \mu_s$ and $\texttt{impratio} = 1$ recovers Case 2; further setting $\mu_s = 0$ recovers Case 1.


