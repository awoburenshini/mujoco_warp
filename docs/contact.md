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

