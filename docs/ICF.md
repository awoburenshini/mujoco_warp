# III
$q \in \mathbb{R}^{n_q}, v\in \mathbb{R}^{n_v}, \dot{q} = N(q) v, v_c = Jv + b$

Symplectic Euler:
$$
\begin{aligned}
M(q_0) (v - v_0) &= \delta t \cdot k_1(q, v) + \delta t \cdot \underbrace{k_2(q_0, v_0)}_{\text{gyroscopic term}} + J(q_0)^\top \gamma\\
q &= q_0 + \delta t \cdot N(q_0) v
\end{aligned}
$$

momentum residual:
$$
m(v) = M(q_0)(v - v_0) - \delta t \cdot k_1(q, v) - \delta t \cdot k_2(q_0, v_0)
$$
We linearizes $m(v)$ at $v^*$, here $m(v^*) = 0$.
we have 
$$
\begin{aligned}
A (v - v^*) &= J^{\top} \gamma
\end{aligned}
$$
and $\frac{\partial m}{\partial v}\big|_{v=v^*} = A + O(\delta t)$.

$$
\begin{aligned}
\min_{v} l_p(v) &= \frac{1}{2} \| v - v^* \|^2_A + l_c(v)
\end{aligned}
$$

And
$$
\begin{aligned}
\gamma(v_c; x_0) &= -\frac{\partial l_c}{\partial v_c}\\
l_c(v_c; x_0) &= \sum_{i = 1}^{n_c} l_{c,i}(v_c; x_0)\\
v_{c,i} &= [v_{t,i}, v_{n,i}]
\end{aligned}
$$

## III: B

**Normal force**

$$
\begin{aligned}
f_n(\phi, v_n) &= k\underbrace{(-\phi)_{+}}_{\text{penetration depth}} (1 - d\cdot \underbrace{v_n}_{\text{normal velocity, scalar, >0 if separating, <0 if approaching}})_{+}\\
\end{aligned}
$$
the normal impulse is
$$
\begin{aligned}
n(v_n; \phi_0) = \delta t \cdot f_n(\phi_0 + \delta t v_n, v_n)
\end{aligned}
$$
note that, this is a scalar function, so definitely we can find a $l_n(v_n; \phi_0)$, such that $n(v_n; \phi_0) = -\frac{\partial l_n}{\partial v_n}$. 

$$
\begin{aligned}
\frac{d^2 l_n}{d v_n^2} &= -\delta t^2 \frac{\partial f_n}{\partial \phi} - \delta t \frac{\partial f_n}{\partial v_n}
\end{aligned}
$$

So, $\frac{\partial f_n}{\partial \phi} \leq 0, \frac{\partial f_n}{\partial v_n} \leq 0$.


**Friction force**

$$
\begin{aligned}
\gamma_t(v_c) &= -\mu \cdot n(v_n; \phi_0) \hat{\bf{t}}_s\\
\hat{t}_s &= \frac{v_t}{\sqrt{\|v_t\|^2 + \epsilon_s^2}}
\end{aligned}
$$

**In total**

$$
\begin{aligned}
\gamma(v_c) &= \begin{bmatrix}\gamma_t(v_c)\\ \gamma_n(v_c)\end{bmatrix}\\
&= \begin{bmatrix}-\mu \cdot n(v_n; \phi_0) \hat{\bf{t}}_s\\ n(v_n; \phi_0)\end{bmatrix}
\end{aligned}
$$

## IV: Convex approximation

### SAP model

...

### Lagged model
$$
\begin{aligned}
l(v_c) = l_t(v_t) + l_n(v_n)
\end{aligned}
$$
the $l_n$ defined in III:B, and $l_t$ is defined as
$$
\begin{aligned}
l_t(v_t) &= \gamma_{n0} ( \sqrt{\|\mu\cdot v_t\|^2 + \epsilon_s^2} - \epsilon_s) 
\end{aligned}
$$

When $\mu$ is a scalar, which is the isotropic friction. When $\mu$ is a SPD matrix, which is the anisotropic friction.