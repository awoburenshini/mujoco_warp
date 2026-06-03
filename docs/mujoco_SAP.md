# II

$$
\dot{q} = N(q) v, q\in \mathbb{R}^{n_q}, v\in \mathbb{R}^{n_v}
$$

$N(q)\in\mathbb{R}^{n_q\times n_v}$, generally $n_q \neq n_v$ (rotation representation):

| Joint | position rep ($q$) | velocity rep ($v$) | $N$ block |
|---|---|---|---|
| HINGE / SLIDE | 1 scalar | 1 scalar | $1$ (identity) |
| BALL | unit **quaternion** (4) | angular velocity $\boldsymbol\omega$ (3) | $\tfrac12\,q\otimes[0,\boldsymbol\omega]$ |
| FREE | 3 pos + 4 quat = **7** | 3 lin + 3 ang = **6** | identity (pos) $\oplus$ quat map |

$$
\dot q_{\text{quat}} = \tfrac12\, q_{\text{quat}} \otimes \begin{bmatrix} 0 \\ \boldsymbol\omega \end{bmatrix}.
$$

`mujoco_warp`: `_integrate_pos` ([forward.py:55](../mujoco_warp/_src/forward.py)) —
`else` branch (HINGE/SLIDE) `qpos += dt * qvel` ($N = I$); `BALL`/`FREE` call
`math.quat_integrate` ([math.py:222](../mujoco_warp/_src/math.py)), fusing eqs. (11)–(12):

$$
q_{t+1} = q_t \otimes \exp\!\big(\tfrac12\,\delta t\,\boldsymbol\omega\big).
$$



## II: A

$(p_i, \hat{n}_i, \phi_i(q), v_{n,i}, v_{t,i}) \in \mathcal{C}(q)$ is the contact set. And $n_c$ is the number of contacts. 

We have the following contact relations:
$$
v_c = J(q) v, J(q) \in \mathbb{R}^{n_c \times n_v}
$$

## II: B

- Normal force 

$$
\frac{\gamma_n}{\delta t} = (- k \phi(q) - \tau_d k v_n)_+ \quad\text{(normal force)}
$$

the equivalent of the complementarity condition 
$$
0\leq (\phi + \tau_d v_n + \frac{\delta t}{k} \gamma_n) \perp \gamma_n \geq 0
$$

- Friction force

$$
\begin{aligned}
\gamma_t &= \arg\min_{\|\xi\| \leq \mu \gamma_n} \mathbf{v}_t \cdot \xi\\
\mathcal{F} &= \{[x_t, x_n] \in \mathbb{R}^3 : \| x_t \| \leq \mu x_n \}
\end{aligned}
$$

The optimal solution
$$
\begin{aligned}
&\mu \gamma_n v_t + \lambda \gamma_t = 0, \\
& 0 \leq \lambda \perp (\mu \gamma_n - \| \gamma_t \|) \geq 0
\end{aligned}
$$

## II: C
Mid-step quantities:
$$
\begin{aligned}
q^{\theta} &= \theta q + (1-\theta) q_0\\
v^{\theta} &= \theta v + (1-\theta) v_0\\
v^{\theta_{vq}} &= \theta_{vq} v + (1-\theta_{vq}) v_0\\
\end{aligned}
$$

we denote the next-step velocity as $v$
$$
\begin{aligned}
M(q^{\theta}(v)) (v - v_0) &= \delta t \cdot k(q^{\theta}(v), v^{\theta}) + J(q_0)^\top \gamma\\
0\leq &(\phi_i(q(v)) + \tau_{d,i} v_{n,i}(q,v) + \frac{c_i}{\delta t} ) \perp \gamma_{n,i} \geq 0\\
&\mu_i \gamma_{n,i} v_{t,i}(q,v) + \lambda_i \gamma_{t,i} = 0, \\
& 0 \leq \lambda_i \perp (\mu_i \gamma_{n,i} - \| \gamma_{t,i} \|) \geq 0\\
\dot{q}^{\theta_{vq}}(v) &= N(q^{\theta}(v)) v^{\theta_{vq}}(v)\\
q(v) &= q_0 + \delta t \cdot \dot{q}^{\theta_{vq}}(v)\\
& i \in \mathcal{C}(q_0)
\end{aligned}
$$

Popular schemes for forward dynamics:

| Scheme | $\theta$ | $\theta_{vq}$ | order |
|---|---|---|---|
| Explicit Euler | $0$ | $0$ | 1st |
| Symplectic Euler | $0$ | $1$ | 1st |
| Implicit Euler | $1$ | $1$ | 1st |
| Symplectic midpoint | $1/2$ | $1/2$ | 2nd |

## II: D

$$
m(v) = M(q^{\theta}(v))(v - v_0) - \delta t \cdot k(q^{\theta}(v), v^{\theta})
$$

we use newton's method to solve $m(v^*) = 0$.

and assume 
$k(q,v) = \underbrace{ k_1(q,v)}_{\text{spring and dampers}} + \underbrace{ k_2(q,v)}_{\text{Coriolis and gyroscopic}}
$

we use 
$$
\begin{aligned}
\frac{\partial m}{\partial v}\Big|_{v=v^*} &= M(q^{\theta}(v^*)) + \underbrace{\frac{\partial M(q^{\theta}(v^*))}{\partial v} \cdot(v^* - v^*)}_{0} - \frac{\partial k_1}{\partial v}\Big|_{v=v^*} - \frac{\partial k_2}{\partial v}\Big|_{v=v^*}\\
\end{aligned}
$$

we remove the $\frac{\partial k_2}{\partial v}$ term since it is generally nonn-negative definite.

$$
\frac{\partial m}{\partial v}\Big|_{v=v^*} = A + O(\delta t)
$$

# III: 

## III: A
$$
\begin{aligned}
\min_{v,\sigma} &\quad l_p(v,\sigma) = \frac{1}{2} \| v - v^*\|^2_A + \frac{1}{2} \|\sigma\|^2_R\\
s.t.& \quad\quad g = (J v - \hat{v}_c + R \sigma)\in \mathcal{F}^*
\end{aligned}
$$

$$
\begin{aligned}
\mathcal{F} &= \{[x_t, x_n] \in \mathbb{R}^3 : \| x_t \| \leq \mu x_n \}
\Leftrightarrow
\mathcal{F}^* = \{[y_t, y_n] \in \mathbb{R}^3 :  \| y_t \| \leq \frac{1}{\mu} y_n \}
\end{aligned}
$$

$$
\begin{aligned}
L(v,\sigma, \gamma) &= \frac12 \| v - v^* \|^2_A + \frac12 \|\sigma\|^2_R - \gamma^\top g
\end{aligned}
$$

Optimality condition with respect to $v$ and $\sigma$:
$$
\begin{aligned}
0 &= \frac{\partial L}{\partial v} = A (v - v^*) - J^\top \gamma\\
0 &= \frac{\partial L}{\partial \sigma} = R \sigma - R^\top \gamma
\end{aligned}
$$
So we have
$$
\begin{aligned}
A (v - v^*) &= J^\top \gamma\\
\sigma &= \gamma
\end{aligned}
$$

now, put $\gamma$ back to the Lagrangian, we have
$$
\begin{aligned}
L(v, \sigma, \gamma) &= \frac12 (v - v^*)^\top A (v - v^*) + \frac12 \|\sigma\|^2_R - \gamma^\top (J v - \hat{v}_c + R \sigma)\\
&= \frac12 \gamma^\top J A^{-1} J^\top \gamma + \frac12 \gamma^\top R \gamma - \gamma^\top R \sigma + \gamma^\top \hat{v}_c - \gamma^\top J(v^* + A^{-1} J^\top \gamma) \\
&= -\frac12 \gamma^\top (J A^{-1} J^\top + R) \gamma + \gamma^\top \hat{v}_c - \gamma^\top v_c^*\\
&= -\frac12 \gamma^\top (J A^{-1} J^\top + R) \gamma + \gamma^\top (\hat{v}_c - v_c^*)
\end{aligned}
$$
So, the dual problem is
$$
\begin{aligned}
\gamma^* &= \arg\min_{\gamma} \frac12 \gamma^\top (J A^{-1} J^\top + R) \gamma + \gamma^\top (v^*_c - \hat{v}_c)
\end{aligned}
$$
And we have
$$
\begin{aligned}
\gamma_i(v_{c,i}) &= P_{\mathcal{F}_i}(y_i(v_{c,i}))\\
&= \arg\min_{\gamma_i \in \mathcal{F}_i} \frac12 (\gamma_i - y_i) R_i (\gamma_i - y_i)\\
\end{aligned}
$$

Finally, we can rewrite the primal problem as
$$
\begin{aligned}
\min_{v} l_p(v) &= \frac{1}{2} \| v - v^* \|^2_A + \frac{1}{2} \| P_{\mathcal{F}}(y(v_c)) \|^2_R\\
\end{aligned}
$$