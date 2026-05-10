# Inertia Matrix and Composite Rigid Body Inertias

$$
\begin{aligned}
T &=  \sum_{b} \int_{\text{Body}_b} \rho(\mathbf{X}) \| \dot{x}^I_b + \dot{R}^I_b X  \|^2 \, dX \\
&= \sum_{b} \int_{\text{Body}_b} \rho(\mathbf{X}) \left( \dot{x}^{I\top}_b \dot{x}^I_b + 2\dot{x}^{I\top}_b \dot{R}^I_b X + X^\top \dot{R}^{I\top}_b \dot{R}^I_b X \right) dX \\
&= \sum_{b} \left( m_b \dot{x}^{I\top}_b \dot{x}^I_b + 2\dot{x}^{I\top}_b \dot{R}^I_b \underbrace{\int_{\text{Body}_b} \rho(\mathbf{X}) X \, dX}_{0} + \int_{\text{Body}_b} \rho(\mathbf{X}) X^\top \dot{R}^{I\top}_b \dot{R}^I_b X \, dX \right) \\
&= \sum_{b} \left( m_b \| \dot{x}^I_b \|^2 + \int_{\text{Body}_b} \rho(\mathbf{X}) X^\top \dot{R}^{I\top}_b \dot{R}^I_b X \, dX \right) \\
&= \sum_{b} \left( m_b \| \dot{x}^I_b \|^2 + \int_{\text{Body}_b} \rho(\mathbf{X}) \| \dot{R}^I_b X \|^2 \, dX \right) \\
&= \sum_{b} \left( m_b \| \dot{x}^I_b \|^2 + \int_{\text{Body}_b} \rho(\mathbf{X}) \| R^I_b [\omega^{I,local}_b]_{\times} X \|^2 \, dX \right)\\
&= \sum_{b} \left( m_b \| \dot{x}^I_b \|^2 + \int_{\text{Body}_b} \rho(\mathbf{X}) \| [\omega^{I,local}_b]_{\times} X \|^2 \, dX \right) \\
&= \sum_{b} \left( m_b \| \dot{x}^I_b \|^2 + \omega^{I,local\top}_b \underbrace{\int_{\text{Body}_b} \rho(\mathbf{X}) \left( \| X \|^2 I - X X^\top \right) dX}_{\text{inertia tensor } I_b \text{, diagonal !}} \omega^{I,local}_b \right) \\
&= \sum_{b}  m_b \| \dot{x}^I_b \|^2 + \omega^{I,local\top}_b I_b^{local} \omega^{I,local}_b
\end{aligned}
$$
Note that $\omega^{I,local}_b = R^{I\top}_b \omega^{I,world}_b$,
So
$$
\begin{aligned}
T &= \sum_{b}  m_b \| \dot{x}^I_b \|^2 + \omega^{I,world\top}_b R^I_b I_b^{local} R^{I\top}_b \omega^{I,world}_b \\
&= \sum_{b}  m_b \| \dot{x}^I_b \|^2 + \omega^{I,world\top}_b I_b^{world} \omega^{I,world}_b\\
&= \sum_{b}  \begin{bmatrix} \omega^{I,world}_b \\ \dot{x}^I_b \end{bmatrix}^\top \begin{bmatrix} I_b^{world} & 0 \\ 0 & m_b I \end{bmatrix} \begin{bmatrix} \omega^{I,world}_b \\ \dot{x}^I_b \end{bmatrix}\\
&= \sum_{b}  \begin{bmatrix} \omega^{I,world}_b \\ \dot{x}^I_b \end{bmatrix}^\top \mathcal{I}_b^{world} \begin{bmatrix} \omega^{I,world}_b \\ \dot{x}^I_b \end{bmatrix}\\
&= \sum_{b}  \dot{q}^T J_b^{I,world\top} \mathcal{I}_b^{world} J_b^{I,world} \dot{q} \\
\end{aligned}
$$

and
$$
\begin{aligned}
\begin{bmatrix}
 \omega^{I,world}_b \\
 \dot{x}^I_b
\end{bmatrix} 
&= J_b^{I, world} \dot{q}\\
&= \sum_{k\in\text{ancestors}(b)} J_{b,k}^{I, world} \underbrace{\dot{q}_k}_{\text{Scalar}}\\
\end{aligned}
$$ 
Now, we compute $J_{b,k}^{I, world}$ 

$$
\begin{aligned}
G^I_b(q) :=\begin{bmatrix}
 R_{b}^I(q)& x_b^I(q)\\
 0 & 1
\end{bmatrix} =P_k^I​(q)G_k​(q_k​)D_{k^+\leftarrow b}^I​(q),
\end{aligned}
$$
where
$$
P_k^I = \begin{bmatrix} R_k^I & a_k \\ 0 & 1 \end{bmatrix}\;(q_{\text{up of }k}),\qquad D_{k^+\leftarrow b}^I = \begin{bmatrix} R_{k^+\leftarrow b}^I & d_{k^+\leftarrow b}^I \\ 0 & 1 \end{bmatrix}\;(q_{\text{down of }k}),\qquad a_k = \texttt{d.xanchor}.
$$

```
   world (body 0)
     │
     │  joint 1   q_1, a_1    ┐
     ●  body 1                │
     │                        │
     │  joint 2   q_2, a_2    │   P_k^I
     ●  body 2                ├── (R_k^I, translation = a_k;
     ⋮                        │    uses q_1, …, q_{k-1})
     ●  body k-1              ┘
     │
     │  joint k   q_k, a_k   ─── G_k(q_k)  about a_k
     │
     ●  body k                ┐
     │                        │   D_{k^+ ← b}
     │  joint k+1   …         ├── (uses q_{k+1}, …, downstream of k)
     ●  body k+1              │
     ⋮                        │
     ●  body b  (CoM x_b^I)   ┘
```

$G^I_b = P_k^I \cdot G_k \cdot D_{k^+\leftarrow b}^I$ — composition along this chain (joint k is the splitting point).

$$
G_k(q_k) = \begin{cases}
\begin{bmatrix} \exp([\bar{\omega}^{\text{local}}_k]_\times q_k) & 0 \\ 0 & 1 \end{bmatrix} & \text{hinge} \\[6pt]
\begin{bmatrix} I & q_k \bar{v}^{\text{local}}_k \\ 0 & 1 \end{bmatrix} & \text{prismatic}
\end{cases}
$$

so, for any $X$ in body $b$'s local frame,
$$
\begin{aligned}
\dot{x} &= \dot{x}^I_b + \dot{R}^I_b X \\
&= \dot{x}^I_b + R^I_b [\omega^{I,local}_b]_\times X \\
&= \dot{x}^I_b + [\omega^{I,world}_b]_\times R^I_b X \\
&= [\omega^{I,world}_b]_\times R^I_b X + \dot{x}^I_b \\
\end{aligned}
$$
which means 
$$
\begin{aligned}
\dot{\hat{x}} &= \begin{bmatrix} [\omega^{I,world}_b]_\times & \dot{x}_b \\ 0&0 \end{bmatrix} \begin{bmatrix} R^I_b X \\ 1 \end{bmatrix} \\
&= \begin{bmatrix} [\omega^{I,world}_b]_\times & \dot{x}_b \\ 0&0 \end{bmatrix} \begin{bmatrix} 1 & -x_b^I \\ 0 & 1 \end{bmatrix} \begin{bmatrix} R^I_b & x_b^I \\ 0 & 1 \end{bmatrix} \begin{bmatrix} X \\ 1 \end{bmatrix} \\
&= \begin{bmatrix} [\omega^{I,world}_b]_\times & \dot{x}_b \\ 0&0 \end{bmatrix} \begin{bmatrix} 1 & -x_b^I \\ 0 & 1 \end{bmatrix} G^I_b(q) \begin{bmatrix} X \\ 1 \end{bmatrix} \\
&= \begin{bmatrix} [\omega^{I,world}_b]_\times & \dot{x}_b - [\omega^{I,world}_b]_\times x_b^I \\  0&0 \end{bmatrix} \hat{x} \\
\end{aligned}
$$


another way to see the same thing: 
$$
\begin{aligned}
\dot{\hat{x}} &= \dot{G}^I_b(q) \underbrace{\hat{X}}_{\begin{bmatrix} X \\ 1 \end{bmatrix}} \\
&= \sum_{k\in\text{ancestors}(b)} \frac{\partial G^I_b}{\partial q_k} \dot{q}_k \hat{X} \\
&= \sum_{k\in\text{ancestors}(b)}  \partial_k [P_k^I G_k(q_k) D_{k^+\leftarrow b}^I] \dot{q}_k \hat{X} \\ 
&= \sum_{k\in\text{ancestors}(b)}  P_k^I \partial_k [G_k(q_k)] D_{k^+\leftarrow b}^I \dot{q}_k \hat{X} \quad\quad \text{remembering that } \dot{q}_k \text{ is a scalar}\\
&= \sum_{k\in\text{ancestors}(b)} \dot{q}_k P_k^I \partial_k [G_k(q_k)] D_{k^+\leftarrow b}^I \hat{X} \\
&= \sum_{k\in\text{ancestors}(b)} \dot{q}_k P_k^I G_k'(q_k) G_k^{-1}(q_k) P_k^{I,-1} P_k^I G_k(q_k) D_{k^+\leftarrow b}^I \hat{X} \\
&= \sum_{k\in\text{ancestors}(b)} \dot{q}_k P_k^I G_k'(q_k) G_k^{-1}(q_k) P_{k}^{I,-1} G_b(q) \hat{X} \\
&= \sum_{k\in\text{ancestors}(b)} \dot{q}_k P_k^I \underbrace{G_k'(q_k) G_k^{-1}(q_k)}_{=\begin{cases} \begin{bmatrix} [\bar{\omega}^{\text{local}}_k]_\times & 0 \\ 0 & 0 \end{bmatrix} & \text{hinge} \\[6pt] \begin{bmatrix} 0 & \bar{v}^{\text{local}}_k \\ 0 & 0 \end{bmatrix} & \text{prismatic} \end{cases}} P_{k}^{I,-1} \hat{x}\\
&= \sum_{k\in\text{ancestors}(b)} \dot{q}_k P_k^I \underbrace{F_k^J}_{\text{from axis to matrix}} P_{k}^{I,-1} \hat{x} \\
\end{aligned}
$$

so we can read off the Jacobian blocks as
$$
\begin{aligned}
\begin{bmatrix} [\omega^{I,world}_b]_\times & \dot{x}_b - [\omega^{I,world}_b]_\times x_b^I \\  0&0 \end{bmatrix} &= \sum_{k\in\text{ancestors}(b)} \dot{q}_k P_k^I F_k^J P_{k}^{I,-1} \\
\end{aligned}
$$

and
$$
\begin{aligned}
 P_k^I F_k^J P_{k}^{I,-1} &= \begin{cases} \begin{bmatrix} R_k^I [\bar{\omega}^{\text{local}}_k]_\times R_k^{I\top} & -R_k^I [\bar{\omega}^{\text{local}}_k ]_\times R_k^{I \top} a_k \\ 0 & 0 \end{bmatrix} & \text{hinge} \\[6pt] \begin{bmatrix} 0 & R_k^I \bar{v}^{\text{local}}_k \\ 0 & 0 \end{bmatrix} & \text{prismatic} \end{cases}\\
 &= \begin{cases} \begin{bmatrix} [R^I_k \bar{\omega}^{\text{local}}_k]_\times & -[R^I_k \bar{\omega}^{\text{local}}_k]_\times a_k \\ 0 & 0 \end{bmatrix} & \text{hinge} \\[6pt] \begin{bmatrix} 0 & R^I_k \bar{v}^{\text{local}}_k \\ 0 & 0 \end{bmatrix} & \text{prismatic} \end{cases}\\
 &= \begin{cases} \begin{bmatrix} [\bar{\omega}^{\text{world}}_k]_\times & -[\bar{\omega}^{\text{world}}_k]_\times a_k \\ 0 & 0 \end{bmatrix} & \text{hinge} \\[6pt] \begin{bmatrix} 0 & \bar{v}^{\text{world}}_k \\ 0 & 0 \end{bmatrix} & \text{prismatic} \end{cases}
\end{aligned}
$$
where $a_k$ is the translation part of $P_k^I$ — the world-frame anchor of joint $k$ (= `d.xanchor` in MuJoCo).

Adopting the convention that $\bar{v}^{\text{world}}_k = 0$ for a hinge and $\bar{\omega}^{\text{world}}_k = 0$ for a prismatic joint, the two cases collapse into a single expression:
$$
P_k^I F_k^J P_{k}^{I,-1} = \begin{bmatrix} [\bar{\omega}^{\text{world}}_k]_\times & -[\bar{\omega}^{\text{world}}_k]_\times a_k + \bar{v}^{\text{world}}_k \\ 0 & 0 \end{bmatrix}.
$$

Plugging this back and matching blocks on both sides of
$$
\begin{bmatrix} [\omega^{I,world}_b]_\times & \dot{x}^I_b - [\omega^{I,world}_b]_\times x_b^I \\  0&0 \end{bmatrix} = \sum_{k\in\text{ancestors}(b)} \dot{q}_k \begin{bmatrix} [\bar{\omega}^{\text{world}}_k]_\times & -[\bar{\omega}^{\text{world}}_k]_\times a_k + \bar{v}^{\text{world}}_k \\ 0 & 0 \end{bmatrix},
$$
the upper-left block gives
$$
\omega^{I,world}_b = \sum_{k\in\text{ancestors}(b)} \dot{q}_k \, \bar{\omega}^{\text{world}}_k,
$$
and the upper-right block gives
$$
\begin{aligned}
\dot{x}^I_b - [\omega^{I,world}_b]_\times x_b^I &= \sum_{k\in\text{ancestors}(b)} \dot{q}_k \left( -[\bar{\omega}^{\text{world}}_k]_\times a_k + \bar{v}^{\text{world}}_k \right) \\
\dot{x}^I_b &= [\omega^{I,world}_b]_\times x_b^I + \sum_{k\in\text{ancestors}(b)} \dot{q}_k \left( -[\bar{\omega}^{\text{world}}_k]_\times a_k + \bar{v}^{\text{world}}_k \right) \\
&= \sum_{k\in\text{ancestors}(b)} \dot{q}_k \left( [\bar{\omega}^{\text{world}}_k]_\times x_b^I -[\bar{\omega}^{\text{world}}_k]_\times a_k + \bar{v}^{\text{world}}_k \right) \\
&= \sum_{k\in\text{ancestors}(b)} \dot{q}_k \left( \bar{\omega}^{\text{world}}_k \times (x_b^I - a_k) + \bar{v}^{\text{world}}_k \right).
\end{aligned}
$$

Stacking the two,
$$
\boxed{\;
\begin{bmatrix} \omega^{I,world}_b \\ \dot{x}^I_b \end{bmatrix} = \sum_{k\in\text{ancestors}(b)} \underbrace{\begin{bmatrix} \bar{\omega}^{\text{world}}_k \\ \bar{\omega}^{\text{world}}_k \times (x_b^I - a_k) + \bar{v}^{\text{world}}_k \end{bmatrix}}_{J_{b,k}^{I,world}} \dot{q}_k.
\;}
$$

Now, we can compute the inertia matrix as
$$
\begin{aligned}
T &= \sum_{b}  \dot{q}^T J_b^{I,world\top} \mathcal{I}_b^{world} J_b^{I,world} \dot{q} \\
&= \dot{q}^T \left( \sum_{b} J_b^{I,world\top} \mathcal{I}_b^{world} J_b^{I,world} \right) \dot{q} \\
\end{aligned}
$$
so the inertia matrix is
$$
\boxed{\;M(q) = \sum_{b} J_b^{I,world\top} \mathcal{I}_b^{world} J_b^{I,world}\;}.
$$

Now, putting the jacobians into it, the $(i,j)$-entry is
$$
M_{ij}(q) = \sum_{b} \big(J_{b,i}^{I,world}\big)^\top \mathcal{I}_b^{world}\, J_{b,j}^{I,world}.
$$

## Composite Rigid Body (CRB)

Since $J_{b,k}^{I,world} = 0$ for $k \notin \text{ancestors}(b)$,
$$
\begin{aligned}
M_{ij}(q)
&= \sum_{b} \big(J_{b,i}^{I,world}\big)^\top \mathcal{I}_b^{world}\, J_{b,j}^{I,world} \\
&= \sum_{b \,\in\, \text{desc}(i)\,\cap\,\text{desc}(j)} \big(J_{b,i}^{I,world}\big)^\top \mathcal{I}_b^{world}\, J_{b,j}^{I,world} \\
&= \begin{cases} \displaystyle\sum_{b \,\in\, \text{desc}(i)} \big(J_{b,i}^{I,world}\big)^\top \mathcal{I}_b^{world}\, J_{b,j}^{I,world} & j \in \text{ancestors}(i) \\[4pt] 0 & \text{otherwise.} \end{cases}
\end{aligned}
$$

Let $r(b) := \texttt{body\_rootid}[b]$ (chain root). The reference point used by cinert/cdof is $\tilde{x}_{r(b)}$ — the subtree CoM rooted at the chain root, NOT $\tilde{x}_b$ (which would be the subtree rooted at $b$ itself, body-dependent). For $b \in \text{desc}(i)$ with $j \in \text{ancestors}(i)$: $r(i) = r(j) = r(b) =: r$, hence $\tilde{x}_{r(b)} = \tilde{x}_r$ throughout the sum.

Split $x_b^I - a_k = (x_b^I - \tilde{x}_r) + (\tilde{x}_r - a_k)$ and define
$$
T_b := \begin{bmatrix} I & 0 \\ -[x_b^I - \tilde{x}_{r(b)}]_\times & I \end{bmatrix}, \qquad S_k := \begin{bmatrix} \bar{\omega}^{\text{world}}_k \\ \bar{\omega}^{\text{world}}_k \times (\tilde{x}_{r(k)} - a_k) + \bar{v}^{\text{world}}_k \end{bmatrix}.
$$
Then $J_{b,k}^{I,world} = T_b\, S_k$ when $r(b) = r(k)$:
$$
T_b S_k = \begin{bmatrix} \bar{\omega}^{\text{world}}_k \\ \bar{\omega}^{\text{world}}_k \times (x_b^I - \tilde{x}_r) + \bar{\omega}^{\text{world}}_k \times (\tilde{x}_r - a_k) + \bar{v}^{\text{world}}_k \end{bmatrix} = J_{b,k}^{I,world}.
$$
$S_i, S_j$ are $b$-independent, so:
$$
\sum_{b \in \text{desc}(i)} J_{b,i}^\top \mathcal{I}_b^{world} J_{b,j}
= S_i^\top \underbrace{\Big( \sum_{b \in \text{desc}(i)} T_b^\top \mathcal{I}_b^{world} T_b \Big)}_{\mathcal{I}^{\text{sub}}_i} S_j.
$$
Expanding $\mathcal{I}^{\text{sub}}_i$ with $d_b := x_b^I - \tilde{x}_{r(b)}$ and $\mathcal{I}_b^{world} = \operatorname{diag}(I_b^{world}, m_b I)$, using $[d]_\times^\top = -[d]_\times$ and $[d]_\times [d]_\times^\top = \|d\|^2 I - d d^\top$:
$$
\mathcal{I}^{\text{sub}}_i = \sum_{b \in \text{desc}(i)} \begin{bmatrix} I_b^{world} + m_b [d_b]_\times [d_b]_\times^\top & m_b [d_b]_\times \\ -m_b [d_b]_\times & m_b I \end{bmatrix}.
$$