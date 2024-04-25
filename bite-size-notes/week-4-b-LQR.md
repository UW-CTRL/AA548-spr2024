# Introduction to Linear Quadratic Regulator (LQR)

## Scope and Objectives

## Introduction

In the field of optimal control, the goal is to control a dynamical system while minimizing the cost that results from the output of said control. However, choosing the parameters of the controller may not be immediately obvious, resulting in inefficient trial-and-error design (such as manual PID parameter tuning). Otherwise, methods such as full state feedback design, using eigenvalue/pole placement to design the controller transfer function, requires physical intuition or understanding may be required to design the parameters. Such intuition or understanding may not be available for complex systems, or be extremely difficult to obtain.

For a linear system, if the cost function is quadratic, the optimal control problem can be solved without the aforementioned issues using the Linear Quadratic Regulator (LQR). The control system engineer would then only need to specify the system dynamics and the cost function, and the optimal control parameters can be calculated. This is the Linear Quadratic Regulator (LQR) problem.

## Preliminaries and Motivation

We consider the discrete time dynamical system at some time step $t = k+1$ described by:

$$ x_{k+1} = Ax_k + Bu_k $$

and the corresponding finite horizon discrete time optimal control problem:

$$ \min_{u \in [u_0,u_K], x \in [x_0, x_k]} \lbrace J_{\text{term}} (x_{k+1}) + \sum_{k=0}^K J(x_k,u_k,k) \rbrace$$

subject to:
$$
  \begin{split}
    x_{k+1} &= Ax_k + Bu_k \\
    x_k &\in \mathcal{X} \\
    u_k &\in \mathcal{U} \\
    x_0 &= x_{\text{init}} \\
    g_i(x_k,u_k,k) &\leq 0, \quad i = 1, \dots, m \\
    h_i(x_k,u_k,k) &= 0, \quad i = 1, \dots, p
  \end{split}
$$

where $J_{term} (x_{k+1})$ is the terminal cost, and $J(x_k,u_k,k)$ is the running/stage cost. The terminal cost is a function of the state at the final time step, and the stage cost is a function of the state and control at each time step. To solve the optimal control problem means to find the optimal control sequence $u^* = [u_0^*, u_1^*, \dots, u_K^*]$ that minimizes this cost function.

In the form this question is written, we must re-solve the problem for each time step, which is computationally expensive. However, if we leverage dynamic programming, we can solve the problem for each time step in a recursive manner, which is much more efficient. For this, we introduce the Bellman equation:

$$ V(x_k, k) = \min_{u_k \in \mathcal{U}} \lbrace g(x_k,u_k,k) + V(x_{k+1}, k+1) \rbrace $$

where $V(x_k, k)$ is the value function, and the omtimization problem is to find the optimal control $u_k$ that minimizes the sum of the immediate cost $g(x_k,u_k,k)$ and the cost-to-go $V(x_{k+1}, k+1)$. Using dynamic programming, we can solve the optimal control problem posed by the Bellman equation by backtracking from the final time step $k+1$ to the initial time step $0$.

Posing the optimal control problem using the Bellman equation *can* give us efficiency, provided that we can actually optimize for the immediate cost and cost-to-go functions. One way of of doing this is to design the cost functions to be convex, for which there are deterministic methods that can be used to solve for the global optimum. As the name LQR suggests, we will choose these cost functions to be quadratic, and therefore convex.

## Linear Quadratic Regulator (LQR) Problem

### Theory

#### Discrete Time, Finite Horizon LQR

The discrete time, finite horizon LQR problem can be written as:

$$ 
  \begin{split}
    \min_{u_k \in \mathcal{U}} &\lbrace x_{k+1}^T Q_T x_{k+1} + \sum_{k=0}^K (x_k^T Q_k x_k + u_k^T R_k u_k) \rbrace \\
    \text{s.t.} &\quad x_{k+1} = Ax_k + Bu_k \\
    & \quad \quad x_0 = x_{\text{init}}
  \end{split}
$$

note that we have no control or state constraints. The dynamics are linear, and the cost function, composed of the next state cost and the running cost, are both quadratic and therefore convex. The regulator regulates to zero for stability. The quadratic cost function guarantees the optimal control input $u_k^*$ is globally optimal at each time step $k$.

The matrices $Q_k$ and $R_k$ are the weights for the state and control costs, respectively. The matrix $Q_T$ is the terminal cost weight, and is used to stabilize the system. The matrices $A$ and $B$ are the system dynamics matrices, and $x_k$ is the state at time step $k$. We require the matrices $Q_k$ and $Q_T$ to be positive semi-definite, and $R_k$ to be positive definite in order to guarantee that the cost function is bowl-shaped and has a global minimum. As a result, we have that:

$$ Q_k = Q_k^T \succeq 0, \quad Q_T^T \succeq 0, \quad R_k^T \succ 0 $$

The positive definite nature of $R_k$ means that any non-zero control input adds to the cost, which ensures that the control is minimized to reduce input effort. The positive semi-definite nature of $Q_k$ and $Q_T$ means that some states may not contribute to the cost, which helps to reduce control effort. Also, $Q_T$ and $Q_k$ ensures that the states are bounded, and the system is stable.

##### Solving the discrete time, finite horizon LQR problem

As suggested by the preliminaries, we can solve the discrete time, finite horizon LQR problem by posing it in the form of the Bellman equation, and then leveraging dynamic programming to solve by backtracking.

Let the value function at time step $k$ be $V(x_k, k) = x_k^T P_k x_k$, where $P_k$ is symmetric and positive semi-definite. Then at the final time step, we have:

$$ V(x_{k+1}, k+1) = x_{k+1}^T P_{k+1} x_{k+1} \implies P_{k+1} = Q_T $$

where $P_{K+1} = Q_T$. This forms the first step of our dynamic programming algorithm. Then, at time step $k$, we have:

$$ V(x_k, k) = \min_{u_k \in \mathcal{U}} \lbrace x_k^T Q_k x_k + u_k^T R_k u_k + V(x_{k+1}, k+1) \rbrace $$

where the sum of the first two terms forms the immediate cost, $V(x_{k+1}, k+1)$ is the cost-to-go. Using the state dynamics, we can write:

$$ V(x_{k+1}, k+1) = V(Ax_k + Bu_k, k+1) = (Ax_k + Bu_k)^T Q_T (Ax_k + Bu_k) $$

We can then notice that the right-hand-side of our expression for $V(x_k, k)$ is quadratic in $u_k$ based on the second and third terms, and can be minimized by simply taking the derivative with respect to $u_k$ and setting it to zero to solve for the optimal control input $u_k^*$. To make differentiation easier, we can re-write $V(x_k,k)$ as:

$$\min_{u_k \in \mathcal{U}} x_k^T (Q_k + A^T P_{k+1} A)x_k + u_k^T (R_k + B^T P_{k+1} B)u_k + 2x_k^T A^T P_{k+1} Bu_k$$

where we used that fact that $P_{k+1} = Q_T$ and combined the quadratic and linear terms for $x_k$ and $u_k$. We now have a standard degree-2 polynomial in $u_k$ which is easy to differentiate w.r.t. $u_k$ and set to zero to solve for the optimal control input $u_k^*$:

$$ \frac{\partial V(x_k, k)}{\partial u_k} = 2(R_k + B^T P_{k+1} B)u_k + 2B^T P_{k+1} A x_k = 0 \implies u_k^* = - (R_k + B^T P_{k+1} B)^{-1} B^T P_{k+1} A x_k $$

#### Solving the discrete time, infinite horizon LQR problem

### Implementation


## Conclusion

## References
