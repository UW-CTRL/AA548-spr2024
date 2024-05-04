# Overview of Model Predictive Control and its Variants

**Scope: Understanding features of MPC variants to choose the right setting for applications** 

Model Predictive Control (MPC) has emerged as a popular framework for achieving high-performance in autonomous systems. 
This report provides a comprehensive overview of Model Predictive Control (MPC) and its diverse various. 
It begins with an explanation of the fundamental concepts of MPC, followed by a detailed discussion on Linear MPC. 
Subsequently, it explores variants of MPC such as Nonlinear MPC, Robust MPC, and Stochastic MPC, highlighting their features and applications.

**Objectives**
- Review the fundamental concepts and principles of MPC.
- Understand the variants of MPC problems.


## Overview of MPC

Model Predictive Control (MPC) is a method that solves optimization problems sequentially over a finite time horizon. 
It has found applications in various fields and has gained significant interest in recent years.

Key characteristics of MPC include:
- Finite horizon makes it computationally easier compared to infinite horizon optimization.
- Sequential optimization improves trajectory quality.
- Flexible handling of nonlinearity and constraints.
- High robustness against disturbances due to model-based control.

Challenges of MPC include:
- Computational cost: MPC involves solving optimization problems sequentially, which can be computationally intensive, especially for systems with complex dynamics and constraints. This can impact real-time performance and implementation complexity.
- Model uncertainty: MPC relies on accurate models of the system, but in practice, these models may be uncertain. Handling this uncertainty is a challenge for MPC.
- Horizon length: Setting a horizon length sufficiently longer than the time constant of the controlled object can be challenging and requires careful tuning.


## Linear MPC
Consider the following dynamical system:

$$
\begin{aligned}
x_{k+1} = Ax_k + Bu_k + Dw_k\\
y_k = Cx_k\\
x_k\in R^n, u_k\in R^m, w_k \in R^n.
\end{aligned}
$$

By repeatedly applying this equation, we can calculate the state at $k$ steps ahead. Given an initial state of $x_0$, $u_0$, and $w_0$,

$$
\begin{aligned}
x_{1} &= Ax_0 + Bu_0 + Dw_0\\
x_{2} &= Ax_1 + Bu_1 + Dw_1\\
&=A(Ax_0 + Bu_0 + Dw_0) + + Bu_1 + Dw_1\\
&= A^2x_0 + 
\left[ 
    \begin{array}{cc} AB & B\end{array} 
\right] 
\left[ 
    \begin{array}{c} u_0 \\
     u_1\end{array} 
\right] + 
\left[ \begin{array}{cc} A & D\end{array} \right]
\left[ 
    \begin{array}{c} w_0 \\
     w_1\end{array} 
\right]\\
x_{3} &= Ax_3 + Bu_3 + Dw_3\\
&=A(A^2x_0 + ABu_0 + Aw_0 + Bu_1 + Dw_1) + Bu_2 + Dw_2\\
&= A^3x_0 + 
\left[ \begin{array}{ccc} A^2B&AB & B\end{array} \right]
\left[ \begin{array}{c} u_0 \\
 u_1\\ 
 u_2 \end{array} 
\right] + 
\left[ \begin{array}{cc} A^2D & AD & D\end{array} \right] 
\left[ \begin{array}{c} w_0 \\
 w_1\\
 w_2 \end{array} \right]\\
\vdots\\
x_{k} &= Ax_{k-1} + Bu_{k-1} + Dw_{k-1}\\
&= A^nx_0 + 
\left[ \begin{array}{cccc} A^{n-1}B&A^{n-2}B & \cdots& B\end{array} \right] 
\left[ \begin{array}{c} u_0 \\
 u_1\\
\vdots\\
u_{n-1}\end{array} \right] + 
\left[ \begin{array}{cc} A^{n-1}D & A^{n-2}D & \cdots & D\end{array} \right] 
\left[ \begin{array}{c} w_0 \\
 w_1\\
\vdots \\
w_{n-1}\end{array} \right]
\end{aligned}
$$

These can be summarized into the following matrix equation:

$$
\begin{aligned}
X = \left[ \begin{array}{c} x_1 \\ 
x_2\\ 
\vdots \\
 x_n\end{array} \right]
&= \left[ \begin{array}{c} A \\
 A^2\\
  \vdots \\
   A^n\end{array} \right]x_0
+
\left[ \begin{array}{cccc}
B & 0 & \cdots & 0 \\
AB & B & \cdots & 0 \\
\vdots & & \ddots & \vdots\\
A^{n-1}B & A^{n-2}B & \cdots & B
\end{array} \right]
\left[ \begin{array}{c} u_0 \\
 u_1\\ 
 \vdots\\
 u_{n-1}\end{array} \right]
+
\left[ \begin{array}{cccc}
D & 0 & \cdots & 0 \\
AD & D & \cdots & 0 \\
\vdots & & \ddots & \vdots\\
A^{n-1}D & A^{n-2}D & \cdots & D
\end{array} \right]
\left[ \begin{array}{c} w_0 \\
 w_1\\ 
 \vdots\\ 
 w_{n-1}\end{array} \right]\\

&=\tilde{A}x_0 + \tilde{B}U + \tilde{D}W\\
\end{aligned}
$$

$$
\begin{aligned}
Y = \left[ \begin{array}{c} y_1 \\
 y_2\\ 
 \vdots \\
  y_n\end{array} \right]
&= 
\left[ \begin{array}{cccc}
C & 0 & \cdots & 0 \\
0 & C & \cdots & 0 \\
\vdots & & \ddots & \vdots\\
0 & 0 & \cdots & C
\end{array} \right]
\left[ \begin{array}{c} x_1 \\ 
x_2\\ 
\vdots\\ 
x_n\end{array} \right]\\
&= \tilde{C}X
\end{aligned}
$$

With the above equations, we have been able to express the system's outputs $Y$ up to $n$ steps ahead in terms of the inputs $U$.
Next, we design the cost function to control the system to track the target value.

$$
\begin{aligned}
J = (Y-Y_{target})^TQ(Y-Y_{target}) + (U-U_{target})^TR(U-U_{target})
\end{aligned}
$$

where $Y_{target}$ and $U_{target}$ are the target values for outputs and controls.
$Q$ and $R$ must be positive semi-definite and positive definite matrices, respectively.
Since this cost function $J$ depends only on the inputs $U$, rearranging $J$ for $U$ yields the following:

$$
\begin{aligned}
J &= (\tilde{C}(\tilde{A}x_0 + \tilde{B}U + \tilde{D}W)-Y_{target})^TQ(\tilde{C}(\tilde{A}x_0 + \tilde{B}U + \tilde{D}W)-Y_{target}) + (U-U_{target})^TR(U-U_{target})\\
&= U^T(\tilde{B}^T\tilde{C}^TQ\tilde{C}\tilde{B} +R)U + 2((\tilde{C}(\tilde{A}x_0+\tilde{D}W) - Y_{target})^TQ\tilde{C}\tilde{B} - U_{target}^TR)U + (const)
\end{aligned}
$$

Therefore, $J$ is a quadratic form of $U$ and can be easily optimized.

Even in the case of nonlinear system dynamics, if the system trajectory is known in advance, linear approximation of the dynamics can be applied at each time step in the vicinity of the trajectory. This allows the application of the aforementioned linear MPC.
However, it should be noted that the system dynamics become time-varying to approximate the dynamics at each time step:

$$
\begin{aligned}
x_{k+1} = A_kx_k + B_ku_k + D_kw_k.
\end{aligned}
$$


## Nonlinear MPC
Consider a system with nonlinear dynamics given by:

$$
x_{k+1} = f(x_k, u_k)\\
y_k = g(x_k, u_k)\\
$$

Furthermore, even if the dynamics are linear, Nonlinear MPC may be applied if the cost function or constraints are not convex. In the Linear MPC section, it was mentioned that even if the system is nonlinear, Linear MPC can be applied by sequentially linearizing the system. However, Nonlinear MPC can also be applied by directly using the nonlinear model.

Nonlinear MPC uses nonlinear optimization, so it has a higher computational cost and takes longer for optimization compared to Linear MPC. However, since it directly optimizes the nonlinear model without using approximations, it can sometimes compute better solutions.

## Robust MPC

One of the control methods for handling disturbances is Robust MPC. It considers the dynamics that were considered in Linear MPC as the system dynamics.
In conventional MPC, the future states (such as positions and orientations) of surrounding objects or robots are predicted for a certain period, and optimal control inputs that satisfy constraints and optimize the state are sought. 
However, if there are errors in the information about the current positions or velocities of objects, the prediction errors can become large, making it difficult to find control inputs that satisfy the constraints. 
Therefore, in Robust MPC, not only a single predicted value but also a set of reachable set is determined, and control inputs that ensure that the entire set satisfies the constraints are sought. 
This ensures that if the prediction errors are within the range of the set, the robot can reliably satisfy the constraints.

The challenge of Robust MPC is that its performance deteriorates in environments with high uncertainty. 
For example, if there is significant noise in the sensors, it is necessary to expand the range of the reachability set assuming that the prediction errors will be large. 
Therefore, in order to ensure that the entire set satisfies the constraints, extremely conservative control is required, leading to a decrease in the robot's performance in executing its intended task.

## Stochastic MPC
Consider a system given by:

$$
x_{k+1} = Ax_k + Bu_k + w_k\\
u_k = \phi_k(X_k),\ X_k=[x_0,\ldots,x_k]\\
x_k\in R^n, u_k\in R^m, w_k \in R^n
$$

where $T$ is finite horizon and $x_0, w_0, \ldots, w_{T-1}$ are random variables.
Then, the objective function is defined as follows:

$$
J = \mathbb{E}(\sum_{k=0}^{T-1} l_k(x_k, u_k)+l_T(x_T))
$$

where $\mathbb{E}$ is expectation function, and $l_k$ and $l_T$ are convex.
The objective function $J$ depends on the control policy $\phi_k$, so the goal of this problem is to choose $\phi_k$ that minimizes $J$.
Since the variable is a function $\phi_k$, this problem is an infinite-dimensional problem.

### Linear Quadratic Stochastic MPC
Stochastic MPC can analytically find solutions in special cases. We make the following assumptions:

$$
\begin{aligned}
&x_0, w_0, \ldots, w_{T-1}\ \mathrm{are\ independent}\\
&\mathbb{E}(x_0) = 0,\ \mathbb{E}(w_k) = 0,\ \mathbb{E}(x_0x_0^T) = \Sigma,\ \mathbb{E}(w_kw_k^T) = W_k 
\end{aligned}
$$

And suppose the objective function is as follows:

$$
l_k(x_k, u_k) = x_k^TQ_kx_k + u_k^TR_ku_k,\ Q_k\succeq 0,\ R_k\succ 0\\
l_T(x_T) = x_T^TQ_kx_T,\ Q_T\succeq 0
$$

Assuming the cost function is quadratic as follows:

$$
V_k(x_k) = x_k^TP_kx_k + q_k,\ k=0,\ldots,T
$$

The value function at time step $k$ can be expressed as follows:

$$
V_k(x_k) = \inf_{u_k} x_k^TQ_kx_k + u_k^TR_ku_k + \mathbb{E}((Ax_k + Bu_k + w_k)^TP_{k+1}(Ax_k + Bu_k + w_k) + q_{k+1})
$$

This results in the following:

$$
P_k = A^TP_{k+1}A-A^TP_{k+1}B(B^TP_{k+1}B+R_k)^{-1}B^TP_{k+1}A+Q_k\\
q_k = q_{k+1} + \mathrm{Tr}(W_kP_{k+1})
$$

Therefore, by setting $P_T = Q_T$ and iteratively calculating $P_{T-1}, \ldots, P_0$, the optimal policy can be computed as follows, independent of the noise term.

$$
\phi_k^*(x_k) = K_kx_k\\
K_k = -(B^TP_{k+1}B+R_k)^{-1}B^TP_{k+1}A
$$

## Conclusion
Model Predictive Control (MPC) and its variants offer robust and flexible control strategies for autonomous systems. 
Linear MPC is effective for linear dynamics, while Nonlinear MPC directly optimizes nonlinear models. 
Robust MPC handles disturbances by considering reachable states, and Stochastic MPC optimizes control under probabilistic uncertainties. 
Understanding these variants is key for selecting the most suitable strategy for different applications.


## Reference
[1] K. Holkar, and L. Waghmare, "An Overview of Model Predictive Control," International Journal of Control and Automation, 2010.

[2] M. Morari, and J. Lee, "Model Predictive Control: Past, Present and Future," Computers & Chemical Engineering, 1999.

[3] D. Mayne, M. Seron, and S. Rakovic, "Robust Model Predictive Control of Constrained Linear Systems with Bounded Disturbances," Automatica, 2005.

[4] A. Mesbah, "Stochastic Model Predictive Control: An Overview and Perspectives for Future Research," IEEE Control System Magazine, 2016.

<!-- [4] Salzmann, Tim, et al., "Real-Time Neural MPC: Deep Learning Model Predictive Control for Quadrotors and Agile Robotic Platforms," IEEE Robotics and Automation Letters, 2023. -->
