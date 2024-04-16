# Introduction to Optimal Control

### Scope + Objectives

In these notes, we aim to provide a brief introduction to optimal control, explaining its relevance to control theory and outlining its basic principles. By the end, readers should understand the fundamental concepts and motivations behind optimal control.

### Objectives


### Introduction

Optimal control is a cornerstone of control theory, focusing on finding control inputs that optimize a certain criterion, such as minimizing costs or maximizing performance. It finds applications in various fields, including engineering, economics, and biology.

The objective of optimal control Theory is _to determine the control signals that will cause a process to satisfy the physical constraints and at the same time minimize (or maximize) some performance criterion._

### Preliminaries

#### Definitions

##### Control System: 

It generates possible behaviors. We will consider the control system to be described by ordinary differential equations (ODEs) that describe its dynamics, which take the form

$$
\dot x = f(t,x,u),\quad x(t_0)=x_0
$$

where $x$ is the _state_ taking values in $\mathbb{R}^n$, $u$ is the _control input_ taking values in some control set $U \subset \mathbb{R}^m$, $t$ is _time_ , $t_0$ is the _initial time_ , and $x_0$ is the _initial state_, and both $x$ and $u$ are functions of time.

##### Cost function: 

It associates a cost with each possible behavior. For a given initial data $(t_0, x_0), the behaviors are parametrized by control functions $u$. So, the cost function assigns a cost value to each admissible control. We will denote cost functions by $J$, which take the form,

$$ 
J(u) := \int_{t_0}^{t_f} L(t,x(t),u(t))dt + K(t_f,x_f)
$$

where $L$ and $K$ are given functions (_running cost_ and _terminal cost_, respectively), $t_f$ is the _final(or terminal) time_, which is either free or fixed, and $x_f:= x(t_f)$ is the _final (or terminal) state_ which is either free or fixed or belongs to some given target set.



Control Input: The variable manipulated to influence the system's behavior.
Objective Function: A measure of system performance to be optimized.

#### Notations 

#### Theorems 
Maximum Principle 
Principle of Optimality 
### Main Body
The problem formulation of an optimal control problem requires:
1. A mathematical description (or model) of the process to be controlled.
2. A statement of the physical constraints.
3. Specifications of the performance criterion.

The general Setup of Optimal Control Problem, in discrete time and finite horizon:

$$
\begin{aligned}
& \min_{\begin{aligned}[t]
& u_0, u_1, \ldots, u_K \\
& x_0, x_1, \ldots, x_{K+1}
\end{aligned}} && \sum_{k=0}^{K} J(x_k, u_k, k) + J_{\text{term}}(x_{K+1}) \\
& \text{s.t.} && x_{k+1} = f(x_k,u_k,k) \quad (k = 0,1,\ldots, K) \\
&&& x_k \in \mathcal{X}, \quad u_k \in \mathcal{U}, \quad x_0 = x_{\text{current}} \\
&&& g_i(x_k,u_k) = 0 \quad (i = 1,\ldots, G) \\
&&& h_i(x_k,u_k) \leq 0 \quad (i = 1, \ldots, H)
\end{aligned}
$$

where $J_{\text{term}}(x_{K+1})$ is the terminal cost,
      $\sum_{k=0}^{K} J(x_k, u_k, k)$ is the summation of the running cost,
      $x_{k+1} = f(x_k,u_k,k) \quad (k = 0,1,\ldots, K)$ is the dynamics of the system,
      $x_k \in \mathcal{X}, \quad u_k \in \mathcal{U}, \quad x_0 = x_{\text{current}}$ are the system constraints,
      $g_i(x_k,u_k) = 0$ is an equality constraint,
      $h_i(x_k,u_k) \leq 0$ is an inequality constraint.




Optimal control problems can be classified into two main types: open-loop and closed-loop control.

Open-loop Control: Also known as trajectory optimization, it involves finding a control input sequence that optimally drives the system from an initial state to a desired final state without considering feedback.
Closed-loop Control: Also known as feedback control, it adjusts the control input based on the system's state feedback to achieve optimal performance in real-time.

### Conclusion
In summary, optimal control is a powerful framework for designing control strategies that optimize system performance. These notes provide a foundational understanding of optimal control principles. For further exploration, readers are encouraged to delve into advanced topics such as dynamic programming and the Hamilton-Jacobi-Bellman equation.

### References
