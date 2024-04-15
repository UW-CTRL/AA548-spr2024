# Introduction to Optimal Control

### Scope + Objectives

In these notes, we aim to provide a brief introduction to optimal control, explaining its relevance to control theory and outlining its basic principles. By the end, readers should understand the fundamental concepts and motivations behind optimal control.

### Introduction

Optimal control is a cornerstone of control theory, focusing on finding control inputs that optimize a certain criterion, such as minimizing costs or maximizing performance. It finds applications in various fields, including engineering, economics, and biology.

The objective of optimal control Theory is _to determine the control signals that will cause a process to satisfy the physical constraints and at the same time minimize (or maximize) some performance criterion._


### Preliminaries
#### Definitions

Control System: A system governed by differential equations that describe its dynamics.
which takes the form
$\dot x = f(t,x,u)$, $x(t_0)=x_0$
Control Input: The variable manipulated to influence the system's behavior.
Objective Function: A measure of system performance to be optimized.

### Main Body
The problem formulation of an optimal control problem requires:
1. A mathematical description (or model) of the process to be controlled.
2. A statement of the physical constraints.
3. Specifications of the performance criterion.

The general Setup of Optimal control Problem:

$$
\min_{u_0, u_1, ..., u_K} \quad 
\begin{aligned}[t]
&= \{x_0,x_1,...,x_{K+1}\} \quad \sum_{k=0}^{K} J(x_k, u_k, k) + J_{\text{term}}(x_{K+1})
\end{aligned}
$$

$$
\min_{u_0, u_1, ..., u_K} \quad
\begin{aligned}
&\text{subject to} \\
&\quad x_0, x_1, ..., x_{K+1} \\
\end{aligned}
\quad \sum_{k=0}^{K} J(x_k, u_k, k) + J_{\text{term}}(x_{K+1})
$$

Optimal control problems can be classified into two main types: open-loop and closed-loop control.

Open-loop Control: Also known as trajectory optimization, it involves finding a control input sequence that optimally drives the system from an initial state to a desired final state without considering feedback.
Closed-loop Control: Also known as feedback control, it adjusts the control input based on the system's state feedback to achieve optimal performance in real-time.

### Conclusion
In summary, optimal control is a powerful framework for designing control strategies that optimize system performance. These notes provide a foundational understanding of optimal control principles. For further exploration, readers are encouraged to delve into advanced topics such as dynamic programming and the Hamilton-Jacobi-Bellman equation.

### References