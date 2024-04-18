# Introduction to Optimal Control Theory

## Scope

We have seen the use of myopic controllers like PID or operational space control, as well as some predictive controllers like trajectory generation. Predictive control allows the controller to make better decisions at the current time to account for future possibilities, particularly for complex and non-linear systems, but they can be very restrictive to one class of systems. Optimal Control addresses these shortcomings in a highly general framework.                  


## Objectives

Optimal Control addresses these shortcomings by optimizing control inputs over time to achieve desired system behavior while considering constraints and minimizing costs. Unlike myopic controllers such as PID, which react to current error signals without considering future implications, optimal control methods anticipate future system states and adjust control inputs to optimize performance. Similarly, while predictive controllers like trajectory generation can improve performance by planning ahead, they may be restricted to one class of systems. Optimal control theory offers a comprehensive approach that can accommodate a wide range of system dynamics and constraints, making it a powerful tool for designing control strategies in diverse applications.

In these notes, we aim to provide a brief introduction to optimal control, how to specify optimal control problems, and the different types of optimal control techniques explaining its relevance to control theory and outlining its basic principles. By the end, readers should understand the fundamental concepts and motivations behind optimal control.

## Introduction

Optimal control is a cornerstone of control theory, focusing on finding control inputs that optimize a certain criterion, such as minimizing costs or maximizing performance. It finds applications in various fields, including engineering, economics, and biology.

The objective of optimal control theory is _to determine the control signals that will cause a process to satisfy the physical constraints and at the same time minimize (or maximize) some performance criterion._
From an engineering point of view, optimality provides a very useful design principle, and the cost to be minimized (or the profit to be maximized) is often naturally contained in the problem itself.
For example, a driver of a car would like to reach a desired location while achieving several other goals: e.g., avoiding obstacles, not driving erratically, and maintaining a comfortable level of acceleration for human passengers. Optimal control allows a control designer to specify the dynamic model and the desired outcomes, and the algorithm will compute an optimized control. 

Some examples of optimal control problems arising in applications include the following:
* Send a rocket to the moon with minimal fuel consumption;
* Produce a given amount of chemical in minimal time and/or with a minimal amount of catalyst used (or maximize the amount produced in a given time);
* Bring sales of a new product to a desired level while minimizing the amount of money spent on the advertising campaign;
* Maximize throughput or accuracy of information transmission over a communication channel with a given bandwidth/capacity.

## Preliminaries

### Definitions

#### Control System: 

It generates possible behaviors. We will consider the control system to be described by ordinary differential equations (ODEs) that describe its dynamics, which take the form

$$
\dot x = f(t,x,u),\quad x(t_0)=x_0
$$

where $x$ is the _state_ taking values in $\mathbb{R}^n$, $u$ is the _control input_ taking values in some control set $U \subset \mathbb{R}^m$, $t$ is _time_ , $t_0$ is the _initial time_ , and $x_0$ is the _initial state_, and both $x$ and $u$ are functions of time.

#### Cost functional: 

It associates a cost with each possible behavior. For a given initial data $(t_0, x_0)$, the behaviors are parametrized by control functions $u$. So, the cost function assigns a cost value to each admissible control. We will denote cost functionals by $J$, which take the form,

$$ 
J(u) := \int_{t_0}^{t_f} L(x(t),u(t),t)dt + K(t_f,x_f)
$$

where $L$ and $K$ are given functions (_running cost(or Instantaneous cost)_ and _terminal cost_, respectively), $t_f$ is the _final(or terminal) time_, which is either free or fixed, and $x_f:= x(t_f)$ is the _final (or terminal) state_ which is either free or fixed or belongs to some given target set.

Since $u$ is a function of time, therefore J is called a _functional_ because it is a function mapping a function to a real number. 

A variety of behaviors can be specified in this framework by modifying the instantaneous cost. For example:

1. **Trajectory tracking** for a trajectory $x_D(t)$ can be implemented by penalizing squared error:
$$L(x, u, t) = {|x - x_D(t)|}^2$$

3. **Minimizing effort** can be defined in terms of a control penalty:
$$L(x, u, t) = \| u \|^2$$

5. **Minimum time to hit a target** $x_{\text{tgt}}$ could be implemented as an indicator function:
$$I[x \neq x_{\text{tgt}}]$$

   where $I[z]$ is 1 if $z$ is true, and 0 otherwise.

7. **Obstacle avoidance** and other feasibility constraints can be implemented as indicator functions as well:
$$\infty \cdot I[x \notin \mathcal{F}]$$

   where $\mathcal{F}$ is the free space.

9. **Smoothed obstacle avoidance** can be implemented by a repulsive barrier that decreases to 0 when the distance to the closest obstacle $d$ exceeds some minimum buffer distance $d_{\text{min}}$ and increases to infinity as the distance shrinks to 0. One common form of this barrier is:
$L(x, u, t) =\frac{1}{{d^2}} - \frac{1}{{d^2_{\text{min}}}}$ when $d < d_{\text{min}}$ and $L(x, u, t) = 0$ otherwise

It is common to mix and match different types of cost functionals using a weighted cost functional:
$$J(x, u) = \sum_{i=1}^{N} w_i J_i(x, u)$$

where each $J_i(x, u)$ is some primitive cost functional and $w_i$ scales its contribution to the final cost. By tuning these weights, a designer can encourage the optimized trajectories to emphasize some aspects of the trajectory over others.

### Notations 

### Theorems 

#### Principle of Optimality 

The principle of optimality is a fundamental concept in optimal control theory that guides the search for optimal control strategies. At its core, it asserts that an optimal control strategy for a complex system can be decomposed into optimal control decisions at each smaller time interval.

The principle of Optimality states that:

For every $(t,x) \in [t_0,t_1) \times \mathbb{R}$ and every $\Delta t \in (0, t_1 - t]$, the value function $V$ satisfies the relation
$$V(t,x) = \inf_{u_{[t,t+\Delta t]}} \quad {\int_{t}^{t + \Delta t}} L(s,x(s),u(s))ds + V(t+\Delta t, x(t+\Delta t))$$
where $x(.)$ on the right-hand side is the state trajectory corresponding to the control  $u_{[t,t+\Delta t]}$ and satisfying $x(t) = x$. The intuition behind that statement is that to search for optimal control, we can search for control that minimizes the cost over a small time interval and then continue the search recursively over subsequent intervals.

Here $V(t,x)$ is called the value function, which is defined as
$$V(t,x) := \inf_{u_{[t,t_1]}} \quad J(t,x,u)$$
where the notation $u_{[t,t_1]}$ indicates that the control $u$ is restricted to the interval $[t,t_1]$. We can think of $V(t,x)$ as the optimal cost (cost-to-go) from a given state $x$ at time $t$, to the terminal time $t_1$. It is the minimum achievable cost over all possible control trajectories starting from $(t,x)$.

The Value function satisfies the boundary condition
$$V(t_1,x) = K(x) \quad \quad \forall x \in \mathbb{R}^n.$$

where $K(x)$ is the terminal cost for all states $x$. The Boundary condition is a consequence of our specific problem formulation.

In simpler terms, we can say that for a given time interval $[t_0,t_1)$ and state $x$, the optimal value function $V(t,x)$ satisfies a recursive relationship, that is, for  any time $t$ within the interval and any small time increment $\Delta t$, the value function $V(t,x)$ can be expressed as the sum of the cost incurred over the current interval and the optimal cost-to-go from the end of the interval.

## Main Body

An optimal control problem involves finding control inputs that optimize a certain criterion while satisfying system dynamics and constraints. The problem formulation of an optimal control problem typically requires:
1. ***A mathematical description (or model) of the process to be controlled***
   - This component involves defining the dynamics of the system to be controlled. The dynamics function $f(x_k, u_k, k)$ describes how the state of the system evolves over time in response to control inputs.
2. ***A statement of the physical constraints***
   - The physical constraints on the system, represented by sets $\mathcal{X}$ and $\mathcal{U}$, define the feasible state and control spaces, respectively. These constraints ensure that the system operates within permissible bounds.
   - Additionally, equality constraints $g_i(x_k, u_k) = 0$ and inequality constraints $h_i(x_k, u_k) \leq 0$ may be imposed to further restrict the feasible space.
3. ***Specifications of the performance criterion***
   - The goal of the optimal control problem is to optimize a performance criterion, which is typically represented by a cost functional. The cost functional $J(x_k, u_k, k)$ evaluates the quality of a control trajectory over a given time horizon.
   - The objective is to minimize the total cost, which consists of both the running cost $$\sum_{k=0}^{K} J(x_k, u_k, k)$$ over the entire trajectory and the terminal cost $J_{\text{term}}(x_{K+1})$ at the final state.
  
#### General Setup of Optimal Control Problem
The general Setup of Optimal Control Problem, in discrete time and finite horizon:

$$\begin{aligned}
& \min_{\begin{aligned}[t]
& u_0, u_1, \ldots, u_K \\
& x_0, x_1, \ldots, x_{K+1}
\end{aligned}} && \sum_{k=0}^{K} J(x_k, u_k, k) + J_{\text{term}}(x_{K+1}) \\
& \text{s.t.} && x_{k+1} = f(x_k,u_k,k) \quad (k = 0,1,\ldots, K) \\
&&& x_k \in \mathcal{X}, \quad u_k \in \mathcal{U}, \quad x_0 = x_{\text{current}} \\
&&& g_i(x_k,u_k) = 0 \quad (i = 1,\ldots, G) \\
&&& h_i(x_k,u_k) \leq 0 \quad (i = 1, \ldots, H)
\end{aligned}$$

where $J_{\text{term}}(x_{K+1})$ is the terminal cost,
      $$\sum_{k=0}^{K} J(x_k, u_k, k)$$ is the summation of the running cost,
      $x_{k+1} = f(x_k,u_k,k) \quad (k = 0,1,\ldots, K)$ is the dynamics of the system,
      $x_k \in \mathcal{X}, \quad u_k \in \mathcal{U}, \quad x_0 = x_{\text{current}}$ are the system constraints,
      $g_i(x_k,u_k) = 0$ is an equality constraint,
      $h_i(x_k,u_k) \leq 0$ is an inequality constraint.

An additional consideration is that Optimal Control solvers require that the cost functional is smooth, so non-differentiable constraints like minimum time and obstacle avoidance must be reformulated as hard constraints, external to the cost functional. As a result, the reformulation becomes essentially an infinite-dimensional constrained optimization problem.

### Solution Techniques for Optimal Control Problems

There are many solution techniques available for optimal control, including, Calculus of Variations, Hamilton-Jacobi-Bellman(HJB) Equation, Direct shooting methods, etc. But for now,  we will discuss just one of these approaches in some detail, that is, Dynamic programming.

#### Dynamic Programming
The dynamic programming approach is quite general, but to fix ideas let's consider the purely discrete case. Consider a system of the form 
$$x_{k+1} = f(x_k, u_k), \quad k = 0, 1, \ldots, T - 1$$

where $x_k$ lives in a finite set $X$ consisting of $N$ elements, $u_k$ lives in a finite set $U$ consisting of $M$ elements, and $T$, $N$, $M$ are fixed positive integers. Suppose each possible transition from some $x_k$ to $x_{k+1}$ corresponding to some control value $u_k$ has a cost assigned to it, and there is also a terminal cost function on $X$. For each trajectory, the total cost accumulated at time $T$ is the sum of the transition costs at time steps $0, \ldots, T - 1$ plus the terminal cost at $x_T$. For a given initial state $x_0$, we want to minimize this total cost, the terminal state $x_T$ being free.

The most naive approach to this problem is as follows: starting from x0, enumerate all possible trajectories going forward up to time T, calculate the cost for each one, then compare them and select the optimal one. It is easy to estimate the computational effort required to implement such a solution: there are $M^T$ possible trajectories and we need T additions to compute the cost for each one, which results in roughly $O(M^TT)$ algebraic operations.

<img src="figs/Discrete_case_going_forward.jpg" alt="Alt text" width="300" caption="Discrete">

Dynamic programming is a versatile method, especially in discrete scenarios. Take a system described by $x_{k+1} = f(x_k, u_k)$ for $k = 0, 1, \ldots, T - 1$, where $x_k$ ranges over a set $X$ of $N$ elements and $u_k$ over a set $U$ of $M$ elements. With fixed positive integers $T$, $N$, and $M$, each transition from $x_k$ to $x_{k+1}$ incurs a cost, along with a terminal cost on $X$. We aim to minimize the total cost for a trajectory up to time $T$, comprising the sum of transition costs and the terminal cost at $x_T$, given an initial state $x_0$. 

The basic approach is to enumerate all possible forward trajectories from $x_0$ up to time $T$, calculate their costs, and select the optimal one. This involves $M^T$ trajectories and approximately $O(M^TT)$ operations.


We now examine an alternative approach, which might initially appear counterintuitive: let us go backward in time. At $k = T$, terminal costs are known for each $x_k$. At $k = T - 1$, for each $x_k$ we find to which $x_{k+1}$ we should jump to have the smallest cost (the one-step running cost plus the terminal cost). Write this optimal “cost-to-go” next to each $x_k$ and mark the selected path. In case of more than one path giving the same cost, choose one of them at random. Repeat these steps for $k = T - 2, \ldots, 0$, working with the costs-to-go computed previously in place of the terminal costs.

<img src="figs/Discrete_case_going_backward.jpg" alt="Alt text" width="300"> 

We claim that when we are done, we will generate an optimal trajectory from each $x_0$ to some $x_T$. The justification of this claim relies on the principle of optimality, an observation that we already made during the proof of the maximum principle. In the present context, this principle says that for each time step $k$, if $x_k$ is a point on an optimal trajectory then the remaining decisions (from time $k$ onward) must constitute an optimal policy for $x_k$ as the initial condition. What the principle of optimality does for us here is a guarantee that the paths we discard going backward cannot be portions of optimal trajectories. On the other hand, in the previous approach (going forward) we are not able to discard any paths until we reach the terminal time and finish the calculations.

Let us assess the computational effort associated with this backward scheme. At each time $k$, for each state $x_k$ and each control $u_k$ we need to add the cost of the corresponding transition to the cost-to-go already computed for the resulting $x_{k+1}$. Thus, the number of required operations is $O(NMT)$. Comparing this with the $O(MT^2)$ operations needed for the earlier forward scheme, we conclude that the backward computation is more efficient for large $T$, with $N$ and $M$ fixed. Of course, the number of operations will still be large if $N$ and $M$ are large (this is the “curse of dimensionality”).

The backward scheme provides much more information: it finds the optimal policy for every initial condition $x_0$, and in fact it tells us what the optimal decision is at every $x_k$ for all $k$. We can restate this last property as follows: the backward scheme yields the optimal control policy in the form of a state feedback law. In the forward scheme, on the other hand, to handle all initial conditions we would need $O(NM T^2)$ operations, and we would still not cover all states $x_k$ for $k > 0$; hence, state feedback is not obtained.

This recursive scheme serves as an example of the general method of dynamic programming.

## Conclusion
In summary, optimal control is a powerful framework for designing control strategies that optimize system performance. 

Optimal Control Theory has been used to obtain solutions to a variety of aerospace engineering problems and holds great promise for other problem areas as well, however, much remains to be accomplished. Hopefully, these notes provide a foundational understanding of optimal control principles. For further exploration, readers are encouraged to delve into advanced topics such as dynamic programming and the Hamilton-Jacobi-Bellman equation.

## References
* https://motion.cs.illinois.edu/RoboticSystems/OptimalControl.html#Section-IV.-DYNAMICS-AND-CONTROL
* https://www.princeton.edu/~aaa/Public/Teaching/ORF523/ORF523_S21_Guest_Lecture.pdf#page=16.00
* Liberzon, D. (2011). Thank you for your interest in this book!.
* Kirk, D. E. (2004). Optimal Control Theory: An Introduction. United States: Dover Publications.
