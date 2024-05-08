# Continuous-Time Constraint Satisfaction in Sequential Convex Programming

**Scope: Continuous-Time Constraints Satisfaction in Discrete-Time Trajectory Optimization**
 Optimal Control is a widely-used mathematical tool in the context of Trajectory Optimization; Sequential Convex Programming (SCP) has been successfully employed for solving Discrete-Time (DT) Optimal Control problems online, allowing vehicles to autonomously replan their reference. The present note frames how to guarantee Continuous-Time Constraint Satisfaction (CTCS) within such DT Optimal Control problems; a drone NMPC controller for obstacle avoidance is presented.

**Objectives**
 - Understand criticalities of DT Trajectory Optimization;
 - Learn how to guarantee CTCS within a CT framework;
 - Verify capabilities of CTCS for application scenarios.

## Introduction
Direct methods are extremely popular for trajectory optimization purposes: they directly treat control as optimization variable while satisfying CT dynamics constraints and CT path constraints of different nature (e.g. max thrust from a rocket engine). Such direct methods are widely used for both offline and online planning purposes: convex formulation of such problems further ensures polynomial-bounded limit on computational time. Direct methods follow a discretize-then-optimize approach, and rely on heuristics to choose a discretization  approach appropriate for the required accuracy and computational performances. State-of-the-art techniques are available to ensure satisfaction of dynamics in a CT framework; on the other hand, no unified method exists to grant satisfaction of path constraints in CT; reason is the following: dynamics can be integrated with high accuracy with 'shooting' approaches (Space Shuttle Guidance is an illustrious example [1]), whereas path constraints can be only imposed at node points, i.e. the time instants at which state and controls are optimization variables. The problem becomes more relevant if few nodes are used, which is common for online applications, e.g. when a SCP techinque is employed. The present note digs into a recently-proposed approach to grant CT satisfaction of path constraints, up to arbitrarily high degree of accuracy [2]; an open-source drone NMPC controller applied to an obstacle-avoidance scenario is used as example [3,4].  

## Preliminaries
### Notation
Apart on common notation, e.g. $\mathbb{R}^n$ to denote the n-dimensional real vectors $\mathbb{R}^{n\times m}$ to denote the n-by-m-dimensional real matrices, and  $\mathbb{R}_+$ to denote nonnegative real numbers, for each scalar $ v $ we define the positive part $|v|_+ = max \left{0, v \right}$; in addition, operators $|\square|_+, |\square|, \square^2$ are defined element-wise for the argument $\square$. Finally, $1$ and $0$ indicate scalars or vectors, depending on the context.

### CT Optimal Control Problem
The standard optimal control problem reads


$$
\begin{array}{rl}
\underset{x,u,t_f}{\text{minimize}} & L(t_f, x(t_f)) \\
\text{subject to} & \dot{x}(t) = f(t, x(t), u(t)) \\
& g(t, x(t), u(t)) \leq 0 \\
& h(t, x(t), u(t)) = 0 \\
& Q(t_i, x(t_i), t_f, x(t_f)) = 0
\end{array}
$$

where $x, u, t$, respectively $x \in \mathbb{R}^n$, $u \in \mathbb{R}^m$, $t \in \mathbb{R}_+$, are the state, the control and time; $L$ is the scalar terminal cost function, $f:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}^m \rightarrow \mathbb{R}^n$ identifies the dynamics function, $g:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}^m \rightarrow \mathbb{R}^g$, $h:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}^m \rightarrow \mathbb{R}^h$ the path constraint functions, and $Q:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}_+\times \mathbb{R}^n \rightarrow \mathbb{R}^Q$ the boundary condition constraint function. Typically $Q$ assumes simple forms, e.g. for prescribed fixed initial and final states $\bar{x}_i, \bar{x}_f$, $Q$ would be the function

where $ x, u, t $, respectively $x \in \mathbb{R}^n$, $u \in \mathbb{R}^m$, $t \in \mathbb{R}_+$, are the state, the control and time; $L$ is the scalar terminal cost function, $f:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}^m \rightarrow \mathbb{R}^n$ identifies the dynamics function, $g:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}^m \rightarrow \mathbb{R}^g$, $h:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}^m \rightarrow \mathbb{R}^h$ the path constraint functions, and $Q:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}_+\times \mathbb{R}^n \rightarrow \mathbb{R}^Q$ the boundary condition constraint function. Typically $Q$ assumes simple forms, e.g. for prescribed fixed initial and final states $\bar{x}_i, \bar{x}_f$, $Q$ would be the function

$$
Q(t_i, x(t_i), t_f, x(t_f)) = \left[\begin{array}{c} x(t_i) - \bar{x}_i \\
                                                     x(t_f) - \bar{x}_f     \end{array}\right]
$$ 

which selects the state at initial and final time and sets to zero their differences with their prescribed values. In addition, the functions outlined in the previous problem formulation are supposed continuously differentiable; although this is not always true in practice, still smooth approximations of such functions can be built, and the problem being solved iteratively, with iteratively more accurate smooth approximations [5]. However, this is another story :guardsman:.

The presented formulation is extremely straightforward and simple from the mathematical perspective; on the other hand, for practical applications, it is hardly ever tractable as it is, but we need to discretize the prescribed time domain, adding nodes to the time domain. With this last process, called discretization, we get into the heart of the note.

## Main body
### DT Optimal Control problem 
### Inter-sample constraint violation
### CTCS-compatible Optimal Control
### Inter-sample constraint satisfaction

## Conclusion


## References
1. NASA Systems Analysis Branch Guidance and Control Division - **Space Shuttle Guidance, Navigation and Control - Design Equations - Volume IV Deorbit and Atmospheric Operations** - NASA Technical Report, 1972 

2. Elango, P., Luo, D., Kamath, A.G., Uzun, S., Kim, T. Acikmese, B. - **Successive Convexification for Trajectory Optimization with Continuous-Time Constraint Satisfaction** - Arxiv, 2024 

3. Uzun, S., Elango, P., Kamath, A.G., Kim, T. Acikmese, B. - **Successive Convexification for Nonlinear Model Predictive Control with Continuous-Time Constraint Satisfaction** - ArXiV, 2024

4. GitHub, sametuzun781, https://github.com/sametuzun781/NMPC-CTCS

5. Malyuta, D., Acikmese, B. - **Fast Homotopy for Spacecraft Rendezvous Trajectory Optimization with Discrete Logic** - Journal of Guidance, Control, and Dynamics, 2023

6.
