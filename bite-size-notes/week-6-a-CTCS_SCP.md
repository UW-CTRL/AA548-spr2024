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
Apart on common notation, e.g. $\mathbb{R}^n$ to denote the n-dimensional real vectors $\mathbb{R}^{n\times m}$ to denote the n-by-m-dimensional real matrices, and  $`\mathbb{R}_+`$ to denote nonnegative real numbers, for each scalar $`v`$ we define the positive part $`|v|_+ = \text{max} \left\{0, v \right\}`$; in addition, operators $`|\square|_+, |\square|, \|\square\|,\square^2`$ are defined element-wise for the argument $\square$. Finally, $1$ and $0$ indicate scalars or vectors, depending on the context. Additional notation is provided throughout the work.


### CT Optimal Control Problem
The standard optimal control problem reads


$$
\begin{array}{rl}
\underset{x,u,t_f}{\text{minimize}} & L(t_f, x(t_f)) \\
\text{subject to} & \dot{x}(t) = f(t, x(t), u(t)) \\
& g(t, x(t), u(t)) \leq 0 \\
& h(t, x(t), u(t)) = 0 \\
& Q(t_0, x(t_0), t_f, x(t_f)) = 0
\end{array}
$$

where $x,u,t$, respectively $x \in \mathbb{R}^n$, $u \in \mathbb{R}^m$, $`t \in \mathbb{R}_+`$, are the state, the control and time; $L$ is the scalar terminal cost function, $`f:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}^m \rightarrow \mathbb{R}^n`$ is the dynamics function, $`g:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}^m \rightarrow \mathbb{R}^g`$, $`h:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}^m \rightarrow \mathbb{R}^h`$ are the path constraint functions, and $`Q:\mathbb{R}_+\times \mathbb{R}^n \times\mathbb{R}_+\times \mathbb{R}^n \rightarrow \mathbb{R}^Q`$ is the boundary condition constraint function.

$L$ may represent as well a running cost: a running cost $\mathcal{L}$ can indeed be embedded in $L$ using numerical integration, i.e. adding a fictitious state $l \in \mathbb{R}$ such that

$$
\begin{array}{c}
l(t_0) = 0, \quad \dot{l}(t) = \mathcal{L}(t, x(t), u(t)), \quad l(t_f) = L(t_f, x(t_f))
\end{array}
$$ 

 Typically $Q$ assumes simple forms, e.g. for prescribed fixed initial and final states $\bar{x}_0, \bar{x}_f$, $Q$ would be the function

$$
Q(t_0, x(t_0), t_f, x(t_f)) = \left[\begin{array}{c} x(t_0) - \bar{x}_0 \\
                                                     x(t_f) - \bar{x}_f     \end{array}\right]
$$ 

i.e. the difference between state at initial and final time and their prescribed values. In addition, $L, f, g, h, Q$ are supposed continuously differentiable; although not always true, smooth approximations of such functions can be built, and the problem solved iteratively, with iteratively more accurate smooth approximations [5]. However, this is another story :guardsman:.

The presented formulation is straightforward and simple from the mathematical perspective; on the other hand, it is not tractable as it is, but a finite number of states $x$ and controls $u$ at defined time instants $t_i$ shall be instead considered. With this last process, called discretization, we get into the heart of the note.

## Main body
### Standard DT Optimal Control problem 
Consider the time horizon $[t_0, t_f]$ and an integer $N>0$. The nodes $t_i, i = 1,\dots,N+1$ are defined in such manner that $t_0 = t_1 \leq \dots \leq t_{N+1} = t_f$. In addition $x_i \coloneqq x(t_i)$, $u_i \coloneqq u(t_i)$ are the discrete states and controls; the subscript $(\square)_i$ The initial problem can be reframed as follows

$$
`
\begin{array}{rl}
\underset{x_i,u_i,t_f}{\text{minimize}} & L(t_{N+1}, x_{N+1}) \\
\text{subject to} & \dot{x}_i = f(t_i, x_i, u_i) \quad i = 1\dots, N\\
& g(t_i, x_i, u_i) \leq 0  \quad i = 1\dots, N\\
& h(t_i, x_i, u_i) = 0  \quad i = 1\dots, N\\
& Q(t_1, x_1, t_{N+1}, x_{N+1}) = 0 \\
& t_1 = t_0, \: t_{N+1} = t_f
\end{array}
`
$$

Eventually, given a parameterized shape $\xi_i(t, p_i)$ for $u_i$, defined over interval $[t_i, t_{i+1}]$, being $p_i$ the set of parameters on which $\xi_i$ depends, the $i$-th dynamics constraint can be treated with a shooting approach, i.e. numerically integrated as follows

$$
x_{i+1} = x_i + \int_{t_{i}}^{t_{i+1}} f(\tau, x(\tau), \xi_i(\tau, p_i))\text{d}\tau
$$

The problem resulting from discretization and application of the shooting approach is tractable, as the amount of unknowns is finite. A common technique to solve the problem consists of applying Sequential Convex Programming (SCP), i.e. repeatedly convexifying and solving the problem itself, up to convergence.

### Inter-sample constraint violation
In the previous discretized formulation, constraints $g$ and $h$ are imposed locally at each node, whereas they should be valid over all time interval. This constitutes a problem for safety-critical applications: the optimizer will tell us that problem is solved, but the actual trajectory may violate constraints $g$ or $h$ between nodes.

Consider a tracking NMPC planar drone problem with dynamics

$$
f(t, x(t), u(t)) = \left[\begin{array}{c} v(t) \\
u(t) - c_d\left\|v(t)\right\|v(t)\end{array}\right]
$$

and running cost

$$
\mathcal{L}(t, x(t), u(t)) = \|r(t) - r_{\text{ref}}(t)\|^2
$$
 
being $r$ the position, $v$ the speed, $u$ the acceleration and $c_d$ a drag coefficient. In addition, the acceleration is limited as $`\|u(t)\|_2 \leq a_{\text{max}}`$. Final condition is not ensured strictly (NMPC standard framework), thus a penalization is present as per final deviation with respect to referene. Notice $r_{\text{ref}}(t)$ is usually computed at low frequency, ignoring the real environment the drone shall move in: hypothesize that the tracking problem, given a finite horizon of length $T$, shall be solved in presence of moving obstacles, not accounted for in $r_{\text{ref}}(t)$. 

Ideally, for each obstacle $j$, the following constraint holds

$$
g^j(t, x(t), u(t)) = 1 - \left\| S^j(r(t) - r^j_{\text{obs}})\right\| \leq 0
$$
 
given the shape operator $S^j$ of the obstacle $j$-th.

Discretizing the previous problem leads, for each node $i$ and obstacle $j$, to 

$$
g^j(t_i, x_i, u_i) = 1 - \left\| S^j(r_i - r^j_{\text{obs}})\right\| \leq 0
$$

What happens between node $i$ and node $i+1$ with the discrete formulation? The result follows: drone, at the first obstacle, goes :boom:. The SCP solver updates the nodes (black dots) in such manner they are not inside of the visible ellipses, i.e. the obstacles. However, SCP is not detecting that **between** the node points, drone is piercing through the obstacles. In other words, for SCP everything is fine, and drone is safe. No way.

![CTCS_noCS](figs/CTCS_noCS.gif)
*Retrieved with permission from [4]

### CTCS-granting Optimal Control
To overcome the inter-sample constraint violation problem, [2] makes use of *exterior penalty functions*, thus allowing a similar approach to the dynamics shooting; consider the fictitious state $y \in \mathbb{R}^{g+h}$ with time derivative

$$
\dot{y}(t) \coloneqq 1^\top|g(t, x(t), u(t))|^2_+ + 1^\top h(t, x(t), u(t))^2
$$

 From $y$ dynamics definition, $\dot{y}(t) \geq 0\; \forall t \in [t_0, t_f]$; if boundary condition $y_0 = y_f$ is satisfied, then $\dot{y}(t) = 0 \;  \forall t \in [t_0, t_f]$, implying constraints $g(t, x(t), u(t)) \leq 0, h(t, x(t), u(t))=0$ are satisfied $\forall t \in [t_0, t_f]$. Therefore, the discretized problem reads

$$
`
\begin{array}{rl}
\underset{x_i,p_i,t_f}{\text{minimize}} & L(t_{N+1}, x_{N+1}) \\
\text{subject to} & x_{i+1} = x_{i} + \int_{t_{i}}^{t_{i+1}} f(\tau, x(\tau), \xi_i(\tau, p_i))\text{d}\tau  \quad i = 1\dots, N\\[1ex]
& y_{i+1} = y_{i} + \int_{t_{i}}^{t_{i+1}} [1^\top |g(\tau, x(\tau), \xi_i(\tau, p_i))|^2_+ + 1^\top h(\tau, x(\tau), \xi_i(\tau, p_i))^2]\text{d}\tau  \quad i = 1\dots, N\\[1ex]
& y_{i+1} - y_{i} \leq \varepsilon \quad i = 1\dots, N\\[1ex]
& Q(t_1, x_1, t_{N+1}, x_{N+1}) = 0 \\[1ex]
& t_1 = t_0, \: t_{N+1} = t_f
\end{array}
`
$$

where the positive variable $\varepsilon$ allows to 1) grant Linear Independence Constraint Qualification (LICQ), making the problem tractable, and 2) leverage an exact penalty approach [2]. In addition, the given $\varepsilon$ is linked with the maximum pointwise violation of the considered path constraints, according to the following relation 

$$
\int_{t_k}^{t_{k+1}} |g^j(\tau, x(\tau), \xi_i(\tau, p_i)) + \delta_{g^j}(\varepsilon)|^2_+ \text{d}t \leq \varepsilon \Rightarrow g^j(\tau, x(\tau), \xi_i(\tau, p_i)) \leq 0, \; \forall t \in [t_k, t_{k+1}]
$$

where the monotonic increasing function $\delta_{g^j}(\varepsilon)$ can be estimated at a first iteration; the SCP approach can then be adopted, for decreasing $\varepsilon$, up to satisfaction of a desired tolerance $\texttt{tol}$: $\delta_{g^j}(\varepsilon) < \text{\texttt{tol}}$.

### Inter-sample constraint satisfaction
The developed framework is now applied to the same drone example as before, but now embedding the obstacle avoidance constraint in the CTCS formulation. Constraint is rported again for clarity here below.

$$
g^j(t, x(t), u(t)) = 1 - \left\| S^j(r(t) - r^j_{\text{obs}})\right\| \leq 0
$$

The NMPC framework now provides far better results with respect to the case outlined before, and nodes are correctly placed in such manner to correctly satisfy the assigned constraints.

![CTCS_yesCS](figs/ctcs_animation.gif)
*Retrieved with permission from [4]

## Conclusion
The present note has outlined one of the most recent advances in SCP-based algorithms, that allows Continuous-Time Constraint Satisfaction in spite of the DT formulation of the original optimal control problem. The developed framework has been applied to an obstacle avoidance drone example; additional ones are as well available in literature [2], such as rocket-landing scenarios and some others are yet to be proposed.

## References
1. NASA Systems Analysis Branch Guidance and Control Division - **Space Shuttle Guidance, Navigation and Control - Design Equations - Volume IV Deorbit and Atmospheric Operations** - NASA Technical Report, 1972 

2. Elango, P., Luo, D., Kamath, A.G., Uzun, S., Kim, T. Acikmese, B. - **Successive Convexification for Trajectory Optimization with Continuous-Time Constraint Satisfaction** - Arxiv, 2024 

3. Uzun, S., Elango, P., Kamath, A.G., Kim, T. Acikmese, B. - **Successive Convexification for Nonlinear Model Predictive Control with Continuous-Time Constraint Satisfaction** - ArXiV, 2024

4. GitHub, sametuzun781, https://github.com/sametuzun781/NMPC-CTCS

5. Malyuta, D., Acikmese, B. - **Fast Homotopy for Spacecraft Rendezvous Trajectory Optimization with Discrete Logic** - Journal of Guidance, Control, and Dynamics, 2023

