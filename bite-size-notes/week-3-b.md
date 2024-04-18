# Nonconvex Trajecotry Optimization

## Scope and Objectives
These notes highlight some examples of non-covex optimal control problems and an overview to solve non-convex optimal control problems.

## Introduction
It is understood that convex programming problems can be reliably solved in polynomial time to global optimality. Furthermore, recent advances have shown that these problems can be solved in realtime by both generic Second Order Cone Programming (SOCP) solvers, and by customized solvers which take advantage of specific problem structures. However, most real-world problems are not such inherently convex, and thus not readily to be solved. These problems involve continuous-time nonlinear dynamics, and possibly non-convex state and control constraints.

An example in aerospace applications is the planetary landing problem. Here, non-convexity arises from minimum thrust constraints (non-convex control constraints) imposed on the vehicle and from non-linear, time-varying gravity fields and aerodynamic forces (nonlinear dynamics). State constaints may render the problem non-convex as well. This is found in the optimal path planning for autonomous vehicles in the presence of obstacles.

Past work has been done to show that non-convexity arising from control constraints, state constraints and nonlinear dynamics can be posed as convex via convexification. Below describes a strategy to tackle non-convexity arising from nonlinear dynamics from *"Successive Convexification of Non-Convex Optimal Control Problems and Its Convergence Properties".*

## Preliminaries

## Main Body
This approach to convexify a non-convex problem caused by nonlinear dynamics involves successively linearize the dynamic equations and solve a sequence of convex subproblems continuously until convergence is achieved.

### Problem Formulation
Assuming control and state constraints as well as the cost function are convex, the following NonConvex Optimal Control Problem (NCOCP) is considered:

*Determine Determine a control function $u^∗ ∈ L_∞[0,T]^m$, and a state trajectory $x^∗ ∈ W_1,∞[0,T]^n$
, which minimizes*

$C(x,u) := ϕ(x(T),T) +\int_0^TL(x(t),u(t),t) dt,$

s.t. :

$\dot x(t) = f(x(t),u(t),t) \quad 0 ≤ t ≤ T,$

$u(t) ∈ U \quad 0 ≤ t ≤ T,$

$x(t) ∈ X \quad 0 ≤ t ≤ T$

### Algorithm Description
The only non-convexity lies in the nonlinear dynamics. A way to convexify it is linearization by using its first order Taylor approximation. However, the solution of the convexified problem won't necessarily be the same as the non-convex problem. To recover optimality, successive linearization can be done $-$ linearize the dynamics about the trajectory in $k^{th}$ succession and the corresponding control in the $(k-1)^{th}$ succession until convergence.

#### Linearization:

Assume the $(i-1)^{th}$ succession gives us a solution $(x^{i-1}(t),u^{i-1}(t))$. Let

$A(t) = \frac{\partial}{\partial x}f(x^{i-1}(t),u^{i-1}(t),t),$

$B(t) = \frac{\partial}{\partial u}f(x^{i-1}(t),u^{i-1}(t),t),$

$D(t) = \frac{\partial}{\partial t}f(x^{i-1}(t),u^{i-1}(t),t),$

$d(t) = x(t) - x^{i-1}(t)$ and $w(t) = u(t) - u^{i-1}(t)$

First order Taylor expansion about that solution will be

$\dot d(t) = A(t)d(t) + B(t)w(t) + D(t) + H.O.T...$

$d(t)$ and $w(t)$ are the new states and control respectively. The linearization gives convexity but introduces *artificial infeasibility* and *approximation error*. To address these respectively, *virtual control* and *trust regions* are introduced in the convexification scheme.

#### Virtual Control
An infeasible problem can arise at various points in the solution space using linearization, even if the nonlinear solution is feasible. This infeasibility is said to be artificial and is undesireable as it obstructs the iteration process, preventing convergence. To prevent this, an additional control input, $v(t)$ called *virtual control* is introduced to the linear dynamics:

$\dot d(t) = A(t)d(t) + B(t)w(t) + E(t)v(t) + D(t)$

$E(t)$ can be chosen based on $A(t)$ such that $A(^.)E(^.)$ is controllable. Since $v(t)$ is unconstrained,
any state in the feasible region can be reachable in finite time. The cost function is adjusted to add $\lambda \gamma (Ev)$, where $\lambda$ is the penalty weight, and $\gamma(^.)$ is the penalty function.

The penalized cost after linearizing: 

$L(d,w) := C(x,u) + \lambda \gamma (Ev)$

The penalized cost before linearizing: 

$J(x,u) := C(x,u) +\lambda \gamma (\dot x− f)$

#### Trust regions
Another concern when linearizing is approximation errors. When large deviation is allowed and occurs, the linear approximation sometimes fails to capture the distinction made by nonlinearity. To overcome this, the linearized trajectory is ensured not to deviate significantly from the nominal one via a trust region on the new control input, 

$||\omega||_∞ ≤ \Delta$ 

thus the new state will be restricted within the trust region as well due to the dynamic equations. Adjusting the trust region radius $\Delta$ with each iteration, the solution will converge eventually.

#### Final problem formulation
Considering the *virtual control* and *trust regions*, a Convex Optimal Control Problem is solved at $k^{th}$ succession:

*Determine $w^∗ ∈ L_∞[0,T]^m$, and $d^∗ ∈ W_1,∞[0,T]^n$
, which minimizes*
 
$L(d,w) := C(x,u) + \lambda \gamma (Ev)$,

s.t. : 

$\dot d(t) = A(t)d(t) + B(t)w(t) + E(t)v(t) + D(t),$

$||\omega||_∞ ≤ \Delta,$

$u^k(t)+w(t) ∈ U,$

$x^k(t)+d(t)∈X$

This convex subproblem can then be solved successively, evaluating if the quality of linear approximations are good or not for each iteration. If it is, accept that step and expand the trust region for the next step. If it isn't, reject it, contract the trust region, and repeat the step. This is done until convergence is achieved.

## Conclusion
This note summarizes the basic idea of convexification of a non-convex problem contributed by nonlinear dynamics as presented in *"Successive Convexification of Non-Convex Optimal Control Problems and Its Convergence Properties".* This reference includes a convergence analysis of the algorithm presented, as well as a simple yet illustrative numerical example that are not convered in this note.
## References
[1] *Yuanqi Mao, Michael Szmuk, Behcet Acıkmese*, "Successive Convexification of Non-Convex Optimal Control Problems and Its Convergence Properties".