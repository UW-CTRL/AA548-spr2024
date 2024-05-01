# Reachability Analysis

## Scope+Objectives
This bite-sized note will cover Reachability Analysis from our week 5 lecture. It will focus on three key concepts: forward reachability, backward reachability, and the Hamilton-Jacobi reachability methods.

## Introduction
Reachability Analysis is used to determining the set of all the final states $x(t_f)$
reachable from an initial state $x(t_0)$ under specific control strategies.


## Forward Reachability

Forward reachability is employed to confirm that a system can attain its desired states while steering clear of undesirable conditions, such as unsafe or unfeasible scenarios. This approach proactively verifies the capabilities and limitations of a system across different operational contexts

## Backward Reachability

Backward reachability analysis identifies all possible initial states from which a dynamic system can evolve to reach a specified final state or set of states, given certain control inputs.

<img width="805" alt="Reachability" src="https://github.com/UW-CTRL/AA548-spr2024/assets/160373451/9a103a0b-d195-4cf0-bf9e-08ea850e22df">

On the figure above you see T is the Target set. Inside target set h(x) is negative, and outside of the set h(x) is positive. Our starting point is $x(t_0)$ and any point on the trajectory is defined as $x(t)$. h function takes the state X and gives a single number R, it is a scalar valued function. We are trying to minimize the cost function h.

<img width="864" alt="Screen Shot 2024-04-29 at 23 53 07" src="https://github.com/kutaydemiralay/AA548-spr2024/assets/160373451/ea162ed5-7012-4308-8910-1fa06eb89572">

We may also care about entering Target set not on final time but any time between [0,T]
## Hamilton-Jacobi Reachability
Hamilton-Jacobi Reachability is a specific method used in the context of reachability analysis. It uses the Hamilton-Jacobi-Bellman (HJB) or Hamilton-Jacobi- Isaacs (HJI) equations, which are partial differential equations that describe the evolution of the reachability boundary with time. 

There is a HJ toolbox written in in stanford ASL written for python, and matlab available, and can be imported with the command: import hj_reachability in phython. It is a toolbow that allows us to solve reachability problems.

-Example From HJ toolbox

-Example System:Air3d  (Pursuer trying to reach to an Evader)

-System Dynamics:

$$
\begin{aligned}
\dot{x} & = v \cos(\theta), \\
\dot{y} & = v \sin(\theta), \\
\dot{\theta} & = \omega.
\end{aligned}
$$

### Code:

```python
import hj # Assuming 'hj' is a hypothetical module for Hamilton-Jacobi analysis

# Initialize the dynamics for a 3D air system
dynamics = hj.systems.Air3d()

# Create a grid with specific boundary conditions and lattice parameters
grid = hj.Grid.from_lattice_parameters_and_boundary_conditions(
    hj.sets.Box(np.array([-6., -10., 0.]), np.array([20., 10., 2 * np.pi])),
    (51, 40, 50),
    periodic_dims=2
)

# Compute the values as the norm of states, adjusted by a radius of 5 units
values = jnp.linalg.norm(grid.states[..., :2], axis=-1) - 5

# Define solver settings with very high accuracy and a specific postprocessor for reachability
solver_settings = hj.SolverSettings.with_accuracy(
    "very_high",
    hamiltonian_postprocessor=hj.solver.backwards_reachable_tube
)
time = 0.
target_time = -2.8
target_values = hj.step(solver_settings, dynamics, grid, time, values, target_time)
```

-"dynamics" line helps setting up dynamics

-"grid"  line specifiies mesh, the grid points we are solving the ODE over

-"vaules" line specifies the terminal point (we need a termenial point to sole ODE backwards in time. And specifies the minimum distance allowable between 2 planes, which is 5, so we form a circle of 5.

-"solver_setting" we set setting with very high precision, solving for the backward reachable tube.

-"target_time" sets that we are gonna go backwards in time for 2.8 seconds.

-"target_values" solver line.

-Solution Set
<img width="906" alt="Screen Shot 2024-04-29 at 22 32 19" src="https://github.com/kutaydemiralay/AA548-spr2024/assets/160373451/00e43353-7caf-4f4e-9e88-bc8990905ce0">

Contours values show that within 2.8 seconds if the pursuer is in the negative contour set, it would catch the evader. When the pursuer is out of the negative set, within 2.8 seconds, it cannot reach the evader, no matter what. Considering (0,0) is your referance frame and you are the evader.
## Preliminaries

-Backward Reachable Set


Optimal Control Problem:


$$ \min_{u \in U} \left( \int_0^T g(x,u,t) \, dt + g_T(x(T), T) \right), \text{s.t.} \quad \dot{x} = f(x,u,t), \quad x(0) = x_0, \quad u(t) \in U $$

This is a simple control problem where we want to find the optimal control input u, with some constraints on u. In backward reachability, we don't care about running cost, we only look for terminal cost. So we can assume the second term of the above equation, our running cost is zero. 

We are left with only our terminal cost for the optimality problem.  Value function represents cost to go, if there is no running cost,  then value function is just terminal cost which leads:

$$V(x,t)=h(X(t))$$

If $v(x, t) \leq 0$, then you will reach the target within $T - t$ seconds.

If $v(x, t) > 0$, then you will not reach the target within $T - t$ seconds.

We will have to do this for each initial state $x_0$. , instead we can use dynamic programming. For discrete cases we can use Bellman equations, however for our continious case we can use Jamilton Jacobi Bellman Equation.


-Hamilton-Jacobi-Bellman (HJB) Equation

$$ \frac{dV}{dt} + \min_{u \in U} \left( g(x, u, t) + \nabla V(x,t)^T f(x, u, t) \right) = 0 $$

Since our g term, running term is zero:

$$ \frac{dV}{dt} + \min_{u \in U} \left( \nabla V(x,t)^T f(x,u,t) \right) = 0 $$

$\frac{dV}{dt}$: This term represents the partial derivative of the value function $V$ with respect to time $t$. The value function $V(x, t)$ typically measures the minimum cost from state $x$ at time $t$ to some terminal state under the optimal control policy.

$\min_{u \in U} \left( \nabla V(x,t)^T f(x,u,t) \right)$: This expression captures the minimum cost of control action. Here, $u$ ranges over the control set $U$, $\nabla V(x,t)$ is the gradient of the value function $V$ with respect to the state $x$, and $f(x, u, t)$ is the system dynamics function which describes how the state $x$ changes over time under control $u$.

$= 0$: The equation setting this expression to zero ensures that the change in the value function over time, plus the minimum control effort needed to follow the system dynamics optimally, equals zero. This balance implies that the value function correctly captures the minimum expected cost from any state $x$ at any time $t$ under the optimal control policy.

If we can solve this equation, we can get the value function for all the states. Which is basically our backward reachable set.

If we are required to enter the Target set not on final time but on any time within the set [0,T], we can modify our reachability HJB equation as:  

$$ \frac{dV}{dt} + \min_{u \in U} \left(0, \nabla V(x,t)^T f(x,u,t)\right) = 0 $$

This is called Backward Reachable Tube.

What happens If there is disturbance in the system and we cannot use Hamilton Jacobi Bellman Equation, we can use Hamilton-Jacobi-Isaacs (HJI) Equation:

-Hamilton-Jacobi-Isaacs (HJI) Equation

$$ \frac{dV}{dt} + \min_{u \in U} \max_{d \in D} \left(\nabla V(x,t)^T f(x,u,d,t)\right) = 0 $$


$\frac{dV}{dt}$: Represents the time derivative of the value function $V$, which measures the optimal cost-to-go from the state $x$ at time $t$.

$\min_{u \in U} \max_{d \in D} \left(\nabla V(x,t)^T f(x,u,d,t)\right)$: This term reflects the game-theoretic nature of the problem, where the control $u$ from the  set $U$ is optimized against the  $d$ from set $D$. This min-max structure is essential in scenarios involving competitive dynamics.

In HJI the dynamics are described as  f(x, u, d, t), different form HJB, there is d term which describes disturbance. HJI must be used when disturbance is present and robustness against disturbance is wanted.

-Modified HJI Equation
$$ \frac{dV}{dt} + \max_{u \in U} \min_{d \in D} \left(\nabla V(x,t)^T f(x,u,d,t)\right) = 0 $$
The modified version of the Hamilton-Jacobi-Isaacs (HJI) equation shifts the focus from reachability to avoidance by swapping the positions of the  max and  min operators. This change tweaks the optimization strategy to prioritize evading certain outcomes, effectively transforming the problem into one of avoidance analysis

This formulation is particularly relevant in scenarios where the system must avoid certain states or zones, which could be unsafe or non-permissible. 

-Backward Reachable Set

$$R_{\text{backward}}(t, T) = \{( x \in X \mid \exists u(\cdot) \in U \text{ s.t. } \tilde{x} = \xi(x(t_0), u(\cdot), f(\cdot, \cdot, \cdot), t), \tilde{x} \in T )\}$$


Given a target set $T$, $x \in R_{\text{backward}}(t, T)$ if it is possible to reach $T$ when starting at $x$ within $t$ seconds; i.e., there exists at least one control signal that can reach $T$ in $t$ seconds.

$x \in X$: This states that $x$ is any state within the state space $X$.

$\exists u(\cdot) \in U$: This means there exists a control input $u(\cdot)$, which is a function of time, belonging to the set of allowable controls $U$.

Such that $\tilde{x} = \xi(x(t_0), u(\cdot), f(\cdot, \cdot, \cdot), t)$ holds: This specifies that $\tilde{x}$ is the state reached at time $t$, when starting from an initial state $x(t_0)$, under the influence of the control $u(\cdot)$ and following the dynamics of the system described by $f$. The function $\xi$ represents the state trajectory function that maps how states evolve over time.

$\tilde{x} \in T$: Means that the state $\tilde{x}$, reached by applying the control $u(\cdot)$, must be a member of the target set $T$.






We can also create the inverse of reachabile set, which is called Avoid set.

-Backward Avoid Set

$$A_{\text{backward}}(t, T) = \{( x \in X \mid \forall u(\cdot) \in U \text{ s.t. } \tilde{x} = \xi(x(t_0), u(\cdot), f(\cdot, \cdot, \cdot), t), \tilde{x} \in T )\}$$

Given a target set $T$, $x \in A_{\text{backward}}(t, T)$ if it is impossible to avoid $T$ when starting at $x$ within $t$ seconds. This means all possible control signals will lead the system into $T$.

This part of the equation indicates that the condition must hold for all possible control inputs $u(\cdot)$ within the set of allowable controls $U$. This means it's not just that there exists one control function that can achieve the target, but every possible control must be able to reach the target from the initial state.

$x \in X$: Indicates that $x$ represents any state within the state space $X$.

$\tilde{x} = \xi(x(t_0), u(\cdot), f(\cdot, \cdot, \cdot), t)$: Specifies that $\tilde{x}$ is the state reached at time $t$ when the system evolves from the initial state $x(t_0)$ under the influence of any control $u(\cdot)$ and following the dynamics described by $f$.

$\tilde{x} \in T$: States that the reached state $\tilde{x}$ must belong to the target set $T$, according to all control functions.



-Forward Reachable Set
$$R_{\text{forward}}(x(t_0), t) = \{ (\tilde{x} \in X \mid \exists u(\cdot) \in U \text{ s.t. } \tilde{x} = \xi(x(t_0), u(\cdot), f(\cdot, \cdot, \cdot), t)) \}$$

If $\tilde{x} \in R_{\text{forward}}(x(t_0), t)$, then there exists a control signal that can take the system from $x(t_0)$ to $\tilde{x}$ in $t - t_0$ seconds under the rules of control constraints and system dynamics. 

$\tilde{x} \in X$: t $\tilde{x}$  which represents any state that can be reached, is an element of the state space X.

$\exists u(\cdot) \in U$: This means there exists a control input $u(\cdot)$, which is a function of time, belonging to the set of allowable controls $U$.

Such that $\tilde{x} = \xi(x(t_0), u(\cdot), f(\cdot, \cdot, \cdot), t)$: It says that $\tilde{x}$ is the state reached at time $t$ when the system evolves from $x(t_0)$ under the influence of the control $u(\cdot)$ and the system dynamics described by $f$. The function $\xi$ represents the state trajectory, which is determined by the initial state, the control inputs, and the system dynamics over time.


## Conclusion

Reachability analysis helps engineers and designers determine whether a desired state or set of states can be reached or avoided from a given initial condition using available controls. This is foundational in control system design, where the goal is often to steer the system from any initial state to a desired final state efficiently and effectively.

-When you need to ensure that a system can achieve a desired state from a known initial condition, forward reachability is appropriate
-When you need to ensure that no initial condition within a given set leads to unsafe or undesirable states use backward reachability. 
-In scenarios where avoiding specific states is paramount (e.g., unstable conditions, safety violations), backward avoid analysis is used to ensure the system can be controlled away from these states
-HJB equation is focused on optimizing a path to a desired outcome in a controlled setting, the HJI equation manages the complexities of different interactions, ensuring strategies are robust against disturbances. Both equations, therefore, enhance the framework of reachability analysis by not just assessing whether a state can be reached, but optimizing how it is reached or avoided under varying conditions


## References
Stanford ASL, hj Toolbox
