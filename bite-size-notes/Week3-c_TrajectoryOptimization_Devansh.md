## Trajectory Optimization - Linear Systems
# Objectives
1. How CBF's and CLF's contribute towards Trajectory Optimization.
2. Convex Formulation for Linear System:

   a. Continuous Time
  
   b. Discrete Time

## Introduction

Traditional approaches of acheiving optimal control with a feedback controller has a lot of limitations, such as they do not work well for systems with state dimensions above 4 or 5, effectiveness of the controller is limited to the specific region of state space where linearization is an approximation of non-linear dynamics and so on. Trajectory Optimization is an essential computational tool to formulate a simpler version of optimization problem by linearizing over a nominal operation point(tajectory), attempting to find optimal control solution which is valid only form a single initial condition rather than finding a solution for the entire state space fro a feedback controller. For this to happen, we represent this solution as a trajectory, x(t), u(t) , defined over a finite interval 'tf'. 

## Prelimnaries

### Control Lyapunov Functions (CLF) and Control Barrier Functions (CBF)

**CLF:**
A Control Lyapunov Function (CLF) is a function used to design controllers that ensure stability of a dynamical system. It measures the "distance" of the system's state from an equilibrium point and drives this distance to zero over time.

**CBF:**
A Control Barrier Function (CBF) is a function used to design controllers that ensure safety of a dynamical system. It enforces constraints on the system's state to avoid unsafe regions of operation, such as collision or violation of operational limits.

## Usage in Trajectory Optimization

In trajectory optimization, CLFs and CBFs play crucial roles in ensuring both stability and safety of the system's trajectory:

1. **Stability (CLF):** CLFs are used to design controllers that stabilize the system along the desired trajectory. By ensuring that the CLF decreases along the trajectory, the controller can drive the system towards the desired state while maintaining stability.

2. **Safety (CBF):** CBFs are used to enforce safety constraints during trajectory optimization. These constraints could include avoiding collisions with obstacles, staying within operational limits (e.g., velocity, acceleration), or respecting physical constraints of the system. By incorporating CBFs into the optimization problem, the trajectory planner can generate feasible trajectories that guarantee safety.

## Combining CLF & CBF: 

CLF's usually is a function to move towards a target as stated above whereas CBF's is a function which confines the system to be in a defined set. 

Here, 

$$\min_{u} \lVert \mathbf{u^des - u} \rVert_2^2$$

$$\text{subject to} \quad u \in U $$

$$\text{CLF} \quad \nabla V(x)^T f(x, u) <= 0 or -\alpha V(x)$$ where $-\alpha V(x)$ has to be positive only.

$$\text{CBF} \quad \nabla V(x)^T f(x, u) <= -\beta b(x)$$

Sometimes, there could be conditions when the regions of CLF constraint and CBF constraint do not overlap each other. This is when one can encounter infeasible problem. 
In such a case, we have to provide a slack or let loose on a constraint, so either of the one (CLF constraint or CBF constraint) can come together and form a feasible region. 


For such a case: 

$$\min_{u} \lVert \mathbf{u^des - u} \rVert_2^2 + \gamma \epsilon^2$$ where $\gamma$ is the weighting and $\epsilon$ is the slack variable.

$$\text{subject to} \quad u \in U $$

$$\text{CLF} \quad \nabla V(x)^T f(x, u) <= \epsilon$$ where this is Linear in u and \epsilon.

$$\text{CBF} \quad \nabla V(x)^T f(x, u) <= -\alpha b(x)$$ where this is linear in u.

And, $\epsilon >= 0$ , hence Linear in $\epsilon$ 

## Problem Formulation
The goal of trajectory optimization is to find the trajectory \( x(t) \) that minimizes (or maximizes) a certain cost function \( J \), subject to constraints. Mathematically, it can be formulated as:

### Discrete Time Dynamics

$$\min_{x_(0)...x_k+1, u_(0)...u_(k)} J_termial(x_(k+1)) + \sum_{k=0}^K J(x_(k), u_(k), k)$$

$$\text{subject to} \quad x_{k+1} = f(x_(k), u_(k), k), \quad k=0,...K$$

$$ \quad \quad \quad x_k \in X, \quad u_(k) \in U_(k), \quad x_(0) = x_current or u_(k) \in U_(k)(x_k)$$ 

Here,

$$J_termial(x_(k+1)$$ is the control effort or the Terminal cost for reaching the point 'k'.  

$$\sum_{k=0}^K J(x_(k), u_(k), k)$$ is the running cost or the cost-to-go 

$$x_{k+1} = f(x_(k), u_(k), k)$$ also determines if the state is feasible and the control is feasible or not. 

Here we understand that: 
1. We are solving only for the current state $(x_0 = x_current)$.
2. One will have to re-solve the entire problem for every different state
3. Sometimes, one will have to solve the problem over and over again.

### Continuous Time Dynamics

$\min_{x_(0)...x_t+1, u_(0)...u_(t)} J_termial(x_(t+1)) + \int_{t=0}^{T_f} J(x_(t), u_(t), t)$$

$$\text{subject to} \quad x_{k+1} = f(x_(t), u_(t), t), \quad k=0,...K$$

$$ \quad \quad \quad x_t \in X, \quad u_(t) \in U_(t), \quad x_(0) = x_current or u_(t) \in U_(t)(t_k)$$ 

## Convex Formulation for Linear Systems

### Continuous-Time Dynamics
For linear systems with continuous-time dynamics, the trajectory optimization problem can be formulated as a convex optimization problem. Consider a continuous-time linear system with state $( x(t) )$ and control input $( u(t) )$, governed by the dynamics:
$$\dot{x}(t) = Ax(t) + Bu(t)$$

### Discrete-Time Dynamics
For discrete-time linear systems, the trajectory optimization problem is naturally formulated as a convex optimization problem. Consider a discrete-time linear system with state $( x_k )$ at time step $( k )$ and control input $( u_k )$, governed by the dynamics:
$$x_{k+1} = Ax_k + Bu_k$$

## Detailed Example: Continuous-Time Dynamics
Consider a simple continuous-time linear system with state $( x(t) = [x_1(t), x_2(t)]^T )$ representing position and velocity, and control input $u(t) = u_1(t)$ representing acceleration. Let $( A )$ and $( B )$ be given matrices defining the system dynamics.

The cost function could be a quadratic form:
$$J = \int_{t_0}^{t_f} \left( x^T Q x + u^T R u \right) \, dt $$

Where (Q), (R) are positive definite weighting matrices.
This problem can be further solved using convex optimization techniques such as QP or other solvers capable of handling convex problems.

## Detailed Example: Discrete-Time Dynamics
Consider the same linear system with discrete-time dynamics:
$$x_{k+1} = Ax_k + Bu_k$$

We want to minimize the cost function:
$$J = \sum_{k=0}^{N-1} \left( x_k^T Q x_k + u_k^T R u_k \right) + x_N^T Q_f x_N$$

Where (Q), (R) and $(Q_f)$ are positive definite weighting matrices.
This problem can be further solved using convex optimization techniques such as QP or other solvers capable of handling convex problems. 

## CONCLUSION

We understand the importance of CLF's and CBF's how they are used in Trajectroy optimization and also about the disjoint sets. Moreover, the problem of optimization stated above stands true only for Linear systems and also which are convex in nature. For non-convex problems, the system is first converted to convex-type and then solved further. 
We have discussed the Direct Transcription method for Trajectory Optimization, but there are other methods such as shooting method which gives a general idea of reaching a goal within a given time interval and also tells us if the trajectory is dynamically feasible or not. We have not quite discussed the QP, quadratic programming part but its a part of Trajectory optimization which can be discussed further. 

## References

1. "Underactuated Robotics - Algorithms for Walking, Running, Swimming, Flying and Manipulation", Russ Tendrake. (https://underactuated.mit.edu/trajopt.html)
2. https://www.matthewpeterkelly.com/tutorials/trajectoryOptimization/index.html
3. https://youtu.be/wlkRYMVUZTs?si=iTx9BR0SJLFSp-WA
 
