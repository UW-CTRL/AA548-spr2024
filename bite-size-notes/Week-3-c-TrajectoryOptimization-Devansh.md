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
