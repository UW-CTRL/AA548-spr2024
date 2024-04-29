# Linear Model Predictive Control (MPC)

## Introduction
Model Predictive Control (MPC) is a control strategy that uses an optimization algorithm to compute the control action by predicting the future behavior of the plant. 

## Theory
Linear MPC uses the linear model of the system to predict future outputs over a certain prediction horizon, based on a sequence of future inputs. The controller optimizes these inputs to achieve the best system performance according to a defined cost function. The cost function typically includes terms for error minimization and control effort. After computing the optimal control inputs, only the first control input is applied. The process repeats at the next time step, incorporating new measurements.

Here is a step-by-step breakdown of the MPC algorithm:

### Step 1: Model the System
Define the system dynamics using a linear state-space representation:
```math
 x_{k+1} = f(x_k, u_k)=Ax_k + Bu_k,
```
where $x_k \in R^n$ is the state vector, $u_k \in R^m$ is the control input, and $A\in R^{n\times n}$, $B\in R^{n\times m}$ are matrices defining the system behavior.
### Step 2: Define Prediction Horizon
Set the prediction horizon $N$, over which the future behavior of the system is predicted and controlled.

### Step 3: Optimization Problem Formulation
#### Objective:
At each time step $k$, formulate an optimization problem to minimize the cost function over the prediction horizon from $k$ to $k+N$. The cost function typically penalizes the deviation from a reference trajectory and the use of control effort:
```math
 J = \sum_{i=k}^{k+N-1} (x_i - x_{\text{ref}})^T Q (x_i - x_{\text{ref}}) + (u_i)^T R (u_i)
```
Where the objective is to minimize the total cost:
```math
\min_{u} J = \sum_{i=0}^{N-1} ((x_{k+i|k} - x_{\text{ref}})^T Q (x_{k+i|k} - x_{\text{ref}}) + (u_{k+i|k})^T R (u_{k+i|k}))
```
#### Constraints:
The optimization is subject to several constraints:
1. **System Dynamics:**
```math
 x_{k+i+1|k} = Ax_{k+i|k} + Bu_{k+i|k}
```
2. **Control and State Constraints:**
   - Control input constraints: $u_{\text{min}} \leq u_{k+i|k} \leq u_{\text{max}}$
   - State constraints: $x_{\text{min}} \leq x_{k+i|k} \leq x_{\text{max}}$
3. **Initial Condition:**
   - $x_{k|k} = x_k$ 


### Step 4: Predict Future States
Predict the future states $x(k+1|k), x(k+2|k), \ldots, x(k+N|k)$ based on the current state $x_k$ and a sequence of hypothetical future control inputs starting from $u_k$.
```math
x_{k+i+1|k} = Ax_{k+i|k} + Bu_{k+i|k}
```
for $i = 0, 1, ..., N-1$ with the initial condition $x_{k|k} = x_k$.
### Step 5: Implement Control
Solve the optimization problem to find the optimal control sequence $u^*|k = [u(k|k), u(k+1|k), \ldots, u(k+N-1|k)]$. Implement the first control action $u(k|k)$ in the plant.

### Step 6: Receding Horizon
At the next time step $k+1$, update the system state $x_{k+1}$, shift the prediction horizon forward, and repeat the process. This shifting or "receding" of the horizon after each time step gives the control strategy its name.

## Example


