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

Given the system dynamics in a linear state-space representation:
```math
x_{k+1} = A x_k + B y_k,
```
where 
```math
A = \begin{bmatrix}
1.1 & 0\\ 0 & 0.9
\end{bmatrix}, \quad B = \begin{bmatrix}
0.1 \\ 0.05
\end{bmatrix}, Q = \begin{bmatrix}
1 & 0\\ 0 & 1
\end{bmatrix}, R=0.01
```
and the total time $T=60$, the horiozn time $N=15$, $x_{\text{ref}} = [1, -0.5]^T$, $x_o = [0, 0]^T$, $-5 \leq u \leq 5$, and $-15 \leq x \leq 15$.
```python
# System parameters
A = np.array([[1.1, 0], [0, 0.9]])
B = np.array([[0.1], [0.05]])
Q = np.eye(2)
R = np.array([[0.01]])

# MPC parameters
N = 15 # Prediction horizon
T = 60  # Total simulation time
u_min, u_max = -5, 5 # Input constraints 
x_min, x_max = -15, 15 # State constraints

# Reference state
x_ref = np.array([1, -0.5])

# Initial state
x0 = np.array([0, 0])
```

The cost function and constraints in MPC:
```python
def mpc_cost(U, x0, N, A, B, Q, R, x_ref):
    x = np.copy(x0)
    cost = 0
    for i in range(N):
        u = U[i]
        u = np.array([u])
        cost += (x - x_ref).T @ Q @ (x - x_ref) + u.T @ R @ u # Cost function
        x = A @ x + B @ u
    return cost

# Constraints function
def mpc_constraints(U, x0, N, A, B, u_min, u_max, x_min, x_max):
    x = np.copy(x0)
    constraints = []
    for i in range(N):
        u = U[i]
        u = np.array([u])
        x = A @ x + B @ u # System dynamics as equality constraints
        # Inequality constraints in inputs and states
        constraints.append({'type': 'ineq', 'fun': lambda u=u: u_max - u})
        constraints.append({'type': 'ineq', 'fun': lambda u=u: u - u_min})
        constraints.append({'type': 'ineq', 'fun': lambda x=x: x_max - x})
        constraints.append({'type': 'ineq', 'fun': lambda x=x: x - x_min})
    return constraints
```
The main controling loop is:
```python
x = np.copy(x0)
states = [x]
optimal_controls = []
control_indices = []

# Simulate the system with MPC over time T
for step in range(T):
    res = minimize(mpc_cost, np.zeros(N), args=(x, N, A, B, Q, R, x_ref),
                   constraints=mpc_constraints(np.zeros(N), x, N, A, B, u_min, u_max, x_min, x_max),
                   method='SLSQP')

    optimal_u = res.x[0]  # Only take the first control input
    optimal_controls.append(optimal_u)
    control_indices.append(step)
    
    # Apply the first control input and update the state
    optimal_u = np.array([optimal_u])
    x = A @ x + B @ optimal_u
    states.append(x)
```


