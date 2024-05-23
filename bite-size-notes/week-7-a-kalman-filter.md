# Kalman Filter: A Closed Form Solution to State Estimation

### Scope
- Linear discrete time system dynamics
### Objectives
- The implementation of a Kalman filter
- The derivation of the Kalman filter
- A brief description of duality

## Introduction: Help, I Don't Know Where my System is!
Many modern state space control techniques make use of full state feedback, or the practice of using all state variables to determine the appropriate control input. However, unless the dynamics are known exactly or a perfect observer exists (Spoiler alert: they don't), the full state is rarely accessible. Instead, an estimator can be used to predict the values of the unknown state variables based on the observable states, but this introduces some error between the true state and the estimated state. One option for dealing with this error is to frame it as a minimization control problem. In other words, can a controller be designed that drives the error to zero? Fortunately, the answer is yes and one such implementation is the Kalman filter.

## Preliminaries
Given the following variables and initial conditions
- $x$ is the true state
- $y$ is the output
- $u$ is the input
- $\hat{x}$ is the estimated state
- $\tilde{x}=x-\hat{x}$ is the estimation error between the true and estimated states
- $K$ is the Kalman filter gain matrix
- $A,B,C,D$ are the system state space matrices
- $\omega\approx\mathcal{N}(0,Q)$ is some Gaussian process noise sampled from a normal distribution with variance $Q=\mathbb{E}[w_kw_k^T]$
- $v\approx\mathcal{N}(0,R)$ is some Gaussian observer noise sampled from a normal distribution with variance $R=\mathbb{E}[v_kv_k^T]$
- $\hat{x}_0\approx\mathcal{N}(\mu_0,\Sigma_0)$ is the initial state estimate

The following linear discrete time Gaussian dynamics with some additive Gaussian noise will be used to represent a system of interest.

$$x_{k+1}=Ax_k+Bu_k+\omega_k$$

$$y_{k+1}=Cx_k+Du_k+v_k$$

For the puposes of these notes, $D$ is assumed to be zero.

Additionally, recall that passing a Gaussian through linear dynamics produces another Gaussian.

## Main Body
The Kalman filter is a method of estimating unknown variables in a system's current state. It is a recursive process that uses successive observations to update an estimate of the true state. This estimate is represented by a normal distribution with some mean $\mu$ (the average estimated state) and covariance $\Sigma$ (the relationships between state variables).
### Kalman Filter Implementation
The Kalman filter is a two-step process. Both steps must be run at each time step $k$.
1) **Estimate the current state based on the previous state**
   
   $$\mu_k^{pred}=A\mu_{k-1}+Bu_{k-1}$$

   $$\Sigma_k^{pred}=A\Sigma_{k-1}A^T+Q$$

   Here the current predicted mean is estimated by passing the previous mean through the system dynamics. The estimation equation for the predicted variance is derived later in these notes and comprises the bulk of the Kalman filter derivation process.

2) **Update the current state estimation using observations of the outputs**

   The current optimal Kalman gain matrix $K_k$ is calculated using the current predicted variance.

   $$K_k=\Sigma_{k}^{pred}C^T(C\Sigma_{k}^{pred}C^T+R)^{-1}$$

   The Kalman gain is then used to correct the predicted mean and covariance using current observations of the state $y_k$.
   
   $$\mu_k=(I-K_kC)\mu_k^{pred}+K_ky_k$$

   $$\Sigma_k=(I-K_kC)\Sigma_k^{pred}$$

$\mu_k$ can then be used to represent the true state $x_k$k for the purposes of control.

It is important to note that the Kalman filter process runs forwards in time.
### Kalman Filter Derivation
Recall the previously defined linear discrete time Gaussian dynamics.

$$x_{k+1}=Ax_k+Bu_k+\omega_k$$

$$y_{k+1}=Cx_k+Du_k+v_k$$

The goal is to determine an estimated state $\hat{x}$ such that $||x_k-\hat{x}_k||_2^2$ is as small as possible, given $y_0...k$ and $u_0...k$. First, the estimated state at the current time is predicted by passing the previous estimated state through the dynamics.

$$\mu_k^{pred}=A\mu_{k-1}+Bu_{k-1}$$

Furthermore, the current covariance is by definition

$$\Sigma_k^{pred}=\mathbb{E}[(x_k-\mu_k^{pred})(x_k-\mu_k^{pred})^T]$$

Where $x_k$ is the true state at the current time step. This is unknown but what is important here is that the above properties can be calculated as functions of the previous values and the dynamics.

$$x_{k}=Ax_{k-1}+Bu_{k-1}$$

Plugging the equations for $x_k$ and $\mu_k$ into the equation for $\Sigma_k^{pred}$ and simplifying yields a recursive equation for the new covariance prediction in terms of the previous covariance.

$$\Sigma_k^{pred}=A\Sigma_{k-1}a^T+Q$$

Notice that these recursive equations are calculating **predictions** of the current estimated state Gaussian. In order to correct these predictions using observations some update equation must account for any error between the observation and the predicted output. For $\mu_k$ this is relatively straighforward. Similar to a controls problem, the error multiplied by some gain  can be used to drive the predicted mean towards the true mean.

$$\mu_k=\mu_k^{pred}+K_k(y_k-C\mu_k^{pred})$$

Rearranging the above equation yields the update equation previously found in the implementation section. 

To find the update equation for the covariance as well as an update equation for $K_k$, consider the definition of the true covariance.

$$\Sigma_k=\mathbb{E}[(x_k-\mu_k)(x_k-\mu_k)^T]$$

$\mu_k$ can be defined as above, but $y_k$ can be instead written in terms of $x_k$ using $y_k=C(x_k+v_k)$, the definition of the output from the linear dynamics.

$$m_k=(I-K_kC)\mu_k^{pred}+K_kC(x_k+v_k)$$

Plugging this into the previous definition of the true covariance and rearranging yields the following equation for the true covariance.

$$\Sigma_k=(I-K_kC)\mathbb{E}[(x_k-\mu_k^{pred})(x_k-\mu_k^{pred})^T] (I-K_kC)^T+K_k \mathbb{E}[v_kv_k^T] K_k^T$$

However, notice that the expected value here is just $\Sigma_k^{pred}$, or the predicted covariance at the current time step, and that the expected value of the squared $v_k$ is just the covariance $R$ of $v$. Plugging this into the above equation yields the equation that corrects the predicted covariance.

$$\Sigma_k=(I-K_kC)\Sigma_k^{pred}(I-K_kC)^T+K_kRK_k^T$$

This equation determines the covariance at the current time step. In order to have a more accurate system, $K_k$ should be choosen such that the covariance, or amount of variation between variables, is minimized. Rather than directly minimizing the matrix, the trace of the above equation is instead minimized to find $K_k$. This yields the optimal gain matrix $K_k^*$.

$$K_k^*=\underset{K_k}{argmin}Tr(\Sigma_k)$$

Solving the above minimization problem provides the following equation for $K_k$

$$K_k^*=\Sigma_k^{pred}C^T(C\Sigma_k^{pred}C^T+R)^{-1}$$

This equation produces the optimal gain matrix for each time step using the predicted covariance.

Interestingly, the Kalman filter shares duality with the LQR control problem. By plugging the update equation for $\Sigma_k$ into the prediction equation for $\Sigma_{k+1}^{pred}$ and doing much algebra, a Ricatti Equation can be recovered of similar form to the Ricatti Equation solved in LQR.

$$\Sigma_k^{pred}=Q+A\Sigma_{k-1}^{pred}A^T-A\Sigma_{k-1}^{pred}C^T(C\Sigma_{k-1}^{pred}C^T+R)^{-1}C\Sigma_{k-1}^{pred}A^T$$

## Conclusion
The Kalman filter is one solution to state estimation that combines observations and dynamics-derived predictions to more accurately estimate a system's current state. By making assumptions about the type of noise (Gaussian) and dynamics (Linear) of the system an optimal closed form solution can be found.

Additional derivation methods can be found in [3].

## Example Code

```
import numpy as np
import jax
import jax.numpy as jnp

# define a simple 2D quadrotor model, derived by Karen Leung from the University of Washington [4]
class BasePlanarQuadrotor:

    def __init__(self):
        # Dynamics constants
        # yapf: disable
        self.x_dim = 6         # state dimension (see dynamics below)
        self.u_dim = 2         # control dimension (see dynamics below)
        self.g = 9.807         # gravity (m / s**2)
        self.m = 2.5           # mass (kg)
        self.l = 1.0           # half-length (m)
        self.Iyy = 1.0         # moment of inertia about the out-of-plane axis (kg * m**2)
        self.Cd_v = 0.25       # translational drag coefficient
        self.Cd_phi = 0.02255  # rotational drag coefficient
        # yapf: enable

        # Control constraints
        self.max_thrust_per_prop = 1.5 * self.m * self.g  # total thrust-to-weight ratio = 1.5
        self.min_thrust_per_prop = 0  # at least until variable-pitch quadrotors become mainstream :D

    def ode(self, state, control, np=jnp):
        #Continuous-time dynamics of a planar quadrotor expressed as an ODE
        x, v_x, y, v_y, phi, omega = state
        F_str, F_stl = control
        return np.array([
            v_x,
            (-(F_str+F_stl) * np.sin(phi) - self.Cd_v * v_x) / self.m,
            v_y,
            ((F_str+F_stl) * np.cos(phi) - self.Cd_v * v_y) / self.m - self.g,
            omega,
            ((F_str-F_stl) * self.l - self.Cd_phi * omega) / self.Iyy,
        ])

    def discrete_step(self, state, control, dt, np=jnp):
        #Discrete-time dynamics (Euler-integrated) of a planar quadrotor
        return state + dt * self.ode(state, control, np)
    
    def discrete_step_SS(self, state, control, dt, np=jnp):
        jacobFun = jax.jacobian(self.ode,[0,1])
        A,B = jacobFun(state,control)
        C = np.eye(6)
        return state + dt * (np.matmul(A,state)+np.matmul(B,control)),A,B,C

qr = BasePlanarQuadrotor()
initial_state = np.array([10,0,10,0,0,0]) #Initial state
u = np.array([0.1+qr.max_thrust_per_prop,0.1+qr.max_thrust_per_prop]) #Constant input
P = np.eye(6) #Initial state covariance
Q = np.eye(6)*0.1 #Process covariance
R = np.eye(6)*0.1 #Measurement covariance

T_final = 5 #Final time (seconds)
steps = 50 #Number of time steps
dt = T_final/steps #get time step

y = qr.discrete_step(initial_state,u,np) #Get the first observed step

for t in range(steps):
   #Predict the next state
   predicted_mean,A,B,C = qr.discrete_step_SS(initial_state,u,np)
   predicted_covariance = A@P@A.T + Q

   #Update the predicted state
   K = predicted_covariance@C.T@np.linalg.inv(C@predicted_covariance@C.T+R)
   estimated_mean = (np.eye(6)-K@C)@predicted_state+K@y
   estimated_covariance = (np.eye(6)-K@C)@predicted_covariance

   #Get the next observation, done here in the loop because the system is simulated
   y = qr.discrete_step(y,u,np)
   
```

## References
[1] Leung, Karen. “Linear Multivariable Control” Lecture, University of Washington, Seattle, 2024-05-6.

[2] [Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation](https://underactuated.csail.mit.edu/) by Russ Tedrake 

[3] H. Masnadi-Shirazi, A. Masnadi-Shirazi, and M.-A. Dastgheib, “A Step by Step Mathematical Derivation and Tutorial on Kalman Filters.” arXiv, Oct. 08, 2019. [Online]. Available: https://arxiv.org/abs/1910.03558

[4] Leung, Karen. “ME 548 Homework 2”, University of Washington, Seattle, 2024-05-15.
