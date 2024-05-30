 # Extended Kalman Filter

## Scope and Objectives
Since the normal Kalman filter can only deal with linear dynamics problems, extended Kalman filter(EKF) is the nonlinear version of the Kalman filter which linearizes about an estimate of the current mean and covariance. 

In this note, I intend to provide a brief introduction to EKF, specifying EKF problems, and explaining its relevance to control theories. After reading this note, readers will understand the basic concepts and motivations with EKF.  

## Introduction
The extended Kalman filter(EKF) is a very useful tool in control theory and estimation. It is widely used for nonlinear control systems. As an extension of traditional Kalman filter, EKF addresses the challenges of nonlinearity, making it be more extensively used in control theory. 

In real world systems, such as autonomous vehicles and aircrafts, exhibit nonlinear behaviors. The EKF provides a systematic approach to estimate the states of those nonlinear systems with noisy environment. Also, in robotics, for localization and mapping(SLAM), where a robot must estimate its position and orientation within an environment. 

The papers related to mathematical foundations of Kalman filter were firstly established between 1959 and 1961. Kalman filter is an optimal way to deal with linear system with noise. However, most of the engineering problems are nonlinear. So attempts are made to solve those nonlinear problems. The EKF used calculus multivariating Taylor Series to linearize a model of a working point. If a system is not well known, praticle filters(PF) are used for estimation. 

## Preliminaries
### Definition
In EKF, the state transition and observation models do not need to be linear, but just differentiable. 

$$ \mathbf{x_k} = f (\mathbf{x_{k-1}}, \mathbf{u_{k-1}}) + \mathbf{w_k} $$

$$ \mathbf{y_k} = h (\mathbf{x_k}) + \mathbf{v_k} $$

where $\mathbf{w_k}$ and $\mathbf{v_k}$ are the process and observation noises, the process and measurement noises are both considered as Gaussian, like

$$ \mathbf{x_{k+1}} = \mathbf{A}\mathbf{x_k} +  \mathbf{B}\mathbf{u_k} $$

$\mathbf{x_k}$ is the state, $\mathbf{u_{k-1}}$ is the control input vector of the previous state. $\mathbf{z_k}$ is the measurement vector at time step k, f is a nonlinear function describing state transition, and h is a nonlinear function relating the state to the measurements. 

We need Jacobian to linearize function f and h

$$ \mathbf{F_{k-1}} = \frac{\partial f}{\partial x} \mid_{x_{k-1}, u_{k-1}} $$

$$ \mathbf{H_{k}} = \frac{\partial h}{\partial x} \mid_{x_k\mid_{k-1}} $$

## Main Body

The EKF operates in two steps: prediction and update.
The first step is prediction. The predicted step estimate is

$$ \mathbf{x_k\mid_{k-1}} = f (\mathbf{x_{k-1 \mid{k-1}}}, \mathbf{u_{k-1}}) $$

The predicted covariance estimate is 

$$ \mathbf{P_k\mid_{k-1}} = \mathbf{{F_{k-1}}P_{k-1 \mid{k-1}}{F_{k-1}}^T}+\mathbf{Q_{k-1}} $$

In update step, the measurement(innovation) residual is 

$$ \tilde{\mathbf{y_k}} = \mathbf{z_k}-h(\hat{\mathbf{x_k\mid_{k-1}}})$$

The innovation covariance is 

$$ \mathbf{S_k} = \mathbf{{H_k}P_{k\mid{k-1}}{H_{k}}^T}+\mathbf{R_{k}} $$

The Kalman Gain is 

$$ \mathbf{K_k} = \mathbf{P_{k\mid{k-1}}{H_{k}}^T{S_{k}}^{-1}} $$

Updated state estimate is 

$$ \hat{\mathbf{x_k\mid_k}} = \hat{\mathbf{x_k\mid_{k-1}}} + \mathbf{K_k}\tilde{\mathbf{y_k}}$$

Updated covariance estimate is 

$$ \mathbf{P_k\mid_k} = \mathbf{({I} - {K_k}{H_k}){P_{k\mid{k-1}}}} $$

### Code Snippet

```

import numpy as np
import matplotlib.pyplot as plt

def f(x, u):
    # Define the state transition function
    # Example: simple motion model
    return np.array([x[0] + u[0] * np.cos(x[2]),
                     x[1] + u[0] * np.sin(x[2]),
                     x[2] + u[1]])

def h(x):
    # Define the measurement function
    # Example: directly observing the state
    return x

def jacobian_f(x, u):
    # Define the Jacobian of the state transition function
    return np.array([[1, 0, -u[0] * np.sin(x[2])],
                     [0, 1, u[0] * np.cos(x[2])],
                     [0, 0, 1]])

def jacobian_h(x):
    # Define the Jacobian of the measurement function
    return np.eye(len(x))

x = np.array([0, 0, 0])  # Initial state
P = np.eye(3)            # Initial state covariance
Q = np.eye(3) * 0.1      # Process noise covariance
R = np.eye(3) * 0.1      # Measurement noise covariance
u = np.array([1, 0.1])   # Control input (velocity, turn rate)
z = np.array([1, 1, 0.1]) # Measurement


x_pred = f(x, u)
F = jacobian_f(x, u)
P_pred = F @ P @ F.T + Q


y = z - h(x_pred)
H = jacobian_h(x_pred)
S = H @ P_pred @ H.T + R
K = P_pred @ H.T @ np.linalg.inv(S)
x = x_pred + K @ y
P = (np.eye(len(x)) - K @ H) @ P_pred

print("Updated state estimate:", x)
print("Updated error covariance:", P)

dt = 0.1
time_steps = 100

true_states = []
measurements = []
state = np.array([0, 0, 0])
for t in range(time_steps):
    state = f(state, u)
    true_states.append(state)
    measurement = state + np.random.multivariate_normal([0, 0, 0], R)
    measurements.append(measurement)

true_states = np.array(true_states)
measurements = np.array(measurements)

estimates = []
x = np.array([0, 0, 0])
P = np.eye(3)
for z in measurements:
    # Prediction
    x_pred = f(x, u)
    F = jacobian_f(x, u)
    P_pred = F @ P @ F.T + Q

    # Update
    y = z - h(x_pred)
    H = jacobian_h(x_pred)
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    x = x_pred + K @ y
    P = (np.eye(len(x)) - K @ H) @ P_pred

    estimates.append(x)

estimates = np.array(estimates)

plt.figure(figsize=(10, 8))
plt.plot(true_states[:, 0], true_states[:, 1], label='True Path')
plt.scatter(measurements[:, 0], measurements[:, 1], label='Noisy Measurements', c='r', s=10)
plt.plot(estimates[:, 0], estimates[:, 1], label='EKF Estimates', linestyle='--')
plt.legend()
plt.xlabel('X position')
plt.ylabel('Y position')
plt.show()
```

### Applications

EKF is widely used in various fields such as robotics, aerospace, automotive, and finance, where systems are mostly nonlinear.

## Conclusion

The key takeaways from this note are
1. EKF operates on nonlinear models
2. Using Jacobian to linearize these models
3. Prediction and updates
4. Code implementation
5. Applications

What was not covered in the notes are:

1. Particle filters (PF)
2. Figure, diagrams, or video/gif about the topic

## Reference

[1] https://en.wikipedia.org/wiki/Extended_Kalman_filter




