# Unscented Kalman Filter (UKF)

# Scope and Objectives

These notes cover the Unscented Kalman Filter (UKF), a sophisticated algorithm used for state estimation in nonlinear systems. The UKF is widely used in control systems, robotics, aerospace, and many other fields requiring precise state estimation. By the end of these notes, you will understand the purpose, principles and calculation procedure of the UKF.

# Introduction

The Unscented Kalman Filter (UKF) is a popular state estimation technique for nonlinear systems. Unlike the Extended Kalman Filter (EKF), which linearizes the process and measurement models, the UKF uses a deterministic sampling technique called the Unscented Transform (UT) to achieve better estimation accuracy.

## Relevance to Controls

Control systems frequently need to estimate the internal state of a system based on noisy measurements. For nonlinear systems, the UKF provides a robust and accurate estimation method, outperforming the EKF in many scenarios.

## Historical Context

The UKF was developed in the late 1990s by Simon J. Julier and Jeffrey K. Uhlmann. They introduced the Unscented Transform to overcome the inaccuracies of the linearization approach used in the EKF.

# Preliminaries

## Definitions and Notation

- **State Vector (x)**: The vector representing the internal state of the system.
- **Measurement Vector (z)**: The vector representing the observed measurements.
- **Process Model (f)**: Describes the evolution of the state over time.
- **Measurement Model (h)**: Describes how the measurements relate to the state.
- **Process Noise (w)**: The noise associated with the process model.
- **Measurement Noise (v)**: The noise associated with the measurement model.
- **Sigma Points**: A set of points used in the UKF to approximate the state distribution.

## Mathematical Background

The UKF relies on key mathematical concepts:
- **Mean and Covariance Propagation**: Instead of linearizing the process, the UKF propagates a set of sigma points through the nonlinear functions to capture the mean and covariance more accurately.
- **Unscented Transform (UT)**: A method to compute the mean and covariance of a random variable undergoing a nonlinear transformation.

# Unscented Kalman Filter Equations and Coding Example

## The Unscented Transform (UT)

The Unscented Transform is central to the UKF. It involves three main steps:
1. **Sigma Point Selection**: Generate a set of sigma points that represent the state distribution.
2. **Propagation**: Pass these sigma points through the nonlinear function.
3. **Reconstruction**: Compute the mean and covariance of the transformed points.

### Sigma Point Generation

For a state vector x with covariance P:

χ₀ = x

χᵢ = x + (√((L+λ)P))ᵢ

χᵢ₊ₗ = x - (√((L+λ)P))ᵢ

where L is the dimension of x and λ is a scaling parameter calculated as λ = α²(L + κ) - L.

- α: Determines the spread of the sigma points. A small α leads to sigma points closer to the mean.
- κ: A secondary scaling parameter, often set to 0 or 3 - L.
- β: Incorporates prior knowledge of the distribution of x (for Gaussian distributions, β = 2).

### Propagation Through Nonlinear Function

For a nonlinear function f:

χᵢ' = f(χᵢ)

Each sigma point is passed through the nonlinear function to obtain a new set of transformed sigma points χᵢ'.

### Mean and Covariance Reconstruction

Calculate the mean and covariance of the transformed points:

x' = Σᵢ₌₀²ᴸ Wᵢ χᵢ'

P' = Σᵢ₌₀²ᴸ Wᵢ (χᵢ' - x')(χᵢ' - x')ᵀ + Q

where Wᵢ are the weights associated with each sigma point and Q is the process noise covariance.

## The UKF Algorithm

The UKF algorithm can be summarized in the following steps:
1. **Initialization**: Set the initial state estimate x₀ and covariance P₀.
2. **Prediction Step**:
   - Generate sigma points from the current state estimate.
   - Propagate sigma points through the process model.
   - Compute the predicted state mean and covariance.
3. **Update Step**:
   - Generate sigma points from the predicted state estimate.
   - Propagate sigma points through the measurement model.
   - Compute the predicted measurement mean and covariance.
   - Compute the cross-covariance and Kalman gain.
   - Update the state estimate and covariance using the new measurement.

### Prediction Step

Generate sigma points from the current state estimate:

χₖ₋₁ = [xₖ₋₁, xₖ₋₁ ± √((L+λ)Pₖ₋₁)]

Propagate sigma points through the process model:

χₖ⁻ = f(χₖ₋₁)

Compute the predicted state mean:

xₖ⁻ = Σᵢ₌₀²ᴸ Wᵢ χₖ⁻

Compute the predicted state covariance:

Pₖ⁻ = Σᵢ₌₀²ᴸ Wᵢ (χₖ⁻ - xₖ⁻)(χₖ⁻ - xₖ⁻)ᵀ + Q

### Update Step

Generate sigma points from the predicted state estimate:

χₖ = [xₖ⁻, xₖ⁻ ± √((L+λ)Pₖ⁻)]

Propagate sigma points through the measurement model:

ζₖ = h(χₖ)

Compute the predicted measurement mean:

zₖ = Σᵢ₌₀²ᴸ Wᵢ ζₖ

Compute the predicted measurement covariance:

Pzz = Σᵢ₌₀²ᴸ Wᵢ (ζₖ - zₖ)(ζₖ - zₖ)ᵀ + R

Compute the cross-covariance between the state and measurement:

Pxz = Σᵢ₌₀²ᴸ Wᵢ (χₖ - xₖ⁻)(ζₖ - zₖ)ᵀ

Compute the Kalman gain:

Kₖ = Pxz Pzz⁻¹

Update the state estimate:

xₖ = xₖ⁻ + Kₖ (zₖ - zₖ)

Update the state covariance:

Pₖ = Pₖ⁻ - Kₖ Pzz Kₖᵀ

## Example: Estimating the Position of a Moving Object

Consider a simple nonlinear system where we estimate the position and velocity of an object moving in one dimension.

### System Model

The state vector is defined as:
xₖ = [pₖ, vₖ]ᵀ

The process model is:
f(xₖ₋₁) = [pₖ₋₁ + vₖ₋₁ Δt, vₖ₋₁]ᵀ
where pₖ is the position, vₖ is the velocity, and Δt is the time step.

The measurement model is:
h(xₖ) = pₖ

### Python Code Implementation

```python
import numpy as np

def generate_sigma_points(x, P, alpha, beta, kappa):
    L = len(x)
    lambda_ = alpha**2 * (L + kappa) - L
    sigma_points = np.zeros((2*L + 1, L))
    sigma_points[0] = x
    U = np.linalg.cholesky((L + lambda_) * P)
    for i in range(L):
        sigma_points[i+1] = x + U[i]
        sigma_points[i+L+1] = x - U[i]
    return sigma_points, lambda_

def unscented_transform(sigma_points, Wm, Wc, noise_cov):
    mean = np.dot(Wm, sigma_points)
    cov = np.zeros((len(mean), len(mean)))
    for i, sigma_point in enumerate(sigma_points):
        diff = sigma_point - mean
        cov += Wc[i] * np.outer(diff, diff)
    cov += noise_cov
    return mean, cov

# Define constants and parameters
alpha = 1e-3
beta = 2
kappa = 0
L = 2  # Dimension of state
Q = np.diag([0.1, 0.1])  # Process noise covariance
R = np.array([[1.0]])  # Measurement noise covariance
x = np.array([0, 1])  # Initial state
P = np.diag([1, 1])  # Initial covariance

# Generate sigma points
sigma_points, lambda_ = generate_sigma_points(x, P, alpha, beta, kappa)
Wm = np.full(2*L+1, 0.5 / (L + lambda_))
Wc = np.full(2*L+1, 0.5 / (L + lambda_))
Wm[0] = lambda_ / (L + lambda_)
Wc[0] = Wm[0] + (1 - alpha**2 + beta)

def predict(sigma_points, Wm, Wc, Q, f):
    sigma_points_pred = np.array([f(sp) for sp in sigma_points])
    x_pred, P_pred = unscented_transform(sigma_points_pred, Wm, Wc, Q)
    return x_pred, P_pred, sigma_points_pred

def process_model(x):
    dt = 1  # Time step
    return np.array([x[0] + x[1] * dt, x[1]])

x_pred, P_pred, sigma_points_pred = predict(sigma_points, Wm, Wc, Q, process_model)

def update(x_pred, P_pred, sigma_points_pred, z, R, h):
    sigma_points_meas = np.array([h(sp) for sp in sigma_points_pred])
    z_pred, P_zz = unscented_transform(sigma_points_meas, Wm, Wc, R)
    P_xz = np.zeros((L, len(z)))
    for i in range(2*L + 1):
        P_xz += Wc[i] * np.outer(sigma_points_pred[i] - x_pred, sigma_points_meas[i] - z_pred)
    K = np.dot(P_xz, np.linalg.inv(P_zz))
    x_upd = x_pred + np.dot(K, (z - z_pred))
    P_upd = P_pred - np.dot(K, np.dot(P_zz, K.T))
    return x_upd, P_upd

def measurement_model(x):
    return np.array([x[0]])

z = np.array([2.0])  # Example measurement
x_upd, P_upd = update(x_pred, P_pred, sigma_points_pred, z, R, measurement_model)

print("Updated state estimate:\n", x_upd)
print("Updated covariance:\n", P_upd)
```

### Explanation of the Code

1. **Sigma Point Generation**:
   - The `generate_sigma_points` function generates sigma points based on the current state estimate and covariance matrix. It uses the Cholesky decomposition to ensure numerical stability.
2. **Unscented Transform**:
   - The `unscented_transform` function computes the mean and covariance of the propagated sigma points. This function is used both in the prediction and update steps.
3. **Prediction Step**:
   - The `predict` function propagates the sigma points through the process model (`process_model`). It then uses the `unscented_transform` function to compute the predicted state mean and covariance.
4. **Update Step**:
   - The `update` function propagates the sigma points through the measurement model (`measurement_model`). It calculates the predicted measurement mean and covariance, the cross-covariance, and the Kalman gain. Finally, it updates the state estimate and covariance based on the measurement.

# Conclusion

The Unscented Kalman Filter (UKF) is a powerful tool for state estimation in nonlinear systems. By leveraging the Unscented Transform, the UKF provides more accurate estimates than the Extended Kalman Filter (EKF) without the need for linearization. The key steps in the UKF involve generating sigma points, propagating them through the process and measurement models, and updating the state estimate using the Kalman gain.

## References

1. Julier, S. J., & Uhlmann, J. K. (1997). "A new extension of the Kalman filter to nonlinear systems." Proceedings of SPIE, 3068, 182-193.
2. Wan, E. A., & Van Der Merwe, R. (2000). "The Unscented Kalman Filter for Nonlinear Estimation." Proceedings of the IEEE 2000 Adaptive Systems for Signal Processing, Communications, and Control Symposium (AS-SPCC).
3. Simon, D. (2006). "Optimal State Estimation: Kalman, H Infinity, and Nonlinear Approaches." Wiley-Interscience.
