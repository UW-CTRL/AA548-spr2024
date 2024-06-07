# Introduction

## Scope + Objectives

These notes aim to provide a concise yet thorough introduction to
continuous Kalman Filters (CKFs). Readers will learn the fundamentals of
CKFs, their applications in control systems, and how they differ from
discrete Kalman Filters. Key questions addressed include:
-   What is a continuous Kalman Filter?
-   How is it formulated mathematically?


## Introduction

Continuous Kalman Filters are essential in control theory and signal
processing for estimating the state of a continuous-time dynamic system.
They are particularly useful in applications requiring real-time state
estimation, such as navigation systems, robotics, and autonomous
vehicles. The continuous Kalman Filter extends the discrete Kalman
Filter to continuous-time processes, offering more accurate and smooth
state estimates for systems modeled by differential equations.

## Continuous-Time Kalman Filter

We start with the continuous time dynamical system with Gaussian noise:

$$
\begin{equation}\tag{1}
    \dot{x}(t) = Ax(t) + Bu(t) + w(t)  \quad\quad\quad\quad w(t) \sim \mathcal{N}(0,Q)
\end{equation}
$$

$$
\begin{equation}\tag{2}
    y(t) = Cx(t) + Du(t) + v(t) \quad\quad\quad\quad v(t) \sim \mathcal{N}(0,R)
\end{equation}
$$

where, $Q$ is Process noise covariance and *R* is measurement noise covariance. We also assume that our measurement is not affected by the control, so $D = 0$.


To derive the Kalman filter equations for these dynamics, we employ
discrete-time dynamics and approximate the equations accordingly. We
write the derivatives as finite difference terms as shown below,
$$\\dot{x} \\approx \\frac{x\_{t+1}-x\_{t}}{\\Delta t}$$
where $Δt → 0$.

Substituting this into continuous dynamics, we get,

We start with the continuous time dynamical system with Gaussian noise:

$$
\begin{equation}\tag{3}
\frac{x_{t+1}-x_t}{\Delta t} = Ax_{t} + Bu_{t} + w
\end{equation}
$$

$$
\begin{equation}\tag{4}
x_{t+1} = (I+A \Delta t)x_{t} + B \Delta t u_{t} + \tilde{w}  \quad \quad \tilde{w} \sim N(0,Q\Delta t)
\end{equation}
$$

Also,

$$
\begin{equation}\tag{5}
y_{t} = Cx_{t} + \tilde{v} \quad \tilde{v} \sim N(0,\frac{R}{\Delta t})
\end{equation}
$$


The noise $w(t)$ is integrated over $Δt$. The resulting covariance of
$w$ after integration is as follows:

$$
\begin{equation}\tag{6}
E\left[\int_{t}^{t+\Delta t} \int_{t}^{t+\Delta t} w(\tau)w(\tau ')^T \, d\tau ' d\tau\right] = Q(t) \Delta t
\end{equation}
$$

Similarly, integrating measurement noise over the time step,

$$
\begin{equation}\tag{25}
a\tilde{v}_{t} = \frac{1}{\Delta t} \int_{t}^{t+\Delta t} v(\tau) \, d\tau
\end{equation}
$$


$$
\begin{equation}\tag{8}
E\left[\tilde{v}_{t} \tilde{v}_{t}^T\right] = \frac{R(t)}{\Delta t} = \frac{R}{\Delta t} \quad \text{(as it is not time-varying)}
\end{equation}
$$


With this, the continuous time dynamics are turned to discrete time dynamics:

$$
\begin{equation}\tag{9}
x_{t+1} = (I+A\Delta t)x_{t} + B\Delta t u_{t} + \tilde{w}_{t} \quad \quad \tilde{w} \sim \mathcal{N}(0, Q\Delta t)
\end{equation}
$$

$$
\begin{equation}\tag{10}
y_{t} = Cx_{t} + \tilde{v}_{t} \quad \quad \tilde{v} \sim \mathcal{N}(0, \frac{R}{\Delta t})
\end{equation}
$$

Just like for discrete systems, we have the predict equations given as:

$$
\begin{equation}\tag{11}
\mu_{t}^{p} = (I+A\Delta t)\mu_{t-1} + B\Delta t u_{t-1}
\end{equation}
$$

$$
\begin{equation}\tag{12}
\Sigma_{t}^{p} = (I+A\Delta t)\Sigma_{t-1}(I+A\Delta t)^{T} + Q\Delta t
\end{equation}
$$

Update equations are as follows:

$$
\begin{equation}\tag{13}
K_{t} = \Sigma_{t}^{p} C^{T} (C \Sigma_{t}^{p} C^{T} + \frac{1}{\Delta t} R)^{-1}
\end{equation}
$$

$$
\begin{equation}\tag{14}
\mu_{t} = \mu_{t}^{p} + K_{t}(y_{t} - C\mu_{t}^{p})
\end{equation}
$$

$$
\begin{equation}\tag{15}
\Sigma_{t} = (I-K_{t}C)\Sigma_{t}^{p}
\end{equation}
$$

Rearranging the gain equation we get,

$$
\begin{equation}\tag{16}
\frac{1}{\Delta t} K_{t} = \Sigma_{t}^{p} C^{T} (C \Sigma_{t}^{p} C^{T} \Delta t + R)^{-1}
\end{equation}
$$

As $\Delta t$ goes to zero, the term on the right goes to a finite value of $\Sigma_{t}^{p} C^{T} R^{-1}$. The only way the left-hand side can be finite is when $K_t$ also goes to zero.
This suggests that when continuous-time dynamics are approximated by
discrete-time dynamics using a small sampling time, the Kalman gain
approaches zero. Consequently, employing a small sampling time in
discrete-time dynamics may not be viable.

### Finite difference version of $Σ$

Substituting the update equations into the predict equations for $\Sigma$, we get,

$$
\begin{equation}\tag{17}
\Sigma_{t}^{p} = (I+A\Delta t)(I-K_{t-1}C)\Sigma_{t-1}^{p}(I+A\Delta t)^{T} + Q\Delta t
\end{equation}
$$

Rearranging these terms will give

$$
\begin{equation}\tag{18}
\frac{1}{\Delta t} (\Sigma_{t}^{p} - \Sigma_{t-1}^{p}) = -\frac{K_{t-1}C\Sigma_{t-1}^{p}}{\Delta t} + (A\Sigma_{t-1}^{p} + AK_{t-1}C\Sigma_{t-1}^{p} + \Sigma_{t-1}^{p}A^{T} - K_{t-1}C\Sigma_{t-1}^{p}A^{T} + Q) + O(\Delta t^{2})
\end{equation}
$$

With the limit $\Delta t \to 0$

We get the finite difference version,

$$
\begin{equation}\tag{19}
\underbrace{\dot{\Sigma}(t) = A\Sigma(t) + \Sigma(t)A^{T} - \Sigma(t)C^{T}R^{-1}C\Sigma(t) + Q}_{\text{Riccati ODE}}
\end{equation}
$$


### Duality in the Riccati Equation
The Linear Quadratic Regulator (LQR) and the continuous Kalman Filter (KF) are similar in that both are based on the solution of Riccati equations, which are central to their respective optimization problems. In LQR, the Riccati equation determines the optimal feedback gain to minimize a quadratic cost function, balancing state error and control effort. In the continuous Kalman Filter, a similar Riccati equation governs the update of the error covariance matrix to optimally estimate the system state from noisy measurements. Both methods use state-space models and quadratic forms, and their dual nature means techniques and insights from one domain often apply to the other, reflecting their mathematical and conceptual interdependence.

CKF: 
$$\dot{\Sigma}=A\Sigma+\Sigma A^{T}-\Sigma C^{T}R^{-1}C\Sigma+Q$$
LQR:  
$$-\dot{P}=A^{T} P+P A-P B R^{-1} B^{T} P+Q$$

|                   **LQR**                   |              **CKF**              |
|:-------------------------------------------:|:---------------------------------:|
|                     $A$                     |             $A^{T}$               |
|                     $B$                     |             $C^{T}$               |
|             $Q_{\text{cost}}$               |           $Q=cov(w)$              |
|             $R_{\text{cost}}$              |             $R=cov(v)$             |
|               $t$ (backward)                | $t_{f}=t$\ (forward)              |
| $P$ from $V(x)=x^{T}Px$ |      $Σ$  Covariance of Est.      |

Differences between LQR and KF.

Combining predict and update equation for mean gives,

$$
\begin{equation}\tag{20}
\dot{\mu}(t) = A\mu(t) + Bu(t) + \Sigma(t) C^{T}R^{-1}(y(t) - C\mu(t))
\end{equation}
$$

### Continuous Time Kalman Filter Summary
Key takeaways include:
- Continuous Kalman Filters provide real-time state estimation for continuous-time systems.
- They extend the principles of discrete Kalman Filters to differential equation models.

Basic equations are summarized as,
-    $\dot{\Sigma}(t)=A\Sigma(t)+\Sigma(t)A^{T}-\Sigma(t)C^{T}R^{-1}C\Sigma(t)+Q$
-   $\dot{\mu}(t) = A\mu(t)+Bu(t)+\Sigma(t) C^{T}R^{-1}(y(t)-C\mu(t))$

## References

Lewis, F.L., Xie, L., & Popa, D. (2008). Optimal and Robust Estimation: With an Introduction to Stochastic Control Theory, Second Edition (2nd ed.). CRC Press. [DOI](https://doi.org/10.1201/9781315221656)

