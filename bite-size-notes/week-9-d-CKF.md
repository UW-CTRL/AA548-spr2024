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
$$\dot{x}(t) = Ax(t) + Bu(t) + w(t)  \quad\quad\quad\quad w(t) \sim \mathcal{N}(0,Q)$$ 
$$y(t) = Cx(t) + Du(t) + v(t) \quad\quad\quad\quad v(t) \sim \mathcal{N}(0,R)$$
where, $Q$ is Process noise covariance and *R* is measurement noise
covariance. We also assume that our measurement is not affected by the
control, so $D = 0$.

To derive the Kalman filter equations for these dynamics, we employ
discrete-time dynamics and approximate the equations accordingly. We
write the derivatives as finite difference terms as shown below,
$$\\dot{x} \\approx \\frac{x\_{t+1}-x\_{t}}{\\Delta t}$$
where $Δt → 0$.

Substituting this into continuous dynamics, we get,

$$\\frac{x\_{t+1}-x_t}{\\Delta t} = Ax\_{t} + Bu\_{t} + w$$
$$x\_{t+1} = (I+A \\Delta t)x\_{t} + B \\Delta t u\_{t} + \tilde{w}  \\quad \\quad \\tilde{w} \\sim N(0,Q\\Delta t)$$
Also,
$$y\_{t} = Cx\_{t} + \\tilde{v} \\quad \\tilde{v} \\sim N(0,\\frac{R}{\\Delta t})$$

The noise $w(t)$ is integrated over $Δt$ The resulting covariance of
$w$ after integration is as follows:

$$E[\int \int_{t}^{t+\Delta t} w(\tau)w(\tau ')^T \; d\tau ' d\tau]$$
$$ = Q(t) \Delta t$$

Similarly, integrating measurement noise over the time step,
$$\\tilde{v}\_{t} = \\frac{1}{\\Delta t} \\int\_{t}^{t+\\Delta t}v(\\tau)d\\tau$$
$$E\[\\tilde{V}\_{t} \\tilde{V}\_{t}^T\]=\\frac{R(t)}{\\Delta t}=\\frac{R}{\\Delta t}\\ \\text{(as it is not time-varying)}$$
With this the continuous time dynamics are turned to discrete time
dynamics:
$$x\_{t+1}=(I+A\\Delta t)x\_{t} + B\\Delta t u\_{t} +\\tilde{w}\_{t} \\quad \\quad \\quad \\tilde{w} \\sim \\mathcal{N}(0, Q\\Delta t)$$
$$y\_{t} = Cx\_{t} + \\tilde{v}\_{t} \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\tilde{v} \\sim \\mathcal{N}(0, \\frac{R}{dt})$$

Just like for discrete systems, we have the predict equations given as:
$$\mu_{t}^{p}=(I+A\Delta t)\mu_{t-1} + B\Delta t u_{t-1}$$
$$\Sigma_{t}^{p}=(I+A\Delta t)\Sigma_{t-1}(I+A\Delta t)^{T}+Q\Delta t$$

Update equations are as follows:
$$K\_{t}=\\Sigma\_{t}^p C^{T} (C \\Sigma\_{t}^{p} C^{T} + \\frac{1}{\\Delta t}R)^{-1}$$
$$\mu_{t}=\mu_{t}^{p}+K_{t}(y_{t}-C\mu_{t}^{p})$$
$$\Sigma_{t}=(I-K_{t}C)\Sigma_{t}^{p}$$

Rearranging the gain equation we get,
$$\\frac{1}{\\Delta t}K\_{t} = \\Sigma\_{t}^{p} C^{T} (C \\Sigma\_{t}^{p} C^{T} \\Delta t +R)^{-1}$$

as $\Delta t$ goes to zero, the term on right goes to a finite value of $\Sigma_{t}^{p} C^{T} R^{-1}$. The only way, left hand side can be finite is when $K_t$ also goes to zero.
This suggests that when continuous-time dynamics are approximated by
discrete-time dynamics using a small sampling time, the Kalman gain
approaches zero. Consequently, employing a small sampling time in
discrete-time dynamics may not be viable.

### Finite difference version of $Σ$

Substituting the update equations into the predict equations for *Σ*, we
get,
$$\\Sigma\_{t}^{p}=(I+A\\Delta t)(I-K\_{t-1}C)\\Sigma\_{t-1}^{p}(I+A\\Delta t)^{T}+Q\\Delta t$$
Rearranging this terms will give

$$\\frac{1}{\\Delta t}(\\Sigma_t^p-\\Sigma\_{t-1}^p) = -\\frac{k\_{t-1}C\\Sigma\_{t-1}^p}{\\Delta t} + (A\\Sigma\_{t-1}^p+AK\_{k-1}C\\Sigma\_{t-1}^p+\\Sigma\_{t-1}^pA^T-K\_{t-1}C\\Sigma\_{t-1}^pA^T+Q) + O(\\Delta t^2)$$

With limit, $Δt → 0$

We get the finite difference version,
$$\\underbrace{\\dot{\\Sigma}(t)=A\\Sigma(t)+\\Sigma(t)A^{T}-\\Sigma(t)C^{T}R^{-1}C\\Sigma(t)+Q}\_{\\text{Riccati ODE}}$$

### Duality in the Riccati Equation

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

### Finite Difference version of *μ*

We can also carry out similar operations to arrive a finite difference equation for $\mu$,
$$\mu_{t} = \mu_{t}^{p} + K_{t}(y_{t}-C\mu_{t}^{p})$$
Plugging this into predict equation will give,
$$\mu_{t}=(I+A\Delta t)\mu_{t-1}+K_{t}(y_{t}-C(I+A\Delta t)\mu_{t-1} + B\Delta tu_{t-1}))$$
Rearranging the terms gives
$$\\frac{1}{\\Delta t}(\\mu\_{t}-\\mu\_{t-1})=A\\mu\_{t}+Bu\_{t}+\\frac{K\_{t}}{\\Delta t}(y\_{t}-C\\mu\_{t}-C(A\\mu\_{t}+Bu\_{t})\\Delta t)$$
With limit, $Δt → 0$

We get the finite difference version,

$$\dot{\mu} = A\mu+Bu+\Sigma C^{T}R^{-1}(y-C\mu)$$
Indicating how mean changes over time.

### Continuous Time Kalman Filter Summary

Basic equations are summarized as,
-    $\dot{\Sigma}(t)=A\Sigma(t)+\Sigma(t)A^{T}-\Sigma(t)C^{T}R^{-1}C\Sigma(t)+Q$
-   $\dot{\mu}(t) = A\mu(t)+Bu(t)+\Sigma(t) C^{T}R^{-1}(y(t)-C\mu(t))$

