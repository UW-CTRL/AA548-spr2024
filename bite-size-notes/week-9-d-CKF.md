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
*ẋ*(*t*) = *A**x*(*t*) + *B**u*(*t*) + *w*(*t*)    *w*(*t*) ∼ 𝒩(0,*Q*)
*y*(*t*) = *C**x*(*t*) + *D**u*(*t*) + *v*(*t*)    *v*(*t*) ∼ 𝒩(0,*R*)
where, *Q* is Process noise covariance and *R* is measurement noise
covariance. We also assume that our measurement is not affected by the
control, so *D* = 0.

To derive the Kalman filter equations for these dynamics, we employ
discrete-time dynamics and approximate the equations accordingly. We
write the derivatives as finite difference terms as shown below,
$$\\dot{x} \\approx \\frac{x\_{t+1}-x\_{t}}{\\Delta t}$$
where *Δ**t* → 0.

Substituting this into continuous dynamics, we get,

$$\\begin{aligned}
    \\frac{x\_{t+1}-x_t}{\\Delta t} &= Ax\_{t} + Bu\_{t} + w\\\\
    x\_{t+1} &= (I+A \\Delta t)x\_{t} + B \\Delta t u\_{t} + \\Tilde{w} \\quad \\quad \\Tilde{w} \\sim N(0,Q\\Delta t)\\end{aligned}$$
Also,
$$y\_{t} = Cx\_{t} + \\Tilde{v} \\quad \\Tilde{v} \\sim N(0,\\frac{R}{\\Delta t})$$

The noise *w*(*t*) is integrated over *Δ**t* The resulting covariance of
*w* after integration is as follows:

*E*\[∫∫<sub>*t*</sub><sup>*t* + *Δ**t*</sup>*w*(*τ*)*w*(*τ*′)<sup>*T*</sup> *d**τ*′*d**τ*\]
 = *Q*(*t*)*Δ**t*
Similarly, integrating measurement noise over the time step,
$$\\Tilde{v}\_{t} = \\frac{1}{\\Delta t} \\int\_{t}^{t+\\Delta t}v(\\tau)d\\tau$$
$$E\[\\Tilde{V}\_{t} \\Tilde{V}\_{t}^T\]=\\frac{R(t)}{\\Delta t}=\\frac{R}{\\Delta t}\\ \\text{(as it is not time-varying)}$$
With this the continuous time dynamics are turned to discrete time
dynamics:
$$x\_{t+1}=(I+A\\Delta t)x\_{t} + B\\Delta t u\_{t} +\\Tilde{w}\_{t} \\quad \\quad \\quad \\Tilde{w} \\sim \\mathcal{N}(0, Q\\Delta t)$$
$$y\_{t} = Cx\_{t} + \\Tilde{v}\_{t} \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\quad \\Tilde{v} \\sim \\mathcal{N}(0, \\frac{R}{dt})$$

Just like for discrete systems, we have the predict equations given as:
*μ*<sub>*t*</sub><sup>*p*</sup> = (*I*+*A**Δ**t*)*μ*<sub>*t* − 1</sub> + *B**Δ**t**u*<sub>*t* − 1</sub>
*Σ*<sub>*t*</sub><sup>*p*</sup> = (*I*+*A**Δ**t*)*Σ*<sub>*t* − 1</sub>(*I*+*A**Δ**t*)<sup>*T*</sup> + *Q**Δ**t*

Update equations are as follows:
$$K\_{t}=\\Sigma\_{t}^p C^{T} (C \\Sigma\_{t}^{p} C^{T} + \\frac{1}{\\Delta t}R)^{-1}$$
*μ*<sub>*t*</sub> = *μ*<sub>*t*</sub><sup>*p*</sup> + *K*<sub>*t*</sub>(*y*<sub>*t*</sub>−*C**μ*<sub>*t*</sub><sup>*p*</sup>)
*Σ*<sub>*t*</sub> = (*I*−*K*<sub>*t*</sub>*C*)*Σ*<sub>*t*</sub><sup>*p*</sup>

Rearranging the gain equation we get,
$$\\frac{1}{\\Delta t}K\_{t} = \\Sigma\_{t}^{p} C^{T} (C \\Sigma\_{t}^{p} C^{T} \\Delta t +R)^{-1}$$

as *Δ**t* goes to zero, the term on right goes to a finite value of
*Σ*<sub>*t*</sub><sup>*p*</sup>*C*<sup>*T*</sup>*R*<sup>−1</sup>. The
only way, left hand side can be finite is when *K*<sub>*t*</sub> also
goes to zero.

This suggests that when continuous-time dynamics are approximated by
discrete-time dynamics using a small sampling time, the Kalman gain
approaches zero. Consequently, employing a small sampling time in
discrete-time dynamics may not be viable.

### Finite difference version of *Σ*

Substituting the update equations into the predict equations for *Σ*, we
get,
$$\\begin{aligned}
    \\Sigma\_{t}^{p}=(I+A\\Delta t)(I-K\_{t-1}C)\\Sigma\_{t-1}^{p}(I+A\\Delta t)^{T}+Q\\Delta t\\end{aligned}$$
Rearranging this terms will give

$$\\begin{aligned}
    \\frac{1}{\\Delta t}(\\Sigma_t^p-\\Sigma\_{t-1}^p) = -\\frac{k\_{t-1}C\\Sigma\_{t-1}^p}{\\Delta t} + (A\\Sigma\_{t-1}^p+AK\_{k-1}C\\Sigma\_{t-1}^p+\\Sigma\_{t-1}^pA^T-K\_{t-1}C\\Sigma\_{t-1}^pA^T+Q) + O(\\Delta t^2)\\end{aligned}$$

With limit, *Δ**t* → 0

We get the finite difference version,
$$\\begin{aligned}
     \\underbrace{\\dot{\\Sigma}(t)=A\\Sigma(t)+\\Sigma(t)A^{T}-\\Sigma(t)C^{T}R^{-1}C\\Sigma(t)+Q}\_{\\text{Riccati ODE}}
 \\end{aligned}$$

### Duality in the Riccati Equation

CKF: *Σ̇* = *A**Σ* + *Σ**A*<sup>*T*</sup> − *Σ**C*<sup>*T*</sup>*R*<sup>−1</sup>*C**Σ* + *Q*
LQR:  − *Ṗ* = *A*<sup>*T*</sup>*P* + *P**A* − *P**B**R*<sup>−1</sup>*B*<sup>*T*</sup>*P* + *Q*

|                   **LQR**                   |              **CKF**              |
|:-------------------------------------------:|:---------------------------------:|
|                     *A*                     |         *A*<sup>*T*</sup>         |
|                     *B*                     |         *C*<sup>*T*</sup>         |
|             *Q*<sub>cost</sub>              |       *Q* = *c**o**v*(*w*)        |
|             *R*<sub>cost</sub>              |       *R* = *c**o**v*(*v*)        |
|               *t* (backward)                | *t*<sub>*f*</sub> = *t* (forward) |
| *P* from *V*(*x*) = *x*<sup>*T*</sup>*P**x* |      *Σ*  Covariance of Est.      |

Differences between LQR and KF.

### Finite Difference version of *μ*

We can also carry out similar operations to arrive a finite difference
equation for *μ*,
*μ*<sub>*t*</sub> = *μ*<sub>*t*</sub><sup>*p*</sup> + *K*<sub>*t*</sub>(*y*<sub>*t*</sub>−*C**μ*<sub>*t*</sub><sup>*p*</sup>)
Plugging this into predict equation will give,
*μ*<sub>*t*</sub> = (*I*+*A**Δ**t*)*μ*<sub>*t* − 1</sub> + *K*<sub>*t*</sub>(*y*<sub>*t*</sub>−*C*(*I*+*A**Δ**t*)*μ*<sub>*t* − 1</sub>+*B**Δ**t**u*<sub>*t* − 1</sub>))
Rearranging the terms gives
$$\\frac{1}{\\Delta t}(\\mu\_{t}-\\mu\_{t-1})=A\\mu\_{t}+Bu\_{t}+\\frac{K\_{t}}{\\Delta t}(y\_{t}-C\\mu\_{t}-C(A\\mu\_{t}+Bu\_{t})\\Delta t)$$
With limit, *Δ**t* → 0

We get the finite difference version,

*μ̇* = *A**μ* + *B**u* + *Σ**C*<sup>*T*</sup>*R*<sup>−1</sup>(*y*−*C**μ*)
Indicating how mean changes over time.

### Continuous Time Kalman Filter Summary

Basic equations are summarized as,

-   *Σ̇*(*t*) = *A**Σ*(*t*) + *Σ*(*t*)*A*<sup>*T*</sup> − *Σ*(*t*)*C*<sup>*T*</sup>*R*<sup>−1</sup>*C**Σ*(*t*) + *Q*

-   *μ̇* = *A**μ* + *B**u* + *P**C*<sup>*T*</sup>*R*<sup>−1</sup>(*y*−*C**μ*)
