<h1>Extended Kalman Filter</h1>

<h3>Objectives</h3>

The objectives of these notes is to highlight the important differences when applying an extended kalman filter as compared to the standard linear kalman filter.

<h3>Introduction</h3>

The Extended Kalman Filter (EKF) is a powerful extension of the Kalman Filter designed to handle nonlinear systems. While the standard Kalman Filter excels in linear scenarios, many real-world systems exhibit nonlinear behavior that the standard filter cannot address adequately. The EKF overcomes this limitation by linearizing the nonlinear system around the current estimate using a first-order Taylor series expansion at each time step.

<h3>Preliminaries: Notation and problem setup</h3>

We will assume Nonlinear Discrete Dynamics with zero mean gaussian process and measurement noise

$$\begin{align*}
x_{k+1}&=f(x_k,u_k,w_k) &&\qquad w_k\sim N(0,Q)\\
y_{k+1}&=g(x_k,v_k) &&\qquad v_k\sim N(0,R)
\end{align*}$$

where f() describes how the dynamics propagate forward in time and g() describes how the states map to measurements of the system.

At each step we will linearize and use the standard kalman filter equations. To linearize we will take the jacobian of both f() and g() w.r.t. the state x, and noise w/v.

$F_{k}^{x}=\frac{\partial f}{\partial x}|_{x_k,u_k,w_k}$
$F_{k}^{w}=\frac{\partial f}{\partial w}|_{x_k,u_k,w_k}$
$G_{k}^{x}=\frac{\partial g}{\partial x}|_{x_k,u_k,v_k}$
$G_{k}^{v}=\frac{\partial g}{\partial v}|_{x_k,u_k,v_k}$

Here subscript denotes the timestep and the superscript denotes what the jacobian is being taken with respect to. Additionally if T is in the superscript like $G_{k}^{xT}$, then that means it is the transpose of $G_{k}^{x}$

For generalization it is assumed that the noise is not additive. If the noise is additive then the partials with respect to w or v will be identity matrices.

<h3>General Extended Kalman Filter Algorithm</h3>

Predict:

$$
\begin{align*}

\mu_{k+1}^p&=f(\mu_k,u_k)\\

\Sigma_{k+1}^p&=F_k^x\Sigma_k F_k^{xT}+F_k^wQF_k^{wT}\\

y_{k+1}^p&=g(\mu_{k+1}^p)
\end{align*}
$$

Update:

$$\begin{align*}

K_{k+1}&=\Sigma_{k+1}^pG_{k+1}^{xT}(G_{k+1}^{x}\Sigma_{k+1}^p G_{k+1}^{xT}+G_{k+1}^{v}RG_{k+1}^{vT})^{-1}\\

\mu_{k+1}&=\mu_{k+1}^p+K_{k+1}(y_{k+1}-y_{k+1}^p)\\

\Sigma_{k+1}&=\Sigma_{k+1}^p-K_{k+1}(G_{k+1}^x\Sigma_{k+1}^pG_{k+1}^{xT}+G_{k+1}^vRG_{k+1}^{vT})K_{k+1}^T

\end{align*}$$

Notice that we use the full nonlinear system in the mean prediction step but we have to use the linearized system when looking at how the 

Simple example: Problem Setup for Nonlinear Pendulum

The nonlinear dynamics of the pendulum can be written as
$$
\begin{align*}

\frac{d}{dt}
\begin{bmatrix}  
    x \\ \dot{x}
\end{bmatrix}
=\begin{bmatrix}
\dot{x} \\ -\sin(x)
\end{bmatrix}
+
\begin{bmatrix}
0 \\ u
\end{bmatrix}
+
\begin{bmatrix}
0 \\ w
\end{bmatrix}
\end{align*}
$$
Which can be discretized using a zero-order hold to
$$
\begin{bmatrix}  
    x_{k+1} \\ \dot{x}_{k+1}
\end{bmatrix}
=\begin{bmatrix}  
    x_k+\dot{x}_k \Delta t \\ \dot{x}_k-\sin(x_k)\Delta t
\end{bmatrix}
+\begin{bmatrix}  
    0 \\ u\Delta t
\end{bmatrix}
+
\begin{bmatrix}
0 \\ w\Delta t
\end{bmatrix}
\\
y_{k+1}=x_{k+1} + v_{k+1}
$$

Notice that we are only measuring the angle of the pendulum (x)

we can calculate our jacobians as

$$\begin{align*}
F_k^{x}&=\frac{\partial f}{\partial x} = \begin{bmatrix} 1 & \Delta t \\  -cos(x_k)\Delta t & 1 \end{bmatrix} \\
F^{w}&=\frac{\partial f}{\partial w} = I\\
G^{x}&=\frac{\partial g}{\partial x} = \begin{bmatrix} 1 \\ 0 \end{bmatrix}\\
G^{v}&=\frac{\partial g}{\partial v} = I
\end{align*}$$

In this case we can see that our dynamics f(x) are nonlinear but our measurement function g(x) is nonlinear. What has resulted is that $G^{x}$ is constant whereas $F_k^x$ will change depending on the state $x_k$, hence it has the subscript k. Also, the noise was assumed to be additive and therefore the jacobians w.r.t. the noise were Identity matrices.

<h3>On optimality and stability:</h3>

One important consequence of linearizing is that this is no longer a truely optimal filter since we are no longer using the actual dynamics but an approximation during the covariance prediction/update steps. This is very important to take into account because the extended kalman filter cannot be used if the nonlinear dynamics is highly nonlinear, such that a local linearization would be invalid and give erroneous estimates.

<h3>Conclusion</h3>

The extended kalman filter is a powerful extension of the linear kalman filter. It performs this extension by approximating the change in the covariance by using a linear approximation of the dynamics and measurement functions f() and g() at each time step using the current estimated state. The linearization can be 

<h3>Refences</h3>

[1] Extended Kalman Filters
- MATLAB & Simulink. (n.d.). https://www.mathworks.com/help/fusion/ug/extended-kalman-filters.html


