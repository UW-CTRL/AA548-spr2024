# Deterministic State Estimation

## Scope + Objectives

These notes aim to provide readers with a thorough understanding and review of state estimation, the Luenberger Observer, and duality. The overall goals of these notes are to provide all background information necessary in the deterministic case to better understand the setup for stochastic state estimators such as the Kalman Filter.

## Introduction

So far in the course, we have considered states of a given system as given quantity in each iteration. However, in real systems, usually only the ouput $y(t)$ and input $u(t)$ are known quantities such that the actual state $x(t)$ needs to be computed. State estimators, also called observers, perform this computation to get an estimate of the state. 

Why don't we know the actual state of the system? How can a determinisitic system still have errors? Usually, the first limiting factor when measuring the state is stochastic noise introduced into the measurement device which the Kalman filter attempts to statistically account for. However, in a determinisic system, there are still plenty of errors that arise to necessitate the need for a state estimator. Model inaccuracies, sensor limitations, parameter variations, and other non-linearities are all sources of error for deterministic state estimators.

## Preliminaries

First, we'll begin with a few definitions of relevant concepts used throughout the rest of the notes.

### Relevant Definitions:

- **Definition 1:**

    A system is considered observable if, for any given time $T > 0$, it is possible to determine the system's state at that time $x(T) \in \mathbb{R}^n$ through measuring control input $u(t)$ and output $y(t)$ where $t \in [0,T]$

    A pair of dynamics $(A,C)$ is observable if the observability matrix $O$ has $rank(O) = n = dim(A)$. The observability matrix is as follows:


$$ \ O =    
\begin{bmatrix}
C \\ CA \\ \vdots \\ CA^{n-1}
\end{bmatrix}\ 
$$

- **Definition 2:**

    The pair $(A, C)$ is detectable if all the unobservable modes are stable.

- **Definition 3:**

    A linear system is reachable if for any $x_o, x_f \in \mathbb{R}^n$, there exists a $T>0$ and $u(t): [0, T] \rightarrow \mathbb{R}$ such that the corresponding solution satisfies $x(0) = x_o$ and $x(T) = x_f \in \mathbb{R}^n$

- **Definition 4:**

    A system (A,B) is reachable if and only if $rank(R) = n$ where $x \in \mathbb{R}^n$ and

$$
R = \begin{pmatrix}
B & AB & \dots & A^{n-1}B
\end{pmatrix}
$$

### Problem Definition:

Given a linear, time invariant system ignoring noise, we have state and measurement output

$$ 
\dot{x}(t) = Ax(t) + Bu(t) \\
y(t) = Cx(t) + Du(t)
$$

From this, we can estimate the state $\hat{x}(t)$ using measurements using control inputs and measured outputs. To do this we want error $\tilde{x}(t) = x(t) - \hat{x}(t)$ to converge to 0. In other words,

$$
\lim_{t\rightarrow \infty}(\tilde{x}) = 0
$$

Therefore, this now becomes a clear control problem as we're effectively trying to minimize our error.

## Main Body

### The Luenberger Observer

The Luenberger Observer is a deterministic state estimator. First, we create create estimator copies of the original system.

$$ 
\dot{\hat{x}}(t) = Ax(t) + Bu(t) \\
\hat{y}(t) = Cx(t) + Du(t)
$$

Now expanding our error dynamics:

$$
\dot{\tilde{x}}(t) = \dot{x}(t) - \dot{\hat{x}}(t) = Ax(t) + Bu(t) - A\hat{x}(t) - Bu(t) = A\hat{e}(t)
$$

Now we can clearly see two situations arise from the following. Either the matrix A has all eigenvalues in the left half-plane which will allow $\hat{e}(t)$ to converge to zero. While this may converge to 0, it may be impractically slow and doesn't utilize the measurement as a feedback system. Additionally, we cannot guarantee the matrix A to be stable. Adding feedback only requires adding matrix $L$ to the estimator.

$$ 
\dot{\hat{x}}(t) = Ax(t) + Bu(t) + L(y(t) - \hat{y}(t))
$$

Again we expand our error dynamics with new estimated state dynamics.

$$
\begin{align*}
\dot{\tilde{x}}(t) &= \dot{x}(t) - \dot{\hat{x}}(t) \\
&= Ax(t) + Bu(t) - A\hat{x}(t) - Bu(t) - L(y(t) - \hat{y}(t)) \\
&= Ax(t) - A\hat{x}(t) - L(Cx(t) - C\hat{x}(t)) \\
&= (A - LC)\tilde{x}(t)
\end{align*}
$$

This is the equation for the Luenberger observer. As the error is now dependent on the matrix $A - LC$ which can be selected with matrix to have left hand poles. This allows the error to converge to 0.

### Duality (Estimation & Control)

The structure of the Luenberger Observer is very similar to state feedback dynamic equation. In a linear, time-invariant system with state-space equations and state feedback:

$$
\begin{align*}
\dot{x} = Ax + Bu
u = -Kx
\end{align*}
$$

The state feedback gain matrix ($K$) can be selected to prioritize certain controls such as stability or response speed. If we use the feedback control in the original state equation, we can derive the closed-loop system:

$$ 
\dot{x} = (A-BK)x
$$

When compared to the Luenberger observer, we can see that both equations have the same form.

$$
\dot{\tilde{x}}(t) = (A^T - L^TC^T)\tilde{x}(t)
$$

To choose L and the pole placement, we need to ensure that the system must be reachable. Recalling Definition 4, we know that it is reachable when $rank(R) = n = \dim(A)$. From the duality realization, we can substitute the estimator's equivalents into our reachability matrix.

$$
\begin{align*}
\tilde{R} &= \begin{pmatrix}
            \tilde{B} & \tilde{A}\tilde{B} & \dots & \tilde{A}^{n-1}\tilde{B}
            \end{pmatrix} \\
        &= \begin{pmatrix}
            C^T & A^T C^T & \dots & (A^T)^{n-1} C^T
            \end{pmatrix} \\
        &= \begin{pmatrix}
            C \\ CA \\ \vdots \\ CA^{n-1}
            \end{pmatrix}^T \\
        &= O^T
\end{align*}
$$

where $O$ is the observability matrix. Since $rank(O) = rank(O^T)$, we can set $rank(O)$ to be n for pole placement. Now we can use the Ackermann formula to determine the state feedback matrix K for pole placement.

$$
K = \begin{pmatrix}
0 & \dots & 1
\end{pmatrix}
R^{-1}p^*_{cl}(A)
$$

where $p^*_{cl}(A)$ is the characteristic polynomial from the closed-loop matrix. Once again, use duality between controls and estimation, we can now write

$$
\begin{align*}
\tilde{K} &= L^T \\
&= \begin{pmatrix}
0 & \dots & 0 & 1
\end{pmatrix}
\begin{pmatrix}
0 & \dots & 0 & 1
\end{pmatrix}^T
\end{align*}
$$

Finally we can write the augmented estimated state dynamics matrix.

$$
\begin{align*}
\begin{bmatrix}
    \dot{x}(t) \\ \dot{\tilde{x}}
\end{bmatrix} = 
\begin{bmatrix}
    A-BK & BK \\ 0 & A - LC
\end{bmatrix}
\begin{bmatrix}
    \dot{x}(t) \\ \tilde{x}
\end{bmatrix}
\end{align*}
$$

Since this matrix is upper block triangular, the eigenvalues of the combined diagonal values only depend on blaock diagonal elements. This means that the control and estimation do not interact with each other and can be design independently. As a general rule, it is advised to design the controller first by placing poles of $A-BK$ in desired LHP regions by determining $K$. Then the observer can be designed by finding $L$ to place poles of $A-LC$ in the LHP. The observer should be much faster than the controller.


## Conclusion

State estimators are essential in real-world control applications where not all states are directly measureable. In a noiseless scenario, the Luenberger observer reconstructs the full state vector from the output measurements and known control inputs to the system. In many applications, sensor limitations can create difficulties in measuring certain states accurately. 

In these notes we have derived this specific estimator/observer starting from the system's state space models. By adding a matrix $L$ to the observer's state estimate, we can begin manipulating the pole placement to achieve desirable convergence in the estimation error. This ensures that, over time, the estimation error decays according to system dynamics. In the process of doing so, we also reviewed multiple topics of control theory such as the Ackermann formula, pole placement, reachability, and the state feedback dynamic equation as well as the duality of state estimation and controls. Finally, we were able to prove that the design of control and estimation are independent of each other and formed a high-level design procedure for a state estimator system.

## References

[1] G. Zardini, "Lecture 10: Estimation", Control Systems II, ETH Zurich, 2018.

[2] S. Boyd, "Observability and State Estimation", EE 363: Linear Dynamical Systems, Standford University, 2009.
