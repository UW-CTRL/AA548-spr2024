# Recursive Bayesian Filters for Linear Gaussian Systems

### Scope

The recursive Bayesian filter describes how the probability distribution of a random signal varies with time.
Astoundingly, if the signal is the output of a linear system with a Gaussian state distribution, the recursive Bayesian filter exactly reproduces the Kalman filter.

### Objective

Derive the Kalman filter from the recursive Bayesian filter subject to linear dynamics and Gaussian distributions, and comment on the connection between the two filters.

## Introduction

The recursive Bayesian filter (RBF) is the most general possible statement of how one's belief about the value of a random signal changes over time.
If one wishes to estimate the state of an uncertain system, the RBF exactly describes how the probability of each state varies over time.
This is the math that underlies any state estimation problem, from localizing the position of an aircraft using GPS to determining the orientation of a satellite using a star tracker.

However, the RBF is challenging to use in practice, as it makes heavy use of Bayes' rule. As a reminder, Bayes' rule states that the probability of an event $X$ given evidence $Y$ is
$$
\begin{equation}
    p(x|y) = \frac{p(y|x) p(x)}{p(y)} = \frac{p(y|x) p(x)}{\int_{\tilde{x}} p(\tilde{x}, y) \, d\tilde{x}}
\end{equation}
$$
If there are lots of possibile values for $X$, then the denominator is extremely hard to compute, as it requires integrating over all of them.
In general, this makes the RBF expensive to use, as most systems of interest have a large number of possible states, or even infinitely many.
For instance, a single vehicle might have 12 continuous states that parametrze its position, velocity, attitude, and angular velocity---and numeric integration over 12 continuous states is never fun.
Due to this difficulty in computation, it is useful to make some simplifying assumptions in order to leverage the power of the RBF.
In particular, we will assume that

1. the signal is the output of a linear system, and
2. the state of the system follows a Gaussian distribution.

These are handy assumptions because a Gaussian distribution subject to linear dynamics remains Gaussian.

## Preliminaries

### Gaussian random variables

For a random variable $X$, a particular value or sample is represented by the lowercase $x \in \mathbb{R}^n$.
When we say $X \sim \mathcal{N}(\mu, \Sigma)$, we mean that $X$ is "Gaussian": it follows a Gaussian or normal distribution with mean $\mu \in \mathbb{R}^n$ and covariance $\Sigma \in \mathbb{R}^{n \times n}$:
$$
\begin{equation}
    p(x) = \mathcal{N}(\mu, \Sigma) = \frac{1}{(2\pi)^{n/2} \det(\Sigma)^{1/2}} \exp\left( -\frac{1}{2} (x - \mu)^T \Sigma^{-1}(x - \mu) \right)
\end{equation}
$$
where $p(x)$ is the probability that $X$ produces $x$. Recall that the mean of $X$ is $\mu := \mathbb{E}[X]$ and the covariance of $X$ is $\Sigma := \mathbb{E}[(X - \mu)(X - \mu)^T]$, where $\mathbb{E}[\cdot]$ is the expected value of an expression. Further, recall that the expected value is linear.

Gaussian random variables remain Gaussian under linear transformations.
Specifically, if $X \sim \mathcal{N}(\mu, \Sigma)$, then for any matrix $M \in \mathbb{R}^{n \times m}$, we have $MX \sim \mathcal{N}(\mu', \Sigma')$ for some $\mu' \in \mathbb{R}^m$ and $\Sigma' \in \mathbb{R}^{m \times m}$.

### Conditional distributions of Gaussian random variables

If $X$ and $Y$ are both Gaussian random variables, then $X | Y$ ("$X$ given $Y$") and $Y | X$ ("$Y$ given $X$") are also Gaussian random variables.
In particular:
$$
\begin{equation}
    \begin{aligned}
        p(x | y) = \mathcal{N}(\mu_{x|y}, \Sigma_{x|y}) \quad &\text{where} \quad \mu_{x|y} = \mu_x + \Sigma_{xy}\Sigma_y^{-1} (y - \mu_y), \quad \Sigma_{x|y} = \Sigma_x - \Sigma_{xy}\Sigma_y^{-1}\Sigma_{yx} \\
        p(y | x) = \mathcal{N}(\mu_{y|x}, \Sigma_{y|x}) \quad &\text{where} \quad \mu_{y|x} = \mu_y + \Sigma_{yx}\Sigma_x^{-1} (x - \mu_x), \quad \Sigma_{y|x} = \Sigma_y - \Sigma_{yx}\Sigma_x^{-1}\Sigma_{xy} \\
        & \text{where} \quad \Sigma_{xy} = \Sigma{yx}^T = Cov(X, Y) := \mathbb{E} [(X - \mu_x)(Y - \mu_y)^T] .
    \end{aligned}
\end{equation}
$$

### Recursive Bayesian filter

The recursive Bayesian filter aims to estimate the state $x_k$ of the system at time $k$ given some history of observations, $y_{1:k}$, where $p(y_{1:k}) = p(y_1, y_2, \dots, y_k)$.
This is done using a two-step recursive process:

1. **Predict:**
$$
\begin{equation}
    p(x_k | y_{1:k - 1}) = \int_{x_{k - 1}} p(x_k | x_{k - 1}) p(x_{k - 1} | y_{1:k - 1}) \, dx_{k - 1}
\end{equation}
$$

2. **Update:**
$$
\begin{equation}
    p(x_k | y_{1:k}) \propto p(y_k | x_k) p(x_k | y_{1:k - 1}) \quad \text{(Bayes' rule)}
\end{equation}
$$

The proportionality constant in the update step is the unique constant that normalizes the posterior probabiity, such that $\int_{x_k} p(x_k|y_{1:k}) \, dx_k = 1$.
This constant can occasionally be computed explicitly, but often can only be approximated.
In the case of a linear Gaussian system, we will be able to compute it explicitly.

## Derivation

We wish to derive the estimate found by the recursive Bayesian filter, given the linear time-invariant dynamics
$$
\begin{equation}
    \begin{aligned}
        x_{k + 1} &= Ax_k + Bu_k + w_k, &&\quad W_k \sim \mathcal{N}(0, Q) \\
        y_k &= Cx_k + v_k, &&\quad V_k \sim \mathcal{N}(0, R)
    \end{aligned}
\end{equation}
$$
where $w_k$ represents the process noise at timestep $k$ and $v_k$ represents the measurement noise.
We assume the value of $u_k$ is known exactly.
We further assume the state is a Gaussian random variable, so that $X \sim \mathcal{N}(\mu_{x_k}, \Sigma_{x_k})$.
The observation $y_k$ is linear in $x_k$ and $v_k$, so it is Gaussian as well:
$$
\begin{equation}
    Y_k = CX_k + V_k \  \sim \  \mathcal{N}(\mu_{y_k}, \Sigma_{y_k}) .
\end{equation}
$$
Let's compute the mean and covariance of $Y_k$. Due to the linearity of the expected value,
$$
\begin{equation}
    \mu_{y_k} := \mathbb{E}[Y_k] = \mathbb{E}[CX_k + V_k] = C\mathbb{E}[X_k] + \mathbb{E}[V_k] = C\mathbb{E}[X_k] = C\mu_{x_k}
\end{equation}
$$
since $V_k$ has zero mean and so zero expected value. Furthermore, we have $Cov(X_k, V_k) = 0$; the current state is uncorrelated with the current process noise. Given all of this, and skipping some algebra,
$$
\begin{equation}
    \begin{aligned}
        \Sigma_{y_k} &= Cov(Y_k) := \mathbb{E}[(Y_k - \mu_{y_k})(Y_k - \mu_{y_k})^T] = ... = C\Sigma_{x_k}C^T + R_k \\
        \Sigma_{x_k y_k} &= Cov(X_k, Y_k) := \mathbb{E}[(X_k - \mu_{x_k})(Y_k - \mu_{y_k})^T] = ... = \Sigma_{x_k} C^T \\
        \Sigma_{y_k x_k} &= Cov(Y_k, X_k) = \Sigma_{x_k y_k}^T = C \Sigma_{x_k}
    \end{aligned}
\end{equation}
$$
where $\Sigma_x = \Sigma_x^T$, since the covariance of a random variable with itself is symmetric.

We can now compute the mean and covariance of $X|Y$, since we have calculated all of the terms in Eq. (2). Plugging in our expressions for $\Sigma_{y_k}$, $\Sigma_{x_k y_k}$, and $\Sigma_{y_k x_k}$ from Eq. (8) into Eq. (2), we find that
$$
\begin{align}
    \mu_{x_k | y_k} &= \mu_{x_k} + \Sigma_{x_k} C^T (C \Sigma_{x_k} C^T + R)^{-1} (y_k - C\mu_{x_k}) \\
    \Sigma_{x_k | y_k} &= \Sigma_{x_k} - \Sigma_{x_k} C^T (C \Sigma_{x_k} C^T + R)^{-1} C\Sigma_{x_k} .
\end{align}
$$

We notice and factor out the common term
$$
\begin{equation}
    K_k = \Sigma_{x_k} C^T (C \Sigma_{x_k} C^T + R)^{-1}
\end{equation}
$$
to obtain
$$
\begin{gather}
    \mu_{x_k | y_k} = (I - K_k C) \mu_{x_k} + K_k y \\
    \Sigma_{x_k | y_k} = (I - K_k C)\Sigma_{x_k} .
\end{gather}
$$

These values describe the probability of being in state $x_k$ given the single observation $y_k$. But what we really want to know is the probability of being in $x_k$ given the history of observations $y_{1:k}$. We don't actually have to change much!

- Assume we have already made the measurements $y_{1:k - 1}$, as of the previous recursive step.
- Our prior (before measurement $y_k$) is $p(x_k | y_{1:k - 1})$ instead of $p(x_k)$. Thus, all of the lone $x_k$'s in Eqs. (11-13) get replaced with $x_k | y_{1:k - 1}$.
- Our posterior (after measurement $y_k$) is $p(x_k | y_{1:k})$ instead of $p(x_k | y_k)$.

Making these replacements in Eqs. (11-13), we get
$$
\begin{gather}
    K_k = \Sigma_{x_k | y_{1:k - 1}} C^T (C \Sigma_{x_k | y_{1:k - 1}} C^T + R)^{-1} \\
    \mu_{x_k | y_{1:k}} = (I - K_k C) \mu_{x_k | y_{1:k - 1}} + K_k y_k \\
    \Sigma_{x_k | y_{1:k}} = (I - K_k C)\Sigma_{x_k | y_{1:k - 1}} .
\end{gather}
$$

This notation has gotten really awful really fast ($\mu_{x_k | y_{1:k - 1}}\ $ ˙◠˙ ), so let's introduce some shorthand:
$$
\begin{equation}
    \begin{aligned}
        x_k | y_{1:k} &\quad \to \quad k|k \\
        x_k | y_{1:k-1} &\quad \to \quad k|k - 1
    \end{aligned}
\end{equation}
$$
The left side of the "given" bar always refers to $x$ at a particular timestep. The right side of the bar always refers to a history of $y$ up to a particular timestep. Exchanging this notation in Eqs. (14-16), we find
$$
\begin{gather}
    K_k = \Sigma_{k|k - 1} C^T (C \Sigma_{k|k - 1} C^T + R)^{-1} \\
    \mu_{k|k} = (I - K_k C) \mu_{k|k - 1} + K_k y_k \\
    \Sigma_{k|k} = (I - K_k C)\Sigma_{k|k - 1} .
\end{gather}
$$

We're now ready to fill in the recursive Bayesian filter from Eq. (3) and (4). We need to determine the distributions

1. **Predict:** $\quad p(k|k - 1) = \mathcal{N}(\mu_{k|k - 1}, \Sigma_{k|k - 1})$

2. **Update:** $\quad p(k|k) = \mathcal{N}(\mu_{k|k}, \Sigma_{k|k})$

Thanks to the linear Gaussian assumption, we can calculate $\mu_{k|k - 1}$ and $\Sigma_{k|k - 1}$ by simply passing $\mu_{k - 1|k - 1}$ and $\Sigma_{k - 1|k - 1}$ through the dynamics:
$$
\begin{gather}
    \mu_{k|k - 1} = A\mu_{k - 1|k - 1} + B u_{k - 1} \\
    \Sigma_{k|k - 1} = A\Sigma_{k - 1|k - 1} A^T + Q
\end{gather}
$$
Then the update step is just an application of Eqs. (18-20).

### Summary

The recursive Bayesian filter for a linear Gaussian system is given by the following recursive predict-update cycle:

1. **Predict:**
$$
\begin{gather}
    \mu_{k|k - 1} = A\mu_{k - 1|k - 1} + B u_{k - 1} \\
    \Sigma_{k|k - 1} = A\Sigma_{k - 1|k - 1} A^T + Q
\end{gather}
$$

2. **Update:**
$$
\begin{gather}
    K_k = \Sigma_{k|k - 1} C^T (C \Sigma_{k|k - 1} C^T + R)^{-1} \\
    \mu_{k|k} = (I - K_k C) \mu_{k|k - 1} + K_k y_k \\
    \Sigma_{k|k} = (I - K_k C)\Sigma_{k|k - 1} .
\end{gather}
$$

Though it was derived in a completely different way, this is exactly identical to the Kalman filter equations.

## Conclusion

From the above derivation, we have proved that the Kalman filter is just a recursive Bayesian filter for the special case of a linear Gaussian system. This is quite profound! The Kalman filter was specifically designed to be optimal per the least-squares error between the predicted mean and actual state. The recursive Bayesian filter, meanwhile, is derived from basic probability, and in this derivation we just used the rules for how Gaussians evolve under linear systems. The recursive Bayesian filter may be hard to compute in general, but in this case it results in the popular and powerful tool that is the Kalman filter.