# Bayes Rule, Recursive Bayesian Estimator, and the Recursive Bayesian Filter

## Bayes's Rule

Bayes's rule, named after Reverend Thomas Bayes, is a fundamental principle in probability theory and statistics. It describes how to update the probability of a hypothesis based on new evidence. Mathematically, Bayes's rule is expressed as:
$$P(X|Y) = \frac{P(Y|X)P(X)}{P(Y)}$$

Each component of the expression is important and has its own name:
- $P(X|Y)$ is the *posterior*, which asks "if y is a measurement, then what is x given y?"
- $P(X)$ is the *prior*, a belief/guess of X before knowing Y
- $P(Y|X)$ is the *likelihood function*, which is what you expect Y to be given X
- $P(Y)$ (also written as $\int_X P(Y|X)P(X)dX$) is the normalizing factor.

Yet what happens if we have multiple measurements? We can write our multiple measurement expression as the following:
$$P(X|Y_1, Y_2) \propto P(Y_2|X,Y_1)P(X|Y_1)$$

Keep in mind, we assume our measurements are independent:
$$\propto P(Y_2|X)P(X|Y_1)$$
$$\propto P(Y_2|X)P(Y_1|X)P(X)$$

This applies for any number of measurements $P(X|Y_1,Y_2,\cdots, Y_N)$.

## Recursive Bayes's Estimator

We use the recursive Bayes's estimator to update the estimate of an unknown quantity as new data becomes available through the application of Bayes's Rule. It can be written as the following expression:
$$P(X|Y_{1:K}) = \frac{P(Y_K|X)P(X|Y_{1:K-1})}{\int_{\tilde{X}} P(Y_K|\tilde{X})P(\tilde{X}|Y_{1:K-1})d\tilde{X}}$$

A tilde is added to the integration term in the denominator in our notation for clarity. As well, note that the prior $P(X|Y_{1:K-1})$ for time step $k$ is simply the posterior from the previous time step $k-1$. Using this, we can then build the Recursive Bayesian Filter, but a few properties and mathematical expressions are worth going over first. The LHS of the RBE is proportional to the following expression:
$$P(X|Y_{1:K}) \propto P(Y_{1:K}|X)P(X)$$
where $P(Y_{1:K}|X)$ can be expanded via the chain rule to:
$$P(Y_{1:K}|X) = P(Y_K|X,Y_{1:K-1})P(Y_{1:K-1}|X)$$

Next, measurements with conditional independence w.r.t. x can be written as:
$$P(Y_K|X)P(Y_{1:K-1}|X)$$

Using this, we can then write the Bayes rule as the following (albeit gross but able to be simplified) expression:
$$P(X|Y_{1:K})=P(Y_K|X)P(Y_{1:K-1}|X)P(X)$$
$$P(X|Y_{1:K-1}) \propto P(Y_{1:K-1}|X)P(X)$$
$$P(X|Y_{1:K}) = \frac{P(Y_K|X)\frac{P(Y_{1:K-1}|X)P(X)}{\int_X P(Y_{1:K-1}|X)P(X)dX}}{\int_X P(Y_K|X) \frac{P(Y_{1:K-1}|X)P(X)}{\int_X P(Y_{1:K-1}|X)P(X)dX} dX}$$

Which can be simplified to:
$$P(X|Y_{1:K}) = \frac{P(Y_K|X)\frac{P(Y_{1:K-1}|X)P(X)}{P(X|Y_{1:K-1})}}{\int_X P(Y_K|X) \frac{P(Y_{1:K-1}|X)P(X)}{P(X|Y_{1:K-1})} dX}$$

### Independence

Given random variables X and Y, $X\perp Y$ iff $P(X,Y)=P(X)P(Y)$
$$X\perp Y \Rightarrow P(X|Y) = P(X)$$
$$P(Y|X) = P(Y)$$

### Conditional Independence

Given measurements $Y_1$ and $Y_2$ which are conditionally independent given $X$
$$P(Y_1|Y_2,X)=P(Y_1|X)$$
$$P(Y_2|Y_1,X)=P(Y_2|X)$$

## Recursive Bayesian Filter

Say we have $X$ which changes over time and assuming the system is *Markov*, meaning "the next state depends on only the current state", we can write this as:
$$P(X_0,X_1,X_2,\cdots,X_K) = P(X_0)\prod_{i=1}^KP(X_i|X_{i-1})$$

However, this is the Markov chain without control. When we insert control, expressed as $P(X_K|X_{K-1}, U_{K-1})$ we get a *Markov Decision Process*. If we include observations and assume some/all of $X$ cannot be observed, we have a *Partially Observable Markov Decision Process*.

The core of the Recursive Bayesian Filter is a two-step process: predict and update.

### Predict

$$P(X_K,X_{K-1}|Y_{1:K-1}) =  P(X_K|X_{K-1},Y_{1:K-1})P(X_{K-1}|Y_{1:K-1})$$
$$P(X_K|X_{K-1})P(X_{K-1}|Y_{1:K-1})$$
$$P(X_K|Y_{1:K-1}) = \int_{X_{K-1}} P(X_K, X_{K-1}|Y_{1:K-1})dX_{K-1}$$

### Update

$$P(X_K|Y_{1:K}) = P(X_K|Y_K,Y_{1:K-1})$$
$$BR \propto P(Y_K|X_K,Y_{1:K-1})P(X_K|Y_{1:K-1})$$
$$\propto P(Y_K|X_K)P(X_K|Y_{1:K-1})$$

where $P(X_K|Y_{1:K-1})$ is from the predict step.

## Conclusion: Why this is important

The Recursive Bayesian Filter is a fundamental tool for state estimation and tracking due to its ability to handle noise and uncertainty in measurements. It is particularly useful in real-time applications since it continuously updates the probability distribution with new data, making the estimation system robust and accurate. Finally, its importance is highlighted given that several other more complex tools build upon the Recursive Bayesian Filter:
- **Kalman Filter** - A specific implementation of the RBF for linear Gaussian systems
- **Extended Kalman Filter** - An extension of the Kalman filter for nonlinear systems
- **Unscented Kalman Filter** - A modification of the EKF to address some of its limitations, specifically attempting to better capture the mean and covariance estimates of the state distribution
- **Particle Filter** - A representation of the posterior distribution using a set of randomly chosen weighted samples or particles, important for non-Gaussian and nonlinear systems
- **Multiple-Model Estimators** - A combination of models to describe different modes of the system dynamics
- **Grid-Based Filters** - Filters that discretize the state space and apply the RBF within said discretized space
- **BraMBLe (Bayesian Multiple-Blob Tracker)** - A tracker using particle filters to handle multiple targets in tracking scenarios
- **Importance Sampling** - A technique used within Particle Filters to weight samples according to their likelihood, allowing for efficient estimation of the posterior distribution in high-dimensional spaces.
