# Kalman Filter: A Closed Form Solution to State Estimation

### Scope
- Linear discrete time system dynamics
### Objectives
- The implementation of a Kalman filter
- The derivation of the Kalman filter
- A brief description of duality

## Introduction: Help, I Don't Know Where my System is!
Many modern state space control techniques make the assumption that the controller knows the true state and output of the system. 

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

$\mu_k$ can then be used to represent $\hat{x}_k$, the current estimated state.

It is important to note that the Kalman filter process runs forwards in time.
### Kalman Filter Derivation
$$\Sigma_k^{pred}=Q+A\Sigma_{k-1}^{pred}A^T-A\Sigma_{k-1}^{pred}C^T(C\Sigma_{k-1}^{pred}C^T+R)^{-1}C\Sigma_{k-1}^{pred}A^T$$
## Conclusion

## References
[1] Leung, Karen. “Linear Multivariable Control” Lecture, University of Washington, Seattle, 2024-05-6.

## What to include in your notes

These notes are supposed to provide the reader with a quick deep dive into a particular topic or concept. To provide some guidelines on what topic/concept to choose, you can choose any topic that is related to that week's lecture. This can include something that was covered extensively during lectures or a passing reference that was made but was not deeply covered. Naturally, your choice of topic will depend on your level of familiarity. Diversity on topic choices is strongly encouraged. An anti-example is summarizing an entire lecture's worth of material that spans multiple topics/concepts.

In constructing your notes, you should include the following aspects:
- **Scope + objectives**: What should one expect to learn from reading these notes? Think about what is the purpose of these notes, what questions will be answered, and what will the reader gain from reading these notes.
- **Introduction**: Introduce the topic. Include things like how it is relevant to controls, in what situation would a reader find this useful, is there some historical context that's important to know, is this related to other topics?
- **Preliminaries**: Set up the mathematics needed to describe the topic/concept. Introduce definitions, notation, theorems, etc.
- **Main body**: Describe the topic/concept at a suitable depth given your selected scope and objectives. Consider including:
   - Figures and diagrams
   - Code snippets
   - Video/gif
   - Derivations
   - Concrete examples (e.g., with a specific system and with actual numbers)
   - Anything else that you think will be helpful!
- **Conclusion**: Summarize the key takeaways, and also mention what was not covered in these notes, and perhaps what are references the reader read to learn more.
- **References**: Please include references to any material you cite, or used in creating these notes.

## Tips and tricks for markdown

You can include a figure like this:
![alt text](figs/leung_cat.jpg "Title")

You can make subsubheadings like this
### subsubheading

or even subsubsubheading
#### subsubsubheading

You can add code snippets like this:
```
import numpy as np

def foo(x):
    print(x)
```

Or you can write equations using LaTeX notation like this:

You can write equations inline like this $x=\frac{2}{7}$ or on it's own line

$$ y=\sin(x) + \sum_{i=0}^T x^TQx$$

But don't forget to add references and acknowledge the sources you used to create these notes

[1] Optimal Control and Estimation by Robert F. Stengel.

[2] [Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation](https://underactuated.csail.mit.edu/) by Russ Tedrake 

[3] Optimal Control Theory: An Introduction by Donald E. Kirk.

[4] [Linear Systems Theory](https://web.ece.ucsb.edu/~hespanha/linearsystems/) by Joao Hespanha

[4] [Safe Autonomy with Control Barrier Functions](https://link.springer.com/book/10.1007/978-3-031-27576-0) by Wei Xiao, Christos G. Cassandras, and Calin Belta.
