# Foundations of Control Barrier Functions 

## Introduction 
In recent years we have had a widespread deployment of autonomous systems, from autonomous vehicles to surgical robots, these systems rely on their ability to perform complex tasks while providing safety assurances. There is a growing increasing number of complex applications with inherent real-world uncertainties using these systems requiring safety-critical autonomous systems... Control barrier functions (CBFs) have recently become a popular technique for **guarenteeing safety** of autonomous systems 

## Preliminaries

### Theoretical Motivation
Control Barrier Functions (CBFs) are motivated from the perspective of control Lyapunov functions (CLFs) which provide guarentees for the stabilization of (nonlinear) dynamical systems. This leads to the "dual" of stability which is safety of a dynamical systems via set invariance and further CBFs in the form of safety sets and enforcing safety through optimization-based controllers. This note will contain a discussion of the theoretical foundations to understand CBFs but will skip a review of Lyapounov Stabiltiy and CLFs (check [2], [3] for more information). 

### Forward Invariance
To first understand the theory behind control barrier functions (CBFs) we must first learn about control invariant sets. The first investigation into safety for dynamical systems was by Nagumo when he provded neccessary and sufficient conditions for set invariance [1]. Lets define what it means to have a positivley invariant set (also called a forward invariant set). We consider a continuous dynamical system: 

$$
 \dot{x}(t) = f(x,t), \hspace{0.2cm} t \geq 0
$$
 
where $x(t) \in \mathbb R^n$ are the state variables, $t$ is the time variable, and $f$ is a continous function. We now provide the definition of the forward invariant set of a dynamical system. 

**Definition (Forward Invariance)** Assume $C$ is a set in $\mathbb R^n$. The set $C$ is called a forward invariant set of the aforementioned dynamical system if $x(0) \in C$ implies $x(t) \in C$ for all $t \geq 0.$ 

In other words, a forward invariant set is a set that once a trajectory of a dynamical system enters the set it will never leave the set for all time. 

## Control Barrier Functions
Now that we have defined the forward invariance of a set, we will formalize the function that gives guarentees for the trajectory of our dynamical system to remain safe for the entirity of its time horizon. If you recall, Lyapunov stability drives a system to a point (or a set) in a fashion similar to an energy function, there is a dissapation of the trajectory to that point. Safety, rather, can be framed in the context of enforcing invariance of a set, i.e., not learing a **safe set**. Particularily, we define a set $C$ as the *superlevel set* of a continuously differentiable function $h : D \subset \mathbb R^n \rightarrow \mathbb R$ giving the following, 

$$
\begin{aligned}
C = \{ x \in D \subset \mathbb R^n : h(x) \geq 0 \}, \\
\partial C = \{x \in D \subset \mathbb R^n : h(x) = 0 \} \\
\text{Int}(C) = \{x \in D \subset \mathbb R^n : h(x) > 0 \}. \\
\end{aligned}
$$

We call $C$ the **safe set**. 

Although motiviated by and a generalization of CLFs, CBFs are different in that fact that we wish to enforce set invariance without strictly requireing a positive definite function $V(x)$. The continuoulsy differentiable function we define with regards to CBFs, $h$, renders our safe set $C$ invariant but not its sublevel sets. This allows a trajectory inside the invariant set to envolve anywhere inside the set rather than dissipating to a certain point in the set (it can even move up to and along the boundary of the invariant set, $\partial C$). 

We first define the extended class $\Kappa_{\infty}$ function as $\alpha : \mathbb R \rightarrow \mathbb R$ that is strictly increasing and zero at zero, i.e., $\alpha (0) = 0$, so the *extended* function is defined on the entire real line: $\mathbb R = (-\infty, \infty)$. With this, we can now define control barrier functions as in [4], [5]: 

**Definition (Controll Barrier Functions)** Let $C \in D \in \mathbb R^n$ be the superlevel set of a continuously differentiable function $h : D \rightarrow \mathbb R$, then $h$ is a *control barrier function (CBF) if there exists an extended class $\Kappa_{\infty}$ function $\alpha$ such that for the control affine system: 

$$
\text{sup}_{u\in U} [L_f h(x) + L_g h(x) u] \geq - \alpha (h(x)) \hspace{0.2cm} \text{for all} x \in D
$$

where













## References
1. F. Blanchini, "Set invariance in control," Automatica, vol. 35, no. 11, pp. 1747-1767, 1999. 

2. Sontag, E. D., "A 'universal' construction of Artstein's theorem on nonliner stabilization," Systems and Control Letters 117-123, 1989. 

3. A. Ames, K. Galloway, K. Sreenath, and J. Grizzle. "Rapidly exponentially stabilizing control Lyapunov functions and hybrid zero dynamics," IEEE Transactions on Automatic Control, 2014. 

4. A. Ames, S. Coogan, M. Egerstedt, G. Notomista, K. Sreenath, P. Tabuada. "Control Barrier Functions: Theory and Applications," 2019. 

5. AD Ames, X XU, JW Grizzle, P Tabuada. "Control barrier function based quadratic programs for safety crtiical systems," IEEE Transactions on Automatic Control, 62 (8), 3861-3876, 2017. 

6. Wikipedia, "Class Kappa Function," https://en.wikipedia.org/wiki/Class_kappa_function. 

7. 