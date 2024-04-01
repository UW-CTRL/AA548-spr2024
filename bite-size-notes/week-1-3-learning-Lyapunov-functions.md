# Learning Lyapunov functions for arbituary non-linear dynamics
Short note for EE 548 week 1 topic 3

## Scope and objective 
We briefly mentioned in the lecture modern, data-driven approaches for developing Lyapunov functions, and in this note I will examine some of these approaches for learning a Lyapunov function for arbituary non-linear dynamics, with or without an explicit model for the dyanmics.
Disclaimer: This will look more like a brief literature review of recent papers rather than actual notes from a lecture, per se. Most of the methods I will include in this note are summarized in [1].

## Introduction and motivations
In the lecture we covered Lyapunov methods for analyzing stability, which are especially useful for non-linear systems (as opposed to linear systems in which it suffices to examine the eigenvalues of state matrices). However, one key factor for Lyapunov stability analysis is to actually find a candidate Lyapunov function. While there exist some heurictics, especially for physical systems (energy of the system, quadratic form, etc), it is in general a very hard task. Modern approaches for learning a Lyapunov function for non-linear systems have been proposed whether or not an explicit form of the dynamics is known (or a good enough simulator for the system is available). 
The methods for the former case mostly divided into model-based and sampling-based approaches; typically, when the dynamical system is known explicity, we can further exploit its properties to simplifiy the process of obtaining a Lyapunov function. For the later case which involve exploiting a given dataset intead of the actual dynamics, I will include a Monte-Carlo sampling approach which ensures probabilitic stability and a novel data-driven approach that ensures strict mathematical stability within the given dataset.

## Math Preliminaries
Recall Lyapunov stability:  
  Given a system $\dot{x} = f(x)$ and some regioon $D \in \mathbb{R^n}$ and $\{0\} \in D$:
  If there exists a continuously differentiable function $V(x)$ such that:  
    1) $V(0) = 0$  
    2) $V(x) > 0 \quad \forall x \in D \setminus \{0\}$  
    3) $\dot{V(x)} = \nabla V(x)^Tf(x) \leq 0 $  
  Then the system is Lyapunov stable.   
  If, further, $\dot{V(x)}< 0 \quad \forall x \in D \setminus \{0\}$, then the system is asymptotically stable.  
  If, further, $\dot{V(x)} \leq -\alpha V(x)$ for some $\alpha > 0 \quad \forall x \in D \setminus \{0\}$, then the system is exponentially stable  

For a linear dynamical system $\dot{x} = Ax$, we can write $V(x) = x^TPx, and the test for Lyapunov stability is equivalent to the existence of a $P>0 that satisfies the linear matrix inequality (LMI) $A^TP + PA<0$. (For linear systems, Lyapunov stability is equivalent to asymptotic, exponenital stability).

## Modern approaches for finding Lyapunov functions
### When an explicit form of the dynamical system (or a good enough simulator) is given:
In this case, we can either estimate a form of the candidate Lyapunov function by exploiting properties of the known dynamics (model-based approaches) or propose a candidate Lyapunov function and penalize it by looking at samples of the dynamics that violate it (sampling-based approaches)
#### Model-based approaches [2]  
One of the methods for generating a candidate Lyapunov function is to "guess" its form as linear polynomical of the state variables, which is affine in the state variables, with undetermined coefficients. The derivative of this function will be equally affine in all the polynomical coefficients, which can then be solved for by establishing a LMI and solving using sum of squares (SOS). As an example proposed in [2]:  
Consider the following dynamical system:  
$$\dot{x_1} = -x_1 -2x_2^2$$  
$$\dot{x_2} = -x_2 -x_1x_2 - 2x_2^3$$  
For this system, let us consider a potential candidate Lyapunov function $V(x) = A^TPB$ (analogus to that of linear systems), where
```math
A = \begin{Bmatrix} 1\\x_1\\x_1^2\\x_1^3\\x_1^4 \end{Bmatrix}, B = \begin{Bmatrix} 1\\x_2\\x_2^2\\x_2^3\\x_2^4 \end{Bmatrix} and P = \begin{Bmatrix} 0&0&c_{02}&c_{03}&c_{04}\\0&c_11&c_12&c_13&0\\c_20&c_21&c_22&0&0\\c_30&c_31&0&0&0\\c_40&0&0&0&0 \end{Bmatrix}
```

#### Sampling-based approaches
### When a dataset of observations from the dyanmical system is given (an explicit form of the dynamical system is not available)
#### Monte-Carlo based sampling
#### (Placeholder) the prepreint

## References
[1] [Robust data-driven Lyapunov analysis with fixed data by Yingzhao Lian](https://arxiv.org/pdf/2305.12813.pdf) by Matteo Tacchi and Colin Jones
[2] [Structured Semidefinite Programs and Semialgebraic Geometry Methods in Robustness and Optimization (https://web.mit.edu/~a_a_a/Public/Publications/refs_for_seb_blog/Parrilo_thesis.pdf) - PhD thesis by Pablo A. Parrilo
