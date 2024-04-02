# Learning Lyapunov functions for arbituary non-linear dynamics
Short note for EE 548 week 1 topic 3

## Scope and objective 
We briefly mentioned in the lecture modern, data-driven approaches for developing Lyapunov functions, and in this note I will examine some of these approaches for learning a Lyapunov function for arbituary non-linear dynamics, with or without an explicit model for the dyanmics.
Disclaimer: This will look more like a brief literature review of recent papers rather than actual notes from a lecture, per se. Most of the methods I will include in this note are summarized in [1].

## Introduction and motivations
In the lecture we covered Lyapunov methods for analyzing stability, which are especially useful for non-linear systems (as opposed to linear systems in which it suffices to examine the eigenvalues of state matrices). However, one key factor for Lyapunov stability analysis is to actually find a candidate Lyapunov function, which is something the theorem doesn't help with. While there exist some heurictics, especially for physical systems (energy of the system, quadratic form, etc), it is in general a very hard task. Modern approaches for learning a Lyapunov function for non-linear systems have been proposed whether or not an explicit form of the dynamics is known (or a good enough simulator for the system is available). 
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
One of the methods for generating a candidate Lyapunov function is to "guess" its form as linear polynomical of the state variables, which is affine in the state variables, with undetermined coefficients. The derivative of this function will be equally affine in all the polynomical coefficients, which can then be solved for by establishing a LMI and solving using sum of squares (SOS).   
As an example proposed in [2]:  
Consider the following dynamical system:  
$$\dot{x_1} = -x_1 -2x_2^2$$  
$$\dot{x_2} = -x_2 -x_1x_2 - 2x_2^3$$  
For this system, let us consider a potential candidate Lyapunov function $V(x) = A^TPB$ (analogus to that of linear systems), where
```math
A = \begin{Bmatrix} 1\\x_1\\x_1^2\\x_1^3\\x_1^4 \end{Bmatrix}, B = \begin{Bmatrix} 1\\x_2\\x_2^2\\x_2^3\\x_2^4 \end{Bmatrix} and \quad P = \begin{Bmatrix} 0&0&c_{02}&c_{03}&c_{04}\\0&c_{11}&c_{12}&c_{13}&0\\c_{20}&c_{21}&c_{22}&0&0\\c_{30}&c_{31}&0&0&0\\c_{40}&0&0&0&0 \end{Bmatrix} \quad is \quad the \quad matrix \quad consisting \quad of \quad coefficients \quad for \quad the \quad polynomial
```
*Note*: It is in general good to start with setting the candidate Lyapunov function to be one order higher than the original dynamics  
This function can then be recast into a quadratic form as 
```math
V(x) = \frac{1}{2} z^tQz, \quad where z = \begin{Bmatrix} x_1\\x_1^2\\x_1x_2\\x_2^2\\x_2 \end{Bmatrix} 
```
and $Q$ is a matrix consisting of terms in $P$ and some arbituary real numbers. 
Now the first and second condition for Lyapunov stability, $V(0) = 0$ and $V(x) > 0 \quad \forall x \in D \setminus \{0\}$ can be achieved by solving a condition in which the coefficients satisfy $Q \geq 0$ (due to the quadratic form). Further, we can write the derivative of the candidate Lyapunov function in a similar manner into quadratic form, in which the third condition of for Lyapunov stability can be reformatted into a LMI as well. These LMIs can then be easily solved using any linear solver.  
  
As it turns out, LMI-based methods can be applied for piecewise-affine functions (PWA) as well, which see frequent applications (for example, the commonly applied ReLU activation function in neural networs is PWA, so is the whole network made up by them).
#### Sampling-based approaches [3]
On the topic of ReLU functions being PWA (and piecewise linear (PWL) as well), this property has been exploited to "learn" Lyapunov functions for PWL dynamical systems. 
  
In [3], the authors built a feedforward neural network with leaky ReLU activation functions (also PWL) to learn a Lyapunov function for PWL systems. This is first achieved by proving that if a PWA Lyapunov function exists for a PWL system, then that Lyapunov function can be represented by "a fully connected neural network with leaky ReLU activation functions and no bias terms". The learning process for generating this neural network is a data-driven, sampling-based method, where the authors used a mixed-integer linear program to solve for samples in the dynamical system which violates the current neural network output (the "current" candidate Lyapunov function) the most; this violation level of this counter-example is then used as the loss function for training the neural network.  
  
This method is known as the counter-example guided inductive synthesis (CEGIS) and exists in many variations. It should be noted that this method does not necessarily demand the explicit algebric form of the dynamical system, instead only relies on our ability to generate samples (at least with high fidelity) from the system.  

Loosening on the requirement, assume we don't have the explicit model for the dynamical system, but can sample arbituarily from the system via, say, a simulator with high fidelity. In those cases, while it is still very hard to find a proper Lyapunov function to establish stability of the system (or lack therefof), it has been shown that using a neural network with supervised learning, one can develop a model predictive controll (MPC) scheme with statistical guarantee on stability [4] (although not a strict mathematical one).
### When a dataset of observations from the dyanmical system is given (an explicit form of the dynamical system is not available)
Some times (more often then not...), we simply don't have an explicit model for the dynamical system. Further, many systems are hard/expensive to arbituarily sample from. In those cases, we'd want to be able to learn a Lyapunov function (if one exists) to prove the stability of the system. In those cases what we might have is a dataset of observations for the system, and the goal is to establish a Lyapunov function that holds true for the *unknown* system defined by the dataset. These are more complicated cases, especially if one aims for establishing stability in the mathematical sense. [1] gave one such example, where the authors proposed an algorithm to establish a robust PWA Lyapunov function and its region of attraction for unknown dynamical system defined solely by a dataset by exploiting convex optimization techniques. It was however also remarked in the paper that due to the extremely limited information available for the dynamical system, even low-dimensional systems pose a big optimization problem, so without other a-priori knowledge of the system the technique might be untractable for more complex systems.  
## Conclusion 
Overall, our ability to establish Lyapunov stability for arbituary non-linear dynamical systems remain very limited. Our ability to find a Lyapunov function (and further region of attraction of the Lyapunov function) for non-linear systems often depends on exploiting certain properties of the dynamical system, for example its polynomial format or the piecewise affine nature. When strict stability is not necessarily required, random-sampling-based machine learning algorithms can also be used to establish statistical stability. Lastly, recent researches have looked at learning Lyapunov functions based on observations of a dynamical system without its explicit form, but such algorithms are still limited to relatively simple systems to minimize the complexity of the problems.
## References
[1] [Robust data-driven Lyapunov analysis with fixed data](https://arxiv.org/pdf/2305.12813.pdf) by Yingzhao Lian, Matteo Tacchi and Colin Jones  
[2] [Structured Semidefinite Programs and Semialgebraic Geometry Methods in Robustness and Optimization](https://web.mit.edu/~a_a_a/Public/Publications/refs_for_seb_blog/Parrilo_thesis.pdf) - PhD thesis by Pablo A. Parrilo  
[3] [Counter-example guided synthesis of neural network Lyapunov functions for piecewise linear systems](https://ieeexplore.ieee.org/document/9304201) by Hongkai Dai, Benoit Landry, Marco Pavoneand and Russ Tedrake  
[4] [Learning an Approximate Model Predictive Controller with Guarantees](https://arxiv.org/pdf/1806.04167.pdf) by Michael Hertneck, Johannes Kohler, Sebastian Trimpe and Frank Allgower    

