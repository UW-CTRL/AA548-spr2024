# A Quick Note of Control Barrier Function 
## Objectives
1. To provide a foundational understanding of Control Barrier Functions, explaining how they are used to ensure system safety by preventing the state of a system from entering unsafe regions.
2. To demonstrate the application of CBFs in real-world scenarios, such as in autonomous vehicle navigation or robotic motion control, where safety is paramount.
3. To discuss the latest advancements and research findings related to CBFs, potentially including different variations or integrations with other control methods like Lyapunov functions.  
## Introduction 
Control Barrier Functions (CBFs) is a critical component in the design of safety-critical control systems. As the demand for autonomous technologies and intelligent control systems continues to rise, ensuring the safety of these systems becomes paramount. CBFs play an indispensable role in this context by providing a robust mathematical framework to enforce safety constraints dynamically, keeping the system states within safe operational boundaries. In this note, we will discuss the fundamentals of CBFs from their theoretical basis to practical applications.  
## Preliminary
### Theoretical Motivation
Before talking about the definition of Control Barrier Function (CBF), it's essential to first grasp two foundational concepts that underpin the development of CBFs. The first one is the notion of a control invariance set. This concept is crucial as it pertains to the sets of states from which the control system can be maintained within desired limits through appropriate control actions. The second key concept is that of a Control Lyapunov Function (CLF), which provides a mathematical tool to assess the stability of a control system. 


#### Definition (Positively Invariant Set)
In mathematical analysis, a positively (or positive) invariant set is a set with the following properties: 
Suppose $\dot{x} = f(x)$ is a dynamical system, $x (t, x_0)$ is a trajectory, and $x_0$ is the initial point. Let $\mathbb{O} $ : = {$x \in \mathbb{R^n} |  \varphi (x) =0 $} where $\varphi$ is a real-valued function. [1]The set $\mathbb{O}$ s said to be positively invariant if $x_0 \in \mathbb{O}$ implies that $x(t, x_0) \in \mathbb{O}$ $\forall$ t $\geq 0$.   

In other words, once a trajectory of the system enters $\mathbb{O}$, it will never leave it again.

#### Definition (Control Lyapunov Function)
Consider an autonomous dynamical system with inputs $\dot{x} = f(x, u)$ where x $\in \mathbb{R^n}$ is the state vector and  $u \in \mathbb{R^m}$ is the control vector. Suppose our goal is to drive the system to an equilibrium $x_* \in \mathbb{R^n}$ from every initial state in some domain. [2]  
A control-Lyapunov function (CLF) is a function $V$ : $D \longrightarrow \mathbb{R}$ that is continuously differentiable, positive-definite, and such that for all $x \in \mathbb{R^n} (x \neq 0)$ , there exist u $\in \mathbb{R^m}$ such that   
$$\dot{V}(x,u)  = \nabla V(X)^T f(x, u) = L_f V(x) + L_g V(x) u< 0 $$
where, $L_f V(x)$ is called lie derivative of $V(x)$ along f(x).   
And a theorem follows: For the nonlinear control system if there exists a control Lyapunov function, then any continuous feedback controller $u(x) $ asymptotically stabilizes the system to $x_*$. 


## Control Barrier Function 
Suppose that we have a nonlinear affine control system: 
$$\dot{x} = f(x) + g(x) u $$   
with $f$ and $g$ locally Lipschitz, $x \in D \subset \mathbb R^n$ and $u \in U \subset \mathbb R^m$ is the set of admissible control inputs.   
Unlike Control Lyapunov Function that promises a **stability** which involves driving a system to a point (or a set) by dissapating energy of the trajectory. **Safety** can be framed in the context of enforcing invariance of a set, i.e., not leaving a safe set once entering the zone. In particular, we consider a set ${Q}$ defined as the superlevel set of a continuously differentiable function $b: {D} \subset \mathbb{R^n} \longrightarrow \mathbb{R}$, yielding: [3]
$$
\begin{aligned}
\mathcal Q = \{ x \in D \subset \mathbb R^n : b(x) \geq 0 \}, \\
\partial \mathcal Q = \{x \in D \subset \mathbb R^n : b(x) = 0 \} \\
\text{Int}(\mathcal Q) = \{x \in D \subset \mathbb R^n : b(x) > 0 \}. \\
\end{aligned}
$$  
We refer to ${Q}$ as the safe set.   

While inspired by and expanding upon Control Lyapunov Functions (CLFs), Control Barrier Functions (CBFs) differ by aiming to enforce set invariance without the strict requirement of a positive definite function. The continuously differentiable function defined for CBFs ensures the invariance of our designated safe set, but not its sublevel sets.
This motivates the formulation of control barrier functions. Before defining these, we note that an extended class ${K_\infty}$ function is a function $\alpha : \mathbb{R} \longrightarrow \mathbb{R}$ that is strictly increasing and with $\alpha(0) =0$. Extended functions are defined on entire real line: $\mathbb R = (-\infty, \infty)$. Then, we can define the Control Barrier Functions now.   
### Definition (Control Barrier Function)
Let $ Q \in D \in \mathbb {R^n}$ be the superlevel set of a continuously differentiable function $b : D \longrightarrow \mathbb {R}$, then $b$ is a Control Barrier Function (CBF) if there exists an extended class $K_{\infty}$ function $\alpha$ such that for the control affine system: [3]
$$\text{sup}_{u\in U} [L_f b(x) + L_g b(x) u] \geq - \alpha (b(x)) $$ 

for all $ x\in D$  
Hence, we can quantify the set of all control inputs at a point $ x \in D$ that can guarantee the system's safety via CBFs. 
$$K_{cbf}(x) = \{u \in U : L_f b(x) + L_g b(x) u + \alpha (b(x)) \geq 0 \}$$  

Common choice of $\alpha$ is to be $: \alpha(r) = \alpha_0(r)$

To better understand the Control Barrier Function, we examine the following cases for the system as it moving to the undesirable set ${C}$. ${C}$ is unsafe region where $C = \{x \in D \mid b(x) \leq 0 \}$ and assume $\alpha_0 =1$. 

1. System stays far away from unsafe region ${C}$: The system is relatively far from the undesired set so it can to move in any direction basically. Notice not all directions are allowed such as going directly toward C. 
$$ L_f b(x) + L_g b(x) u   \geq - 3 $$


## Conclusion 
## References 
1. Wikipedia, "Positively Invariant Set", https://en.wikipedia.org/wiki/Positively_invariant_set
2. Wikipedia,"Control Lyapunov Function", https://en.wikipedia.org/wiki/Control-Lyapunov_function
3. A. Ames, S. Coogan, M. Egerstedt, G. Notomista, K. Sreenath, P. Tabuada. "Control Barrier Functions: Theory and Applications," 2019.
4.     