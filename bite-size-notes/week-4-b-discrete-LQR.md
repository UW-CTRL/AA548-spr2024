# Introduction to Discrete Time Linear Quadratic Regulator (LQR)

## Scope and Objectives

In these notes, we will focus on the discrete time Linear Quadratic Regulator (LQR), specifically:
1. Motivate why the LQR exists and how it could be useful in an optimal controls problem
2. Introduce the Bellman Equation, which lays the foundation of LQR formulation
3. Define the discrete time LQR problem
4. Derive the mathematical formulation for discrete time LQR for both the finite and infinite horizon case
5. Discuss possible methodologies for choosing LQR weights
6. Discuss the pros and cons of using LQR to solve an optimal control problem

## Introduction

In the field of optimal control, the goal is to control a dynamical system while minimizing the cost that results from the output of said control. However, choosing the parameters of the controller may not be immediately obvious, resulting in inefficient trial-and-error design of control inputs. Otherwise, methods such as full state feedback design, using eigenvalue/pole placement to design the controller transfer function, requires physical intuition or understanding may be required to design the parameters. Such intuition or understanding may not be available for complex systems, or be extremely difficult to obtain.

For a linear system, if the cost function is quadratic, the optimal control problem can be solved without the aforementioned issues using the LQR. The control system engineer would then only need to specify the system dynamics and the cost function, and the optimal control input can be calculated, which can be done efficiently using computational methods. The specification of the cost function can be entirely avoidant of physical intuition of the system being controlled. While it also can be an iterative process in which different cost functions are compared with respect to the control objective, the design process of the cost function can be simpler than individually specifying control inputs, and can also be more intuitive.

## Preliminaries and Motivation

We consider the discrete time dynamical system at some time step $t = k+1$ described by:

$$ x_{k+1} = Ax_k + Bu_k $$

and the corresponding finite horizon discrete time optimal control problem:

$$ \min_{u \in [u_0,u_K], x \in [x_0, x_k]} \lbrace J_{\text{term}} (x_{k+1}) + \sum_{k=0}^K J(x_k,u_k,k) \rbrace$$

subject to:

$$
\begin{split}
x_{k+1} &= Ax_k + Bu_k \\
x_k &\in \mathcal{X} \\
u_k &\in \mathcal{U} \\
x_0 &= x_{\text{init}} \\
g_i(x_k,u_k,k) &\leq 0, \quad i = 1, \dots, m \\
h_i(x_k,u_k,k) &= 0, \quad i = 1, \dots, p
\end{split}
$$

where $J_{term} (x_{k+1})$ is the terminal cost, and $J(x_k,u_k,k)$ is the running/stage cost. The terminal cost is a function of the state at the final time step, and the stage cost is a function of the state and control at each time step. To solve the optimal control problem means to find the optimal control sequence $u^*$ that minimizes this cost function.

In the form this question is written, we must re-solve the problem for each time step, which is computationally expensive. However, if we leverage dynamic programming, we can solve the problem for each time step in a recursive manner, which is much more efficient. For this, we introduce the Bellman equation:

$$ V(x_k, k) = \min_{u_k \in \mathcal{U}} \lbrace g(x_k,u_k,k) + V(x_{k+1}, k+1) \rbrace $$

where $V(x_k, k)$ is the value function, and the omtimization problem is to find the optimal control $u_k$ that minimizes the sum of the immediate cost $g(x_k,u_k,k)$ and the cost-to-go $V(x_{k+1}, k+1)$. Using dynamic programming, we can solve the optimal control problem posed by the Bellman equation by backtracking from the final time step $k+1$ to the initial time step $0$.

Posing the optimal control problem using the Bellman equation *can* give us efficiency, provided that we can actually optimize for the immediate cost and cost-to-go functions. One way of of doing this is to design the cost functions to be convex, for which there are deterministic methods that can be used to solve for the global optimum. As the name LQR suggests, we will choose these cost functions to be quadratic, and therefore convex.

## Linear Quadratic Regulator (LQR) Problem

### Discrete Time LQR

The discrete time LQR problem can be written as:

$$ 
\begin{split}
  \min_{u_k \in \mathcal{U}} &\lbrace x_{k+1}^T Q_T x_{k+1} + \sum_{k=0}^K (x_k^T Q_k x_k + u_k^T R_k u_k) \rbrace \\
  \text{s.t.} &\quad x_{k+1} = Ax_k + Bu_k \\
  & \quad \quad x_0 = x_{\text{init}}
\end{split}
$$

note that we have no control or state constraints. The dynamics are linear, and the cost function, composed of the next state cost and the running cost, are both quadratic and therefore convex. The regulator regulates to zero for stability. The quadratic cost function guarantees the optimal control input $u_k^*$ is globally optimal at each time step $k$.

The matrices $Q_k$ and $R_k$ are the weights for the state and control costs, respectively. The matrix $Q_T$ is the terminal cost weight, and is used to stabilize the system. The matrices $A$ and $B$ are the system dynamics matrices, and $x_k$ is the state at time step $k$. We require the matrices $Q_k$ and $Q_T$ to be positive semi-definite, and $R_k$ to be positive definite in order to guarantee that the cost function is bowl-shaped and has a global minimum. As a result, we have that:

$$ Q_k = Q_k^T \succeq 0, \quad Q_T^T \succeq 0, \quad R_k^T \succ 0 $$

We also sometimes assume that $Q_{k+1} \succeq Q_k$ and $R_{k+1} \succeq R_k$ for all $k$ to ensure that the cost function is monotonically decreasing.

The positive definite nature of $R_k$ means that any non-zero control input adds to the cost, which ensures that the control is minimized to reduce input effort. The positive semi-definite nature of $Q_k$ and $Q_T$ means that some states may not contribute to the cost, which helps to reduce control effort. Also, $Q_T$ and $Q_k$ ensures that the states are bounded, and the system is stable.

### Solving the discrete time, finite horizon LQR problem

For simplicity, we let $K$ be some finite time horizon, and we will solve the discrete time, finite horizon LQR problem.

As suggested by the preliminaries, we can solve the discrete time, finite horizon LQR problem by posing it in the form of the Bellman equation, and then leveraging dynamic programming to solve by backtracking.

Let the value function at time step $k$ be $V(x_k, k) = x_k^T P_k x_k$, where $P_k$ is symmetric and positive semi-definite. Then at the final time step, we have:

$$ V(x_{k+1}, k+1) = x_{k+1}^T P_{k+1} x_{k+1} \implies P_{k+1} = Q_T $$

where $P_{K+1} = Q_T$. This forms the first step of our dynamic programming algorithm. Then, at time step $k$, we have:

$$ V(x_k, k) = \min_{u_k \in \mathcal{U}} \lbrace x_k^T Q_k x_k + u_k^T R_k u_k + V(x_{k+1}, k+1) \rbrace $$

where the sum of the first two terms forms the immediate cost, $V(x_{k+1}, k+1)$ is the cost-to-go. Using the state dynamics, we can write:

$$ V(x_{k+1}, k+1) = V(Ax_k + Bu_k, k+1) = (Ax_k + Bu_k)^T Q_T (Ax_k + Bu_k) $$

We can then notice that the right-hand-side of our expression for $V(x_k, k)$ is quadratic in $u_k$ based on the second and third terms, and can be minimized by simply taking the derivative with respect to $u_k$ and setting it to zero to solve for the optimal control input $u_k^*$. To make differentiation easier, we can re-write $V(x_k,k)$ as:

$$\min_{u_k \in \mathcal{U}} x_k^T (Q_k + A^T P_{k+1} A)x_k + u_k^T (R_k + B^T P_{k+1} B)u_k + 2x_k^T A^T P_{k+1} Bu_k$$

where we used that fact that $P_{k+1} = Q_T$ and combined the quadratic and linear terms for $x_k$ and $u_k$. We now have a standard degree-2 polynomial in $u_k$ which is easy to differentiate w.r.t. $u_k$ and set to zero to solve for the optimal control input $u_k^*$:

$$ \frac{\partial V(x_k, k)}{\partial u_k} = 2(R_k + B^T P_{k+1} B)u_k + 2B^T P_{k+1} A x_k = 0 \implies u_k^* = - (R_k + B^T P_{k+1} B)^{-1} B^T P_{k+1} A x_k $$

Here we notice that in the expression 

$$ u_k^* = - (R_k + B^T P_{k+1} B)^{-1} B^T P_{k+1} A x_k $$

the term $(R_k + B^T P_{k+1} B)^{-1} B^T P_{k+1} A$ is entirely composed linear operations and can be pre-computed as $A, B$ are known from the system dynamics and $P_{k+1} = Q_T$ and $R_k$ are chosen by the control system engineer. This means that the optimal control input $u_k^*$ can be computed in **in constant time** at each time step $k$. We can simply denote this term as $K_k$, and so we have:

$$ u_k^* = -K_k x_k $$

where $K_k = (R_k + B^T P_{k+1} B)^{-1} B^T P_{k+1} A$. Using this optimal control input, we can then backtrack and compute the value function at time step $k$ and therefore $P_k$, which we can use to compute the optimal control input at time step $k-1$, and so on until we reach the initial time step $0$. Letting $u_k = -K_k x_k$, and noting that we had originally let $V(x_k, k) = x_k^T P_k x_k$, we plug this back into the minimization problem for $V(x_k, k)$ to get:

$$
\begin{split}
V(x_k, k) = x_k^T P_k x_k &= x_k^T (Q_k + A^T P_{k+1} A)x_k + u_k^T (R_k + B^T P_{k+1} B)u_k + 2x_k^T A^T P_{k+1} Bu_k \\
&= x_k^T (Q_k + A^T P_{k+1} A)x_k + x_k^T B^T P_{k+1} B K_k K_k^T x_k - 2x_k^T A^T P_{k+1} B K_k x_k \\
&= x_k^T (Q_k + A^T P_{k+1} A - (A^T P_{k+1} B) (R_k + B^T P_{k+1} B)^{-1} (B^T P_{k+1} A))x_k
\end{split}
$$

lots of Markdown, but hopefully showing more of the derivation is helpful! Finally, we have:

$$ P_k = Q_k + A^T P_{k+1} A - (A^T P_{k+1} B) (R_k + B^T P_{k+1} B)^{-1} (B^T P_{k+1} A) $$

which we can use to continue in the dynamic programming algorithm to solve for the optimal control input at each time step $k$. While this equation is long to look at, the computation is entirely linear, and it is known what the matrices $A, B, Q_k, R_k, Q_T, P_{k+1}$ are, so solving for $P_k$ is straightforward in code. The $u_k$ yielded at each step $k$ is the optimal control input that minimizes the cost function at that time step, and by the principle of optimality, this guarantees that the control vector $u^* = [u_0^*, u_1^*, \dots, u_K^*]$ is globally optimal.

Considering the constant time nature of solving for each $u_k^*$, the LQR problem with dynamic programming is known to run in $O(Kn^3)$ time, where $n$ is the number of states in the system, which is significantly faster than solving the LQR by brute force, which would run in $O(K^3 n m^2)$ time [1], where $m$ is the number of control inputs. This makes the dynamic programming approach not dependent on the dimensionality of the control inputs, and therefore more scalable. It is also forseeably more efficient than computing $u_k$ by brute force, which would may not be boundable through big $O$ notation. However, we see that solving LQR using DQ suffers from the curse of dimensionality, as with any dynamic programming approach [2].

### Solving the discrete time, infinite horizon LQR problem

Now that we have derived the solution to the discrete time, finite horizon LQR problem, we can extend this to the infinite horizon LQR problem. The infinite horizon LQR problem is the same as the finite horizon LQR problem, except that we let $K \to \infty$. This means that the terminal cost $Q_T$ is the same as the running cost $Q_k$ for all $k$, and we can denote this as $Q$. We also simply denote the control cost $R_k$ as $R$ for all $k$.

This also means that $P_k$ converges to some steady-state value $P$ as $k \to \infty$ since the value function $V(x_k, k)$ converges to some steady-state value $V(x_k)$, by virtue of the positive semi-definite nature of $Q$ and $R$, which penalizes non-zero states and control inputs. Using this information and our finite horizon derivation for $P_k$, we have:

$$ P = Q + A^T P A - (A^T P B) (R + B^T P B)^{-1} (B^T P A) $$

which is called the discrete algebraic Riccati equation (DARE). The solution to the DARE gives us the steady-state value of $P$, which we can then use to compute the optimal control input $u^* = - (R + B^T P B)^{-1} B^T P A x$, with the subscript $k$ dropped since the steady-state value of $P$ is the same for all time steps. This optimal control input is globally optimal for the infinite horizon LQR problem.

### Choosing Weights for the LQR Problem

The above derivations are classically known, and there are also ready implementations of both dynamic programming (for the finite horizon case) and the DARE (for the infinite horizon case) in many computational libraries [3].

The choice of the weights $Q_k, R_k, Q_T$ is then a critical, and non-trivial, part of the LQR problem. In many cases systems are designed such that they converge for all time if left unperturbed, so the design choices boil down to choosing $Q$ and $R$. 

While this can be either trial-and-error, or be based on physical intuition/understanding of the system itself, the abstraction provided by the LQR allows for a more systematic and efficient approach. From the derivation, we see that $Q$ is the matrix that weighs state vector $\vec{x}$, and $R$ is the matrix that weighs control vector $\vec{u}$. For simplicity, $Q$ and $R$ are often chosen to be diagonal matrices, where the diagonal elements are the weights for each state and control input, respectively. This then provides directing and independent weighting of the states and control inputs in the cost function, which can be much easier to tune than parameters in a full state feedback design which can affect the entire state or control, or choosing control inputs without clear knowledge of how each impacts the states. The weights can be tuned directly based on output observations of the system, without clearly understanding the system internals.

This fact leads to Bryson's Rule [4], which is a popular method for determining a starting point for $Q$ and $R$. In fact, MATLAB's example documentation on LQR defaults on using Bryson's rule to choosing $Q$ and $R$ [5]. Bryson's Rule states that, for diagonal matricies $Q$ and $R$, the diagonal elements are chosen such that:

$$
\begin{split}
Q_{ii} &= \frac{1}{\text{maximum allowable value of }x_i} \\
R_{ii} &= \frac{1}{\text{maximum allowable value of }u_i}
\end{split}
$$

Using Bryson's rule scales the maximum value of each term that appears in the cost function to 1. This can be especially important when state/control inputs are in different units, or are of vastly different magnitudes. This allows the weighting for each state and control term to be roughly comparable, which can be helpful as a starting point as the contributions of each state and control input to the cost function can be more easily compared.

Of course, Bryson's rule is just an elementary method of choosing weights. There is no definite systematic method for choosing weights, but there are interesting methods to explore, such as the algebraic approach outlined in [6] in which the time domain design specifications of the system are directly synthesized into the cost function, and therefore the weights are chosen in direct relation to the control objective.


## Conclusion

In the above, we have shown how the discrete time LQR problem is derived and solved for both the finite and infinite horizon case. In doing so, we notice the following advantages and disadvantages of using LQR to solve an optimal control problem:

#### Pros:
1. **Abstracted Design**: The LQR problem allows for the design of the control input to be abstracted from the physical intuition or understanding of the system being controlled. This can be especially useful for complex systems where physical intuition may be difficult to obtain.
2. **Efficiency**: The finite horizonLQR problem can be solved in $O(Kn^3)$ time using dynamic programming, which can significantly faster than brute force or naive methods. The infinite horizon case can be solved in constant time using the DARE. Both methods are also efficient to implement in code.
3. **Optimal Control**: The LQR problem is convex and guarantees that the control input $u^*$ is globally optimal for the cost function at each time step $k$.
4. **Independent Tuning**: The weights $Q$ and $R$ can be tuned independently for each state and control input, which can be easier than tuning parameters in PID, or pole placement design.

#### Cons:
1. **Curse of Dimensionality**: The finite horizon LQR problem suffers from the curse of dimensionality, as with any dynamic programming approach, which can make the problem difficult for high-dimensional systems.
2. **Abstracted Design**: The abstraction provided by the LQR problem can make it difficult to understand how the control input affects the system. As opposed to full state feedback design, the LQR problem does not provide any specific insights regarding how the system responds to control inputs.
3. **Non-Systematic Weight Choice**: There is no systematic method for choosing the weights $Q$ and $R$, and the choice of weights can be non-trivial and may be tedious through trial-and-error.
4. **Model and Cost Limitation** The LQR problem assumes that the system dynamics are linear, and the cost function is quadratic. This limits it to the types of systems it control and the complexity of the cost function. For non-linear models, linearization methods may not provide an accurate enough model for LQR to be effective.
5. **constraint handling**: The LQR formulation does not explicitly handle state and control constraints, which can add complexity to the problem if constraints are present.

In conclusion, LQR can be an efficient and powerful tool when designing control systems that can be modelled as linear systems with quadratic cost functions. It can be especially useful as a starting point when designing control systems for complex systems where physical intuition may be difficult to obtain. However, the same abstraction can also be detrimental as relationship between controller parameters and controller behavior may need to be clearly understood, in which case full state feedback methods are preferred. The non-trivial and non-systematic choice of weights can also limit the usefulness of LQR in practical applications.

## References

[1] [Stanford EE363 Lecture Notes](https://stanford.edu/class/ee363/lectures/dlqr.pdf) by Prof. Stephen Boyd
[2] [Wikipedia: Curse of dimensionality](https://en.wikipedia.org/wiki/Curse_of_dimensionality)
[3] [SciPy DARE documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_discrete_are.html)
[4] [Lecture notes on LQR/LQG controller design](https://staff.uz.zgora.pl/wpaszke/materialy/kss/lqrnotes.pdf) by Prof. Joao P. Hespanha
[5] [MATLAB Linear-Quadratic Regulator (LQR) design](https://www.mathworks.com/help/control/ref/lti.lqr.html)
[6] E. V. Kumar, J. Jerome and K. Srikanth, "Algebraic approach for selecting the weighting matrices of linear quadratic regulator," 2014 International Conference on Green Computing Communication and Electrical Engineering (ICGCCEE), Coimbatore, India, 2014, pp. 1-6, doi: 10.1109/ICGCCEE.2014.6922382.
[7] UBC ELEC441 Lecture Notes by Prof. Maryam Kamgarpour
[8] [Wikipedia: Linear Quadratic Regulator](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator)
[9] UW AA548 Spring 2024 Lecture Notes by Prof. Karen Leung
