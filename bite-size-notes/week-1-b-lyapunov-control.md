## Lyapunov Direct Method for Stability Analysis

### Scope
Through these notes, you will learn the importance of Lyapunov stability including its definition and application in assessing system stability. Furthermore, we will cover the conditions that these functions must satisfy for a system to be considered Lyapunov stable. An example will be provided to gain an understanding of how Lyapunov stability is applied in practice, illustrated through diagrams and code snippets.

### Motivation
In industrial processes, you have to control many variables such as a position of an object to the temperature of a process. Ideally, you want all process variables to acheive a stability at a desired level. There are many concepts of stability:
* **Lyapunov stable**
* Asymptotically stable
* Exopnentially stable
* BIBO stable
* Internally stable
* etc.


Lyapunov is a powerful tool for assessing the stability of equilibrium points without solving the system's differential equations directly. It's widely used in control systems, robotics, and anywhere system stability is critical.

### Definitions and Notation
__Lyapunov Stability__ - an equilibrium point is stable if all system trajectories starting at nearby points stay nearby

Lyapunov functions must satisfy the following statements and conditions to be Lyapunov stable:

**Lyapunov Functions (no control input)** \
&nbsp;&nbsp; Given a system $\dot{x} = f(x)$ and some region $D \subset R^n$ with $0 \in D$, then if there exists a continuous-differentiable function $V(x)$ such that:
* $V(0) = 0 \rightarrow$ the function at the zero-state is zero
* $V(x) > 0, \forall x \in D \backslash \ 0 \rightarrow$ the function must be greater than zero for all x in the domain except for the zero state
* $\dot{V}(x(t)) = \nabla V(x)^T f(x) \leq 0 \rightarrow$ The time derivative of the Lyapunov function V, as a function of x must be less than or equal to zero

Intuitively, the Lyapunov function " $V(x)$ " is similar to the "energy" of the system, this is nontrivial to find for arbitrary nonlinear systems

### Example: Damped Pendulum
Equation of motion of the damped simple pendulum:

$$ \ddot{\theta} = -\frac {g}{l} \sin{\theta} - k \dot{\theta} $$

State vector:

$$ x = 
\begin{bmatrix}
\theta \\
\dot{\theta}
\end{bmatrix}
$$

Lyapunov Function (total system energy, kinetic + potential):

$$ V(x) = mgl(1 - \cos{\theta}) + \frac{1}{2} m l^2 \dot{\theta^2} $$

Determine if $V(x)$ is a valid Lyapunov function from the conditions above:

&nbsp;&nbsp; **1. $V(0) = 0$** \
&nbsp;&nbsp; Set the state vector $x$ to 0:

$$ x = 
\begin{bmatrix}
\theta \\
\dot{\theta}
\end{bmatrix} =
\begin{bmatrix}
0 \\
0
\end{bmatrix}
$$

&nbsp;&nbsp; Plug into Lyapunov function $V(x)$:

$$ V (0) = mgl(1 - \cos{0} ) + \frac{1}{2} m l^2 0^2 $$

$$ V (0) = 0 $$

&nbsp;&nbsp; The first condition is **satisfied**

&nbsp;&nbsp; **2. $V(x) > 0, \forall x \in D \backslash 0$**

&nbsp;&nbsp; $V(x) = mgl(1 - \cos{\theta}) + \frac{1}{2} m l^2 \dot{\theta^2}$

* Cosine term is always $\leq 1$, thus $(1 - \cos{\theta}) \geq 0$
* Mass of pendulum, $m$, is postitive
* Acceleration due to gravity, $g$, is positive
* Length of pendulum, $l$, is postitive
* Anglular velocity, $\dot{\theta}$, is squared which will be positive

&nbsp;&nbsp; The second condition is **satisfied** 
