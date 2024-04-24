# Linearization

**Scope: Linearization in Multivariable Control System** 
- Many control models in real world are mostly made up of multiple inputs and outputs. Using simple examples, pendulum, which is often covered in classes, we will study how linearization can be applied in multivariable control system.

**Objectives**
- To review the definition and the mathematical foundation of linearization.
- To remind how to apply linearization using the pendulum example.
- To look over code snippets such as JAX for linearizing the control system.

## Introduction
Linearization simplifies nonlinear dynamics into linear models, making complex problems easier to analyze, understand and predict in fields like robotics, physics, economics, and ecology. It is key for dealing with real-world challenges, bridging theory with practice, and enhancing system stability and responsiveness, integrating well with various control strategies.


## Preliminaries
- **Nonlinear vs Linear control system**:
  |          category             |           Nolinear          |         Linear               |
  |----------------------------   | ----------------------------| -----------------------------|
  |       Characteristics         |   Complex, Curved graph     |  Simple, straight line graph |
  |   In continous-time system    |      <i>ẋ = f(x, u)</i>     |      <i>ẋ = Ax + Bu</i>      |
  |   In discrete-time system     |     <i>x<sub>k+1</sub> = f(x<sub>k</sub>, u<sub>k</sub>)</i> | <i>x<sub>k+1</sub> = Ax<sub>k</sub> + Bu<sub>k</sub></i> |
  | Equilibrium point | Many points | Only one point |
  
- **Linearization(or linear approximation)** explains local behavior of a nonlinear system by a linear system. We can linearize a sytem by using 'Taylor series expansion'. The linear approximation of a function is the first order Taylor expansion around the point of interest.

<p align="center">
  <img width="400" height="200" src=https://mathinsight.org/media/image/image/tangent_line_graph.png>
</p>

- **Talor series expansion**
  - Pick a point 'a'
  - For 1-Dimension: $f(x) = f(a) + f'(a)(x-a) \textcolor{red}{\left( + \frac{1}{2!} f''(a)(x-a)^2 + \cdots + \frac{1}{n!} f^{(n)}(a)(x-a)^n + \cdots \right)}$
    
    Here, the red part is Higher order Term $\textcolor{red}{(H.O.T.) ≈ 0}$
  - For N-Dimension: $f(\vec{x}) = f(\vec{a}) + \nabla f(a)^{T} (\vec{x} - \vec{a})$

The transpose of the gradient of \(f(x)\) can be represented as:

$$
\nabla f(x)^T = \left[ \begin{array}{cccc}
\frac{\partial f_1}{\partial x_1} & \frac{\partial f_1}{\partial x_2} & \cdots & \frac{\partial f_1}{\partial x_n} \\
\frac{\partial f_2}{\partial x_1} & \frac{\partial f_2}{\partial x_2} & \cdots & \frac{\partial f_2}{\partial x_n} \\
\vdots & \vdots & \ddots & \vdots \\
\frac{\partial f_n}{\partial x_1} & \frac{\partial f_n}{\partial x_2} & \cdots & \frac{\partial f_n}{\partial x_n}
\end{array} \right]
= \left[ \begin{array}{c}
\nabla f_1(x)^T \\
\nabla f_2(x)^T \\
\vdots \\
\nabla f_n(x)^T
\end{array} \right]
, \ where \ f(x) = \left[ \begin{array}{c}
f_1(x)\\
f_2(x)\\
\vdots \\
f_n(x)
\end{array} \right]
$$

The extended state vector $\tilde{x}\$ is defined as:

$$
\tilde{x} = \left[ \begin{array}{c}
x\\
u\\
\end{array} \right]
$$

The function $f(x, u)$ and its linear approximation around the operating point $\(x_0, u_0)\$ are given by:

$$
f(x,u) = f(x_0,u_0) + \nabla_x f(x_0,u_0)^T (x - x_0) + \nabla_u f(x_0,u_0)^T (u - u_0), \ where \ (x_0, u_0) \ is \ constant.
$$

The simplified linear model can be expressed as:

$$
f(x,u) = \textcolor{green}{\nabla_x f(x_0,u_0)^T} x + \textcolor{blue}{\nabla_u f(x_0,u_0)^T}u - \textcolor{red}{\nabla_x f(x_0,u_0)^T x_0 - \nabla_u f(x_0,u_0)^T u_0 + f(x_0,u_0)}
$$

$$
f(x,u) = \textcolor{green}{A} x + \textcolor{blue}{B}u + \textcolor{red}{C}    \quad \text{($C$ is constant)}
$$

## Main Body

### Let's apply Linearization to Pendulum example!

<p align="center">
  <img width="400" height="300" src=https://i.stack.imgur.com/RrmTf.jpg>
</p>

 #### 1. First, we will solve Pendulum problem in **mathematical way**, which we studied in Preliminary section.

The given pendulum equation is:

$$
\dot{x} = \left[ \begin{array}{c}
\dot{x}_1 \\
\dot{x}_2
\end{array} \right]
 = \left[ \begin{array}{c}
x_2 \\
 \-\omega^2 \sin(x_1) - r x_2
\end{array} \right]
$$

- $x_1$: Angular position of the pendulum, measured in radians. It indicates how far and in which direction the pendulum has swung from the vertical.
- $x_2$: Angular velocity of the pendulum, showing the rate at which the angular position $x_1$ changes. It tells us how fast and in which direction (clockwise or counterclockwise) the pendulum is moving.
- $\dot{x_1} = x_2$: This states that the rate of change of the angular position is equal to the angular velocity, linking the position to its derivative, velocity.
- $\dot{x_2}$: Angular acceleration of the pendulum, influenced by gravitational and damping forces.
   - $-\omega^2 sin(x_1)$: The gravitational component, where $\omega^2 = g/L$ is influenced by gravity $g$ and the pendulum length $L$. The sine function captures the restoring force, which depends on the pendulum's angle.
   - $-rx_2$: The damping force, where $r$ is a coefficient representing energy loss due to factors like air resistance and friction, acting in opposition to the velocity.
  
Here, the control input u is autonomous force(no additional external control input applied), so we will consider u = [0, 0]ᵀ

The equilibrium points $f(x) = 0$ lead to 

$$
\bar{x} = x_0 = \left[ \begin{array}{c}
0 \\
0 
\end{array} \right]
$$

Let's linearize around the equilibrium points. The linearization around an equilibrium point $x_0$ can be expressed using the Taylor series as follows:

$$
f(x) ≈ ∇_x f(x_0)^T (x - x_0) + f(x_0)
$$

For the pendulum system, the Jacobian $∇_x f(x_0)^T$ around the equilibrium point $x_0 = [0, 0]^T$ is:

$$
∇_x f(x_0) = \left[ \begin{array}{cc}
\frac{\partial \dot{x}_1}{\partial x_1} & \frac{\partial \dot{x}_1}{\partial x_2} \\
\frac{\partial \dot{x}_2}{\partial x_1} & \frac{\partial \dot{x}_2}{\partial x_2}
\end{array} \right]
 = \left[ \begin{array}{cc}
0 & 1 \\
-\omega^2 \cos(x_1) & -r
\end{array} \right]
 = \left[ \begin{array}{cc}
0 & 1 \\
-\omega^2 & -r
\end{array} \right]
$$

Thus, the linear approximation of $f(x)$ near $x_0 = [0, 0]^T$ would be:

$$
f(x) ≈ ∇_x f(x_0)^Tx - ∇_x f(x_0)^T x_0 + f(x_0)
$$

$$
f(x) ≈ \textcolor{green}{\left[ \begin{array}{cc}
0 & 1 \\
-\omega^2 & -r
\end{array} \right]} * 
\left[ \begin{array}{c}
x_1 \\
x_2
\end{array} \right] - 
\textcolor{red}{\left[ \begin{array}{cc}
0 & 1 \\
-\omega^2 & -r
\end{array} \right] *
\left[ \begin{array}{c}
0\\
0
\end{array} \right] +
\left[ \begin{array}{c}
0\\
0
\end{array} \right]}
$$

$$
f(x) ≈ \textcolor{green}{A} x + \textcolor{red}{C}
$$

In this pendulum example, the linearization process around the equilibrium point simplifies the equations of motion from trigonometric expressions to linear relationships.

<p align="center">
  <img width="400" height="400" src=https://upload.wikimedia.org/wikipedia/commons/2/24/Oscillating_pendulum.gif>
</p>

  
#### 2. Now, we will use **codes** for the Pendulum example. Look into which **libraries** can efficiently perform for **linearization**:

(i) **JAX:** can automatically compute gradients of functions using forward-mode differentiation 'jax.jacfwd' and reverse-mode differentiation 'jax.grad'

(ii) **TensorFlow** excels in scalable deep learning, offering 'tf.GradientTape' for automatic differentiation and optimizations via 'tf.function'. It uniquely integrates with tools like TensorBoard for visualization and TensorFlow Lite for mobile, standing out for deployment capabilities.

(iii) **PyTorch** is useful for research with its dynamic computation graphs and intuitive, Pythonic syntax. It's particularly user-friendly for rapid prototyping and iteration, differentiating itself with a more accessible approach to graph construction and debugging.

(iv) **Auto_diff (Pseudocode)** conceptualizes automatic differentiation, simplifying the understanding of gradient and Jacobian computation without tying to a specific API(Application Programming Interface), highlighting the core principles of auto differentiation technologies.

- Here are the **packages/modules** of each library as follows:

```
# JAX
import jax.numpy as jnp
from jax import grad, jacfwd

def pendulum_dynamics(x, omega=1.0, r=0.1):
    x1, x2 = x
    dx1dt = x2
    dx2dt = -omega**2 * jnp.sin(x1) - r * x2
    return jnp.array([dx1dt, dx2dt])

def linearize_system(x, omega=1.0, r=0.1): 
    jacobian_func = jacfwd(pendulum_dynamics, argnums=0)
    jacobian_at_x = jacobian_func(x, omega, r)   
    return jacobian_at_x

x_example = jnp.array([0.0, 0.0])  # Equilibrium point
omega = 1.0  # Natural frequency
r = 0.1      # Damping coefficient
jacobian_at_equilibrium = linearize_system(x_example, omega, r)
print("Jacobian matrix at the equilibrium point:", jacobian_at_equilibrium)
```
```
# TensorFlow
import tensorflow as tf
@tf.function
def pendulum_tf(x):
    x1, x2 = x[0], x[1]
    return tf.stack([x2, -omega**2 * tf.sin(x1) - r * x2])
x_tf = tf.Variable([0.0, 0.0])
with tf.GradientTape() as tape:
    tape.watch(x_tf)
    y_tf = pendulum_tf(x_tf)
A_tf = tape.jacobian(y_tf, x_tf)
```
```
# PyTorch
import torch
def pendulum_torch(x):
    x1, x2 = x.unbind()
    return torch.stack([x2, -omega**2 * torch.sin(x1) - r * x2])
x_torch = torch.tensor([0.0, 0.0], requires_grad=True)
y_torch = pendulum_torch(x_torch)
A_torch = torch.autograd.functional.jacobian(pendulum_torch, x_torch)
```
```
# Auto_diff (Pseudocode)
def pendulum_auto_diff(x, omega=1.0, r=0.1):
    x1, x2 = x
    return [x2, -omega**2 * auto_diff.sin(x1) - r*x2]
x0_auto_diff = [0.0, 0.0]
A_auto_diff = auto_diff.compute_jacobian(pendulum_auto_diff, x0_auto_diff)
```

## Conclusion
Linearization simplifies complex systems into easier linear models, crucial for better understanding and control in many areas. This guide shows how to use linearization on a pendulum, with clear math explanations and code examples in JAX, TensorFlow, and PyTorch, showcasing how these tools make analyzing systems straightforward.

## Reference
[1] Leung, Karen. “Linear Multivariable Control” Lecture, University of Washington, Seattle, 2024-03-27.

[2] Taghvaei, Amir. “Nonlinear Control Systems” Lecture, University of Washington, Seattle, 2023-10-05.

[3] D.Lazard. "Nonliear system." *Wikipedia*. Last Modification: 2024-03-08. Last Accessed:2024-04-03. [URL](https://en.wikipedia.org/w/index.php?title=Nonlinear_system&action=history).

[4] Nonliear Linear System by Hassan K. Khalil, 3rd Edition

[5] *JAX 101: JAX Basics*. JAX Documentation.[https://jax.readthedocs.io/en/latest/jax-101/01-jax-basics.html](https://jax.readthedocs.io/en/latest/jax-101/01-jax-basics.html). Last accessed: 2024-04-03.

[6] Introduction to gradients and automatic differentiation. https://www.tensorflow.org/guide/autodiff. Last accessed:2024-04-03.

[7] JACOBIANS, HESSIANS, HVP, VHP, AND MORE: COMPOSING FUNCTORCH TRANSFORMS. https://pytorch.org/functorch/stable/notebooks/jacobians_hessians.html#jacobians-hessians-hvp-vhp-and-more-composing-functorch-transforms. Last accessed:2024-04-03.
