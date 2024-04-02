# Linearization
 
**Scope: Linearization in Multivariable Control System** 
- Many control models in real world are mostly made up of multiple states and a inputs. Using one of familiar examples, unicycle model, which is often covered in our course, we will study how linearization can be applied in multivariable control system.

**Objectives**
- To understand the definition and the mathematical foundation of linearization.
- To learn how to apply linearization in practical control system problem.
- To look over code snippets(JAX, auto_diff) for linearizing the control system.

## Introduction
Linearization simplifies nonlinear dynamics into linear models, making complex problems easier to analyze, understand and predict in fields like robotics, physics, economics, and ecology. It is key for dealing with real-world challenges, bridging theory with practice, and enhancing system stability and responsiveness, integrating well with various control strategies.


## Preliminaries

- **Nonlinear and Linear System** 
  - 
- **Linearization** is a method for assessing the local stability of an equilibrium point of a system of nonlinear differential equations or discrete dynamical systems. It simplifies the system to `f(x) ≈ f(a) + f'(a) · (x-a)` for analysis and control.

- **Discretization:** Conversion of continuous-time dynamics (differential equations) into discrete-time models (difference equations) essential for digital processing.

- **Automatic Differentiation (Autodiff):** A computational technique for exact derivative calculation, crucial for linearization processes.

## Main Body

### Linearizing Continuous-Time Dynamics

Consider a pendulum with dynamics `\(\ddot{\theta} + \frac{g}{L}\sin(\theta) = 0\)`. Linearizing around `\(\theta = 0\)` simplifies this to `\(\ddot{\theta} + \frac{g}{L}\theta ≈ 0\)`.

### Discretizing Dynamics

**Euler Integration Method:** For state `\(x\)` with dynamics `\(\dot{x} = f(x)\)`, discretization with time step `\(\Delta t\)` is `\(x_{t+1} = x_t + \Delta t \cdot f(x_t)\)`.

### Automatic Differentiation for Linearization

**Using JAX:** Simplifies derivative calculation for linearization.

```python
import jax.numpy as jnp
from jax import grad

def f(x): return x**2
df = grad(f)
print(df(2.0))  # Outputs 4.0
```
## Conclusion
Linearization, discretization, and automatic differentiation form a powerful trio for control systems design and analysis. This note covers the basics, setting a foundation for further exploration into control strategies and nonlinear dynamics.
