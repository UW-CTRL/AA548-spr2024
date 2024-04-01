# Linearization and Discretization in Multi-Variable Control System: An Introduction
 
**Purpose:** 
- To study the transformative process of linearizing complex continuous-time dynamics and their conversion into discrete-time models within multi-variable control systems. 
- To study the significant role of automatic differentiation in streamlining the linearization process.

**Objectives:**
- Clarify linearization's role in control systems analysis.
- Explore discretization techniques for adapting continuous-time dynamics to digital control applications.
- Find how to use the automatic differentiation tools like JAX in control system linearization eifficiently.

## Introduction

Linearization and discretization are foundational to control systems engineering, enabling simplified analysis and design of digital control systems from complex nonlinear dynamics. Automatic differentiation emerges as a key player in enhancing the efficiency and precision of this process.

## Preliminaries

- **Linearization:** Approximation of nonlinear systems as linear around an equilibrium. It simplifies the system to `f(x) ≈ f(a) + f'(a) · (x-a)` for analysis and control.

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
