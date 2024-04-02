# Linearization
 
**Scope: Linearization in Multivariable Control System** 
- Many control models in real world are mostly made up of multiple states and a inputs. Using one of familiar examples, unicycle model, which is often covered in our course, we will study how linearization can be applied in multivariable control system.

**Objectives**
- To understand the definition and the mathematical foundation of linearization.
- To learn how to apply linearization in practical control system problem.
- To look over code snippets(JAX, auto_diff) for linearizing the control system.

## Introduction
Linearization simplifies nonlinear dynamics into linear models, making complex problems easier to analyze, understand and predict in fields like robotics, physics, economics, and ecology. It is key for dealing with real-world challenges, bridging theory with practice, and enhancing system stability and responsiveness, integrating well with various control strategies.


## Preliminaries(100-150)

- **Nonlinear and Linear System**
- **State Space Rrepresentation** 
- **Linearization**(diagrams)

## Main Body(200-250)

### Numerical Linearization(figure, diagrams or gif)
- **Taylor Series Expansion**
- **Jacobian Matrix**
- **Application: Unicycle model**
Consider a pendulum with dynamics `\(\ddot{\theta} + \frac{g}{L}\sin(\theta) = 0\)`. Linearizing around `\(\theta = 0\)` simplifies this to `\(\ddot{\theta} + \frac{g}{L}\theta ≈ 0\)`.

### Snippet codes for Linearization (code)

- **Using JAX:** Simplifies derivative calculation for linearization.

```python
import jax.numpy as jnp
from jax import grad

def f(x): return x**2
df = grad(f)
print(df(2.0))  # Outputs 4.0
```
## Conclusion(50-70)
Linearization, discretization, and automatic differentiation form a powerful trio for control systems design and analysis. This note covers the basics, setting a foundation for further exploration into control strategies and nonlinear dynamics.

## Reference
