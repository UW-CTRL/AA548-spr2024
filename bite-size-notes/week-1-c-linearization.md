# Linearization

**Scope: Linearization in Multivariable Control System** 
- Many control models in real world are mostly made up of multiple states and a inputs. Using one of familiar examples, unicycle model, which is often covered in our course, we will study how linearization can be applied in multivariable control system.

**Objectives**
- To review the definition and the mathematical foundation of linearization.
- To remind how to apply linearization in practical control system problem.
- To look over code snippets(JAX, auto_diff) for linearizing the control system.

## Introduction
Linearization simplifies nonlinear dynamics into linear models, making complex problems easier to analyze, understand and predict in fields like robotics, physics, economics, and ecology. It is key for dealing with real-world challenges, bridging theory with practice, and enhancing system stability and responsiveness, integrating well with various control strategies.


## Preliminaries(100-150)
- **Nonlinear vs Linear control system**:
  |          category             |           Nolinear          |         Linear               |
  |----------------------------   | ----------------------------| -----------------------------|
  |       Characteristics         |   Complex, Curved graph     |  Simple, straight line graph |
  |   In continous-time system    |      <i>ẋ = f(x, u)</i>     |      <i>ẋ = Ax + Bu</i>      |
  |   In discrete-time system     |     <i>x<sub>k+1</sub> = f(x<sub>k</sub>, u<sub>k</sub>)</i> | <i>x<sub>k+1</sub> = Ax<sub>k</sub> + Bu<sub>k</sub></i> |
  
- **Linearization(or linear approximation)** explains local behavior of a nonlinear system by a linear system. We can linearize a sytem by using 'Taylor series expansion'. The linear approximation of a function is the first order Taylor expansion around the point of interest.

- **Talor series expansion**
  - Pick a point 'a
 
   ![alt text](figs/eom_linearization.PNG "Title")
  - For 1-Dimension: $f(x) = f(a) + f'(a)(x-a) \textcolor{red}{\left( + \frac{1}{2!} f''(a)(x-a)^2 + \cdots + \frac{1}{n!} f^{(n)}(a)(x-a)^n + \cdots \right)}$
    
    Here,the red part is Higher order Term $\textcolor{red}{(H.O.T.) ≈ 0}$
  - For N-Dimension: $f(\vec{x}) = f(\vec{a}) + \nabla f(\vec{a})^{T} (\vec{x} - \vec{a})$
 

<p>Gradient ∇f(x)<sup>T</sup> is a matrix:</p>
<p>[ (df<sub>1</sub>/dx<sub>1</sub>) (df<sub>1</sub>/dx<sub>2</sub>) ⋯ (df<sub>1</sub>/dx<sub>n</sub>)<br>
   (df<sub>2</sub>/dx<sub>1</sub>) (df<sub>2</sub>/dx<sub>2</sub>) ⋯ (df<sub>2</sub>/dx<sub>n</sub>)<br>
   ⋮ ⋮ ⋱ ⋮<br>
   (df<sub>n</sub>/dx<sub>1</sub>) (df<sub>n</sub>/dx<sub>2</sub>) ⋯ (df<sub>n</sub>/dx<sub>n</sub>) ]<br>
   = [ ∇f<sub>1</sub>(x)<sup>T</sup> ∇f<sub>2</sub>(x)<sup>T</sup> ⋮ ∇f<sub>n</sub>(x)<sup>T</sup> ]</p>

<p>Jacobian jax = jacobian(…), where the first row is f<sub>1</sub> gradient.</p>

<p>f(x) as a vector function:</p>
<p>[ f<sub>1</sub>(x) f<sub>2</sub>(x) ⋮ f<sub>n</sub>(x) ]</p>

<p>Extended state vector x<sub>~</sub> = [ x u ]</p>

<p>f(x,u) = f(x<sub>0</sub>,u<sub>0</sub>) + ∇<sub>x</sub>f(x<sub>0</sub>,u<sub>0</sub>)<sup>T</sup>(x-x<sub>0</sub>) + ∇<sub>u</sub>f(x<sub>0</sub>,u<sub>0</sub>)<sup>T</sup>(u-u<sub>0</sub>), where (x<sub>0</sub>,u<sub>0</sub>) is constant.</p>

<p>After simplification, for equilibrium point linearization:</p>
<p>f(x,u) = ∇<sub>x</sub>f(x<sub>0</sub>,u<sub>0</sub>)<sup>T</sup>x + ∇<sub>u</sub>f(x<sub>0</sub>,u<sub>0</sub>)<sup>T</sup>u - ∇<sub>x</sub>f(x<sub>0</sub>,u<sub>0</sub>)<sup>T</sup>x<sub>0</sub> - ∇<sub>u</sub>f(x<sub>0</sub>,u<sub>0</sub>)<sup>T</sup>u<sub>0</sub> + f(x<sub>0</sub>,u<sub>0</sub>)</p>

<p>To remove C, consider shifted coordinates: x<sub>~</sub> = x - x<sub>0</sub>, u<sub>~</sub> = u - u<sub>0</sub>.</p>

## Main Body(200-250)

### Numerical Linearization(figure, diagrams or gif)
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
