# Intro to Convex Optimization

## Scope and Objective
These notes will serve as a brief introduction to Convex Optimization within the field of Controls. Since this is also my introduction to optimization, it will cover more of the basics of convex optimization and what distinguishes it from other optimization problems.  

## Introduction to optimization
Optimization is a topic that is used in mathematics, computer science, engineering, and economics. It is a method of finding the best or optimal solution to a defined problem or system. Optimization problems can occur on an incredibly broad scale, going from optimizing everyday things like coffee consumption based on your relaxed constraints in your everyday life to optimizing aircraft performance based on 500 strict mechanical, aerodynamic, and financial constraints. Almost any system can be made into an optimization problem, and due to advancements in mathematical computing, most problems can be solved using a standard method and code base.

In controls, optimization allows us not just to choose a feasible control input for our system, or even just a good control for our system, but to actually find the best control input for our system based on our constraints, desired behavior, and current state. This can be a powerful tool. However, there are some limiting factors. While the mathematical framework for setting up an optimization problem can be used to define most problems, actually solving a problem can yield very crude solutions. The key to solving an optimization problem simply and convincingly is to    

## Preliminaries  
### Standard form of an optimization problem 
minimize    $$f_0 (x)$$  
subject to  $$f_i (x) \leq 0, i = 1, ...., m$$ 
             $$g_i (x) = 0, i = 1, ...., p$$
* $x \in \pmb{R^n}$ is (vector) variable to be chosen (n scaler variables $x_1, ....., x_n$)
* $f_0$ is the **objective function** to be minimized
* $f_1, ..., f_m$ are the **inequality constraint functions**
* $g_1, ..., g_p$ are the **equality constraint functions**  
[1]

### Standard form of convex optimization problem
minimize    $$f_0 (x)$$  
subject to  $$f_i (x) \leq 0, i = 1, ...., m$$ 
             $$Ax = b$$
* $x \in \pmb{R^n}$
* equality constraints are linear
* $f_1, ... f_m$ are **Convex**: for $\theta \in$ [0,1],  
$$f_i(\theta x+ (1- \theta)y \leq \theta f_i(x) + (1-\theta)f_i(y))$$  
i.e., $f_i$ have nonnegative (upward) curvature
[1]

### Affine set


### Convex Set

### discrete vs. continuous



### example in python


[1] https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf 
[2] Convex Optimization Stephen Boyd Lieven Vandenberghe https://web.stanford.edu/~boyd/cvxbook/bv_cvxslides.pdf
