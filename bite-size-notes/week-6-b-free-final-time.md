
# Introduction to Free Final Time

The problem of free final time refers to leaving the terminal time as a optimizable variable in an optimal control problem. This topic is well-understood for linear systems where analytical solutions are commonly available, but requires numerical methods to handle the increasing complexity of nonlinear dynamics and/or higher-order linear systems.

<!--
Add a photo here that helps reader visualize free final time: trajectory and alternate trajectories would be interesting.
-->

Free final time is an important topic in aerospace engineering, as fuel costs are inextricably tied to time of flight and many astronautical missions rely on a window of opportunity. Having the flexibility of directly optimizing time also aids in the robustness of any system that might experience changing dynamical situations commonly encountered in aerospace.

## Scope

These notes will focus on introducing free final time on a simple linear system with theory and code, explain difficulties with implementation on nonlinear and higher-order linear systems, and provide resources to explore the subject further.

Learning objectives include...
* Discovering the difference between *functions* and *functionals*.
* Understanding how the Euler-Lagrange equation is used to determine extremizing functions within the cost functional.
* Determining optimal control with the Hamiltonian under Pontryagin's Minimum Principle.
* Constructing a free final time problem using a simple linear system.
* Coding the same free final time problem in Python.

The reader is expected to have prior knowledge of undergraduate-level controls and topics such as basic optimization and the Lagrangian.

## Prerequisites
Why do we need new math to solve free final time? For free final time problem, terminal time $T$ is no longer fixed. This means

[explain why we need these theorems (we want to find extrema functions, boundary conditions, and transversality conditions)]

The mathematical theorems used for the analytical solution to free final time are the calculus of variations and Pontryagin's Minimum Principle. A summarization of the equations/theorems and their impacts to the problem are detailed below.

### Calculus of variations
Functionals are mappings from a set of functions to the set of real numbers. Intuitively, one can think of them as "functions of functions", as functions are mappings from a set of numbers to another set of numbers. The calculus of variations uses small changes of functions and functionals, known as variations, to determine the extrema of functionals.

The purpose behind using the calculus of variations for free final time is to find a extrema function that minimizes/maximizes a functional: analogous to finding the critical points by setting the derivative of a function equal to zero.

Our optimization cost is then represented by a cost functional $J[x(t),u(t),t]$ rather than a cost function $J\big(x(t),u(t),t\big)$. More precisely,

$$J[x(t),u(t),t] = \int_0^T L\big(x(t), u(t), t\big) \,dt$$

where $L$ is the Lagrangian and $T$ is the terminal time.



One of the most important results from the calculus of variations is the Euler–Lagrange equation. [1]

$$
\frac{\partial L}{\partial f} - \frac{d}{dx} \frac{\partial L}{\partial f'} = 0
$$

<details close>
<summary> <em>Derivation of the Euler–Lagrange equation.</em> </summary>
<br>
<div markdown="1">

Consider the functional
$$
J[y] =  \int_{a}^{b} L(x, y(x), y'(x)) \,dx
$$
where $a,b$ are constants, 

</div>
</details>

### Pontryagin's Minimum Principle

A formal definition
[formal definition and equations]

## Main Body

Now let's look at a standard optimal control problem, but this time with terminal time being an optimization variable.

### Theory
The cost can be written as a function of the state, the control, and the terminal time.

$$
J\[x(t),u(t),T\]
$$

#### Common methods for constraints


### Example
Let's revisit a common example: the 2D unicycle. All variables here are functions of time.
* state variable $z$
  * positions $x$, $y$
  * velocity $v$
  * angle $\theta$
* control input $u$
  * acceleration $a$
  * angular velocity $\omega$

Then the state and dynamics vectors, still functions of time, are:

$$
z = \begin{bmatrix}
  x \\
  y \\
  v \\
  \theta
\end{bmatrix}, \qquad
\text{and}
\qquad \dot{z} = \begin{bmatrix}
  v \cos \theta \\
  v \sin \theta \\
  a \\
  \omega
\end{bmatrix}
$$

The minimization problem can be written as:



### Code


## Conclusion

In these notes, we've hopefully provided a base understanding of free final time in an analytical sense and a computational sense. We've also highlighted how this concept is more difficult to implement in more complex systems without use of numerical methods. With these notes, it's hoped that the reader gains a more intuitive understanding of free final time.

To continue exploring this topic, references that were used to craft these notes as well as papers that utilize modern numerical techniques to innovate on this subject are provided in the next section. References [1] through [#] were used for the notes, references [#] through [#] are papers that go over numerical methods that solve free final time.

## References
[1] 

<!--
Scope + objectives: What should one expect to learn from reading these notes? Think about what is the purpose of these notes, what questions will be answered, and what will the reader gain from reading these notes.

Introduction: Introduce the topic. Include things like how it is relevant to controls, in what situation would a reader find this useful, is there some historical context that's important to know, is this related to other topics?

Preliminaries: Set up the mathematics needed to describe the topic/concept. Introduce definitions, notation, theorems, etc.

Main body: Describe the topic/concept at a suitable depth given your selected scope and objectives. Consider including:
Figures and diagrams
Code snippets
Video/gif
Derivations
Concrete examples (e.g., with a specific system and with actual numbers)
Anything else that you think will be helpful!

Conclusion: Summarize the key takeaways, and also mention what was not covered in these notes, and perhaps what are references the reader read to learn more.
References: Please include references to any material you cite, or used in creating these notes
-->
