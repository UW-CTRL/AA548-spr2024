# Introduction to Free Final Time

Free final time refers to leaving the terminal time as a optimizable variable in an optimal control problem. This topic is well-understood [for linear? systems] [not so well understood for higher order systems or nonlinear]

## Scope

These notes will focus on introducing free final time on a simple system with theory and code, explain difficulties with implementation on higher-order systems, and provide resources to explore the subject further.

The reader is assumed to have a entry-level understanding of optimal control theory such as optimality, minimization/maximization, numerical methods, etc, etc.

## Prerequisites

Free final time [explain why we need these theorems (we want to find extrema functions, boundary conditions, and transversality conditions)]

Some mathematical theorems used in the free final time theorem are the calculus of variations and Pontryagin's Minimum Principle. A summarization of the theorems and their impacts to the problem are detailed below.

### Calculus of variations

[short sentence about what the CoV is.]

The purpose behind using the calculus of variations for free final time is to find a extrema function: analogous to finding the critical points by setting the derivative of a function equal to zero.

[explain what functionals are] Intuitively, one can think of them as "functions of functions".
[explain extrema finding]
[explain similarities to optimization] This shares similarities to the first derivative test

One of the most important results from the calculus of variations is the Euler–Lagrange equation. [1]

$$
\frac{\partial L}{\partial f} - \frac{d}{dx} \frac{\partial L}{\partial f'} = 0
$$

<details close>
<summary> <em>Derivation of the Euler–Lagrange equation.</em> </summary>
<br>
Well, you asked for it!
</details>

More formally:
[formal defintion and equations]

[point about ]

### Pontryagin's Minimum Principle

A formal definition
[formal defintion and equations]

## Main Body

Now let's look at a standard optimal control problem, but this time with terminal time being an optimization variable.

### Theory

The cost can be written as a function of the state, the control, and the terminal time.

$$
J\[x(t),u(t),T\]
$$


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

Given the complexity of sequential convex programming both as a method and 
An understanding of what kinds of problems should be solved by sequential convex programming. An elementary understanding of the theory behind the topic. Concrete ways to start with sequential convex programming. References to learn more.

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
