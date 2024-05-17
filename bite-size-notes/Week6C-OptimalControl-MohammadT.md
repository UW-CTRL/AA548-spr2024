# Optimization-Based Control Primer

## Scope + Objectives

These notes aim to provide readers with a thorough understanding and review of optimization-based control, spanning fundamental principles, techniques, applications, and computational methods. By delving into this overview, readers can expect to:

1. **Grasp** the fundamental principles underpinning optimization-based control and its significance in diverse engineering applications.
   
2. **Explore** various techniques employed in optimization-based control, including Control Barrier Functions (CBFs), Linear Quadratic Regulator (LQR) Control, and Trajectory Optimization.
   
3. **Learn** how to formulate optimization problems for control systems and solve them using mathematical optimization techniques.
   
4. **Analyze** practical examples demonstrating the application of optimization-based control in real-world scenarios, such as robotic motion planning and system stabilization.
   
5. **Familiarize** themselves with the mathematical formulations and computational methods associated with optimization-based control techniques.

By offering a comprehensive overview, these notes endeavor to equip readers with the knowledge and tools required to effectively understand, implement, and analyze optimization-based control strategies.

## Introduction

Optimization-based control stands as a cornerstone in control engineering, providing a systematic framework for designing control strategies to achieve desired system behaviors while adhering to various constraints. Evolving from classical control theory, optimization-based control has witnessed significant advancements, particularly with the advent of computational techniques and optimization algorithms.

Its relevance extends across a myriad of engineering disciplines, finding applications in robotics, autonomous vehicles, aerospace systems, chemical processes, and more. By addressing complex control problems involving nonlinear dynamics, uncertain environments, and multiple objectives, optimization-based control offers a versatile approach to tackle contemporary engineering challenges.

These notes aim to elucidate how optimization-based control techniques provide a robust toolkit for engineers, facilitating the effective resolution of real-world control problems. Furthermore, understanding optimization-based control concepts lays a solid foundation for exploring advanced control strategies and interdisciplinary research in fields like machine learning and artificial intelligence.

## Preliminaries

Before delving into the intricacies of optimization-based control, it is essential to establish a solid foundation in certain prerequisites to grasp the concepts effectively. Prospective learners are advised to have a basic understanding of control theory, including concepts such as state-space representation, stability analysis, and feedback control. Additionally, familiarity with mathematical optimization techniques, such as convex optimization and gradient-based methods, would be beneficial for comprehending the optimization algorithms employed in optimization-based control. Proficiency in mathematical modeling of dynamical systems and differential equations is also advantageous for formulating control problems and analyzing system dynamics. Moreover, a working knowledge of programming languages commonly used in scientific computing, such as MATLAB, Python, or Julia, is recommended for implementing and simulating control algorithms. By possessing these prerequisites, readers can approach the study of optimization-based control with a solid footing, enabling a deeper understanding of the principles and methodologies elucidated in the subsequent sections.

### Relevant Mathematical Formulas:

- **Control Barrier Functions (CBFs):**

$$ \ \frac{d}{dt}h(x) \leq \alpha(h(x)) + \beta(u) \ $$

- **Principle of Optimality:**

$$ V(x) = \min_u \big[ \{ L(x,u) + V(f(x,u)) \} \big]$$

- **Linear Quadratic Regulator (LQR) Control:**

$$ \ A^T P + PA - PB(R + B^T P B)^{-1} B^T P + Q = 0 \ $$

- **Hamilton Jacobi (HJ) Reachability:**

$$ \ \frac{\partial V}{\partial t} + H(x, \nabla V) = 0 \ $$

- **Trajectory Optimization:**

$$ \ \int_{t_0}^{t_f} L(x(t), u(t)) dt \ $$

## Main Body

### Control Barrier Functions (CBFs)

Control Barrier Functions (CBFs) serve as indispensable tools in ensuring the safety of dynamical systems by enforcing constraints and averting the system from entering unsafe regions. These functions act as safety nets, maintaining the system within predefined boundaries and mitigating the risk of undesirable outcomes.

Mathematically, h(x) must adhere to the inequality:

$$ \ \frac{d}{dt}h(x) \leq \alpha(h(x)) + \beta(u) \ $$

α(⋅) and β(⋅) are functions selected based on the specific application, system dynamics, and desired control or safety requirements.

#### Example:

Consider a scenario involving a quadcopter drone navigating within a constrained indoor environment. Here, the CBF h(x) could be defined to ensure that the distance between the drone and any obstacle remains above a certain threshold. Should the drone approach too close to an obstacle, the CBF would enact corrective actions, adjusting control inputs such as velocity or orientation to steer the drone away and maintain a safe distance. Thus, CBFs effectively serve as safety mechanisms, averting collisions and ensuring the secure operation of autonomous systems.

### Principle of Optimality

The Principle of Optimality states that making the best decisions at each step leads to an optimal overall outcome. In the context of dynamic programming, this principle is captured by the Bellman equation:

$$ V(x) = \min_u \big[ \{ L(x,u) + V(f(x,u)) \} \big]$$

where V(x) is the value function representing the best expected outcome from state x, L(x,u) is the immediate cost of taking action u in state x, and f(x,u) is the transition function that defines the next state given the current state and action.

This equation essentially says that the optimal value of being in state x is the minimum cost achievable by taking any action u in that state, plus the expected value of being in the resulting state f(x,u). By recursively applying this principle, we can compute the optimal value function for the entire system.


#### Example:

Consider a robotic entity traversing through a grid-based environment, facing decisions at each grid cell regarding movement direction. The Bellman equation empowers the robot to discern the optimal action to undertake at each cell, weighing the immediate cost of movement against the anticipated cost of reaching the goal from the subsequent cell. By iteratively unraveling the Bellman equation, the robot can chart the optimal path to its destination, while minimizing the overall cost incurred.

### Linear Quadratic Regulator (LQR) Control

LQR control emerges as a potent technique for stabilizing linear systems characterized by quadratic cost functions. Its primary objective revolves around minimizing a cost function subject to state space of the system where x signifies the system state, u denotes the control input, Q represents a positive definite weighting matrix for the state, and R stands as a positive definite weighting matrix for the control input.

The optimal control law for LQR control is delineated by u* = -Kx wherein K denotes the solution to the associated algebraic Riccati equation:

$$ \ A^T P + PA - PB(R + B^T P B)^{-1} B^T P + Q = 0 \ $$
$$ \ K = (R + B^T P B)^{-1} B^T P A \ $$

Once K is ascertained, it can be wielded to compute the optimal control input u* for any bestowed state x, thereby ensuring stability and optimal performance.

#### Example:

Consider an inverted pendulum system tasked with maintaining the pendulum in an upright position. LQR control calculates the optimal control input (typically a force) requisite to stabilize the pendulum around the upright orientation. By modulating the control input predicated on the prevailing state of the pendulum, LQR control adeptly upholds stability even amidst perturbations.

### HJ Reachability

HJ reachability represents a mathematical framework instrumental in analyzing safe regions within dynamical systems constrained by limits. This framework entails solving the Hamilton-Jacobi partial differential equation (PDE):

$$ \ \frac{\partial V}{\partial t} + H(x, \nabla V) = 0 \ $$

Here, V embodies the value function denoting the optimal cost-to-go from a conferred state, t designates time, x signifies the state vector, and nabla V encapsulates the gradient of V with respect to x. The function 
$$ \(H(x, \nabla V)\) $$

The Hamiltonian, often referred to as H(x,∇V) represents the system's dynamics.

By solving the Hamilton-Jacobi partial differential equation (HJ PDE), one can determine the value function V(x), which provides insights into the reachable set of the system. This set includes all states that can be reached from a given initial state while satisfying specific safety constraints.

The reachable set arises from the level sets of the value function V, marking regions in the state space where the system can operate safely without violating constraints.

#### Example:

Envisage an autonomous vehicle navigating through a congested environment. HJ reachability analysis can be leveraged to compute the vehicle's reachable set, factoring in obstacles and safety constraints such as minimum distance stipulations. By discerning the reachable set, one can discern safe trajectories for the vehicle to traverse, ensuring collision-free and constraint-compliant locomotion.

### Trajectory Optimization

Trajectory optimization emerges as a potent technique for identifying the optimal path for a system to traverse over time, meticulously considering its dynamics and constraints. Prevalent in robotics, aerospace, and autonomous systems, trajectory optimization engenders the formulation of smooth and efficient motion trajectories.

Mathematically, trajectory optimization endeavors to minimize a cost function of the form:

$$ \ \int_{t_0}^{t_f} L(x(t), u(t)) dt \ $$

Trajectory optimization strives to pinpoint the optimal trajectory that minimizes the cost function while upholding the system's dynamics and constraints. This necessitates solving a formidable optimization problem, often replete with nonlinear dynamics, non-convex cost functions, and intricate constraints.

#### Example:

Ponder a quadcopter drone tasked with navigating through a cluttered environment to a designated target location while evading obstacles. Trajectory optimization can be harnessed to chart a seamless and collision-free path for the drone by optimizing its trajectory over time. By cognizantly considering the drone's dynamics, environmental constraints, and mission objectives, trajectory optimization guarantees the drone's safe and efficient navigation to its destination.

## Conclusion

In conclusion, optimization-based control offers a robust framework for designing and analyzing control systems across a wide range of engineering applications. Each concept discussed, from Control Barrier Functions (CBFs) ensuring system safety to Trajectory Optimization identifying optimal paths considering system dynamics and constraints, contributes to a comprehensive understanding of control strategies. By leveraging these principles, engineers can develop sophisticated control systems capable of addressing complex challenges while optimizing performance. The versatility of optimization-based control extends to various fields, including robotics, aerospace, and autonomous systems, where precise control and efficient decision-making are paramount. Through the integration of mathematical formulations and computational methods, optimization-based control provides engineers with powerful tools to model, analyze, and optimize complex systems in real-time, enabling the development of innovative solutions that drive technological advancements and enhance efficiency across industries.

Furthermore, the interdisciplinary nature of optimization-based control fosters collaboration between control engineers, mathematicians, computer scientists, and domain experts, facilitating the integration of diverse perspectives to tackle multifaceted challenges. As technology continues to evolve, optimization-based control remains at the forefront of innovation, enabling the development of intelligent systems capable of autonomous decision-making and adaptive behavior. By advancing our understanding of optimization-based control and its applications, we empower future generations of engineers to push the boundaries of what is possible, paving the way for transformative advancements in engineering and beyond.

## References

[1] L. T. Ashchepkov, D. V. Dolgy, T. Kim, and R. P. Agarwal, *Optimal Control*. Cham: Springer, 2022.

[2] H. Purnawan, Mardlijah, and E. B. Purwanto, “Design of linear quadratic regulator (LQR) control system for flight stability of LSU-05,” *Journal of Physics: Conference Series*, vol. 890, p. 012056, Sep. 2017. doi:10.1088/1742-6596/890/1/012056

[3] R. K. Sarin, “Value function,” *Wiley StatsRef: Statistics Reference Online*, Sep. 2014. doi:10.1002/9781118445112.stat03615

[4] “The principle of optimality,” *Dynamic Programming*, pp. 377–400, Sep. 2010. doi:10.1201/ebk0824740993-17

[5] W. Xiao, C. G. Cassandras, and C. Belta, “Control Barrier functions,” *Safe Autonomy with Control Barrier Functions*, pp. 7–18, 2023. doi:10.1007/978-3-031-27576-0_2

[6] Optimal Control and Estimation by Robert F. Stengel.

[7] Optimal Control Theory: An Introduction by Donald E. Kirk.
