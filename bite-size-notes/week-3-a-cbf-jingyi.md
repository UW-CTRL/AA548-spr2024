## Table of Contents
- [Objectives](#objectives)
- [Preliminaries](#preliminaries)
- [Control Barrier Function Introduction](#control-barrier-function-introduction)
- [Applications](#applications)
- [Comparison with Control Lyapunov Function](#comparison-with-control-lyapunov-function)
- [Combine CBF Constraints with CLF Constraints](#combine-cbf-constraints-with-clf-constraints)
- [Conclusion](#conclusion)

## Objectives
The purpose of this section is to introduce Control Barrier Functions (CBFs), their significance in ensuring safety in dynamical systems, and the mathematical conditions that define them. Readers will gain a basic understanding of how CBFs contribute to controller synthesis for safety-critical systems.
## Preliminaries
#### Control Invariant Set
A control invariant set is a subset of the state space within which the system can be kept by an appropriate choice of control inputs. Formally, a set $C$ is control invariant if for every state $x$ in $C$, there exists a control input $u$ such that the system dynamics $f(x,u)$ keep the state within $C$. This ensures that once the system enters the safe set $C$, it can remain there indefinitely by selecting appropriate control actions.
#### Control Lyapunov Function
A control Lyapunov function is a Lyapunov-like function that can be used to design stabilizing controllers for a dynamical system. A CLF $V(x)$ is a positive definite, radially unbounded function of the system state $x$, such that there exists a control input $u$ that makes the time derivative of $V(x)$ along the system dynamics $f(x,u)$ negative definite. This ensures that the system state will converge to the desired equilibrium point or origin asymptotically. The CLF framework allows for systematic synthesis of stabilizing controllers by solving optimization problems that incorporate the CLF condition.
## Control Barrier Function Introduction
[Barrier functions](https://en.wikipedia.org/wiki/Barrier_function) are Lynapunov-like functions, which have been used for optimization and multi-objective control[^1]. CBFs extend the concept of barrier functions, which enables controller synthesis for safety requirements specified by forward invarianse of a set using Lyapunov-like condition[^2]. 
#### Motivation
In safety-critical applications, it is imperative to guarantee that a dynamic system remains within a designated safe region, known as a [control invariant set](#control-invariant-set), throughout its operation. CBFs offer a robust mathematical framework for defining and enforcing these sets, playing a pivotal role in the design of controllers aimed at ensuring system safety.
![alt text](https://homes.cs.washington.edu/~mohnishi/images/pubs/TCST_1.png)
*Illustration provided by Motoya Ohnishi[^3]
The illustration above vividly captures the concepts of Limited-duration Safety and Safety (Forward Invariance) within the context of CBFs. In the left image, we observe a limited-duration safety region where the system is only obligated to remain safe for a finite duration. Conversely, the right image portrays a control invariant set, demanding the system to uphold forward invariance, thereby ensuring that the system state remains within the safe set for all time.
These concepts serve as the cornerstone in defining the safety requirements that CBFs aim to enforce. By leveraging CBFs, engineers can develop controllers capable of robustly maintaining system safety, even in the face of uncertain environments and disturbances. The integration of such rigorous safety mechanisms is paramount in guaranteeing the reliability and stability of safety-critical systems across diverse domains.
#### Definitaion
Consider a dynamicical system described by $\dot{x} = f(x,u)$ where $x \in \mathbb{R}^n$ and $u \in \mathbb{R}^m$. Let $b$: $\mathbb{R}^n \rightarrow \mathbb{R}$ be a continuous differentialbe function. Define $C \in \mathbb{R}^n$ and $C=\lbrace x \in X | b(x) \le 0 \rbrace$, then $b$ is a valid CBF $\exists \alpha$ s.t. $\underset{u \in \mathcal{U}}\sup \nabla b(x)^Tf(x,u) \ge -\alpha(b(x))$.
- Explanation
    1. The function $b(x)$ is a scalar-valued, continuous, and differentiable function that defines the safe set $C = \lbrace x \in X | b(x) \le 0\rbrace$.
    2. The CBF condition requires that there exists a class $\mathcal{K}$ function $\alpha$, which is a monotonically increasing function with $\alpha(0) = 0$, such that the rate of change of the barrier function $b(x)$ along the system dynamics $f(x,u)$ is always greater than or equal to $-\alpha(b(x))$.
    3. This condition ensures that as the system state approaches the boundary of the safe set (where $b(x) = 0$), the rate of change of $b(x)$ becomes more negative, effectively "pushing" the system state back into the interior of the safe set. Once the system state is at the boundary, the CBF condition requires that the system can either move along the boundary or turn back into the interior of the safe set.
- Additional Notes
    - Class $\mathcal{K}{\infty}$ is a continuous function where $\alpha:(-\infty, \infty) \rightarrow (-\infty, \infty)$, belongs to the extended class $\mathcal{K}{\infty}$ if it is strictly increasing, $\alpha(0)=0$, and $\lim_{r \rightarrow \pm \infty}\alpha (r) = \pm \infty$.
## Applications
CBFs find applications in a wide range of fields, from robotics and autonomous vehicles to aerospace and manufacturing. For example, in robotics, CBFs can ensure that a robot arm avoids collisions with obstacles or stays within its workspace. In autonomous vehicles, CBFs can enforce safe distance constraints and prevent collisions. These real-world applications highlight the importance of CBFs in ensuring safety and reliability in complex systems.
## Comparison with Control Lyapunov Function
| Aspect                  | CBFs                                                              | CLFs                                                              |
|-------------------------|-------------------------------------------------------------------|-------------------------------------------------------------------|
| Objective               | Ensure safety by enforcing control invariant sets               | Ensure stability by demonstrating Lyapunov-like properties       |
| Mathematical Formulation | Ensure forward invariance of sets using a barrier function       | Prove existence of a Lyapunov function demonstrating stability properties  |
| Conditions              | Existence of a control action ensuring forward invariance        | Existence of a Lyapunov function demonstrating global stability   |
| Focus                   | Prioritize safety by preventing the system from entering unsafe states  | Emphasize stability by ensuring convergence to a desired equilibrium point  |
| Applications            | Commonly used in safety-critical systems such as robotics and autonomous vehicles | Widely applied in control systems to analyze stability and convergence properties  |
| Robustness              | Provide robustness against disturbances and uncertainties       | Offer robustness against perturbations by ensuring stability around an equilibrium point  |
| Trade-offs              | May be conservative in their safety guarantees                  | Require stronger assumptions about the system dynamics and stability properties  |
| Complementary Nature    | Focus on safety-critical aspects, ensuring the system remains within safe operating bounds | Address stability concerns, guaranteeing convergence to desired equilibrium points  |
## Combine CBF Constraints with CLF Constraints
#### Motivation
In many practical scenarios, we may want to design a system that aims to reach a desired goal while simultaneously avoiding obstacles. This can be achieved by combining Control Lyapunov Function (CLF) constraints and Control Barrier Function (CBF) constraints.
#### Stpes
To balance the objective of reaching a goal and avoiding obstacles, we can formulate an optimization problem with the following steps:
1. Choose a desired control input $u^{dsr}$ that is as close as possible to the actual control input $u$. This can be achieved by minimizing the squared norm $\underset{u}\min \lVert u^{dsr}-u \rVert^2_2$, subject to the constraint $u \in \mathcal{U}$.
2. Include the CLF constraint $\nabla V(x)^T f(x,u) \le -\beta V(x)$, where $V(x)$ is a Control Lyapunov Function and $\beta$ is a positive parameter. This ensures that the system is making progress towards the desired goal.
3. Include the CBF constraint $\nabla b(x)^T f(x,u) \ge -\alpha(b(x))$, where $b(x)$ is the Control Barrier Function and $\alpha$ is a class $\mathcal{K}$ function. This ensures that the system stays within the safe set defined by $b(x) \le 0$.
4. However, combining the CLF and CBF constraints can lead to an infeasible optimization problem. To address this, we can introduce a slack variable $\epsilon \ge 0$ and relax the CLF constraint to $\nabla V(x)^T f(x,u) \le \epsilon$. This allows the system to slightly deviate from the desired goal for the sake of safety.
5. To minimize the violation of the CLF constraint, we can add a penalty term $\gamma\epsilon^2$ to the objective function, where $\gamma$ is a weighting factor. The final optimization problem becomes: $\underset{u, \epsilon}\min \lVert u^{dsr}-u \rVert^2_2 + \gamma \epsilon^2$ subject to:
    - $u \in \mathcal{U}$
    - $\nabla V(x)^T f(x,u) \le \epsilon$
    - $\nabla b(x)^T f(x,u) \ge -\alpha(b(x))$
#### Conclusion
The resulting optimization problem is a Quadratic Program (QP) because the objective function is quadratic, and the constraints are linear in the control input $u$ and the slack variable $\epsilon$. This formulation allows the system to balance the competing objectives of reaching the desired goal and avoiding obstacles.
- Additional Notes
    - If the system dynamics are control-affine, i.e., $f(x,u) = f_0(x) + f_1(x)u$, then the optimization problem becomes linear in the control input $u$.
## Conclusion
Control Barrier Functions (CBFs) offer a robust framework for ensuring safety in dynamic systems, especially in safety-critical applications. By defining control invariant sets and enforcing forward invariance through Lyapunov-like conditions, CBFs enable the design of controllers that guarantee system safety.
Throughout these notes, we explored the core concepts of CBFs, including their mathematical formulation, significance in safety-critical applications, and integration with Control Lyapunov Functions (CLFs) to balance safety and performance objectives. 
These notes provide a foundational understanding of Control Barrier Functions, laied a foundation for further exploration and application in control theory and engineering. Continued research and refinement of CBF principles will lead to advancements in designing safe and robust control systems.

[^1]: Xiao, Wei, and Calin Belta. "High-order control barrier functions." IEEE Transactions on Automatic Control 67, no. 7 (2021): 3655-3662.
[^2]: Xu, Xiangru, Paulo Tabuada, Jessy W. Grizzle, and Aaron D. Ames. "Robustness of control barrier functions for safety critical control." IFAC-PapersOnLine 48, no. 27 (2015): 54-61.
[^3]: Motoya Ohnishi, "Constraint Learning for Control Tasks with Limited Duration Barrier Functions," Accessed April 17th 2024, Available at: [[Link](https://homes.cs.washington.edu/~mohnishi/publication/TCST2019/)].
