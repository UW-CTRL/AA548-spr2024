# Continuous-Time Constraint Satisfaction in Sequential Convex Programming

**Scope: Continuous-Time Constraints Satisfaction in Discrete-Time Trajectory Optimization**
 Optimal Control is a widely-used mathematical tool in the context of Trajectory Optimization; Sequential Convex Programming (SCP) has been successfully employed for solving Discrete-Time (DT) Optimal Control problems online, allowing vehicles to autonomously replan their reference. The present note frames how to guarantee Continuous-Time Constraint Satisfaction (CTCS) within such DT Optimal Control problems; a drone NMPC controller for obstacle avoidance is presented.

**Objectives**
 - Understand criticalities of DT Trajectory Optimization;
 - Learn how to guarantee CTCS within a CT framework;
 - Verify capabilities of CTCS for application scenarios.

## Introduction
Direct methods are extremely popular for trajectory optimization purposes: they directly treat control as optimization variable while satisfying CT dynamics constraints and CT path constraints of different nature (e.g. max thrust from a rocket engine). Such direct methods are widely used for both offline and online planning purposes: convex formulation of such problems further ensures polynomial-bounded limit on computational time. Direct methods follow a discretize-then-optimize approach, and rely on heuristics to choose a discretization  approach appropriate for the required accuracy and computational performances. State-of-the-art techniques are available to ensure satisfaction of dynamics in a CT framework; on the other hand, no unified method exists to grant satisfaction of path constraints in CT; reason is the following: dynamics can be integrated with high accuracy with 'shooting' approaches (Space Shuttle Guidance is an illustrious example [1]), whereas path constraints can be only imposed at node points, i.e. the time instants at which state and controls are optimization variables. The problem becomes more relevant if few nodes are used, which is common for online applications, e.g. when a SCP techinque is employed. The present note digs into a recently-proposed approach to grant CT satisfaction of path constraints, up to arbitrarily high degree of accuracy [2]; the necessity of such formulation is highlighted with a drone NMPC controller applied to an obstacle-avoidance scenario [3,4].  

## Preliminaries
### Notation


## Main body
### DT Optimal Control problem 
### Inter-sample constraint violation
### CTCS-compatible Optimal Control
### Inter-sample constraint satisfaction

## Conclusion


## References
[1] NASA Systems Analysis Branch Guidance and Control Division - **Space Shuttle Guidance, Navigation and Control - Design Equations - Volume IV Deorbit and Atmospheric Operations** - NASA Technical Report, 1972 \\
[2] Elango, P., Luo, D., Kamath, A.G., Uzun, S., Kim, T. A$\cc$ikme$\ss$e, B. - **Successive Convexification for Trajectory Optimization with Continuous-Time Constraint Satisfaction** - Arxiv, 2024 \\
[3] 
[4] 
[5] 
[6] 