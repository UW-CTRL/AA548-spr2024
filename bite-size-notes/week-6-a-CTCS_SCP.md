# Continuous-Time Constraint Satisfaction in Sequential Convex Programming

**Scope: Continuous-Time Constraints Satisfaction in Discrete-Time Trajectory Optimization**
 - Optimal Control is a widely-used mathematical tool in the context of Trajectory Optimization; Sequential Convex Programming (SCP) has been successfully employed for solving Discrete-Time (DT) Optimal Control problems online, allowing vehicles to autonomously replan their reference. The present note frames how to guarantee Continuous-Time Constraint Satisfaction (CTCS) within such DT Optimal Control problems; a drone NMPC controller for obstacle avoidance and a rocket landing-trajectory planner are preseted.

**Objectives**
 - Understand criticalities of DT Trajectory Optimization;
 - Learn how to guarantee CTCS within a CT framework;
 - Verify capabilities of CTCS for application scenarios.

## Introduction
Direct methods are extremely popular for trajectory optimization purposes: they directly treat control as optimization variable while satisfying CT constraints associated with vehicle dynamics and CT path constraints of different nature (e.g. max thrust from a rocket engine). Such direct methods are widely used for both offline and online planning purposes: convex formulations of such problems ensures polynomial-bounded limit on computational time. Direct methods follow a discretize-then-optimize approach, and rely on accurate discretization in order to trade-off accuracy and computational performances. State-of-the-art techniques are available to ensure satisfaction of dynamics in a CT framework; on the other hand, no unified method exists to grant satisfaction of path constraints in CT; reason is simple: dynamics can be integrated with high accuracy (Space Shuttle guidance is an illustrious example [1]), whereas path constraints can be only imposed at node points, i.e. the time instants at which state and controls are optimization variables.

## Preliminaries


## Main body


## Conclusion


## References
[1] NASA TR - 1972 - Space Shuttle Guidance, Navigation and Control - Design Equations - Volume IV Deorbit and Atmospheric Operations 
[2] 
[3] 
[4] 
[5] 
[6] 