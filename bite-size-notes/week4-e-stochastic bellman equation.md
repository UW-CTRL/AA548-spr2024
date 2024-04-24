---
# classic #gaia #uncover
theme: classic
paginate: true
class:
 - lead
size: 16:9

marp: true
footer: "ME 548 SPRING 2024, YOUNG_JAE_RYU" 

---
# Stochastic Bellman Equation
## Preliminaries 
---
![Transition Graph](C:/study/Spring24/ME548/Bite-Size-Note/1.png)

For this bite size note, 
Also, be aware of the terminology for value function is quite different from where it uses from reinforcement learning. 
## Introduction 
---
The model is the concept of a system state, capturing all mutable and influential components of the system, represented at a particular point in time $t$ by a variable $x(t)$. The evolution of the system’s state is in turn governed by a set of constraints, called the system dynamics. When such a system can be said to be influenced by some control signal $u$, the task in control theory is to design the control signal to generate desirable behavior relative to a goal. 
Our overall task will be to determine what is the “optimal” control for our system. For this we need to specify performance for each controls. Let the state represent as a node and control sequences correspond to paths originating at the initial state and terminating at one of the goal state correspond to the Terminal stage $N$. So if we view the cost of path as its length, we see that a determinisitic finite state finite horizon problem is equivalent to finding a minimun-length path from the initial state of the env to the terminal state.


However, this is the deterministic optimazation problem. 

Consider the finite-horizon discrete-time system which forms:
$$
    x_{k+1} = f_k(x_k, u_k, \omega_k), \quad k=0,1,...,N-1
$$

where as $x_k$ is an element of some state space $S_k$, the control $u_k$ is an element of some control space. The cost per stage is denoted $g_k(x_k,u_k,\omega_k)$ and also depends on the random disturbance $\omega_k$. Then we can intepret transition graph as a Markov Decision Process. The control $u_k$ is constrained to take values in a given subset $U(x_k)$, which depends on the current state $x_k$.

The stochastic finite horizon optimal control problem differs from the deterministic version primarily in the nature of the discrete-time dynamic system that governs the evolution of the state $x_k$. This evolution is also be called as the the state transition probability $p(x_k | x_{k−1} , u_k)$ where probability distribution assigns a probability or density value to each possible state with regards to the true state. Also, stochastic system includes a random "disturbance" $\omega_k$, which is characterized by a probability distribution $p(\cdot|x_k, u_k)$ that may depend explicitly on $x_k$ and $u_k$, but not on values of prior disturbances $\omega_{k-1}, \omega_{k-2}, ...., \omega_0$. This is so called the __markov assumption__. In reality, this disturbance also contains random and contains uncertainties. Generally, disturbance are mostly originate from the observation, such that we can call as the measurement probability $p(z_k|x_{k}, u_{k-1}) = p(z_k|x_{k})$  where intuition can be given as the "noisy projection of the state". The state transition probability and the measurement probability together describe the dynamical stochastic system.




## Major Difference between deterministic and stochastic optimal control
An important difference is that we optimize not over control sequences ${u_0,u_1,...u_{N-1}}$, but rather over policies (also called closed-loop control laws or feedback policies) that consist of a sequence of functions. 
$$
    \pi = {\mu_0, \mu_1,...,\mu_{N-1}}
$$
where $\mu_k$ maps states $x_k$ into controls $u_k = \mu_k(x_k)$, and satisfies the control constraints, i,e is such that $\mu_k(x_k)\in U_k(x_k)$ for all $x_k \in S_k$. Such policies will be called __admissible__. Polices are more general objects than control sequences, and in the presence of stochastic uncertainty, they can result in improved cost, since they allow choices of controls $u_k$ that incorporate knowledge of the state $x_k$. Without this knowledge, the controller cannot adapt appropriately to unexpected values of the state, and as the result the cost can be adversely affected. This is a fundamental distinction between deterministic and stochastic optimal control problems.

Another distinction between deterministic and stochastic problems is that the evaluation of various quatities such as cost function values involves forming expected values, and this may necessitate the use of Monte Carlo simulation. Monte Carlo simulation is a computational technique used to understand the impact of uncertainty and randomness in mathematical, engineering systems. In a Monte Carlo simulation, a model is run many times with random inputs, sampling from probability distributions that represent the uncertain parameters in the system. By repeating this process thousands or even millions of times, the simulation generates a range of possible outcomes and their probabilities.

---
## Formation
As following the same process of the deterministic value function we learned from the lecture, given an initial state $x_0$ and a policy $\pi = {\mu_0,..\mu_{N-1}}$, the future states $x_k$ and disturbances $\omega_k$ are random variables with probability distributions(i.e Guassian Normal Distribution) defined through the system equation,

$$
     x_{k+1} = f_k(x_k, u_k, \omega_k), \quad k=0,1,...,N-1
$$

Thus, for given functions $g_k$, $k=0,1,..., N$, the expected cost of the policy $\pi$ starting $x_0$ is :
$$
    J_{\pi}(x_0) = \mathbb{E} \{g_N(x_N) + \sum^{N-1}_{k=0} g_k(x_k, \mu_k(x_k), \omega_k) \}
$$
where the expected value operation  $\mathbb{E}\{\cdot\}$ is over all the random variables $\omega_k$ and $x_k$. An optimal policy $\pi^*$ is one that minimizes this cost; i.e,

$$
    J_{\pi^*}(x_0) = \min_{\pi\in\Pi}J_\pi(x_0)
$$
where $\Pi$ is the set of all __admissible__ policies.

The optimal cost depends on $x_0$ and is denoted by $J^*(x_0)$; 
$$
    J^*(x_0) = \min_{\pi \in \Pi} J_{\pi}(x_0)
$$
It is useful to view $J^*$ as a function that assigns to each initial state $x_0$ the optimal cost $J^*(x_0)$, and call it the __optimal cost function or optimal value function__, particularly in problems of maximizing the reward.

---
## Finite Horizon Stochastic Dynamic Programming

Optimization problems such as the one stated above are efficiently solved via dynamic programming (DP). DP relies on the following obvious fact: if a given state-control action sequence is optimal, and we were to remove the first state and control action, the remaining sequence is also optimal (with the second state of the original sequence now acting as initial state). This is the Bellman optimality principle. The optimality principle can be revised in similar language: the choice of optimal actions in the future is independent of the past actions which led to the present state. This property resembles the markov assumption.

The DP algorithm for the stochastic finite horizon optimal control problem has a similar form to its deterministic version, and shares several of its major characteristics:
- Using tail subproblems to break down the minimization over multiple stages to single stage minimizations.
- Generating backwards for all $k$ and $x_k$ the values ${J^*}_k(x_k)$, which give the optimal cost-to-go starting at stage $k$ at state $x_k$
- Obtaining an optimal policy by minimization in the DP equations.
- A structure that is suitable for approximation in value space, whereby we replace $J_k^*$ by approximations $\bar{J}_k^*$, and obtain a suboptimal policy by the corresponding minimization.

### DP Algorithm for Stochastic Finite Horizon Problems
- Start with
$$
J^*_N (x_N ) = g_N(x_N)
$$
- and for $k = 0, . . . , N−1$, let
$$
J^*_k(x_k) = \min_{u_k \in U_k (x_k)} \mathbb{E} \{g_k(x_k, u_k, w_k) + J^*_{k+1}(f_k(x_k, u_k, w_k)) \}
$$
- If $u^∗_k = \mu^∗_k(x_k)$ minimizes the right side of this equation for each $x_k$ and $k$, the policy $\pi^∗ = {\mu^*_0, . . . , \mu^*_{N −1}}$ is optimal.


The key fact is that for every initial state $x_0$, the optimal cost $J^*(x_0)$ is equal to the function $J^*_0(x_0)$, obtained at the __last step__ of the above DP algorithm.

As in deterministic problems, the DP algorithm can be very time-consuming, but more so since it involves the expected value operation. This motivates suboptimal control techniques, such as approximation in value space whereby we replace $J_k^*$ with easier obtainable approximation $\bar{J}_k$.

## Conclusion
During the execution of a controlled system, at any point in time the system trajectory may be split in two. 
- The optimal future trajectory is independent of the past (Markov Assumption), as the system’s state captures the full history of what occurred, and
thus each trajectory segment is itself optimal (Bellman's Optimality). 
- Goal of Optimal Control in stocahstic horizon is to minimize expected value of the sum of costs, subject to dynamics and other constraints and disturbances.