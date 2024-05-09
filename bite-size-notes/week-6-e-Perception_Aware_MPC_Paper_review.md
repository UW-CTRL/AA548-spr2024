# Paper review-Perception Aware Model Predictive Control for Quadrotors 

Authors - Davide Falanga, Philipp Foehn, Peng Lu, and Davide Scaramuzza

## Why is this important?

As we are advancing into a world of autonomous control of vehicles, there have been some key developments that have accelerated the research in the domain. One of these, is the Perception Aware Model Predictive Control (PAMPC) for Quadrotors, which I am going to review in this document. The focus of this review is to summarize the Model Predictive Control (MPC) pipeline used along with its cost function and control constraints.

## Introduction

Previously, perception and control action were treated as two separate problems in controls. To solve this problem, the authors of the paper designed a Model Predictive Control that optimizes the control input for trajectory optimization and simultaneously optimizing the perception of the sensing algorithms to maximize the vision needed to execute a control action. An example of this can be the path planning and trajectory optimization for the quadrotor to pass through a narrow gap, at the same time making sure the gap is always visible. A big advantage of this approach is that it is independent of the task and can be used in variety of other applications, other than quadrotors, that involve a vision-based localization and target tracking.

## Methodology

The way they designed the MPC was by including both, the quality of perception, and the system dynamics within the cost function that will be optimized to find the minimum control input for the trajectory. Moreover, the perception objectives are added as a part of components to be optimized, rather than as constraints, eventually merging control action and perception into a unified optimization problem. The unified optimization problem uses the velocity of the projection of the point of interest in the image plane. For using the MPC they derived the quadrotor dynamics, the perception objectives and the action objectives. The dynamical model of the quadrotor is described as folowing : 

$$
\begin{align*}
\dot{\mathbf{p}}_{W B} & =\mathbf{v}_{W B} \\
\dot{\mathbf{v}}_{W B} & ={ }_{W} \mathbf{g}+\mathbf{q}_{W B} \odot \mathbf{c}  \tag{3}\\
\dot{\mathbf{q}}_{W B} & =\frac{1}{2} \Lambda\left(\boldsymbol{\Omega}_{B}\right) \cdot \mathbf{q}_{W B}
\end{align*}
$$
where, 
$\mathbf{p}_{W B}=\left(p_{x}, p_{y}, p_{z}\right)^{\top}$ and $\mathbf{q}_{W B}=\left(q_{w}, q_{x}, q_{y}, q_{z}\right)^{\top}$ are the position and the orientation of the body frame with respect to the world frame $W$, expressed in world frame. Additionally, let $\mathbf{v}_{W B}=\left(v_{x}, v_{y}, v_{z}\right)^{\top}$ be the linear velocity of the body, expressed in world frame, and $\boldsymbol{\Omega}_{B}=\left(\omega_{x}, \omega_{y}, \omega_{z}\right)^{\top}$ its angular velocity, expressed in the body frame. Finally, let $\mathbf{c}=(0,0, c)^{\boldsymbol{\top}}$ be the mass-normalized thrust vector, where $c=\left(f_{1}+f_{2}+f_{3}+f_{4}\right) / m, f_{i}$ is the thrust produced by the i-th motor, and $m$ is the mass of the vehicle. Moreover, where ${ }_{W} \mathbf{g}=(0,0,-g)^{\top}$ is the gravity vector, with $g=$ $9.81 \mathrm{~m} / \mathrm{s}^{2}$. The state and the input vectors of the system are $\mathbf{x}=\left[\mathbf{p}_{W B}, \mathbf{v}_{W B}, \mathbf{q}_{W B}\right]^{\top}$ and $\mathbf{u}=\left[c, \boldsymbol{\Omega}_{B}^{\top}\right]^{\top}$, respectively.

The perception objectives can be formulated such that it guarantees a robust vision-based perception. Let ${ }_{W} \mathbf{p}_{f}=\left({ }_{W} p_{f x}, W p_{f y}, W p_{f z}\right)$ be the 3D position of a point of interest (landmark) in the world frame $W$ (cf. Fig. 22). We assume the body to be equipped with a camera having extrinsic parameters described by a constant rigid body transformation $T_{B C}=\left[\mathbf{p}_{B C}, \mathbf{q}_{B C}\right]$, where $\mathbf{p}_{B C}$ and $\mathbf{q}_{B C}$ are the position and the orientation of $C$ with respect to $B$. The coordinates $C \mathbf{p}_{f}=\left({ }_{C} p_{f x}, C p_{f y}, C p_{f z}\right)^{\top}$ of ${ }_{W} \mathbf{p}_{f}$ in the camera frame $C$ are given by:

$$
\begin{align*}
C \mathbf{p}_{f}= & \left(\mathbf{q}_{W B} \mathbf{q}_{B C}\right)^{-1} \odot  \tag{4}\\
& \left({ }_{W} \mathbf{p}_{f}-\left(\mathbf{q}_{W B} \odot \mathbf{p}_{B C}+\mathbf{p}_{W B}\right)\right)
\end{align*}
$$

The point $C \mathbf{p}_{f}$ in camera frame is projected into the image plane coordinates $\mathbf{s}=(u, v)^{\top}$ according to classical pinhole camera model :

$$
\begin{equation*}
u=f_{x} \frac{C p_{f x}}{C p_{f z}}, \quad v=f_{y} \frac{C p_{f y}}{C p_{f z}} \tag{5}
\end{equation*}
$$

where $f_{x}, f_{y}$ are the focal lengths for pixel rows and columns, respectively.
Later, these equations can be used to express the projection velocity as a function of the quadrotor state and input vectors as we are interested in reducing the velocity of its projection onto the image plane. 

## Problem Formulation

The unified optimization problem can be formulated as following:

$$
\begin{align*}
& \min _{\mathbf{u}} \int_{t_{0}}^{t_{f}} \mathcal{L}_{a}(\mathbf{x}, \mathbf{u})+\mathcal{L}_{p}(\mathbf{z}) d t  \tag{1}\\
& \text { subject to } \quad \mathbf{r}(\mathbf{x}, \mathbf{u}, \mathbf{z})=0 \\
& \mathbf{h}(\mathbf{x}, \mathbf{u}, \mathbf{z}) \leq 0
\end{align*}
$$

Where

$$
\begin{aligned}
& \dot{\mathbf{x}}=\boldsymbol{f}(\mathbf{x}, \mathbf{u}) \text { is the system dynamics } \\
& \mathbf{z}=\boldsymbol{f}_{p}(\mathbf{x}, \mathbf{u}, \sigma) \cdot \text { is the state vector for the perception dynamics } \\
& \mathcal{L}_{a}(\mathbf{x}, \mathbf{u}) \text { defines an action cost } \\
& \mathcal{L}_{p}(\mathbf{z}) \text { defines a cost for perception objectives. }
\end{aligned}
$$

After deriving the quadrotor dynamics and perception objectives the following cost function was employed in the Model Predictive Control :

$$
\begin{align*}
\mathcal{L}= & \overline{\mathbf{x}}_{N}^{\top} \mathcal{Q}_{x, N} \overline{\mathbf{x}}_{N}+ \\
& +\sum_{i=1}^{N-1}\left(\overline{\mathbf{x}}_{i}^{\top} \mathcal{Q}_{x, i} \overline{\mathbf{x}}_{i}+\overline{\mathbf{z}}_{i}^{\top} \mathcal{Q}_{p, i} \overline{\mathbf{z}}_{i}+\overline{\mathbf{u}}_{i}^{\top} \mathcal{R}_{i} \overline{\mathbf{u}}_{i}\right) \tag{10}
\end{align*}
$$

where the values  $\overline{\mathbf{x}}, \overline{\mathbf{z}}, \overline{\mathbf{u}}$ refer to the difference with respect to the reference of each value.

The inputs $\mathbf{u}$, consisting of $c$ and $\boldsymbol{\Omega}_{B}$, as well as the velocity $\mathbf{v}_{W B}$ are limited by the constraints:

$$
\begin{gather*}
c_{\min } \leq c \leq c_{\max }  \tag{11}\\
-\Omega_{\max } \leq \boldsymbol{\Omega}_{B} \leq \Omega_{\max }  \tag{12}\\
-v_{\max } \leq \mathbf{v}_{W B} \leq v_{\max } \tag{13}
\end{gather*}
$$

where $c_{\min }, c_{\max }, \Omega_{\max }, v_{\max } \in \mathcal{R}_{+} \cdot$

## Conclusion

The aforementioned cost function and constraints used in PAMPC are then used with the hardware of the quadrotors to execute the control action in real time. The authors proved successful results using this approach via experimentation. An example of it was a room with
obstacles (boxes) on the ground where the quadrotors were able to manage to encircle the obstacles and control the speed when they were around.

![](https://cdn.mathpix.com/cropped/2024_05_09_879380217ac9dd082806g-3.jpg?height=835&width=1645&top_left_y=396&top_left_x=240)

The figure above shows the quadrotors maintaining their velocity at the same time maximizing their vision space around objects.

## References

[1] Falanga, D., Foehn, P., Lu, P., \& Scaramuzza, D. (2018). PAMPC: Perception-aware model predictive control for quadrotors. 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). https://doi.org/10.1109/iros.2018.8593739 

The referencing of the equations is from the main research paper whose link is provided above. 

