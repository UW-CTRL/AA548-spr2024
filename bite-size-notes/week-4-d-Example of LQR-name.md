# Objectives: 
1. Review the LQR 
2. Walkthough a example of invertid pendulem 
# Introduction: 
LQR (Linear Quatratic Regulater) is the a optimal solution of a full state feedback control to minimine the control efforts. Let break down this sentence and to understand what does this tell us ? 
## What is a fullstate feedback control.
A standar full feedback controle has the block diagram. 
 ![fullfeedback](https://github.com/p8410077/AA548-spr2024/assets/11802603/bea8ed3a-5062-4d60-93c9-b213d4d856d5)
A classic fullstate feecback controller regulates the system to all 0 states, (There are deviation out of the scope) 
##what is pole placement ?
The open loop state space representices of the system is:
$$ 
\dot{x}(t) = Ax(t)+ Bu(t) \\
y(t) = Cx(t) +Du(t) \\
u(t) = -Kx(t)
$$

By sustituting u(t) with -Kx(t), the closed loop state space representices of the system is:
$$ 
\dot{x}(t) = (A-BK)x(t) \\
y(t) = (C-DK)x(t)
$$
We know that the eigenvalue of the matrix (A-BK) is the poles of the system. We can using K matrix to put the poles at desired location and therefore change the respose of the system. 
$$ 
x_{t+1} = x_{t} + \dot{x}_{t} \Delta t
$$
## How to chose K ?  
Since we can change the respoce of the system by poles placement. We might ask ourself where should I put the poles, in other words, how to chose the K ? One idea is to construcet the problem into optimization framework that minimize the state error and control namely LQR. 

## Summarize different deviations of LQR in class  
| -----------------| Continuous time finite horizon   | Discret time finite horizon | Continuous time infinite horizon | Discret time infinite horizon |
| LQR form         | $$ min x_{k+1} $$ | --------------------------- | -------------------------------- | ----------------------------- |
| Riccati Equation | -------------------------------- | --------------------------- | -------------------------------- | ----------------------------- |

	

# Preliminaries 
	OPT
# Main body: 
# Conclusion: 
advantage 
Disadvantage 

# Refernece: 
