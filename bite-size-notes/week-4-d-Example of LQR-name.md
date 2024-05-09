# Objectives: 
1. Motivation of LQR
2. Summarize the variation of LQR and corresponding Riccati equation and optimal control. 
3. Using ode45 to solve Riccati equation.
# Introduction: 
LQR (Linear Quadratic Regulator) is an optimal solution for full-state feedback control, aiming to minimize a cost function chosen by the designer, typically reducing state error and control effort. Let's break down this sentence to understand what LQR is
## What is a fullstate feedback control.
We want to stabilize a system and drive all states to 0. A standard full-state feedback control can achieve this goal. (A standard full-state feedback means the system doesn't have a reference input. Other variations, such as systems with reference inputs or error tracking, are out of scope.) \
A standar full feedback controle has the block diagram. 
 ![fullfeedback](https://github.com/p8410077/AA548-spr2024/assets/11802603/bea8ed3a-5062-4d60-93c9-b213d4d856d5)

By using the full state control, we can using the method called pole placement, literally, we are putting the poles of the system at the desired location on the complex plane.

## what is pole placement ?
The open loop state space representices of the system is:

$$
	\dot{x}(t) = Ax(t)+ Bu(t) \\
$$

$$
	y(t) = Cx(t) +Du(t) \
$$

By finding the location of the poles of matrix A, we can understand if the system is stable. We can design a controller that stabilize the system, if the system is unstable, or drive the system to converge faster, if the system is stable. 

By implementing the full state feed back, the closed loop state space representices of the system is:

$$
\dot{x}(t) = Ax(t)+ Bu(t) \\
$$

$$
y(t) = Cx(t) +Du(t) \\
$$

$$
u(t) = -Kx(t)
$$

By sustituting u(t) with -Kx(t), the closed loop state space representices of the system is:

$$ 
\dot{x}(t) = (A-BK)x(t) \\
$$

$$
y(t) = (C-DK)x(t)
$$

We know that the eigenvalue of the matrix (A-BK) is the poles of the system. We can using K matrix to put the poles at desired location and therefore change the respose of the system. 


## How to chose K ?  
Since we can change the respoce of the system by poles placement. We might ask ourself where should I put the poles, in other words, how to chose the K ? One idea is to construcet the problem into optimization framework that minimize the state error and control nameed LQR.

In class, we already disccused, by using HJB, the optimal control input u^* is a function of A,B,R,Q,P where

$$
\begin{aligned}
	A \text{ is the A matrix of the system (know)};\\
	B \text{ is the B matrix of the system (know)};\\
	Q \text{ is a states weighing function (know)};\\
	R \text{ is a control weighing function  (know)};\\
	P \text{ is a matrix variable form the Riccati equation (Uknow)};
 \end{aligned}
$$

And by solving the Ricatti equation, we can find the optimal control input $u^*$
The variation of the optimal representies of LQR and correponding Riccati equation and control u are summirized below.  

|                                    | LQR form                         | Riccati Equation                |                                
| ---------------------------------  | -------------------------------- |-------------------------------- |
| Continuous time finite horizon     | $\min \int_{0}^{T}x(t)^T Q(t) x(t) + u(t)^T R(t) u(t) + x(T)^T Q(T) x(T)$ | $0 =\dot{P}(t) + P(t)A+A^TP(t)+Q-P(t)BR^{-1}B^TP(t)$ | 
| Discret time finite horizon   | $$\sum_{n=1}^{\infty} x_{k}^T Q_k x_{k} + u_{k}^T R_k u_{k}\$$ | $P_k = Q_k +A^T P_{k+1}A-(A^TP_{k+1}B)(R_k+B^TP_{k+1}B)^{-1}(B^TP_{k+1}A)$ | 
| Continuous time infinite horizon   | $\min \int_{0}^{\inf}x(t)^T Q(t) x(t) + u(t)^T R(t) u(t)$ |$PA+A^TP+Q-PBR^{-1}B^TP=0$ |
| Discret time infinite horizon      | $$\min x_{k+1}^T Q_T x_{k+1} + \sum_{k=0}^k x_{k}^T Q_k x_{k} + u_{k}^T R_k u_{k}$$ | $P = Q +A^TPA-(A^TPB)(R+B^TPB)^{-1}(B^TPA)$| 

|                                    |  u          |                     
| ---------------------------------  | -------------------------------- |
| Continuous time finite horizon     | $u^*(t) =-R^{-1}B^TP(t)x$ |
| Discret time finite horizon        | $u^*_k =-(R _{k+1}+B^TP _{k+1}B^{-1}BP _{k+1}A) X _k$  |
| Continuous time infinite horizon   | $u^* =-R^{-1}B^TPx$|
| Discret time infinite horizon      | $u^*_k =-(R+B^TPB^{-1}(B^TPA)x$ |



Fron this point, we narrow down the optimal problem to solving a differentail matrix equation. And I want to show how to solving the Ricatti equation for the continuous finite time using matlab ode45. 
 
# Main body 
## Solving the Recatti equation using matlab ode45 (Method from AA550 Nonlinear optimal control by Prof. Santosh Divasia)

Please refer to the attached code. 

The optimal control problem in terns on LQR is:

$$
	\min \int_{0}^{T}x(t)^T Q(t) x(t) + u(t)^T R(t) u(t) + x(T)^T Q(T) x(T) 
$$

$$
	\text{s.t  } \dot{x} = Ax + Bu
$$

The Riccati equtaion is :

$$
0 =  \dot{P} (t) + (P(t)A+A^TP(t)+Q-P(t)BR^{-1}B^TP(t))
$$

$$
\dot{P} (t) =  -P(t)A-A^TP(t)-Q-P(t)BR^{-1}B^TP(t))
$$

Optimal control input is :

$$u^*(t) =-R^{-1}B^TP(t)x$$

By applying boundery condition, P matrix at final time, P(tf), is Q(T). 
Therefore we can sovle the P for $\dot{P}$, by solving the differential equation backward in time with initial condition $P(0) = Q(T)$.

The command in matlab is: \

[t,PV]=ode45(@(t,p) dpp(t,p,A,B,Q,R,E,F),[0 (tf-ti)],Ptf,options);


The first step is rewrite the Riccati equation as in reversive form: 

$$\dot{P}(t) =  P(t)A+A^TP(t)+Q-P(t)BR^{-1}B^TP(t)$$

To define a function named "dpp" as the reversed Riccati equation, which will be a anonymous function passing to ode45. 

Since the inpout of the ode45 is a vactor, We need to reshape the vector to the original P matrix in the dpp founciton and reshape the output, $\dot{P}$ as a vector. 

After solving the P, we will have Pm*Pm-by-n matrix. Pm is the dimention of the state or the roll number of the P matrix, n is the time stes number solving by ode45. 

Because the solution is backward in time, we need to reverse the oder by using the matlab function fliup(). to reorder the PV vector and the corresponding time step in time. 

Finally, we compute the optimal control u^*(t), at the time t = tm, by extracting the PV vector in corresponding row at tm into the original matrix P(tm) and plug it into the equation

$u^*(tm) = R^{-1}BP(tm)$

from t = 0 to t = final. The result is the series of the optimal control $u^*$. 


Let's do a example:
The optimal control problem in terns on LQR is:

$$
	$\min \int_{0}^{1}x(t)^T Q(t) x(t) + u(t)^T R(t) u(t) + x(T)^T Q(T) x(T)$ 
	s.t \dot{x} = Ax + Bu
$$

Where

$$
A =  \begin{bmatrix} 0 & 1  \\\ 3 & 0 \end{bmatrix} \\
B =  \begin{bmatrix} 0  \\\ 3  \end{bmatrix} \\
Q =  \begin{bmatrix} 0  \\\ 3  \end{bmatrix} \\
R= 1 \\;
Q(T) = \begin{bmatrix} 50 & 0  \\\ 0 & 50  \end{bmatrix} \\
x = \begin{bmatrix} x1  \\\ x2 \end{bmatrix} \\
$$

$$
\text{u is one dimention.} \\
$$

$$
\text{initial state} x(0) = \begin{bmatrix} 1  \\\ 1 \end{bmatrix} \\
$$

##code 

```Matlab 
clear
close all
clc
% System matrices 
    A = [0 1; 3 0]; B=[0;3]; C = [1 0]; D =0;
    % cost function matrices
    Q = 1*[1 0; 0 1]; R = 1; F = 50*[1,0;0,1] ;
    ti = 0; tf =1;
    % initial conditions 
    x0 = [1;1]; 

E = B*inv(R)*B';
Ptf = zeros(4,1);
Ptf(1,1) = F(1,1);
Ptf(2,1) = F(1,2);
Ptf(3,1) = F(2,1);
Ptf(4,1) = F(2,2);
nfig = 0;
options=odeset('RelTol',1e-10);
[tk,PV]=ode45(@(t,p) dpp(t,p,flag,A,B,Q,R,E,F),[0 (tf-ti)],Ptf,options);


% Solving for P in a vector form PV
%
%
% PV is in vector form, each row corresponds to row in time t
% flip the PV vector 
PV = flipud(PV);
% redefine the time vector
tk = flipud(tk); 
tk = -tk +(tf)*ones(size(tk));
%
%
% computing the gain matrix K(t) as row vector
%
[mP,nP] = size(PV);
K = zeros(mP,2);
%
for i = 1:mP
Ptm = zeros(2) ; 
Ptm(1,1) = PV(i,1);
Ptm(1,2) = PV(i,2);
Ptm(2,1) = PV(i,3);
Ptm(2,2) = PV(i,4);
K(i,:) = inv(R)*B'*Ptm;

end


[y2, dyx] = ode45(@(t,x) dyn(t,x,A,B),[0 (tf-ti)],x0,options);

figure 
plot(y2,dyx,'o','LineWidth',3)
xlabel('time')
ylabel('State')
legend('uncontrol State x1' ,'uncontrol State x2')
set(gca,'FontSize',20)

% plot the control gain K
nfig = nfig+1; figure(nfig)
figure
plot(tk,K,'r','LineWidth',3)
xlabel('time')
ylabel('Kalman gain K')
set(gca,'FontSize',20)

%return
% simulate the sytem response using ode45
options=odeset('RelTol',1e-10);
[t,x]=ode45(@(t,x) xdiff(t,x,flag,A,B,tk,K),[0 (tf-ti)],x0,options);
% plot the state x
nfig = nfig+1; figure(nfig)
figure
plot(t,x,'b','LineWidth',3)
xlabel('time')
ylabel('State')
legend('Controled State x1' ,'Controled State x2')
set(gca,'FontSize',20)

%return
% find and plot the input u

[m,n]=size(x);     % Length of time, x is m
[mR,nR] = size(R); % number of inputs = mR
Kt = interp1(tk,K,t); % interpolate the K matrix
u = zeros(m,mR);    % initialize the input
for jj=1:1:m
    u(jj) = -Kt(jj,:)*(x(jj,:))';
end

figure
plot(t,u,'r','LineWidth',3)
xlabel('time')
ylabel('Input u')
set(gca,'FontSize',20)










%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Function to find the solution to Riccati Eq
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dpv = dpp(t,p,flag,A,B,Q,R,E,F)
%
%    Called by PSolve to compute the Riccati matrix P.
%
%    [t,GP] = ode45('gramdiff',[ti tf],zeros(NT,1),[],A,B,Q,R,E).
%
%    See also PSOLVE.
%
%
% Finding the P matrix PM
% 
PM = zeros(2);
PM(1,1) = p(1);
PM(1,2) = p(2);
PM(2,1) = p(3);
PM(2,2) = p(4);
%
%
% computing P matrix derivative 
% Backward in time
dp = A'*PM + PM*A -PM*E*PM +Q;
%
%
% computing P vector derivative
dpv = zeros(4,1);

dpv(1,1) = dp(1,1) ;
dpv(2,1) = dp(1,2) ;
dpv(3,1) = dp(2,1) ;
dpv(4,1) = dp(2,2) ;
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Function to find the solution to system with optimal control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xdiff = xdiff(t,x,flag,A,B,tK,K)
%
% solving xdot = AX + Bu

Kt = interp1(tK,K,t);
ut = -Kt*x;
xdiff = A*x +B*ut;
end

function dx = dyn(t,x,A,B)

dx = A*x; 
end
```

# Results: 
First we can we plot the states without control, and find the system blows up, since the eigenvalue of the A matrix is on the RHP.
![uncon](https://github.com/p8410077/AA548-spr2024/assets/11802603/b7168051-1b6c-4784-8c36-71f37404da7b)

By inplementing LQR, the system converge to 0 in 1 second. 
![image](https://github.com/p8410077/AA548-spr2024/assets/11802603/389fc3c2-0a2a-4066-815f-6815c59bd34f)

# Conclusion: 
1. We succesfully use ode45 to solve Ricatti equation and find the optimal control for continuous finite time problem.
2. For other variation of the LQR need different technic to solve, for example: the infinit time problen is not a ODE, therefore can't use ode45.
# Refernece: 
[1] [Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation](https://underactuated.mit.edu/lyapunov.html) by Russ Tedrake \
[2] https://www.youtube.com/watch?v=E_RDCFOlJx4 \
[3] https://www.youtube.com/watch?v=E_RDCFOlJx4 \
[4] AA550 (Prof. Santosh Divasia)
