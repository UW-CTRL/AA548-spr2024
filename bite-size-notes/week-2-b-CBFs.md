# Control Barrier Functions 

## Introduction 
In recent years we have had a widespread deployment of autonomous systems, from autonomous vehicles to surgical robots, these systems rely on their ability to perform complex tasks while providing safety assurances. There is a growing increasing number of complex applications with inherent real-world uncertainties using these systems requiring safety-critical autonomous systems... Control barrier functions (CBFs) have recently become a popular technique for **guarenteeing safety** of autonomous systems 

## Preliminaries

### Forward Invariance
To first understand the theory behind control barrier functions (CBFs) we must first learn about control invariant sets. The first investigation into safety for dynamical systems was by Nagumo when he provded neccessary and sufficient conditions for set invariance [1]. Lets define what it means to have a positivley invariant set (also called a forward invariant set). We consider a continuous dynamical system: 

$$
 \dot{x}(t) = f(x,t), \hspace{0.2cm} t \geq 0
 $$
 
where $x(t) \in \mathbb R^n$ are the state variables, $t$ is the time variable, and $f$ is a continous function. We now provide the definition of the forward invariant set of a dynamical system. 

**Definition (Forward Invariance)** Assume $C$ is a set in $\mathbb R^n$. The set $C$ is called a forward invariant set of the aforementioned dynamical system if $x(0) \in C$ implies $x(t) \in C$ for all $t \geq 0.$ 








## References
1. F. Blanchini, "Set invariance in control," Automatica, vol. 35, no. 11, pp. 1747-1767, 1999. 