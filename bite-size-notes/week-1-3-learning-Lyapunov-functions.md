# Learning Lyapunov functions for arbituary non-linear dynamics
Short note for EE 548 week 1 topic 3

## Scope and objective 
We briefly mentioned in the lecture modern, data-driven approaches for developing Lyapunov functions, and in this note I will examine some of these approaches for learning a Lyapunov function for arbituary non-linear dynamics, with or without an explicit model for the dyanmics.
Disclaimer: This will look more like a brief literature review of recent papers rather than actual notes from a lecture, per se. Most of the methods I will include in this note are summarized in [1].

## Introduction and motivations
In the lecture we covered Lyapunov methods for analyzing stability, which are especially useful for non-linear systems (as opposed to linear systems in which it suffices to examine the eigenvalues of state matrices). However, one key factor for Lyapunov stability analysis is to actually find a candidate Lyapunov function. While there exist some heurictics, especially for physical systems (energy of the system, quadratic form, etc), it is in general a very hard task. Modern approaches for learning a Lyapunov function for non-linear systems have been proposed whether or not an explicit form of the dynamics is known (or a good enough simulator for the system is available). The methods for the former case mostly divided into model-based and sampling-based approaches and I will include 2 examples for each in this note. For the later case which involve exploiting a given dataset intead of the actual dynamics, I will include a Monte-Carlo sampling approach which ensures probabilitic stability and a novel data-driven approach that ensures strict mathematical stability within the given dataset.

## Math Preliminaries
Recall Lyapunov stability:
  Given a system $\dot{x} = f(x)$ and some regioon $D \in \mathbb{R^n}$ and ${0} \in D$:
  If there exists a continuously differentiable function $V(x)$ such that:  
  1) $V(0) = 0$
  2) $V(x) > 0 \forall x \in D\\{0}$
  3) $\dot{V(x)} = \nabla V(x)^T\f(x) \leq 0 $

## References
[1] [Robust data-driven Lyapunov analysis with fixed data by Yingzhao Lian](https://arxiv.org/pdf/2305.12813.pdf) by Matteo Tacchi and Colin Jones
