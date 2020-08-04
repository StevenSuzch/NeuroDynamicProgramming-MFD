# NeuroDynamicProgramming-MFD [[PDF]](https://www.researchgate.net/publication/340090082_Neuro-dynamic_programming_for_optimal_control_of_macroscopic_fundamental_diagram_systems)


## Reference
This repository contains the *source code* of the following research paper:

* Z.C. Su, Andy H.F. Chow, N. Zheng, Y.P. Huang, E.M. Liang, R.X. Zhong, Neuro-dynamic programming for optimal control of macroscopic fundamental diagram systems,
Transportation Research Part C: Emerging Technologies,
Volume 116,
2020,
102628,
ISSN 0968-090X,
[[Link]](https://doi.org/10.1016/j.trc.2020.102628.)

If our paper hleps, please cite it as:

```javascript
@article{ZCSU2020,
title = "Neuro-dynamic programming for optimal control of macroscopic fundamental diagram systems",
journal = "Transportation Research Part C: Emerging Technologies",
volume = "116",
pages = "102628",
year = "2020",
issn = "0968-090X",
author = "Z.C. Su and Andy H.F. Chow and N. Zheng and Y.P. Huang and E.M. Liang and R.X. Zhong",
}
```

## Introduction
The macroscopic fundamental diagram (MFD) can eﬀectively reduce the spatial dimension involved in dynamic optimization of trafc performance for large-scale networks. Solving the
Hamilton-Jacobi-Bellman (HJB) equation takes center stage in yielding solutions to the optimal
control problem. At the core of solving the HJB equation is the value function that represents
choosing a sequence of actions to optimize the system performance. However, this problem
generally becomes intractable for possible discontinuities in the solution and the curse of dimensionality for systems with all but modest dimension. To address these challenges, a neural
network is used to approximate the value function to obtain the optimal controls through policy
iteration. Furthermore, a saturated operator is embedded in the neural network approximator to
handle the difculty caused by the control and state constraints. This policy iteration can be
implemented as an iterative data-driven technique that integrates with the model-based optimal
design based on real-time observations. Numerical experiments are conducted to show that the
neuro-dynamic programming approach can achieve optimization goals while stabilizing the
system by regulating the traffic state to the desired uncongested equilibrium.

## Quick Start

### Scenario A: Minimizing system energy (set-point control)
1. Open the main file <Main_NeuroDynamicMFD.m>
2. Check the simulation settings:
    
    * Initial states of two regions
    * Set points of two regions
    * Demand profile (static) of each directions
    * Control time span
    * Weighted matrix for value function
    * Sample time of Monte-carlo sampling
    * The number of iterations of the Policy Iteration

3. Choose the appropriate kernel function:
    
    * Open the function file <Calculate_dPHI.m>
    * Select one of those provided kernel functions, and comment others
    * For kernel functions are provided:
        
        $$  \hat{V}_{129}(x_1,x_2,x_3,x_4)=\sum_{k=1}^{129} w_{k}x_1^ix_2^jx_3^mx_4^n  \quad \quad   i+j+m+n=2,4,6 $$
        