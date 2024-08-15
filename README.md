# A Data-Driven Approach To Preserve Safety and Reference Tracking for Constrained Cyber-Physical Systems Under Network Attacks

## Description 
This repository contains the codes for "A Data-Driven Approach To Preserve Safety and Reference Tracking for Constrained Cyber-Physical Systems Under Network Attacks" by [Mehran Attar](https://scholar.google.com/citations?user=nnLTy-oAAAAJ&hl=en) and [Walter Lucia](https://users.encs.concordia.ca/~wlucia/index.html) which has been submitted to publish in xxx --> [Arxiv link](https://arxiv.org/abs/2312.00658)


## Abstracat 
This paper proposes a data-driven control architecture capable of preserving the safety and tracking performance of constrained Cyber-Physical Systems (CPSs) in the presence of network attacks. In particular, by adopting a worst-case approach, the objective is to ensure the safety of the systems under attack while minimizing any potential degradation in tracking performance.  	To this end, on the controller side, a data-driven robust anomaly detector is designed to detect cyber-attack occurrences. Moreover, an add-on tracking supervisor module allows safe open-loop tracking control operations in case of unreliable measurements due to cyber-attacks. On the plant side, a safety verification module and a local emergency controller are designed to manage severe attack scenarios that cannot be handled on the controller's side. These two modules resort to worst-case reachability and controllability data-driven arguments to detect potential unsafe scenarios and replace, whenever strictly needed, the tracking controller with emergency actions whose objective is to steer the plant's state trajectory in a predefined set of admissible and safe robust control invariant region until an attack-free scenario is restored.
		The effectiveness of the proposed solution has been demonstrated through a simulation example.
		
## Running
1- Download [CORA release 2022](https://tumcps.github.io/CORA/) and [MPT3 release 2022](https://www.mpt3.org/) 

2- Add CORA and MPT folder and subfolders to the Matlab (in this work, [MATLAB R2021-a](https://www.mathworks.com/products/new_products/release2021a.html) has been used) path.

3- Add the repo folder and subfolders to the Matlab path.

## Files Description
One scenario has been considered in this work. 
### Scenario: Attack on the measurement channel 
To simulate this scenario, please run "main.m" 

#### Function Descriptions
- "compute_AB.m": computes all possible system matrices that are consistent with the data
- "computeRPI.m": computes a model-based RCI set based on the proposed method in **"Invariant approximations of the minimal robust positively invariant set", by Rakovic et al.
- "computing_ROSC_sets.m": computes the family of ROSC sets by considering a target set $\hat{\mathcal{T}}^0$
- "compute_intersec.m": computes the intersection of polyhedrons
- "data_driven_controller.m": computes the data-driven tracking controller for a constrained discrete-time linear system
- "data_driven_safety_guard.m": checks the safety of the plant using the received control 
- "one_step_ctrl.m" computes the data-driven ST-MPC control commands. 
- "poly_approx.m": computes a zonotopic inner approximation of a polyhedron 
- "set_index.m": computes the set membership index of a state for the data-driven ROSC sets.
- "detector_data_driven.m": simulates the data-driven anomaly detector local to the tracking controller, which is in charge of detecting anomalies caused by FDI attacks


### Figures
![Proposed architecture]([https://github.com/attarmehran/Data-Driven-Safety-Preserving-Control-Architecture-for-Constrained-CPS/blob/main/Figures/case_A_alarm_safety.jpg](https://github.com/PreCyseGroup/data-driven-control-architecture-for-preserving-the-safety-and-tracking-performance-of-CPS/blob/main/Figures/proposed_architecture.jpg)
![State trajectory: proposed solution with attacks (blue solid
line) vs trajectory in attack-free scenario (purple dashed line).](https://github.com/PreCyseGroup/data-driven-control-architecture-for-preserving-the-safety-and-tracking-performance-of-CPS/blob/main/Figures/trajectories.jpg)
![State evolution: no attack, proposed approach, \cite{attar2024safety}](https://github.com/PreCyseGroup/data-driven-control-architecture-for-preserving-the-safety-and-tracking-performance-of-CPS/blob/main/Figures/state_evolutions.jpg)
