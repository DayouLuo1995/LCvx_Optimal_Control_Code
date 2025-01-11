# LCvx Optimal Control Code

This repository contains MATLAB code implementing the methods described in the paper:

**"Revisiting Lossless Convexification: Theoretical Guarantees for Discrete-time Optimal Control Problems"**

Authors: Dayou Luo, Kazuya Echigo, Behçet Açıkmeşe  (https://arxiv.org/abs/2410.09748)

## Overview

This repository includes MATLAB scripts for applying the lossless convexification (LCvx) method to discrete-time optimal control problems. It focuses on both normal and long-horizon cases, providing three different examples to illustrate the method.

### Examples:

1. **Example 1: Standard LCvx** (`standard_LCvx.m`)  
   This script implements the standard LCvx method as described in the paper, demonstrating how convexification is applied in normal optimal control cases.

2. **Example 2: Perturbation** (`purturbation_example2.m`)  
   This script handles a perturbed system, showing how the LCvx method performs under small disturbances in the system dynamics.

3. **Example 3: Long Horizon** (`lcvx_overrelaxed_main.m`)  
   This script addresses the long-horizon case, where the temporal grid points exceed the state dimension. It includes the over-relaxation technique and bisection search for improved results.

## Dependencies:

- MATLAB 2024 or later
- MoseK
- Yamlip

## How to Use

1. Clone the repository and ensure all dependencies are installed.
2. Run the corresponding `.m` scripts for each example in MATLAB:
   - Example 1: Run `standard_LCvx.m`
   - Example 2: Run `purturbation_example2.m`
   - Example 3: Run `lcvx_overrelaxed_main.m`

## Reference

Luo, D., Echigo, K., Açıkmeşe, B., "Revisiting Lossless Convexification: Theoretical Guarantees for Discrete-time Optimal Control Problems," arXiv:2410.09748, 2024.
