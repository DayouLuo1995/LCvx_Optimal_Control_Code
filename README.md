LCvx Optimal Control Code
This repository contains MATLAB code implementing the methods described in the paper:

"Revisiting Lossless Convexification: Theoretical Guarantees for Discrete-time Optimal Control Problems"

Authors: Dayou Luo, Kazuya Echigo, Behçet Açıkmeşe
Date: October 14, 2024

Abstract
This code demonstrates the lossless convexification (LCvx) method for discrete-time optimal control problems, as detailed in the paper. The method transforms nonconvex optimal control problems with control constraints into convex problems using convex relaxations. The repository includes three examples that illustrate how LCvx is applied to both normal and long-horizon cases, incorporating perturbations and bisection search.

Features:
Standard LCvx example: A basic example demonstrating the LCvx process.
Perturbation example: An extension that considers small perturbations to system dynamics.
Long-horizon example: Demonstrates the LCvx approach for longer time horizons and temporal grid points.
Dependencies:
MATLAB 2024 or later
MoseK
Yamlip
How to Use:
Clone the repository and run the scripts for the corresponding examples.
Make sure all dependencies are installed for the code to function properly.
Reference:
Luo, D., Echigo, K., Açıkmeşe, B., "Revisiting Lossless Convexification: Theoretical Guarantees for Discrete-time Optimal Control Problems," [Journal Name], 2024.

