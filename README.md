# Overview
This repository contains the prototype implementation of the data-driven DCARP framework using SUMO, a traffic simulation software. The main concept of the framework is presented in the paper __*A Data-driven Solution for The Dynamic Capacitated Arc Routing Problem*__ by Zsuzsanna Nagy and Agnes Werner-Stark. The paper is available [here](https://www.conferences-scientific.cz/file/9788088203247) (pp. 64-83).

This repository is a modified version of [this repository](https://github.com/HawkTom/DCARP-SUMO-Simulation).

# Added files
- [optimizer_utils.py](utils/optimizer_utils.py): Contains the Python implementations of the following algorithms:
  - Minimal Rerouting (RR1) algorithm.
  - Artificial Bee Colony algorithm for CARP (CARP-ABC).
  - Hybrid Metaheuristic Approach (HMA).
  - Hybrid Local Search (HyLS) algorithm.
- [julia_utils.py](utils/julia_utils.jl): Contains the Julia implementations of the following algorithms:
  - Minimal Rerouting (RR1) algorithm.
  - Artificial Bee Colony algorithm for CARP (CARP-ABC).
  - Hybrid Metaheuristic Approach (HMA).
- [main1.py](main1.py): A modified version of the `main.py` file that uses the Python implementations of the (D)CARP solvers.
- [main2.py](main2.py): A modified version of the `main.py` file that uses the Julia implementations of the (D)CARP solvers.
