# 🚀 Multi-Objective Trajectory Optimization
This repository contains the code for the design and optimization of trajectories specifically for Mars Ascent Vehicles (MAVs), using a Multi-Objective Trajectory Optimization (MOTO) framework, with detailed methodology described in [1]. The problem is formulated as an optimal control problem, incorporating a range of dynamic and geometric constraints. An initial Pareto front is generated using a direct weighted product scalarization approach with uniform weights. To improve resolution and better capture the trade-offs between objectives, an adaptive bi-objective weighted product method is implemented, following the refinement procedure described in [2]. Trajectory optimization and discretization are implemented using Pyomo [3], while nonlinear programs are solved using IPOPT. This repository also provides a foundation for more advanced Multidisciplinary Design Optimization (MDO) research related to planetary ascent and autonomous mission planning.

---

### Key Features:

  - Six Degrees of Freedom (6-DoF) Dynamics: The MAV flight is modeled using full 6-DoF equations of motion.

  - Quaternion and Euler Representations: Both are implemented. Due to numerical feasibility and constraint sensitivity, only one can be activated at a time by commenting the other out.

  - Multi-Objective Optimization: Simultaneously maximizes downrange distance and final payload mass (or equivalently minimizes fuel consumption).

  - Pareto Front Generation: Captures trade-offs between objectives using scalarization techniques.

  - Adaptive Weighted Product Method: Refines the Pareto front adaptively for better solution diversity and resolution.

  - Mars-Relevant Physics: Includes aerodynamic and propulsion models tailored for Mars ascent conditions.

---

### Technical Implementation

  - Built with Pyomo, a Python-based optimization modeling framework.

  - Solved using IPOPT, a nonlinear programming solver.

---

### Results and Visualizations

**Pareto Front:**

<img width="500" height="790" alt="image" src="https://github.com/user-attachments/assets/5a953d8f-b606-4172-8524-c29f813e2384" /> <br>

**Sample Optimized Trajectories:**

<img width="500" height="707" alt="image" src="https://github.com/user-attachments/assets/541dce2c-32b3-4cde-9b88-8530612796d3" />

---

### References

[1] Abraham, M. A., Gardi, A., & Kara, O. (2025). Multi-Objective Trajectory Optimization of Mars Hybrid Rockets with In Situ Propellants. Acta Astronautica. https://doi.org/10.1016/j.actaastro.2025.04.036 <br>
[2] Kim, I. Y., & De Weck, O. L. (2005). Adaptive weighted sum method for multiobjective optimization: a new method for Pareto front generation. Structural and Multidisciplinary Optimization, 31(2), 105–116. https://doi.org/10.1007/s00158-005-0557-6 <br>
[3] Schlossman, R., Williams, K., Kozlowski, D., & Parish, J. J. (2021). Open-Source, Object-Oriented, Multi-Phase pseudospectral optimization using Pyomo. AIAA SCITECH 2022 Forum. https://doi.org/10.2514/6.2021-1951 <br>
