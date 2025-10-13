
---

# MPC Simulation for Differential Drive Robot 

This code implements a **Model Predictive Control (MPC)** simulation for a differential-drive mobile robot.
The objective is to track a predefined trajectory such as a **circle** or **figure-eight** curve with high accuracy while minimizing control effort.

---

## Overview

The system predicts the robot’s future motion over a finite horizon (`N_pred`) and computes an optimal control sequence that minimizes a quadratic cost.
At each step, only the **first control input** is applied, and the process repeats (receding horizon strategy).

The optimization is solved using **`fmincon` (SQP algorithm)** under input constraints on linear and angular velocity.

---

## Simulation Flow

1. **Reference generation**

   * A smooth trajectory (`circle` or `figure_eight`) is generated using kinematic equations.
   * The reference heading angle is computed from the path derivatives and rate-limited for smoothness.

2. **Initialization**

   * The robot starts from the initial point of the trajectory.
   * Weight matrices `Q` and `R` define the balance between tracking accuracy and control smoothness.
   * Optional Gaussian noise can be added to simulate sensor uncertainty.

3. **MPC Loop**

   * At each step, the next `N_pred` reference points are extracted.
   * An optimization problem is solved to minimize:
J = Σ (x_k − r_k)ᵀ Q (x_k − r_k) + u_kᵀ R u_k,   for k = 1 ... N_pred

   * The first control command is applied, and the state is updated using the robot’s kinematic model.
   * The process repeats until the end of the trajectory.

4. **Evaluation**

   * The mean position error and average solver time are calculated.
   * Plots show trajectory tracking, control input evolution, and cost trends.

---

## Parameters

| Category      | Parameter                          | Description                     |
| ------------- | ---------------------------------- | ------------------------------- |
| Time          | `dt`                               | Control period (s)              |
| Horizon       | `N_pred`                           | Prediction horizon length       |
| Trajectory    | `trajectory_type`                  | `'circle'` or `'figure_eight'`  |
| Weights       | `Q`, `R`                           | State and input weight matrices |
| Constraints   | `v_min`, `v_max`, `w_min`, `w_max` | Velocity and turn-rate limits   |
| Noise         | `add_noise`                        | Enable/disable Gaussian noise   |
| Visualization | `realtime_stride`                  | Plotting frequency              |
| Offset        | `test_offsets`                     | Y offset of initial position    |
---

## Extended Evaluation: Horizon Sweep

The script includes an automated **`N_pred` sweep** experiment.
It tests multiple horizon lengths (3–30) under both noise-free and noisy conditions to evaluate:

* **Mean tracking error (m)**
* **Average computation time (ms)**

The results demonstrate the trade-off between control accuracy and real-time feasibility, typically showing optimal performance at `N_pred ≈ 7–10`.

---

## Summary

* **Control method**: Model Predictive Control (receding horizon)
* **Model**: Differential-drive kinematics
* **Solver**: `fmincon (SQP)`
* **Reference**: Circle / Figure-eight trajectory
* **Metrics**: Mean position error, average solve time
* **Result**: Stable tracking within ~0.05 m accuracy and real-time feasibility for `N_pred ≈ 7–10`.

---
