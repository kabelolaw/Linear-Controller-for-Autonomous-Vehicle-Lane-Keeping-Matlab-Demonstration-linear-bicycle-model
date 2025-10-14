
---

# Linear Controller for Autonomous Vehicle Lane-Keeping

**MATLAB/Simulink Demonstration – Linear Bicycle Model, LQR Control**



## 📘 Project Overview

This project demonstrates how to design and simulate a **Linear Quadratic Regulator (LQR)** controller for an **autonomous vehicle lane-keeping system** using MATLAB and Simulink.

It’s based on the **linear bicycle model**, a simplified representation of a vehicle’s lateral dynamics, and shows how LQR control can minimize lateral and heading errors while keeping steering smooth and stable.

If you’re new to vehicle dynamics or control systems — don’t worry! This project is designed to help you understand **how MATLAB and Simulink work together** to create, tune, and visualize a linear control system.


## 🧠 What You’ll Learn

By the end of this project, you’ll understand:

* How to **model vehicle lateral dynamics** using the **linear bicycle model**.
* How to **derive state-space equations** for autonomous lane-keeping.
* How to **design and tune an LQR controller** using MATLAB.
* How MATLAB and Simulink **integrate seamlessly** for simulation and testing.
* How to balance **fast convergence**, **smooth control**, and **aggressive response** through LQR weight tuning.

## ⚙️ System Model

The **state-space model** used in this project represents the lateral dynamics of a car.
The state vector is defined as:

[
x = [e, \psi, v_y, r]^T
]

Where:

* **e** = lateral position error (distance from lane center)
* **ψ** = heading angle error (difference between vehicle heading and lane direction)
* **v_y** = lateral velocity
* **r** = yaw rate

The simplified dynamic equations are:

```
ė = v * ψ + v_y
ψ̇ = r - v * κ
```

These describe how the vehicle’s lateral position and heading angle change over time based on speed (v), curvature (κ), and motion dynamics.

## MATLAB–Simulink Workflow

This project uses both MATLAB and Simulink, each playing a specific role:

### 🔹 MATLAB

* Defines **vehicle parameters** (mass, geometry, tire stiffness, etc.).
* Derives the **state-space matrices** A, B, C, and D.
* Computes the **LQR gain matrix** using:

  ```matlab
  [K_lqr, S, poles] = lqr(A, B, Q, R);
  ```
* Saves all parameters into a `.mat` file:

  ```matlab
  save('lateral_lqr_params.mat');
  ```

### Simulink

* Loads the `.mat` file from MATLAB.
* Implements the **closed-loop system** using the state-space block.
* Uses an **LQR Controller block** that applies `u = -K_lqr * x`.
* Includes **saturation limits** to restrict steering to realistic values (±0.5 rad).
* Displays results like **lateral deviation**, **yaw rate**, and **steering angle** on scope blocks.

This setup allows you to adjust the LQR weights in MATLAB and instantly see how the vehicle behaves in Simulink — no manual data entry needed!


## ⚖️ LQR Weight Tuning Explained

The LQR controller minimizes the cost function:

[
J = \int (x^T Q x + u^T R u) dt
]

Where:

* **Q** = State weighting matrix (controls how strongly errors are penalized).
* **R** = Input weighting (controls how much we penalize large steering angles).

### Key Tuning Insights

* Increasing **qₑ** or **q_ψ** → Faster convergence to lane center and heading, but steering becomes aggressive.
* Increasing **R** → Smoother steering, less aggressive control, but slower convergence.
* Balanced tuning gives **stable**, **smooth**, and **responsive** lane-keeping.

You can experiment with the following sample weights:

```matlab
q_e = 5; 
q_psi = 10; 
q_vy = 1; 
q_r = 1; 
Q = diag([q_e, q_psi, q_vy, q_r]);
R = 500;
```

## 📈 Simulation Results (What You’ll Observe)

After running the Simulink model, you should see:
✅ Lateral error converging smoothly to zero.
✅ Heading error stabilizing quickly.
✅ Steering input remaining within realistic bounds.
✅ No oscillations or instability (if weights are tuned correctly).


## 🔍 How to Run the Project

1. **Open MATLAB** and run the script:

   ```matlab
   lateral_lqr_init.m
   ```

   This will save all required parameters to `lateral_lqr_params.mat`.

2. **Open Simulink** and load:

   ```
   LaneKeeping_LQR.slx
   ```

3. **Run the simulation** (Press ▶).
   Observe the lane-keeping performance in the scope outputs.

4. Adjust the LQR weights in the MATLAB file and re-run the simulation to see how the system behavior changes.


## Discussion

This project demonstrates the trade-offs between **tracking performance** and **control smoothness**.

* High Q → precise tracking, more aggressive control.
* High R → smoother steering, slower convergence.

By adjusting these parameters, you learn how to design controllers that balance **comfort**, **stability**, and **accuracy** — just like in real-world autonomous driving systems.

## 🧾 References and Further Reading

* Rajamani, R. (2012). *Vehicle Dynamics and Control*. Springer.
* Gáspár, P., et al. (2018). *Design of Robust Vehicle Steering Control*.
* González, D., Pérez, J., et al. (2016). *A Review of Motion Planning Techniques for Automated Vehicles*.
* MATLAB Documentation – [LQR Controller Design](https://www.mathworks.com/help/control/ref/lqr.html)

## 🤖 Author

Developed as part of a research assignment for Linear Contol module(EEEN 411) on:
**“Linear Controller for Autonomous Vehicle Lane-Keeping: MATLAB/Simulink Demonstration – Linear Bicycle Model, LQR (Autonomous Vehicles)”**


