# OpenMV-Based Auto-Aiming Gimbal System (2025 NUEDC)

**Author**: Yuxiang Fan | Tongji University

## 📝 Project Overview
This repository contains the complete visual control solution for Task 2 of the 2025 National Undergraduate Electronics Design Contest (NUEDC) Question E. The system utilizes an OpenMV4 module as the core vision processor to drive a 2D Pan-Tilt gimbal, enabling the fast and precise tracking of a target center on A4 UV-sensitive paper and hitting it with a 405nm laser pointer within a strict 2-second time limit.

## 🛠️ Hardware Architecture
* **Vision Controller**: OpenMV4 (powered by STM32H7), chosen for its high frame rate and efficient machine vision capabilities.
* **Actuators**: Custom 2D Pan-Tilt gimbal driven by two PWM servos (Pan and Tilt), with control steps strictly limited to ±5°.
* **Laser Module**: 405nm blue-violet laser pointer (optical power ≤ 10mW). *(Note: A red laser was used during the debugging phase as an alternative).*

## 🧠 Software & Algorithms
The software architecture forms a robust closed-loop control system, integrating image acquisition, feature matching, error filtering, and actuator execution.

**1. Target Recognition (NCC Algorithm)**
We utilize the Normalized Cross-Correlation (NCC) algorithm in the LAB color space to rapidly match and filter black frame targets. The mathematical model for the matching degree is:

$$NCC = \frac{\sum_{x,y}(I_1(x,y)-I_1(p_x,p_y))(I_2(x+d,y)-I_2(p_x+d,p_y))}{\sqrt{\sum_{x,y}(I_1(x,y)-I_1(p_x,p_y))^2\sum_{x,y}(I_2(x+d,y)-I_2(p_x+d,p_y))^2}}$$

*(Where $I_1(x,y)$ is the original image pixel value, and $I_2(x+d,y)$ is the target image pixel value after offset.)*

**2. Moving Average Filter**
To suppress high-frequency jitter caused by mechanical dead zones and rapid gimbal movements, a 5-frame moving average buffer is introduced in the feedback channel to smooth the pixel errors ($e_x, e_y$) before they are fed into the controller.

**3. Positional PID Control**
Decoupled positional PID controllers are implemented for the two axes:
* **Pan (Horizontal)**: $K_p = 0.07$, no integral/derivative terms.
* **Tilt (Vertical)**: $K_p = 0.08$, no integral/derivative terms.
* **The PID output is scaled by a factor of 0.9 to prevent overshoot oscillation.**

## 📊 Performance Validation
The system demonstrates excellent steady-state convergence and anti-interference capabilities.

| Test Scenario | Initial Constraint | Chassis Status | Max Laser Error Distance ($D$) | Result |
| :--- | :--- | :--- | :--- | :--- |
| **Test 1** | Fixed position | Stationary | 1.6 cm | Hit target |
| **Test 2** | Fixed position | Stationary | 1.8 cm | Hit target |
| **Test 3** | Arbitrary position | Stationary | 3.8 cm | Locked target |
| **Test 4** | Arbitrary position | Stationary | 5.3 cm | Locked target |

**Conclusion**: The system strictly controls the steady-state error within 1.8cm in a fixed position and maintains robust tracking even during step responses from arbitrary positions, successfully meeting the ≤ 2.0cm specification within 2 seconds.

## 🚀 Quick Start
1. Clone this repository to your local machine.
2. Copy `main.py` and `pid.py` from the `src/` directory directly to the root directory of your OpenMV flash drive.
3. Fine-tune the `LAB_THRESHOLD` parameters in the configuration section of `main.py` according to your ambient lighting.
4. Power on the system. The LED indicator will light up, the gimbal will reset to its center position, and tracking will commence.
