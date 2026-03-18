# OpenMV-Based Auto-Aiming Gimbal System (2025 NUEDC)

**Author**: Yuxiang Fan | Tongji University

## 📝 Project Overview
This repository provides a vision-based control implementation for Task 2 of the 2025 National Undergraduate Electronics Design Contest (NUEDC), Question E. The system utilizes an OpenMV4 module as the primary vision processor to coordinate a 2D Pan-Tilt gimbal. The design objective is to identify target centers on A4 UV-sensitive paper and direct a 405nm laser pointer to the target within a 2-second interval.

## 🛠️ Hardware Configuration
* **Vision Processor**: OpenMV4 (STM32H7-based), utilized for image acquisition and real-time feature processing.
* **Actuators**: A 2D Pan-Tilt gimbal driven by two PWM-controlled servos. Servo movements are constrained to ±5° increments per control cycle.
* **Laser Module**: 405nm blue-violet laser (optical power ≤ 10mW). A red laser was employed for calibration during the development phase.

## 🧠 Control Logic & Algorithms
The system implements a closed-loop control architecture integrating feature recognition, signal filtering, and positional adjustment.

### 1. Target Recognition (NCC Algorithm)
The implementation employs the Normalized Cross-Correlation (NCC) algorithm within the LAB color space to identify rectangular frames. The matching degree is defined by the following mathematical model:

$$NCC = \frac{\sum_{x,y}(I_1(x,y)-\bar{I}_1)(I_2(x+d,y)-\bar{I}_2)}{\sqrt{\sum_{x,y}(I_1(x,y)-\bar{I}_1)^2 \sum_{x,y}(I_2(x+d,y)-\bar{I}_2)^2}}$$

*(Where $I_1$ represents the source image pixel values and $I_2$ represents the target template values after coordinate offset $d$.)*

### 2. Error Filtering
To mitigate high-frequency jitter resulting from mechanical backlash or rapid transitions, a 5-frame moving average filter is applied to the calculated pixel errors ($e_x, e_y$) prior to the control stage.

### 3. Positional PID Control
Decoupled proportional control is applied to the horizontal and vertical axes:
* **Pan (Horizontal)**: $K_p = 0.07$
* **Tilt (Vertical)**: $K_p = 0.08$
* **Output Scaling**: A damping factor of $0.9$ is applied to the final output to reduce potential overshoot and oscillation during convergence.

## 📊 Experimental Results
The following table summarizes the measured performance across different test scenarios.

| Scenario | Initial Condition | Target Status | Max Deviation ($D$) | Observation |
| :--- | :--- | :--- | :--- | :--- |
| **Test 1** | Fixed position | Stationary | 1.6 cm | Successful tracking |
| **Test 2** | Fixed position | Stationary | 1.8 cm | Successful tracking |
| **Test 3** | Arbitrary offset | Stationary | 3.8 cm | Target locked |
| **Test 4** | Arbitrary offset | Stationary | 5.3 cm | Target locked |

**Summary**: Experimental data indicates that the steady-state error is maintained within 1.8 cm at fixed positions. The system demonstrates reliable convergence from arbitrary starting coordinates, meeting the requirement of ≤ 2.0 cm deviation within the specified 2-second timeframe.

## 🚀 Deployment
1. Clone the repository to the local environment.
2. Transfer `main.py` and `pid.py` from the `src/` directory to the root directory of the OpenMV module.
3. Adjust the `LAB_THRESHOLD` parameters in `main.py` to account for specific ambient lighting conditions.
4. Upon power-up, the system undergoes a centering routine before entering the active tracking state.
