# 🐁 Autonomous Micromouse Robot

Source code and control logic for an autonomous Micromouse robot. This project is engineered to navigate, map, and solve complex mazes efficiently by combining hardware optimization with advanced pathfinding and closed-loop control systems.

## 🛠️ Hardware Specifications
* **Microcontroller:** ESP32 development board (tailored to custom PCB design).
* **Sensors:** Distance sensor array (e.g., VL53L0X TOF or IR sensors) for rapid wall detection.
* **Actuators:** N20 micro DC gear motors equipped with high-resolution encoders for odometry.
* **Motor Driver:** TB6612 motor driver IC.
* **Chassis & Mechanics:** Custom PCB designed in Altium Designer, integrating round-headed screws as low-friction free wheels (casters) to ensure smooth pivot turns and minimize drag.

## 🧠 Algorithms & Control Theory
* **Maze Navigation:** Implements timed pathfinding algorithms to explore the maze, map the walls, and calculate the shortest route to the center.
* **Path Optimization:** Utilizes fitness functions to evaluate and refine the optimal trajectory for the final "speed run" after the initial mapping phase.
* **Closed-Loop Kinematics:** Advanced PID control system—modeled using Laplace transforms and block diagrams—to maintain accurate wall-following, minimize steady-state errors, and execute precise 90°/180° rotational maneuvers.

## 📸 Media & PCB Design
<img width="1927" height="2560" alt="image" src="https://github.com/user-attachments/assets/63b8e8f2-afd3-4fd3-83f1-618929fb3077" />

## ⚙️ Setup & Deployment
1. Open the source code in your preferred C++ environment (e.g., Arduino IDE, STM32CubeIDE, or PlatformIO).
2. Configure the hardware pinouts corresponding to the custom PCB schematics.
3. Tune the PID parameters (`Kp`, `Ki`, `Kd`) and adjust the fitness function variables for optimal straight-line speed and turning stability.
4. Compile and upload the firmware to the microcontroller.
