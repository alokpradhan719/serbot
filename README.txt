# SafePath — Autonomous Dynamic Robot

**An ESP32-Based Autonomous Mobile Robot with Dynamic Obstacle Avoidance, Occupancy-Grid Mapping, A* Path Planning, and Web-Based Monitoring**

## Project Overview

SafePath is an autonomous dynamic robot project designed to navigate indoor environments, detect obstacles, update its environment map, and plan paths toward a destination. The system combines ultrasonic sensors, servo-based scanning, motor control, Wi-Fi communication, occupancy-grid mapping, and A* path planning.

The ESP32 handles low-level hardware operations, including sensor acquisition, servo control, motor control, Wi-Fi communication, and local safety logic. A web-based dashboard supports environment mapping, path planning, mission management, calibration, and real-time robot monitoring.

The objective of SafePath is to develop and demonstrate an autonomous mobile robotic system capable of responding to environmental changes and navigating around detected obstacles.

## Project Name

**SafePath — Autonomous Dynamic Robot**

## Key Features

- Dynamic obstacle detection using ultrasonic sensors.
- Servo-based scanning using the front ultrasonic sensor.
- Motor control through an L298N motor driver.
- Occupancy-grid environment mapping.
- A* algorithm for path planning.
- Obstacle-aware route planning and replanning.
- Wi-Fi-based communication between the ESP32 and dashboard.
- Web-based robot monitoring, mapping, and mission controls.
- Local safety logic and stop commands.

*Note: Actual autonomous navigation and dynamic obstacle avoidance must be validated through physical hardware testing.*

## Technologies Used

- ESP32 and Arduino framework
- C/C++
- HTML, CSS, and JavaScript
- Ultrasonic sensors and servo motor
- L298N motor driver and DC motors
- Wi-Fi and HTTP API
- Occupancy-grid mapping
- A* path-planning algorithm

## Repository

GitHub: https://github.com/alokpradhan719/serbot
