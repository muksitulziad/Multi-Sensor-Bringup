# Multi-Sensor Bringup & Diagnostics

A modular collection of unified hardware bringup scripts designed to verify sensor health and demonstrate reliable data acquisition across embedded boards and robotics platforms.

## Unified Bringup Workflow
Each sensor module in this repository is self-contained inside a single standalone script that handles the complete bringup pipeline from start to finish:

1. **Hardware & Connection Verification:** Automatically probes the target port/bus (USB, UART, I2C, SPI), verifies baud rate alignment, and confirms valid packet streaming (`[PASS]` / `[FAIL]`).
2. **Clean Data Acquisition:** Once verified, initializes a structured, reusable reader interface that handles framing/timeouts and outputs clean, parsed sensor readings ready for integration into larger robotics or embedded projects.

## Repository Structure
* `lidar/lidar_bringup.py` — 2D/3D LiDAR port check and laser scan array acquisition
* `imu/imu_bringup.py` — IMU register/stream check and orientation/acceleration readout
* `gps/gps_bringup.py` — GNSS NMEA/RTK fix verification and coordinate parsing
* `cameras/camera_bringup.py` — Camera device check, FPS verification, and frame grabber
* `encoders/encoder_bringup.cpp` — Quadrature pulse verification and velocity/odometry calculation
