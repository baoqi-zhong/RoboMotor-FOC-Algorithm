# RoboMotor-FOC-Algorithm

![Cover](./Image/cover.jpg)

C++ FOC Algorithm Library for RoboMotor Project.

## Overview

**RoboMotor-FOC-Algorithm** is a high-performance Field Oriented Control (FOC) algorithm library designed for the RoboMotor project. It provides a robust control stack for BLDC motors, emphasizing predictability, simplicity, and efficiency on embedded targets like the STM32G4 series.

## Key Features

- **Predictable Real-Time Execution**  
  The core control loop operates entirely within deterministic interrupt routines. The codebase strictly avoids dynamic memory allocation (`malloc`/`new`) to prevent heap fragmentation and nondeterministic timing.

- **"C-with-Class" Architecture**  
  Written in a classic C++ style that leverages namespaces and classes for clean encapsulation without the overhead of complex inheritance or virtual polymorphism. This approach combines modern organizational benefits with the raw performance and transparency of C.

- **Optimized Control with Feedforward**  
  Implements a standard FOC loop with Park and Clarke transformations, enhanced by feedforward control logic. This improves dynamic response and tracking accuracy under varying load conditions.

- **Modular Design**  
  Decouples the control algorithm from hardware specifics. The `Drivers/` and `Boards/` modules abstract low-level peripherals (FDCAN, Flash, Encoders), allowing the `Control/` logic to remain portable and focused on motion physics.

- **Hardware-Aware Optimizations**  
  Includes specialized utilities for embedded performance:
  - **CORDIC Acceleration**: Efficient trigonometric computations using hardware or fixed-point software CORDIC helpers.
  - **Lookup Table Calibration**: Comprehensive sensor calibration using lookup tables for linearization and error compensation.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
