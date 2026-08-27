# AGENTS.md — Development & Contribution Guide for AI Agents & Developers

This document serves as the standard engineering guideline for AI coding agents and human contributors working on the `ydlidar_driver` repository. All future modifications should strictly adhere to these practices.

---

## 1. Project Architecture & Philosophy

The `ydlidar_driver` package is an ultra-lightweight, high-performance ROS 2 driver for the **YDLidar X4 Pro**.

### Core Architecture
- **No External SDK**: Communicates directly over UART via POSIX `termios`/`ioctl`.
- **Decoupled Core Library**:
  - `x4pro_serial.hpp / .cpp`: Direct serial port management, custom baudrate support (`BOTHER`), DTR motor control, and buffered I/O.
  - `x4pro_protocol.hpp / .cpp`: Pure protocol encoding/decoding, packet structs, angle interpolation, and checksum verification. Completely hardware/ROS independent for easy unit testing.
  - `x4pro_lidar.hpp / .cpp`: Lidar device state machine, background packet stream processing, and callback dispatch.
  - `ydlidar_node.cpp`: ROS 2 Node lifecycle, parameter management, TF frame assignment, services (`start_scan`, `stop_scan`), and `sensor_msgs::msg::LaserScan` publishing.

---

## 2. Test-Driven Development (TDD) Workflow

When fixing bugs, refactoring, or introducing new protocol features:

### Step-by-Step TDD Process:
1. **Red**: Write a failing test in `test/test_*.cpp` that asserts the expected behavior.
2. **Green**: Implement the minimal necessary logic in `src/` to make the test pass.
3. **Refactor**: Clean up the implementation, optimize performance, and ensure tests remain green.

### Running Tests:
```bash
# Build with testing enabled
colcon build --packages-select ydlidar_driver --cmake-args -DBUILD_TESTING=ON

# Run test suite
colcon test --packages-select ydlidar_driver

# View test summary & verbose results
colcon test-result --verbose
```

---

## 3. Code Style & Formatting Guidelines

- **C++ Standard**: C++17
- **Style Standard**: Google C++ Style Guide / ROS 2 Standards
- **Formater**: Run `clang-format -i <file>` using the root `.clang-format` before finalizing changes.
- **Header Files**: Use `#pragma once`. Include paths should be clean and structured.
- **Documentation**: Use Doxygen-style docstrings (`/// @brief ...`, `/// @param ...`, `/// @return ...`).
  - **Avoid verbose narrations, agent self-talk, or redundant commentary.**
  - Keep comments concise, factual, and informative.

---

## 4. Performance & Memory Guidelines

- **Zero-Copy Publishing**: Use `std::make_unique<sensor_msgs::msg::LaserScan>()` and publish via `std::move(msg)`.
- **Buffered I/O**: Do not invoke `read()` syscalls byte-by-byte in tight loops. Use block reads and parse through sliding/ring buffers.
- **Memory Preallocation**: Always `reserve()` vectors when the maximum or estimated capacity is known (e.g. `points.reserve(360)`).
- **Fast Arithmetic**: Avoid repeated trigonometric operations or iterative loops where closed-form arithmetic applies (e.g. angle normalization and linear interpolation).

---

## 5. Security & Hygiene Checklist

Before submitting changes:
- [ ] No hardcoded tokens, passwords, private IPs, or API keys.
- [ ] No large binary files or IDE artifacts committed.
- [ ] `.gitignore` and `.dockerignore` properly updated.
- [ ] All tests pass (`colcon test`).
- [ ] `clang-format` applied.
