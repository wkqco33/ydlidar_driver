# Contributing to YDLidar X4 Pro ROS 2 Driver

We welcome contributions from the community! This document outlines the guidelines and workflow for contributing to this project.

## Development Principles & Philosophy

1. **Lightweight & Dependency-Minimal**: Maintain zero external SDK dependencies. Keep direct serial communication fast, predictable, and maintainable.
2. **TDD (Test-Driven Development)**: Write or update unit tests for any protocol or data processing changes before/alongside feature implementation.
3. **High Code Quality & Consistency**: Adhere to ROS 2 C++ standards and format code with the provided `.clang-format`.

---

## Getting Started

### 1. Fork & Clone
```bash
git clone https://github.com/<your-username>/ydlidar_driver.git
cd ydlidar_driver
```

### 2. Build & Run Tests
Ensure you have a ROS 2 environment sourced (e.g. Jazzy or Humble):
```bash
colcon build --packages-select ydlidar_driver --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ydlidar_driver
colcon test-result --verbose
```

### 3. Code Formatting
Format all C++ and header files before submitting a pull request:
```bash
clang-format -i src/*.cpp src/*.hpp test/*.cpp
```

---

## Pull Request Guidelines

1. **Atomic Commits**: Create small, focused commits with descriptive commit messages (e.g. `feat: add checksum validation`, `fix: serial timeout handling`).
2. **Include Tests**: If you fix a bug or add a feature, provide test cases under `test/`.
3. **No Sensitive Data**: Never commit hardcoded secrets, private IPs, credentials, or large binary files.
4. **Documentation**: Update `README.md` and comments if configuration parameters, topics, or services change.

Thank you for helping improve the YDLidar driver!
