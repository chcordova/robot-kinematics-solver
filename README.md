# Robot Kinematics Solver

> A simulator and kinematics solver for a 3-DOF articulated robotic arm, featuring implementations of Forward Kinematics (FK) and Inverse Kinematics (IK) using the Denavit-Hartenberg convention. This project was developed as part of a university course on computational resources.

## Table of Contents

- [About The Project](#about-the-project)
- [Key Features](#key-features)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#usage)
- [Roadmap](#roadmap)
- [License](#license)
- [Author](#author)

## About The Project

This project provides a Python-based solution for analyzing the motion of a 3-DOF (Degrees of Freedom) robotic arm. It allows users to calculate the end-effector's position from given joint angles (Forward Kinematics) and to determine the required joint angles to reach a desired position (Inverse Kinematics).

The mathematical model is built upon the **Denavit-Hartenberg (DH)** convention, a standard method for modeling robotic manipulators. The project includes a visual simulator to display the arm's movement and validate the kinematic calculations.

## Key Features

- **Forward Kinematics (FK):** Calculates the end-effector position and orientation from joint angles.
- **Inverse Kinematics (IK):** Solves for the joint angles required to reach a specific target point (using a geometric approach).
- **Denavit-Hartenberg Modeling:** Systematic parameterization of the robot's geometry.
- **2D/3D Visualization:** A simple visual interface to simulate and validate the arm's movements.

## Getting Started

To get a local copy up and running, follow these simple steps.

### Prerequisites

Make sure you have Python 3 and pip installed.
- Python 3.x
- pip

You will also need the libraries listed in the `requirements.txt` file.
```bash
pip install numpy matplotlib
