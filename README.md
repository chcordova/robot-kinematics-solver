# Robot Kinematics Solver

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

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

The mathematical model is built upon the **Denavit-Hartenberg (DH)** convention, a standard method for modeling robotic manipulators. The project includes a visual simulator (planned for later phases) to display the arm's movement and validate the kinematic calculations.

## Key Features

- **Forward Kinematics (FK):** Calculates the end-effector position (x, y, z) from given joint angles (θ₁, θ₂, θ₃).
- **Inverse Kinematics (IK):** Solves for the required joint angles (θ₁, θ₂, θ₃) to reach a specified target position (x, y, z) using a geometric approach suitable for a 3-DOF arm.
- **Denavit-Hartenberg Modeling:** Utilizes the DH convention to establish a systematic coordinate frame for each link of the robot.
- **Modular Python Implementation:** Well-structured code for easy understanding and potential expansion.

## Getting Started

To get a local copy up and running, follow these steps.

### Prerequisites

Make sure you have Python 3 and pip installed on your system.

- [Python 3.x](https://www.python.org/downloads/)
- [pip](https://pip.pypa.io/en/stable/installing/)

You will also need the following Python libraries, which can be installed using pip:

- [NumPy](https://numpy.org/) (for numerical operations, especially matrix manipulation)
- [Matplotlib](https://matplotlib.org/) (for plotting and potential visualization in later stages)

### Installation

1.  Clone the repository to your local machine:
    git clone [https://github.com/your\_username/robot-kinematics-solver.git](https://github.com/chcordova/robot-kinematics-solver.git)
2.  Navigate to the project directory:
    cd robot-kinematics-solver
3.  It is recommended to create a virtual environment to manage dependencies:
    python3 -m venv venv
    source venv/bin/activate  # On Linux/macOS
    venv\Scripts\activate.bat  # On Windows
4.  Install the required Python libraries:
    pip install -r requirements.txt

    *(Create a file named `requirements.txt` in your project root directory with the following content:)*
    
    
    numpy
    matplotlib

## Usage

1.  Ensure you are in the project directory (and have activated the virtual environment, if used).
2.  Run the main script (the name might vary depending on your primary implementation file, e.g., `kinematics_solver.py` or `main.py`):
    python [your_main_script_name.py]

    *(Replace `your_main_script_name.py` with the actual name of your Python script.)*

3.  Follow the prompts or interact with the script's functions to calculate forward and inverse kinematics. The script will likely take joint angles as input for FK and target coordinates as input for IK, displaying the calculated results in the terminal.

*(Remember to update this section with specific instructions on how to use your actual program once it's more developed.)*

## Roadmap

- [x] **Phase 1: Virtual Design & Kinematic Fundamentals (Completed: June 23, 2024)**
    - [x] Defined the virtual robot's geometry (link lengths: [Specify your link lengths, e.g., L1=10cm, L2=15cm, L3=12cm]).
    - [x] Assigned coordinate systems to each joint based on the Denavit-Hartenberg (DH) convention.
    - [x] Created and verified the Denavit-Hartenberg (DH) parameter table for the 3-DOF arm:

        | Link | θᵢ      | dᵢ   | aᵢ      | αᵢ      |
        |------|---------|------|---------|---------|
        | 1    | θ₁      | d₁   | a₁      | α₁      |
        | 2    | θ₂      | d₂   | a₂      | α₂      |
        | 3    | θ₃      | d₃   | a₃      | α₃      |

        *(Replace the placeholders in the table above with your actual DH parameters.)*

- [ ] **Phase 2: Simulator Development & Applied Kinematics (Due: June 30, 2024)**
    - [ ] Implement the Forward Kinematics function in Python using NumPy for matrix operations.
    - [ ] Develop and implement the geometric solution for the Inverse Kinematics problem for the 3-DOF arm.
    - [ ] Choose and integrate a Python library (e.g., Matplotlib) for basic 2D visualization of the arm's configuration.
    - [ ] Validate the accuracy of the FK and IK implementations through testing and visualization.

- [ ] **Phase 3: Final Report & Presentation (Due: July 02, 2024)**
    - [ ] Write the comprehensive final report, detailing the project's objectives, methodology (including the DH parameters and kinematic equations), implementation details, and results.
    - [ ] Prepare a presentation summarizing the project and demonstrating the functionality of the kinematics solver.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

**\[Your Full Name(s)]**

- GitHub: \[[Your GitHub Profile URL](https://github.com/your_username)]
- Email: \[your\_email@example.com]

Thank you for checking out the Robot Kinematics Solver! Feel free to contribute or reach out with any questions.
