# src/robot_kinematics_solver/__init__.py

"""
Robot Kinematics Solver Package

Un paquete de Python para analizar y simular la cinemática de un brazo
robótico de 3 GDL utilizando múltiples enfoques algorítmicos.
"""

# Se exponen las clases y funciones principales para que puedan ser
# importadas directamente desde el paquete.
# Ejemplo de importación: from robot_kinematics_solver import RobotArm
from .kinematics import RobotArm, KinematicsSolver, GeometricSolver, MatrixDHSolver, RTBSolver
from .kinematics import plot_arm, run_demonstration

__version__ = "1.0.0"