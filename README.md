# Robot Kinematics Solver (Solucionador de Cinemática Robótica)

[![Python: 3.11+](https://img.shields.io/badge/Python-3.11+-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![made-with-python](https://img.shields.io/badge/Made%20with-Python-1f425f.svg)](https://www.python.org/)

Un simulador en Python para analizar, comparar y visualizar la cinemática de un brazo robótico de 3 Grados de Libertad (GDL). El proyecto implementa múltiples enfoques de solución y está diseñado con una arquitectura de software modular y extensible.

Este proyecto fue desarrollado como parte del curso de **Recursos Computacionales**, con el objetivo de aplicar conceptos teóricos de álgebra lineal, cinemática y patrones de diseño de software en una implementación funcional.

![Demo del Simulador](https://i.imgur.com/g8e1O7U.jpg) 
*(Te sugiero reemplazar esta URL con una captura de pantalla de tu propio gráfico final)*

---

## 🏛️ Arquitectura del Software

El proyecto está estructurado como un paquete de Python y sigue el **Patrón de Diseño Strategy** para permitir la intercambiabilidad de los algoritmos de cinemática.

* **`src/robot_kinematics_solver/`**: Es el paquete principal de Python.
    * `kinematics.py`: Contiene toda la lógica del proyecto, incluyendo:
        * `KinematicsSolver`: Una clase base abstracta que define la interfaz común para todos los solvers.
        * `GeometricSolver`, `MatrixDHSolver`, `RTBSolver`: Implementaciones concretas de cada estrategia de solución.
        * `RobotArm`: La clase de contexto que utiliza un solver específico para realizar los cálculos.
    * `__init__.py`: Convierte el directorio en un paquete de Python y expone las clases principales.

* **`notebooks/`**: Contiene los cuadernos de Jupyter (`.ipynb`) utilizados para la exploración, pruebas y análisis interactivo de cada enfoque cinemático.

* **`docs/`**: Almacena la documentación teórica del proyecto, como el archivo `Análisis de Cinemática.pdf`.

---

## 🚀 Cómo Empezar

Sigue estos pasos para configurar el entorno con Conda y ejecutar el simulador.

### Prerrequisitos

* Tener **Miniconda** o **Anaconda** instalado.
* Una versión de Python estable como la 3.11 o 3.12.

### Instalación

1.  **Clona el repositorio:**
    ```sh
    git clone [https://github.com/chcordova/robot-kinematics-solver.git](https://github.com/chcordova/robot-kinematics-solver.git)
    cd robot-kinematics-solver
    ```

2.  **Crea y activa el entorno de Conda:**
    Este comando crea un ambiente llamado `robotica` con Python 3.12 y una versión de NumPy compatible.
    ```sh
    conda create --name robotica python=3.12 "numpy<2" -y
    conda activate robotica
    ```

3.  **Instala las dependencias:**
    El archivo `requirements.txt` contiene todas las librerías necesarias.
    ```sh
    pip install -r requirements.txt
    ```

---

## 🛠️ Uso del Simulador

Una vez que el entorno está configurado y activado, puedes ejecutar la suite de demostración completa.

El script probará automáticamente las tres estrategias de solución (`matrix`, `geometric`, `rtb`) y generará los gráficos comparativos para cada una.

**Ejecuta el script como un módulo de Python desde la carpeta raíz del proyecto:**

```sh
python -m src.robot_kinematics_solver.kinematics
```

---

## 📄 Licencia

Este proyecto está bajo la Licencia MIT. Consulta el archivo `LICENSE` para más detalles.