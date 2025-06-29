# Robot Kinematics Solver (Solucionador de Cinemática Robótica)

[![Python: 3.8+](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Un simulador en Python para analizar y visualizar la cinemática directa e inversa de un brazo robótico de 3 Grados de Libertad (GDL) utilizando un enfoque matricial basado en Denavit-Hartenberg (D-H).

Este proyecto fue desarrollado como parte del curso de **Recursos Computacionales**, con el objetivo de aplicar conceptos teóricos de álgebra matricial y robótica en una implementación funcional.

![Demo del Simulador](https://i.imgur.com/eBwFKEG.png)  ---

## 🚀 Cómo Empezar

Sigue estos pasos para tener una copia local del proyecto y ejecutar el simulador.

### Pre-requisitos

Asegúrate de tener instalado Python 3.8 o superior.

### Instalación

1.  **Clona el repositorio:**
    ```sh
    git clone [https://github.com/](https://github.com/)[TU_USUARIO]/robot-kinematics-solver.git
    cd robot-kinematics-solver
    ```

2.  **Crea un entorno virtual (Recomendado):**
    ```sh
    python -m venv venv
    source venv/bin/activate  # En macOS/Linux
    # venv\Scripts\activate   # En Windows
    ```

3.  **Instala las dependencias:**
    Este proyecto utiliza las librerías listadas en el archivo `requirements.txt`.
    ```sh
    pip install -r requirements.txt
    ```

---

## 🛠️ Uso del Simulador

El script principal se encuentra en la carpeta `src`. Para ejecutar la demostración que calcula la cinemática directa e inversa y muestra los gráficos:

```sh
python src/robot_arm_kinematics.py
```

Esto ejecutará el bloque `if __name__ == "__main__":`, que contiene un ejemplo de uso completo y la verificación de los algoritmos.

---

## 🏛️ Enfoque Arquitectónico

El proyecto está estructurado para separar la lógica del robot de su ejecución y análisis.

* **`src/robot_arm_kinematics.py`**: Contiene la clase principal `RobotArm`. Esta clase encapsula toda la lógica matemática (cinemática directa e inversa) y las propiedades geométricas (longitudes, parámetros D-H) del brazo robótico. También incluye una función de utilidad `plot_arm` para la visualización.

* **`notebooks/`**: Esta carpeta contiene cuadernos de Jupyter (`.ipynb`) utilizados para la exploración, desarrollo y presentación de los diferentes enfoques (geométrico, matricial, etc.). Permiten un análisis interactivo de los algoritmos.

* **`docs/`**: Contiene la documentación teórica del proyecto, incluyendo el desarrollo matemático detallado en el archivo `Análisis de Cinemática.pdf`.

---

## 📄 Licencia

Este proyecto está bajo la Licencia MIT. Consulta el archivo `LICENSE` para más detalles.

*(Recuerda crear un archivo LICENSE en tu repositorio si aún no lo has hecho).*