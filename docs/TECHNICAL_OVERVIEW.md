
# 🤖 Análisis de Código de Cinemática para Brazo Robótico 3GDL

Este repositorio contiene una implementación completa y modular de la cinemática directa e inversa para un brazo robótico de 3 grados de libertad, utilizando tres enfoques: **geométrico**, **matricial (Denavit-Hartenberg)** y **Robotics Toolbox (RTB)**.

---

## 🧩 Estructura del Código

### 0. Importaciones y Robustez

```python
import numpy as np
import matplotlib.pyplot as plt
from abc import ABC, abstractmethod
```

- **numpy**: operaciones matemáticas vectoriales.
- **matplotlib**: visualización 2D/3D.
- **abc**: define interfaces abstractas.
- **roboticstoolbox (opcional)**: librería de simulación avanzada.

---

### 1. `KinematicsSolver` (Patrón de Diseño Strategy)

Define una interfaz base para resolver cinemática directa e inversa:

```python
class KinematicsSolver(ABC):
    def forward(self, thetas): ...
    def inverse(self, target_pos, elbow_config='up', q0=None): ...
```

---

### 2. `GeometricSolver`: Enfoque Trigonométrico

Calcula ángulos y posiciones utilizando relaciones trigonométricas:

- Usa senos, cosenos y ley de cosenos.
- Permite elegir entre solución "elbow up" o "elbow down".

---

### 3. `MatrixDHSolver`: Matrices Homogéneas y DH

Emplea la convención Denavit-Hartenberg:

- Calcula matrices de transformación para cada articulación.
- Encadena transformaciones para obtener posición final.
- Usa inversión de marcos para simplificar la cinemática inversa.

---

### 4. `RTBSolver`: Robotics Toolbox

Utiliza funciones avanzadas como `ikine_LM`:

- Modela el robot con `DHRobot`.
- Aplica métodos iterativos numéricos.
- Ideal para simulación realista.

---

### 5. `RobotArm`: Clase Contenedora

Selecciona e inicializa el solver apropiado:

```python
robot = RobotArm(lengths=[10, 12, 8], solver_strategy='matrix')
```

---

### 6. `plot_arm`: Visualización 3D con Matplotlib

Dibuja el robot y sus ángulos, posiciones y objetivo:

- Articulaciones (azul)
- Eslabones (líneas)
- Objetivo (estrella dorada)
- Etiquetas dinámicas y escala ajustada.

---

### 7. `run_demonstration`: Simulación y Comparación

- Ejecuta CD e imprime posición del efector final.
- Ejecuta CI para recuperar ángulos.
- Verifica que CI reproduzca la CD.
- Grafica ambas soluciones.

---

### 8. Bloque Principal

```python
if __name__ == "__main__":
    ...
```

- Corre las 3 estrategias: `'matrix'`, `'geometric'`, `'rtb'`.
- Usa ángulos de entrada `[40°, 60°, -50°]`.

---

## 📦 Requisitos

```bash
pip install numpy matplotlib roboticstoolbox-python spatialmath-python
```

---

## 📘 Créditos y Licencia

Proyecto educativo para entender cinemática robótica.  
Autor: Charles Cordova.

---

