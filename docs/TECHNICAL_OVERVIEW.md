
# 🤖 Análisis del Código de Cinemática para Brazo Robótico 3GDL

Este informe documenta una implementación modular de cinemática directa (CD) e inversa (CI) para un brazo robótico con 3 grados de libertad (3GDL), usando tres enfoques diferentes: **Geométrico**, **Matriz Denavit-Hartenberg (DH)** y **Robotics Toolbox (RTB)**. La arquitectura del código sigue principios de diseño como el **Patrón Strategy**, y está optimizado para visualización 3D y validación numérica.

---

## 🧩 Estructura del Código

### 1. Importaciones y Control de Dependencias

Se utilizan bibliotecas científicas (`numpy`, `matplotlib`), abstracciones (`ABC`) y se verifica la disponibilidad de `roboticstoolbox`.

---

### 2. Clase Abstracta `KinematicsSolver`

Define la interfaz común para resolver CD y CI:

```python
class KinematicsSolver(ABC):
    def forward(self, thetas): ...
    def inverse(self, target_pos, elbow_config='up', q0=None): ...
```

---

### 3. Implementaciones Concretas de Solvers

#### 🧮 `GeometricSolver`

- Utiliza trigonometría clásica para resolver la cinemática.
- Soporta configuraciones "elbow up" y "elbow down".
- Operaciones explícitas en los planos XY y XZ.

#### 🔢 `MatrixDHSolver`

- Usa matrices homogéneas según la convención DH.
- Aplica transformación e inversión de marcos.
- Alta precisión en la reconstrucción de la posición final.

#### 🧰 `RTBSolver`

- Usa la librería `roboticstoolbox-python` con modelo `DHRobot`.
- Implementa solución numérica con `ikine_LM`.
- Ideal para pruebas avanzadas y extensibilidad en ROS/Simulación.

---

### 4. `RobotArm`: Contenedor de Alto Nivel

Instancia un solver y ejecuta las operaciones de CD y CI de manera uniforme:

```python
robot = RobotArm(lengths=[10, 12, 8], solver_strategy='matrix')
```

---

### 5. `plot_arm`: Visualización en 3D

Renderiza el brazo en `matplotlib` con:

- Eslabones (líneas)
- Articulaciones (puntos)
- Objetivo (estrella dorada)
- Etiquetas interactivas y escala adaptativa

---

### 6. `run_demonstration`: Validación de Estrategias

Función principal que compara resultados de CD y CI:

- Calcula posición del efector final desde ángulos.
- Recupera ángulos desde la posición con CI.
- Compara si CI reproduce los ángulos originales.
- Visualiza ambas soluciones lado a lado.

---

### 7. Ejecución Principal

```python
if __name__ == "__main__":
    for strategy in ['matrix', 'geometric', 'rtb']:
        run_demonstration(strategy, [10, 12, 8], [40, 60, -50])
```

---

## 📦 Requisitos del Proyecto

```bash
pip install numpy matplotlib roboticstoolbox-python spatialmath-python
```

---

## 📘 Créditos y Licencia

Proyecto educativo de simulación y análisis de cinemática robótica.  

---
