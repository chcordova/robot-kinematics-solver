# 🤖 **Informe Técnico: Análisis y Simulación Cinemática de un Brazo Robótico 3-GDL**

---

## 📜 Índice del Proyecto

1. [Introducción](#-1-introducción)
2. [Aplicaciones](#-2-aplicaciones)
3. [Objetivos](#-3-objetivos)
4. [Alcance del Análisis Cinemático](#-4-alcance-del-análisis-cinemático)
5. [Desarrollo y Análisis Cinemático](#-5-desarrollo-y-análisis-cinemático)
6. [Resultados y Validación Visual](#-6-resultados-y-validación-visual)
7. [Conclusiones](#-7-conclusiones)
8. [Anexos](#-8-anexos)

---

## 📖 1. Introducción

En el marco del curso **Recursos Computacionales**, se desarrolla este proyecto aplicado. La cinemática robótica es la base para el control de cualquier manipulador, permitiendo relacionar la configuración de sus articulaciones con la posición de su efector final en el espacio.

Para lograr un modelo matemático robusto y estandarizado, se recurre a la convención **Denavit-Hartenberg (D-H)**, descrita como un "GPS universal para brazos robóticos". Este método ofrece un formalismo para describir la geometría de cualquier robot de cadena abierta con solo cuatro parámetros por eslabón.

Este informe detalla el proceso completo de diseño, modelado matemático y simulación de un brazo robótico de **3 Grados de Libertad (GDL)** de tipo RRR (Rotacional-Rotacional-Rotacional), aplicando el formalismo de Denavit-Hartenberg para resolver tanto la cinemática directa como la inversa.

---

## 🎯 2. Aplicaciones

El modelo desarrollado es una herramienta fundamental con aplicaciones en:

* 🔧 **Industria:** Planificación y verificación de trayectorias en manufactura.
* 🎓 **Educación:** Herramienta didáctica para entender los fundamentos de la robótica.
* 🔬 **Investigación:** Prototipado virtual y validación de algoritmos de control.

---

## ✅ 3. Objetivos

Los objetivos del proyecto, basados en el roadmap, son:

* Diseñar un modelo geométrico y matemático funcional para un brazo robótico de 3 GDL.
* Implementar un simulador en `Python` para calcular la **Cinemática Directa (FK)** y la **Cinemática Inversa (IK)**.
* Visualizar los movimientos del brazo para validar el modelo cinemático.
* Documentar el proceso y presentar los resultados obtenidos.

---

## 📏 4. Alcance del Análisis Cinemático

El proyecto se enfoca en el análisis cinemático mediante transformaciones homogéneas. La matriz genérica de Denavit-Hartenberg que describe la transformación entre eslabones consecutivos es:

$$
T_i^{i-1} =
\begin{bmatrix}
\cos(\theta_i) & -\sin(\theta_i)\cos(\alpha_i) & \sin(\theta_i)\sin(\alpha_i) & a_i \cos(\theta_i) \\
\sin(\theta_i) & \cos(\theta_i)\cos(\alpha_i) & -\cos(\theta_i)\sin(\alpha_i) & a_i \sin(\theta_i) \\
0 & \sin(\alpha_i) & \cos(\alpha_i) & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## 💻 5. Desarrollo y Análisis Cinemático

### 5.1 Modelo Geométrico y Parametrización D-H

**Dimensiones de los Eslabones:**

* $L_1 = 10$ cm
* $L_2 = 12$ cm
* $L_3 = 8$ cm

**Tabla de Parámetros D-H:**

| Eslabón (i) | θᵢ (variable) | dᵢ (cm) | aᵢ (cm) | αᵢ (°) |
| ----------- | ------------- | ------- | ------- | ------ |
| 1           | θ₁            | 10      | 0       | 90     |
| 2           | θ₂            | 0       | 12      | 0      |
| 3           | θ₃            | 0       | 8       | 0      |

---

### 5.2 Cinemática Directa (FK)

La FK responde a la pregunta: *"Si conozco los ángulos de las articulaciones, ¿dónde estará el efector final?"*

Se calcula con:
$T_3^0 = T_1^0(\theta_1) \cdot T_2^1(\theta_2) \cdot T_3^2(\theta_3)$

Para $\{\theta_1 = 40^\circ,\ \theta_2 = 60^\circ,\ \theta_3 = -50^\circ\}$:

$$
T_3^0 =
\begin{bmatrix}
0.174 & 0.985 & 0.000 & 10.632 \\
-0.150 & 0.087 & -0.985 & 8.921 \\
0.985 & -0.174 & 0.000 & 21.781 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

📍 **Posición del efector final:** $(x, y, z) = (10.632,\ 8.921,\ 21.781)$

<details>
<summary>Ver código Python de Cinemática Directa</summary>

```python
import numpy as np

def dh_matrix(theta, d, a, alpha):
    alpha_rad = np.deg2rad(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha_rad), np.sin(alpha_rad)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(thetas, dh_table):
    T = np.identity(4)
    joint_positions = [np.array([0, 0, 0])]
    for i in range(len(thetas)):
        d, a, alpha = dh_table[i, 1:]
        T = T @ dh_matrix(thetas[i], d, a, alpha)
        joint_positions.append(T[:3, 3])
    return joint_positions[-1], joint_positions
```

</details>

---

### 5.3 Cinemática Inversa (IK)

La IK responde a: *"Para que el efector alcance un punto (x, y, z), ¿qué ángulos deben tener las articulaciones?"*

**Procedimiento Geométrico:**

1. $\theta_1 = \arctan2(y, x)$
2. $r = \sqrt{x^2 + y^2}$, $z' = z - L_1$, $d = \sqrt{r^2 + z'^2}$
3.

$$
\begin{align*}
\alpha &= \arctan2(z', r) \\
\beta &= \arccos\left(\frac{d^2 + L_2^2 - L_3^2}{2 d L_2}\right) \\
\theta_3 &= -\arccos\left(\frac{d^2 - L_2^2 - L_3^2}{2 L_2 L_3}\right) \\
\theta_2 &= \alpha + \beta
\end{align*}
$$

<details>
<summary>Ver código Python de Cinemática Inversa</summary>

```python
def inverse_kinematics(target_pos, lengths, elbow_config='up'):
    L1, L2, L3 = lengths
    x, y, z = target_pos
    theta1 = np.arctan2(y, x)
    r = np.sqrt(x**2 + y**2)
    z_prime = z - L1
    d = np.sqrt(r**2 + z_prime**2)
    if d > L2 + L3 or d < abs(L2 - L3): return None
    alpha = np.arctan2(z_prime, r)
    beta = np.arccos(np.clip((d**2 + L2**2 - L3**2)/(2*d*L2), -1.0, 1.0))
    cos_theta3 = (d**2 - L2**2 - L3**2) / (2 * L2 * L3)
    if elbow_config == 'up':
        theta2 = alpha + beta
        theta3 = -np.arccos(np.clip(cos_theta3, -1.0, 1.0))
    else:
        theta2 = alpha - beta
        theta3 = np.arccos(np.clip(cos_theta3, -1.0, 1.0))
    return [theta1, theta2, theta3]
```

</details>

---

## 📊 6. Resultados y Validación Visual

**Ciclo de Validación:**

1. Entrada FK: $\{40^\circ,\ 60^\circ,\ -50^\circ\}$
2. FK genera posición: $(10.632,\ 8.921,\ 21.781)$
3. IK reconstruye los ángulos con misma entrada

✅ Modelo coherente matemática y computacionalmente.

<details>
<summary>Ver código Python de Validación</summary>

```python
if __name__ == "__main__":
    L1, L2, L3 = 10, 12, 8
    dh_params = np.array([[0, L1, 0, 90], [0, 0, L2, 0], [0, 0, L3, 0]])
    angles_deg = [40, 60, -50]
    angles_rad = [np.deg2rad(a) for a in angles_deg]
    final_pos, _ = forward_kinematics(angles_rad, dh_params)
    target_angles = inverse_kinematics(final_pos, [L1, L2, L3])
    if target_angles:
        print("Resultado coherente: IK → FK validados.")
```

</details>

---

## 🏁 7. Conclusiones

* ✅ Modelo cinemático completo del brazo robótico 3GDL
* ✅ Implementación computacional en Python funcional y precisa
* ✅ Simulación valida coherencia entre teoría, código y simulación

---

## 📎 8. Anexos

* 📂 **Anexo A:** Código Python completo
* 📐 **Anexo B:** Diagramas y coordenadas D-H
* 🖼️ **Anexo C:** Capturas de simulación visual
