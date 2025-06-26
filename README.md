# 🤖 Informe Técnico: Análisis y Simulación Cinemática de un Brazo Robótico 3-GDL

## 📜 Índice del Proyecto

1.  [**Introducción**](#-1-introducción)
2.  [**Aplicaciones**](#-2-aplicaciones)
3.  [**Objetivos**](#-3-objetivos)
4.  [**Alcance del Análisis Cinemático**](#-4-alcance-del-análisis-cinemático)
5.  [**Desarrollo y Análisis Cinemático**](#-5-desarrollo-y-análisis-cinemático)
    * [5.1. Modelo Geométrico y Parametrización D-H](#51-modelo-geométrico-y-parametrización-d-h)
    * [5.2. Cinemática Directa (FK): Teoría y Validación Práctica](#52-cinemática-directa-fk-teoría-y-validación-práctica)
    * [5.3. Cinemática Inversa (IK): Teoría y Validación Práctica](#53-cinemática-inversa-ik-teoría-y-validación-práctica)
6.  [**Resultados y Validación Visual**](#-6-resultados-y-validación-visual)
7.  [**Conclusiones**](#-7-conclusiones)
8.  [**Anexos**](#-8-anexos)

---

### 📖 1. Introducción
En el marco del curso **Recursos Computacionales**, se desarrolla este proyecto aplicado. La cinemática robótica es la base para el control de cualquier manipulador, permitiendo relacionar la configuración de sus articulaciones con la posición de su efector final en el espacio. Para lograr un modelo matemático robusto y estandarizado, se recurre a la convención **Denavit-Hartenberg (D-H)**, descrita en nuestra documentación como un "GPS universal para brazos robóticos". Este método ofrece un formalismo para describir la geometría de cualquier robot de cadena abierta con solo cuatro parámetros por eslabón.

Este informe detalla el proceso completo de diseño, modelado matemático y simulación de un brazo robótico de **$3$ Grados de Libertad (GDL)** de tipo RRR (Rotacional-Rotacional-Rotacional), aplicando el formalismo de Denavit-Hartenberg para resolver tanto la cinemática directa como la inversa.

---

### 🎯 2. Aplicaciones
El modelo desarrollado es una herramienta fundamental con aplicaciones en:
* **Industria:** Para la planificación y verificación de trayectorias en manufactura.
* **Educación:** Como una herramienta didáctica para entender los fundamentos de la robótica.
* **Investigación:** Para el prototipado virtual y la validación de algoritmos de control.

---

### ✅ 3. Objetivos
Los objetivos del proyecto, basados en el roadmap, son:
* **Diseñar** un modelo geométrico y matemático funcional para un brazo robótico de $3$ GDL.
* **Implementar** un simulador en `Python` para calcular la **Cinemática Directa (FK)** y la **Cinemática Inversa (IK)**.
* **Visualizar** los movimientos del brazo para validar el modelo cinemático.
* **Documentar** el proceso y presentar los resultados obtenidos.

---

### 📏 4. Alcance del Análisis Cinemático
El alcance de este proyecto se centra en el **análisis y la implementación del modelo cinemático** del manipulador. El enfoque es matricial, utilizando las transformaciones homogéneas de Denavit-Hartenberg. La matriz genérica que describe la transformación entre eslabones consecutivos ($T_{i}^{i-1}$) y que forma la base de nuestro análisis es:

$$
T_{i}^{i-1} = \begin{bmatrix} \cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\ 0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

El proyecto aborda tanto la cinemática directa como la inversa, limitado a una simulación virtual sin considerar dinámicas ni colisiones.

---

### 💻 5. Desarrollo y Análisis Cinemático

#### **5.1. Modelo Geométrico y Parametrización D-H**
La "estructura" física del robot se define matemáticamente mediante sus dimensiones y el modelo D-H.
* **Dimensiones de los Eslabones**:
    * $L_1 = 10 \text{ cm}$
    * $L_2 = 12 \text{ cm}$
    * $L_3 = 8 \text{ cm}$

* **Tabla de Parámetros D-H**:

| **_i_** | **θ~i~ (rotación Z)** | **d~i~ (traslación Z)** | **a~i~ (traslación X)** | **α~i~ (rotación X)** |
|:---:|:---:|:---:|:---:|:---:|
| 1 | θ~1~ (variable) | _L_~1~ = 10 | 0 | 90° |
| 2 | θ~2~ (variable) | 0 | _L_~2~ = 12 | 0° |
| 3 | θ~3~ (variable) | 0 | _L_~3~ = 8 | 0° |

#### **5.2. Cinemática Directa (FK): Teoría y Validación Práctica**
La FK responde a la pregunta: *"Si conozco los ángulos de las articulaciones, ¿dónde estará el efector final?"*.

* **Procedimiento Matemático Matricial**
    El método consiste en multiplicar secuencialmente las matrices de transformación para encontrar la transformación total. La ecuación fundamental es:
    $$
    T_{3}^{0} = T_{1}^{0}(\theta_1) \cdot T_{2}^{1}(\theta_2) \cdot T_{3}^{2}(\theta_3)
    $$

* **Caso de Estudio Práctico (FK)**
    Para la configuración articular de ejemplo $\{\theta_1, \theta_2, \theta_3\} = \{40^\circ, 60^\circ, -50^\circ\}$, las matrices individuales son:
    $$
    \displaylines{
    T_{1}^{0} = \begin{bmatrix} 0.766 & 0 & 0.643 & 0 \\ 0.643 & 0 & -0.766 & 0 \\ 0 & 1 & 0 & 10 \\ 0 & 0 & 0 & 1 \end{bmatrix} \\ \\
    T_{2}^{1} = \begin{bmatrix} 0.5 & -0.866 & 0 & 6 \\ 0.866 & 0.5 & 0 & 10.392 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} \\ \\
    T_{3}^{2} = \begin{bmatrix} 0.643 & 0.766 & 0 & 5.144 \\ -0.766 & 0.643 & 0 & -6.128 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
    }
    $$
    El producto de estas matrices da como resultado la matriz de transformación total:
    $$
    T_{3}^{0} = \begin{bmatrix} 0.174 & 0.985 & 0.000 & 10.632 \\ -0.150 & 0.087 & -0.985 & 8.921 \\ 0.985 & -0.174 & 0.000 & 21.781 \\ 0 & 0 & 0 & 1 \end{bmatrix}
    $$
    De esta matriz se extrae la posición cartesiana del efector final: **$(x, y, z) = (10.632, 8.921, 21.781)$**.

* **Implementación en Python (FK)**
    ```python
    def forward_kinematics(thetas, dh_table):
        T_acumulada = np.identity(4)
        joint_positions = [np.array([0, 0, 0])]
        for i in range(len(thetas)):
            d, a, alpha = dh_table[i, 1:]
            T = dh_matrix(thetas[i], d, a, alpha)
            T_acumulada = T_acumulada @ T
            pos_actual = T_acumulada[:3, 3]
            joint_positions.append(pos_actual)
        return joint_positions[-1], joint_positions
    ```

#### **5.3. Cinemática Inversa (IK): Teoría y Validación Práctica**
La IK responde a la pregunta: *"Para que el efector final alcance un punto $(x, y, z)$, ¿qué ángulos deben tener las articulaciones?"*.

* **Procedimiento Teórico-Matricial**
    Teóricamente, una vez que se tiene la ecuación $T_{3}^{0} = T_{obj}$, la IK se resuelve despejando las variables angulares. Un enfoque matricial consiste en pre-multiplicar la ecuación por la inversa de cada matriz para aislar las articulaciones:
    $$
    (T_{1}^{0})^{-1} \cdot T_{obj} = T_{2}^{1}(\theta_2) \cdot T_{3}^{2}(\theta_3)
    $$

* **Enfoque Práctico (Solución Geométrica)**
    Para este robot, es más eficiente utilizar un **método geométrico** basado en el desacoplamiento cinemático.

* **Implementación en Python (IK)**
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
        beta_cos_val = (d**2 + L2**2 - L3**2) / (2 * d * L2)
        beta = np.arccos(np.clip(beta_cos_val, -1.0, 1.0))
        cos_theta3 = (d**2 - L2**2 - L3**2) / (2 * L2 * L3)
        if elbow_config == 'up':
            theta2 = alpha + beta
            theta3 = -np.arccos(np.clip(cos_theta3, -1.0, 1.0))
        else: # 'down'
            theta2 = alpha - beta
            theta3 = np.arccos(np.clip(cos_theta3, -1.0, 1.0))
        return [theta1, theta2, theta3]
    ```
* **Validación del Modelo (IK)**
    Al usar la posición objetivo $(10.632, 8.921, 21.781)$, el código de la IK calcula los ángulos articulares, resultando en $\{\theta_1, \theta_2, \theta_3\} = \{40.0^\circ, 60.0^\circ, -50.0^\circ\}$, que son idénticos a los ángulos de entrada originales.

---

### 📊 **6. Resultados y Validación Visual**
La validación numérica se complementa con la visualización gráfica del simulador.

**Gráfico 1: Visualización del Brazo por Cinemática Directa**
> *Posición del brazo para los ángulos de entrada $\{\theta_1, \theta_2, \theta_3\} = \{40^\circ, 60^\circ, -50^\circ\}$. Valida que el cálculo de la FK es correcto.*

![Visualización del Brazo Robótico por Cinemática Directa](https://via.placeholder.com/800x600.png?text=Gráfico+de+Cinemática+Directa)

**Gráfico 2: Verificación de la Cinemática Inversa**
> *El efector final (círculo hueco) alcanza con precisión el punto objetivo (estrella dorada), validando la solución de la IK.*

![El Brazo Robótico Alcanzando un Punto Objetivo](https://via.placeholder.com/800x600.png?text=Gráfico+de+Cinemática+Inversa)

---

### 🏁 **7. Conclusiones**
* Se diseñó y validó con éxito un modelo matemático para un brazo robótico de $3$ GDL.
* Se implementaron en `Python` funciones robustas para la cinemática directa e inversa.
* La validación de ciclo completo (FK → IK) confirmó la consistencia y precisión del simulador.
* El proyecto constituye una base sólida y escalable para futuras aplicaciones en robótica.

---

### 📎 **8. Anexos**
* **Anexo A:** Código fuente completo en `Python`.
* **Anexo B:** Esquemas detallados de los sistemas de coordenadas asignados.
* **Anexo C:** Gráficos y capturas de pantalla de la simulación.
