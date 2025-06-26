# 🤖 **Informe Técnico: Análisis y Simulación Cinemática de un Brazo Robótico 3-GDL**

## 📜 **Índice del Proyecto**

1.  [**Introducción**](#-1-introducción)
2.  [**Aplicaciones**](#-2-aplicaciones)
3.  [**Objetivos**](#-3-objetivos)
4.  [**Alcance del Análisis Cinemático**](#-4-alcance-del-análisis-cinemático)
5.  [**Desarrollo y Análisis Cinemático**](#-5-desarrollo-y-análisis-cinemático)
6.  [**Resultados y Validación Visual**](#-6-resultados-y-validación-visual)
7.  [**Conclusiones**](#-7-conclusiones)
8.  [**Anexos**](#-8-anexos)

---

### 📖 1. Introducción
En el marco del curso **Recursos Computacionales**, se desarrolla este proyecto aplicado. La cinemática robótica es la base para el control de cualquier manipulador, permitiendo relacionar la configuración de sus articulaciones con la posición de su efector final en el espacio. Para lograr un modelo matemático robusto y estandarizado, se recurre a la convención **Denavit-Hartenberg (D-H)**, descrita en nuestra documentación como un "GPS universal para brazos robóticos". Este método ofrece un formalismo para describir la geometría de cualquier robot de cadena abierta con solo cuatro parámetros por eslabón.

Este informe detalla el proceso completo de diseño, modelado matemático y simulación de un brazo robótico de **3 Grados de Libertad (GDL)** de tipo RRR (Rotacional-Rotacional-Rotacional), aplicando el formalismo de Denavit-Hartenberg para resolver tanto la cinemática directa como la inversa.

---

### 🎯 2. Aplicaciones
El modelo desarrollado es una herramienta fundamental con aplicaciones en:
* **Industria:** Para la planificación y verificación de trayectorias en manufactura.
* **Educación:** Como una herramienta didáctica para entender los fundamentos de la robótica.
* **Investigación:** Para el prototipado virtual y la validación de algoritmos de control.

---

### ✅ 3. Objetivos
Los objetivos del proyecto, basados en el roadmap, son:
* **Diseñar** un modelo geométrico y matemático funcional para un brazo robótico de 3 GDL.
* **Implementar** un simulador en `Python` para calcular la **Cinemática Directa (FK)** y la **Cinemática Inversa (IK)**.
* **Visualizar** los movimientos del brazo para validar el modelo cinemático.
* **Documentar** el proceso y presentar los resultados obtenidos.

---

### 📏 4. Alcance del Análisis Cinemático
El proyecto se enfoca en el análisis cinemático mediante transformaciones homogéneas. La matriz genérica de Denavit-Hartenberg que describe la transformación entre eslabones consecutivos es:
```
T_i^{i-1} = 
⎡ cos(θᵢ)  -sin(θᵢ)cos(αᵢ)   sin(θᵢ)sin(αᵢ)   aᵢ cos(θᵢ) ⎤
⎢ sin(θᵢ)   cos(θᵢ)cos(αᵢ)  -cos(θᵢ)sin(αᵢ)   aᵢ sin(θᵢ) ⎥
⎢   0         sin(αᵢ)          cos(αᵢ)          dᵢ      ⎥
⎣   0           0                0               1       ⎦
```

---

### 💻 5. Desarrollo y Análisis Cinemático

#### **5.1. Modelo Geométrico y Parametrización D-H**
* **Dimensiones de los Eslabones:**
    * L₁ = 10 cm
    * L₂ = 12 cm
    * L₃ = 8 cm

* **Tabla de Parámetros D-H:**
  ```
  ------------------------------------------------------------------
  | i |   θᵢ (variable)   |   dᵢ (traslación) |   aᵢ (longitud) |  αᵢ (torsión) |
  |---|-------------------|------------------|----------------|---------------|
  | 1 |        θ₁         |     L₁ = 10      |        0       |      90°      |
  | 2 |        θ₂         |        0         |     L₂ = 12      |       0°      |
  | 3 |        θ₃         |        0         |      L₃ = 8      |       0°      |
  ------------------------------------------------------------------
  ```

#### **5.2. Cinemática Directa (FK): Teoría y Validación Práctica**
La FK responde a la pregunta: *"Si conozco los ángulos de las articulaciones, ¿dónde estará el efector final?"*.

* **Procedimiento Matemático Matricial**
    El método consiste en multiplicar secuencialmente las matrices de transformación para encontrar la transformación total desde la base ($S_0$) hasta el efector final ($S_3$). La ecuación fundamental es:
      ```
      T₃⁰ = T₁⁰(θ₁) · T₂¹(θ₂) · T₃²(θ₃)
      ```
    Para nuestro caso de estudio con los ángulos `{40°, 60°, -50°}`, se construyen las matrices individuales:
      ```
      T₁⁰ =                        T₂¹ =                         T₃² =
      ⎡ 0.766   0     0.643    0   ⎤   ⎡ 0.5   -0.866   0     6      ⎤   ⎡ 0.643   0.766   0     5.144  ⎤
      ⎢ 0.643   0    -0.766    0   ⎥   ⎢ 0.866  0.5     0    10.392  ⎥   ⎢-0.766   0.643   0    -6.128  ⎥
      ⎢ 0       1     0       10   ⎥   ⎢ 0      0       1     0      ⎥   ⎢ 0       0       1     0      ⎥
      ⎣ 0       0     0        1   ⎦   ⎣ 0      0       0     1      ⎦   ⎣ 0       0       0     1      ⎦
      ```
    El producto de estas matrices da como resultado la matriz de transformación total:
      ```
      T₃⁰ = 
      ⎡  0.174   0.985   0.000   10.632 ⎤
      ⎢ -0.150   0.087  -0.985    8.921 ⎥
      ⎢  0.985  -0.174   0.000   21.781 ⎥
      ⎣  0       0       0        1     ⎦
      ```
    De esta matriz se extrae la posición cartesiana del efector final: **(x, y, z) = (10.632, 8.921, 21.781)**.

* **Implementación en Python (FK)**
    <details>
    <summary>Ver código Python</summary>
    
    ```python
    def dh_matrix(theta, d, a, alpha):
        # Construye la matriz de transformación homogénea (4x4)
        alpha_rad = np.deg2rad(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha_rad), np.sin(alpha_rad)
        T = np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        return T

    def forward_kinematics(thetas, dh_table):
        # Resuelve el Problema Cinemático Directo (PCD)
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
    </details>

#### **5.3. Cinemática Inversa (IK): Teoría y Validación Práctica**
La IK responde a la pregunta: *"Para que el efector final alcance un punto (x, y, z), ¿qué ángulos deben tener las articulaciones?"*.

* **Procedimiento Teórico-Matricial y Geométrico**
    Teóricamente, la IK se resuelve despejando las variables angulares de la ecuación $T_{3}^{0} = T_{obj}$. Un enfoque matricial consiste en pre-multiplicar la ecuación por la inversa de cada matriz para aislar las articulaciones. Si bien este es el fundamento, para un robot de 3 GDL es más eficiente implementar una **solución geométrica**. Este método se detalla a continuación:

    1.  **Cálculo del Ángulo de la Base (θ₁):** Se calcula proyectando el punto objetivo (x, y) sobre el plano base.
        ```
        θ₁ = atan2(y, x)
        ```
    2.  **Reducción del Problema a 2D:** Se transforma el problema 3D en un triángulo planar 2D. Para ello se calculan:
        * La distancia radial al punto: `r = √(x² + y²)`
        * La altura efectiva desde el "hombro": `z' = z - L₁`
        * La distancia directa entre el hombro y la muñeca: `d = √(r² + z'²)`
    3.  **Resolución del Triángulo Planar (Ley de Cosenos):** Se resuelven los ángulos internos del triángulo formado por los eslabones L₂ y L₃.
        * `α = atan2(z', r)`
        * `β = arccos((d² + L₂² - L₃²) / (2 · d · L₂))`
        * `θ₃ = -arccos((d² - L₂² - L₃²) / (2 · L₂ · L₃))`
    4.  **Cálculo de θ₂:** Se combinan los ángulos para obtener la solución final para la configuración "codo arriba".
        ```
        θ₂ = α + β
        ```

* **Validación del Modelo (Ciclo Completo)**
    Se realiza un ciclo completo para verificar la consistencia del modelo:
    1.  **Entrada FK:** Se parte de los ángulos `{θ₁, θ₂, θ₃} = {40°, 60°, -50°}`.
    2.  **Salida FK:** El cálculo de la cinemática directa nos da la posición `P = (10.632, 8.921, 21.781)`.
    3.  **Entrada IK:** Se utiliza la posición `P` como objetivo para la cinemática inversa.
    4.  **Salida IK y Verificación:** La función devuelve los ángulos `{θ'₁, θ'₂, θ'₃} = {40°, 60°, -50°}`.
    
    Al ser los ángulos de salida idénticos a los de entrada, se confirma que el modelo es matemáticamente correcto.

* **Implementación en Python (IK)**
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

### 📊 6. Resultados y Validación Visual

La validación numérica se complementa con la visualización gráfica, implementada con el siguiente código:
<details>
<summary>Ver código Python de Visualización</summary>

```python
def plot_arm(joint_positions, target=None):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    points = np.array(joint_positions)
    # Dibuja los eslabones y articulaciones
    ax.plot(points[:, 0], points[:, 1], points[:, 2], 'o-', color='purple', lw=4, markersize=10, markerfacecolor='blue', label='Eslabones y Articulaciones')
    # Dibuja el efector final
    final_joint = points[-1]
    ax.scatter(final_joint[0], final_joint[1], final_joint[2], s=350, facecolors='none', edgecolors='darkviolet', lw=2, zorder=4, label='Efector Final')
    # Dibuja el punto objetivo
    if target is not None:
        ax.scatter(target[0], target[1], target[2], c='gold', s=250, marker='*', label='Objetivo', zorder=3, edgecolor='black')
    # Configuración del entorno gráfico
    ax.set_xlabel('Eje X (cm)'); ax.set_ylabel('Eje Y (cm)'); ax.set_zlabel('Eje Z (cm)')
    ax.set_title('Simulador Robótico 3-DOF (Enfoque Matricial DH)')
    max_range = sum([L1, L2, L3])
    ax.set_xlim([-max_range, max_range]); ax.set_ylim([-max_range, max_range]); ax.set_zlim([0, max_range])
    ax.legend(); ax.grid(True)
    plt.show()
```
</details>

**Gráfico 1: Visualización por Cinemática Directa**
> *Configuración del brazo para los ángulos de entrada {40°, 60°, -50°}.*

``

**Gráfico 2: Validación de Cinemática Inversa**
> *El efector final alcanza con precisión el punto objetivo.*

``

---

### 🏁 7. Conclusiones
* Se desarrolló exitosamente el modelo matemático y computacional de un brazo robótico de 3 GDL.
* Las implementaciones de FK e IK en Python validaron el modelo teórico de forma precisa.
* La simulación confirmó la coherencia entre la teoría, el desarrollo matemático y el resultado numérico a través de un caso de estudio práctico.

---

### 📎 8. Anexos
* **Anexo A:** Código fuente completo en Python
* **Anexo B:** Esquemas de coordenadas D-H
* **Anexo C:** Capturas del simulador
