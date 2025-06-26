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

#### **5.2. Cinemática Directa (FK)**
La FK responde a la pregunta: *"Si conozco los ángulos de las articulaciones, ¿dónde estará el efector final?"*. Se resuelve mediante el producto de matrices de transformación.
  ```
  T₃⁰ = T₁⁰(θ₁) · T₂¹(θ₂) · T₃²(θ₃)
  ```
Para el caso de estudio con los ángulos `{40°, 60°, -50°}`, el resultado final es:
  ```
  T₃⁰ = 
  ⎡  0.174   0.985   0.000   10.632 ⎤
  ⎢ -0.150   0.087  -0.985    8.921 ⎥
  ⎢  0.985  -0.174   0.000   21.781 ⎥
  ⎣  0       0       0        1     ⎦
  ```
* **Posición del efector final:** (x, y, z) = (10.632, 8.921, 21.781)

* **Implementación en Python (FK)**
    <details>
    <summary>Ver código Python de Cinemática Directa</summary>
    
    ```python
    import numpy as np
    
    def dh_matrix(theta, d, a, alpha):
        """
        Construye la matriz de transformación homogénea (4x4) para un eslabón,
        basada en los cuatro parámetros de Denavit-Hartenberg.
        """
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
        """
        Resuelve el Problema Cinemático Directo (PCD) mediante la composición secuencial
        de las matrices de transformación de cada articulación.
        """
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

#### **5.3. Cinemática Inversa (IK)**
La IK responde a la pregunta: *"Para que el efector final alcance un punto (x, y, z), ¿qué ángulos deben tener las articulaciones?"*.

* **Procedimiento Matemático (Solución Geométrica)**
    Para este robot de 3 GDL, se utiliza un método geométrico que se resuelve en los siguientes pasos:
    1.  **Cálculo del Ángulo de la Base (θ₁):** Se calcula proyectando el punto objetivo (x, y) sobre el plano base.
        ```
        θ₁ = atan2(y, x)
        ```
    2.  **Reducción del Problema a 2D:** Se transforma el problema 3D en un triángulo planar 2D, calculando las distancias auxiliares `r`, `z'` y `d`.
        ```
        r = √(x² + y²)
        z' = z - L₁
        d = √(r² + z'²)
        ```
    3.  **Resolución del Triángulo Planar:** Se utiliza la Ley de los Cosenos para encontrar los ángulos internos `α` y `β`, que permiten despejar `θ₂` y `θ₃`.
        ```
        α = atan2(z', r)
        β = arccos((d² + L₂² - L₃²) / (2 · d · L₂))
        θ₃ = -arccos((d² - L₂² - L₃²) / (2 · L₂ · L₃))
        θ₂ = α + β
        ```
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

### 📊 6. Resultados y Validación

#### **6.1. Proceso de Validación (Ciclo Completo FK → IK)**
Para demostrar la consistencia del modelo, se realiza una validación de ciclo completo:
1.  **Punto de Partida (FK):** Se usan los ángulos iniciales `{θ₁, θ₂, θ₃} = {40°, 60°, -50°}`.
2.  **Cálculo de Posición:** La cinemática directa nos da la posición objetivo `P = (10.632, 8.921, 21.781)`.
3.  **Cálculo de Ángulos (IK):** Se introduce la posición `P` en la función de cinemática inversa.
4.  **Resultado y Verificación:** La función devuelve los ángulos `{θ'₁, θ'₂, θ'₃} = {40°, 60°, -50°}`.

Como los ángulos finales son idénticos a los iniciales, se concluye que el modelo es matemáticamente correcto.

#### **6.2. Código de Ejecución y Validación**
<details>
<summary>Ver código Python de Validación</summary>

```python
if __name__ == "__main__":
    # --- Parámetros del Robot ---
    L1, L2, L3 = 10, 12, 8
    dh_params = np.array([[0, L1, 0, 90], [0, 0, L2, 0], [0, 0, L3, 0]])

    # --- Prueba del Problema Cinemático Directo (PCD) ---
    print("--- RESULTADO CINEMÁTICA DIRECTA ---")
    angles_deg = [40, 60, -50]
    angles_rad = [np.deg2rad(angle) for angle in angles_deg]
    final_pos, all_joints = forward_kinematics(angles_rad, dh_params)
    print(f"Para los ángulos articulares: {angles_deg}°")
    print(f"La posición cartesiana del efector final es: (x={final_pos[0]:.3f}, y={final_pos[1]:.3f}, z={final_pos[2]:.3f})")
    
    # --- Prueba del Problema Cinemático Inverso (PCI) ---
    print("\n" + "="*50 + "\n")
    print("--- RESULTADO CINEMÁTICA INVERSA ---")
    target_position = final_pos
    target_angles = inverse_kinematics(target_position, [L1, L2, L3], elbow_config='up')
    print(f"Para la posición objetivo: (x={target_position[0]:.3f}, y={target_position[1]:.3f}, z={target_position[2]:.3f})")
    if target_angles:
        target_angles_deg = [np.rad2deg(angle) for angle in target_angles]
        print(f"Los ángulos articulares calculados son: (θ1={target_angles_deg[0]:.1f}°, θ2={target_angles_deg[1]:.1f}°, θ3={target_angles_deg[2]:.1f}°)")
```
</details>

#### **6.3. Validación Visual**
* **Gráfico 1: Visualización por Cinemática Directa**
``

* **Gráfico 2: Validación de Cinemática Inversa**
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
