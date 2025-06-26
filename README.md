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
| **i** | **θᵢ (variable)** | **dᵢ** | **aᵢ** | **αᵢ** |
|:---:|:---:|:---:|:---:|:---:|
| 1 | θ₁ | 10 | 0 | 90° |
| 2 | θ₂ | 0 | 12 | 0° |
| 3 | θ₃ | 0 | 8 | 0° |

#### **5.2. Cinemática Directa (FK)**
* **Ecuación General:**
  ```
  T₃⁰ = T₁⁰(θ₁) · T₂¹(θ₂) · T₃²(θ₃)
  ```
* **Matrices de Transformación (Ejemplo: {40°, 60°, -50°})**
  ```
  T₁⁰ =                        T₂¹ =                         T₃² =
  ⎡ 0.766   0     0.643    0   ⎤   ⎡ 0.5   -0.866   0     6      ⎤   ⎡ 0.643   0.766   0     5.144  ⎤
  ⎢ 0.643   0    -0.766    0   ⎥   ⎢ 0.866  0.5     0    10.392  ⎥   ⎢-0.766   0.643   0    -6.128  ⎥
  ⎢ 0       1     0       10   ⎥   ⎢ 0      0       1     0      ⎥   ⎢ 0       0       1     0      ⎥
  ⎣ 0       0     0        1   ⎦   ⎣ 0      0       0     1      ⎦   ⎣ 0       0       0     1      ⎦
  ```
* **Resultado Final:**
  ```
  T₃⁰ = 
  ⎡  0.174   0.985   0.000   10.632 ⎤
  ⎢ -0.150   0.087  -0.985    8.921 ⎥
  ⎢  0.985  -0.174   0.000   21.781 ⎥
  ⎣  0       0       0        1     ⎦
  ```
* **Posición del efector final:** (x, y, z) = (10.632, 8.921, 21.781)

#### **5.3. Cinemática Inversa (IK)**
* **Procedimiento:** Se implementa una solución geométrica en Python para encontrar los ángulos articulares a partir de una posición objetivo.
    <details>
    <summary>Ver código Python de Cinemática Inversa</summary>
    
    ```python
    import numpy as np

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

* **Validación:** La entrada IK para (x, y, z) = (10.632, 8.921, 21.781) devuelve correctamente los ángulos originales: θ₁≈40°, θ₂≈60°, θ₃≈-50°.

---

### 📊 6. Resultados y Validación Visual

**Gráfico 1: Visualización por Cinemática Directa**
> *Configuración del brazo para los ángulos de entrada {40°, 60°, -50°}.*

![Visualización del Brazo Robótico por Cinemática Directa](https://raw.githubusercontent.com/chcordova/robot-kinematics-solver/main/docs/fk_arm_visualization.png)

**Gráfico 2: Validación de Cinemática Inversa**
> *El efector final alcanza con precisión el punto objetivo.*

![El Brazo Robótico Alcanzando un Punto Objetivo](https://raw.githubusercontent.com/chcordova/robot-kinematics-solver/main/docs/ik_validation.png)

---

### 🏁 7. Conclusiones
* Se desarrolló exitosamente el modelo de 3 GDL.
* Las implementaciones de FK e IK en Python validaron el modelo matemático.
* La simulación confirmó la coherencia entre la teoría y el resultado numérico.

---

### 📎 8. Anexos
* **Anexo A:** Código fuente completo en Python
* **Anexo B:** Esquemas de coordenadas D-H
* **Anexo C:** Capturas del simulador
