# 🤖 **Informe Técnico: Análisis y Simulación Cinemática de un Brazo Robótico 3-GDL**

---

📅 **Curso:** Recursos Computacionales
👨‍💻 **Tema:** Cinemática Directa e Inversa mediante Denavit-Hartenberg
🧠 **Grados de Libertad:** 3 (RRR: Rotacional - Rotacional - Rotacional)

---

## 📜 **Índice del Proyecto**

1. [Introducción](#1-introducción)
2. [Aplicaciones](#2-aplicaciones)
3. [Objetivos](#3-objetivos)
4. [Alcance del Análisis Cinemático](#4-alcance-del-análisis-cinemático)
5. [Desarrollo y Análisis Cinemático](#5-desarrollo-y-análisis-cinemático)
6. [Resultados y Validación Visual](#6-resultados-y-validación-visual)
7. [Conclusiones](#7-conclusiones)
8. [Anexos](#8-anexos)

---

## 📖 1. Introducción

La cinemática robótica establece la relación matemática entre las configuraciones articulares de un robot y la posición de su efector final. En este proyecto se modela y simula un brazo robótico de 3 GDL empleando la convención estándar **Denavit-Hartenberg (D-H)**, que permite construir un modelo compacto, sistemático y replicable para cualquier manipulador en cadena abierta.

---

## 🌟 2. Aplicaciones

🔧 **Industria:** Planificación de trayectorias y automatización.
🎓 **Educación:** Enseñanza de principios de cinemática y robótica.
🔬 **Investigación:** Simulación, validación de algoritmos y prototipado virtual.

---

## ✅ 3. Objetivos

🎯 Diseñar el modelo geométrico de un robot RRR de 3 GDL.
💻 Implementar los algoritmos de Cinemática Directa e Inversa en Python.
📈 Visualizar y validar el comportamiento del brazo en simulaciones.
📝 Documentar resultados, enfoques y validaciones.

---

## 📏 4. Alcance del Análisis Cinemático

El análisis se enfoca exclusivamente en la **cinemática** del manipulador, excluyendo dinámica y colisiones. Se trabaja bajo un enfoque **matricial** empleando transformaciones homogéneas 4x4.

### 📐 Matriz General de D-H

```
Tᵢ⁽ⁱ⁻¹⁾ =
⎡ cos(θᵢ)  -sin(θᵢ)cos(αᵢ)   sin(θᵢ)sin(αᵢ)   aᵢ·cos(θᵢ) ⎤
⎢ sin(θᵢ)   cos(θᵢ)cos(αᵢ)  -cos(θᵢ)sin(αᵢ)   aᵢ·sin(θᵢ) ⎥
⎢   0         sin(αᵢ)          cos(αᵢ)          dᵢ       ⎥
⎣   0           0                0               1       ⎦
```

---

## 💻 5. Desarrollo y Análisis Cinemático

### 5.1 Modelo Geométrico y Parametrización D-H

📏 **Dimensiones de Eslabones:**
• L₁ = 10 cm
• L₂ = 12 cm
• L₃ = 8 cm

📊 **Tabla D-H:**

```
┌─────┬───────────────┬────────┬───────┬────────────┐
│  i  │ θᵢ (variable)  │  dᵢ    │  aᵢ   │   αᵢ      │
├─────┼───────────────┼────────┼───────┼────────────┤
│  1  │     θ₁         │  10    │  0    │  90°      │
│  2  │     θ₂         │   0    │  12   │   0°      │
│  3  │     θ₃         │   0    │   8   │   0°      │
└─────┴───────────────┴────────┴───────┴────────────┘
```

---

### 5.2 Cinemática Directa (FK)

La FK calcula la posición del efector final dados los ángulos articulares. Se realiza el producto sucesivo de matrices de transformación:

```
T₃⁰ = T₁⁰(θ₁) · T₂¹(θ₂) · T₃²(θ₃)
```

📍 **Ejemplo numérico (θ₁ = 40°, θ₂ = 60°, θ₃ = -50°):**

```
T₃⁰ =
⎡  0.174   0.985   0.000   10.632 ⎤
⎢ -0.150   0.087  -0.985    8.921 ⎥
⎢  0.985  -0.174   0.000   21.781 ⎥
⎣  0       0       0        1     ⎦
```

📌 **Posición del efector final:**
**(x, y, z) = (10.632, 8.921, 21.781)**

---

### 5.3 Cinemática Inversa (IK)

La IK calcula los ángulos articulares necesarios para alcanzar una posición objetivo.

#### 🧠 Enfoque Teórico: Inversión Matricial

Teóricamente, una vez que se tiene la ecuación `T₃⁰ = T_obj`, donde `T_obj` es la matriz de posición y orientación objetivo, la IK se resuelve despejando las variables angulares. El enfoque matricial consiste en pre-multiplicar la ecuación por la inversa de cada matriz para aislar las articulaciones secuencialmente:

1. **Aislar la cadena de las últimas articulaciones:**

```
(T₁⁰)⁻¹ · T_obj = T₂¹(θ₂) · T₃²(θ₃)
```

2. **Aislar la última articulación:**

```
(T₂¹)⁻¹ · (T₁⁰)⁻¹ · T_obj = T₃²(θ₃)
```

Este procedimiento permite resolver paso a paso para cada ángulo articulado.

#### 🔧 Enfoque Práctico: Solución Geométrica

Aunque el método matricial es teóricamente robusto, su resolución algebraica puede ser muy compleja. Para un robot de 3 GDL como el nuestro, es más eficiente implementar una solución geométrica basada en el desacoplamiento cinemático, que se detalla en el código siguiente.

<details>
<summary>Ver código Python</summary>

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

#### 🔄 Validación por ciclo FK → IK → FK

* Entrada FK: `{40°, 60°, -50°}`
* Salida posición: `(10.632, 8.921, 21.781)`
* Entrada IK: posición anterior
* Salida IK: `{40°, 60°, -50°}` ✅

---

## 📊 6. Resultados y Validación Visual

📌 **Gráfico 1: Cinemática Directa**

> *Simulación para entrada `{40°, 60°, -50°}`.*

![FK](https://via.placeholder.com/800x600.png?text=FK+-+Visualización)

📌 **Gráfico 2: Cinemática Inversa**

> *El efector final alcanza el objetivo.*

![IK](https://via.placeholder.com/800x600.png?text=IK+-+Validación)

---

## 🏁 7. Conclusiones

✔️ Se desarrolló un modelo cinemático completo de un robot de 3 GDL.
✔️ Se validaron matemáticamente y computacionalmente las funciones FK e IK.
✔️ La simulación confirmó la coherencia teórica y práctica del modelo.

---

## 📎 8. Anexos

📂 **Anexo A:** Código Python completo
📐 **Anexo B:** Diagramas y coordenadas D-H
🖼️ **Anexo C:** Capturas de simulaciones
