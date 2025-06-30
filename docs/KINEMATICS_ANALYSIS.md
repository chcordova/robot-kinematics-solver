
# 🤖 Análisis de Cinemática

Desarrollo paso a paso de la Cinemática Inversa (IK) para un robot RRR (3-DOF) utilizando un enfoque matricial. Este método es una forma algebraica basada en la inversión de matrices de transformación homogénea, comúnmente utilizadas en el modelo de Denavit-Hartenberg (D-H).

---

## 🎯 Objetivo del Problema

**Datos Proporcionados:**
- Posición y orientación objetivo del efector final
- Parámetros geométricos del robot:
  - L₁ = 10
  - L₂ = 12
  - L₃ = 8
- Configuración articular para CD:
  - θ₁ = 40°
  - θ₂ = 60°
  - θ₃ = -50°

**Objetivo:**  
Determinar los ángulos articulares θ₁, θ₂, θ₃ que logran la posición objetivo.

---

## 📚 Paso 0: Fundamento de la Cinemática Directa (CD)

La CD calcula la posición del efector a partir de ángulos articulares dados (θ₁, θ₂, θ₃) usando el producto de matrices homogéneas:

**Fórmula General:**

$$
T_3^0 = T_1^0(θ₁) · T_2^1(θ₂) · T_3^2(θ₃)
$$

Cada matriz representa un cambio de sistema de coordenadas entre eslabones según D-H.

---

## 📝 Paso 1: Construcción de la Cinemática Directa

**Tabla D-H del robot RRR:**

| i | θᵢ       | dᵢ  | aᵢ | αᵢ  |
|---|----------|-----|----|-----|
| 1 | 40°      | 10  | 0  | 90° |
| 2 | 60°      | 0   | 12 | 0°  |
| 3 | −50°     | 0   | 8  | 0°  |

**Conversión a radianes:**
- θ₁ = 0.6981 rad
- θ₂ = 1.0472 rad
- θ₃ = -0.8727 rad

---

## 🧮 Paso 2: Construcción de las Matrices de Transformación

**Fórmula D-H:**

$$
T_i^{i-1} =
\begin{bmatrix}
\cosθᵢ & -\sinθᵢ\cosαᵢ & \sinθᵢ\sinαᵢ & aᵢ\cosθᵢ \\
\sinθᵢ & \cosθᵢ\cosαᵢ & -\cosθᵢ\sinαᵢ & aᵢ\sinθᵢ \\
0 & \sinαᵢ & \cosαᵢ & dᵢ \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

**Matrices individuales:**
- T₁⁰
- T₂¹
- T₃²

---

## ✅ Paso 3: Producto Total de Matrices

Multiplicando:

$$
T_3^0 = T_1^0 · T_2^1 · T_3^2
$$

**Resultado del efector final:**
- x = 10.632
- y = 8.921
- z = 21.781

---

## 🔄 Paso 4: Inversión Cinemática (IK)

### 🎯 Objetivo:
Recuperar θ₁, θ₂, θ₃ desde la posición objetivo usando el enfoque matricial inverso.

### 🔁 Paso 4.1: Aislamiento de θ₁
Extraer coordenadas del vector de posición y aplicar inversa de T₁⁰.

### 🔁 Paso 4.2: Aislamiento de θ₂
Multiplicar por inversa de T₂¹ para obtener T₃².

### 🔁 Paso 4.3: Aislamiento de θ₃
Comparar T₃² con la matriz simbólica para extraer el valor de θ₃.

---

## 🧠 Sistema Algebraico

Resolver el sistema no lineal con sympy o métodos numéricos.  
Resultado esperado:  
- θ₁ = 40°, θ₂ = 60°, θ₃ = -50°

---

## 🔎 Paso 5: Verificación Cruzada CD vs. CI

- CD: con [θ₁, θ₂, θ₃] obtenemos (x, y, z)
- CI: con (x, y, z) recuperamos los mismos [θ₁, θ₂, θ₃]

✅ Coincidencia total → el modelo D-H y la inversión funcionan correctamente.

---
