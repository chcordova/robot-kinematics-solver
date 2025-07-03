
# 🤖 Simulador de Brazo Robótico de 3 Grados de Libertad (3GDL)

**Curso:** Recursos Computacionales  
**Profesor:** Anthony Luiggi Inostroza Campos  
**Equipo:** Charles Javier Cordova, Esteffany Huamanraime, Ricardo Phill, Vladimir Vilca  

---

## 🧠 Introducción

El presente proyecto simula un brazo robótico de 3GDL empleando Python y librerías científicas como NumPy y Matplotlib. Se enfoca en el estudio de la cinemática directa e inversa en un entorno accesible, educativo y sin hardware físico.

---

## 🛠️ Aplicación

Este simulador tiene aplicaciones potenciales en educación, automatización industrial y robótica experimental. Puede ser usado para desarrollar tareas de manipulación, programación de trayectorias y enseñanza de fundamentos robóticos.

---

## 🎯 Objetivos

- Diseñar un brazo robótico virtual con 3 GDL.
- Implementar funciones de cinemática directa e inversa.
- Visualizar y validar los movimientos del efector final.
- Fomentar el uso de herramientas libres como Python.

---

## 📐 Alcance

- Cinemática directa basada en ángulos de entrada.
- Cinemática inversa mediante métodos geométricos, matriciales y numéricos.
- Visualización 2D y 3D del modelo y trayectorias.
- No se incluye integración física o simulación dinámica.

---

## ⚙️ Desarrollo

### Dimensiones Asignadas
- Longitudes: `L₁ = 10 cm`, `L₂ = 12 cm`, `L₃ = 8 cm`
- Grados de Libertad: 3
- Sistema tipo RRR (rotacional puro)

### Implementación
- `GeometricSolver`: Resuelve CI y CD con trigonometría.
- `MatrixDHSolver`: Aplica la convención Denavit-Hartenberg.
- `RTBSolver`: Usa Robotics Toolbox para cálculos iterativos.

### Visualización
Se utilizan gráficos de Matplotlib en 3D con etiquetas dinámicas para ilustrar articulaciones, eslabones y objetivos.

---

## 📊 Tabla de Resumen de Articulaciones

| Articulación | Ángulo | Eje de Giro | Movimiento         | Analogía  |
|--------------|--------|-------------|--------------------|-----------|
| Base         | θ₁     | Eje Z       | Giro Horizontal    | Cintura   |
| Hombro       | θ₂     | Eje Y local | Elevación          | Brazo     |
| Codo         | θ₃     | Eje Y local | Flexión            | Codo      |

---

## 🧮 Cinemática: Representación Visual

<img src="/assets/arm-3GDL.png" alt="Brazo Robótico" width="600" height="auto" style="display: block; margin: 0 auto;">
  
---

## 📘 Conclusiones

1. Integración efectiva de programación, matemáticas y visualización.
2. La simulación es una herramienta didáctica poderosa para aprender robótica.
3. Se validó correctamente el uso de CD e CI en múltiples estrategias.
4. Se comprobó la eficiencia del entorno libre Python.
5. El entorno simulado permite análisis rápidos pero tiene limitaciones físicas.

---

## 📎 Anexos

- Código fuente completo (ver repositorio GitHub):  
  [github.com/chcordova/robot-kinematics-solver](https://github.com/chcordova/robot-kinematics-solver)
- Tablas DH y coordenadas
- Bocetos y diagramas del brazo
- Capturas del simulador en ejecución

---

## 📚 Glosario

- **Brazo Robótico**: Mecanismo articulado que imita el brazo humano.
- **GDL**: Grados de libertad (número de movimientos independientes).
- **Cinemática Directa**: Determina posición del efector a partir de ángulos.
- **DH**: Denavit-Hartenberg, método de parametrización robótica.
- **Python**: Lenguaje de programación interpretado y versátil.
- **NumPy**: Librería para álgebra lineal y vectores.
- **Matplotlib**: Librería para visualización 2D/3D en Python.

---
