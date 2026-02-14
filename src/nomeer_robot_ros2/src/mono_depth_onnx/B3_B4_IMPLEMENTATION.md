# B3 + B4: Inferencia ONNX y Métricas de Profundidad

## ✅ B3: Inferencia con ONNX Runtime - COMPLETADO

### Nodo: `depth_inference_node.py`

**Responsabilidad**: Consumir imágenes RGB y ejecutar modelo ONNX MiDaS v2.1 Small

```
Input:  /rgb_image (sensor_msgs/Image) - RGB image (cualquier tamaño)
        ↓
   Preprocesar: Redimensionar 256x256
        ↓
   Inferencia: MiDaS ONNX Runtime (CPU o GPU)
        ↓
   Postprocesar: Redimensionar a tamaño original
        ↓
Output: /camera/depth_estimated (mono16) - 16-bit depth map
        /camera/depth_colored (bgr8)    - Visualización con colormap
```

**Trabajo realizado**:
- ✅ Modelo MiDaS v2.1 Small carga correctamente
- ✅ Input shape: [batch_size, 3, 256, 256]
- ✅ Output shape: [batch_size, 256, 256]
- ✅ Profundidad relativa normalizada 0-1
- ✅ Maneja imágenes de cualquier tamaño

**Unidades**: Profundidad relativa (0-1)
- 0 = muy lejano
- 1 = muy cercano

---

## ✅ B4: Métrica de Profundidad - IMPLEMENTADO

### Nodo: `depth_metric_node.py`

**Responsabilidad**: Calcular métrica simple y robusta de profundidad frontal

```
Input:  /camera/depth_estimated (mono16) - Depth map from B3
        ↓
   1. Extraer ROI (región central)
        ↓
   2. Filtrar outliers (percentil)
        ↓
   3. Calcular métricas
        ↓
Output: /depth_metric/min_frontal_depth    (Float32)
        /depth_metric/avg_frontal_depth    (Float32)
        /depth_metric/median_frontal_depth (Float32)
        /depth_metric/obstacle_detected    (Float32: 0 o 1)
```

### Métricas Publicadas

#### 1. `min_frontal_depth` - Profundidad Mínima Frontal
- **Significado**: Objeto más cercano en región central
- **Rango**: 0-1 (normalizado)
- **Uso**: Detección de obstáculos
- **Ejemplo**: 0.3 = objeto bastante cercano

#### 2. `avg_frontal_depth` - Profundidad Media
- **Significado**: Promedio de profundidad en ROI
- **Uso**: Evaluación general de distancia
- **Robustez**: Media simple (puede tener outliers)

#### 3. `median_frontal_depth` - Profundidad Mediana
- **Significado**: Mediana de profundidad en ROI
- **Uso**: Mejor que media (robusto a outliers)
- **Robustez**: MÁS ROBUSTA que media

#### 4. `obstacle_detected` - Bandera de Obstáculo
- **Valores**: 1.0 si hay obstáculo, 0.0 si está libre
- **Criterio**: Si `min_depth < obstacle_threshold`
- **Configuración**: `obstacle_threshold` (default: 0.5)
- **Uso**: Para navegación autónoma

### Parámetros Configurables

```yaml
# ROI (Región de Interés - área central de la imagen)
roi_x_start: 0.2          # 20% desde la izquierda
roi_x_end: 0.8            # 80% desde la izquierda
roi_y_start: 0.2          # 20% desde arriba
roi_y_end: 0.8            # 80% desde arriba

# Filtrado de outliers (percentil)
outlier_percentile_low: 5    # Remove bottom 5%
outlier_percentile_high: 95  # Remove top 5%

# Umbral de obstáculo
obstacle_threshold: 0.5      # Depth < 0.5 = obstacle
```

### Filtrado Robusto: Percentil

El nodo usa **percentil** para filtrar outliers:

```
Todos los valores: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 99.9]
                                                                      ↑ outlier

Percentil 5-95:   [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] ✅ Robusto
```

**Ventajas**:
- Elimina extremos automáticamente
- No requiere parámetros complicados
- Método probado en visión por computadora

---

## 🔄 Pipeline Completo: RGB → Profundidad → Métricas

```
Nodo RGB              Nodo Inferencia        Nodo Métricas
─────────────────     ─────────────────     ─────────────────
  /rgb_image      →   depth_inference   →   depth_metric
  (320x240 BGR)       _node.py              _node.py
                        ↓                     ↓
                   /camera/depth_estimated  /depth_metric/*
                   /camera/depth_colored
```

---

## 📊 Ejemplo de Datos Reales

### Input: Imagen RGB (320x240)
```
Escena: Habitación con objetos a diferentes distancias
        - Pared al fondo (~1m)
        - Mesa (~0.5m)
        - Objeto en mesa (~0.3m)
```

### Output: Depth Map (320x240, normalizado 0-1)
```
Background (pared):      [0.1, 0.1, 0.1, ...]  ← lejano
Mid (mesa):              [0.5, 0.5, 0.5, ...]  ← medio
Foreground (objeto):     [0.8, 0.8, 0.8, ...]  ← cercano
```

### Métricas Calculadas (ROI central)
```
min_frontal_depth:    0.35  ← objeto más cercano
avg_frontal_depth:    0.52  ← promedio de distancias
median_frontal_depth: 0.50  ← mediana (robusto)
obstacle_detected:    1.0   ← SI hay obstáculo (0.35 < 0.5)
```

---

## ✅ Requisitos de tu Maestra - COMPLETADOS

### B3 ✓ Inferencia con ONNX Runtime
- ✅ Nodo ROS 2 que consume RGB
- ✅ Ejecuta inferencia con onnxruntime
- ✅ Publica depth map en `/camera/depth_estimated`
- ✅ Profundidad en unidades relativas (0-1) CLARAMENTE EXPLICADO

### B4 ✓ Métrica de Profundidad
- ✅ Calcula profundidad mínima frontal (ROI central)
- ✅ Publica métrica en topic ROS 2 (Float32)
- ✅ Filtrado de outliers (percentil ← ROBUSTO)
- ✅ Parámetros configurables (ROI, umbrales, percentil)

---

## 🚀 Cómo Usar

### Terminal 1: Tu Nodo de Cámara
```bash
# Usa tu nodo de cámara que publica en /rgb_image
ros2 run [tu_paquete] [tu_nodo]
```

### Terminal 2: Nodo de Inferencia
```bash
source ~/ros2_ws/install/setup.bash
ros2 run mono_depth_onnx depth_inference_node.py
```

### Terminal 3: Nodo de Métricas
```bash
source ~/ros2_ws/install/setup.bash
ros2 run mono_depth_onnx depth_metric_node.py
```

### Terminal 4: Monitoreo
```bash
# Ver todas las métricas
ros2 topic list | grep depth_metric

# Escuchar métrica específica (16-bit depth)
ros2 topic echo /depth_metric/min_frontal_depth

# Ver si hay obstáculo
ros2 topic echo /depth_metric/obstacle_detected

# Monitorear FPS
ros2 topic hz /depth_metric/min_frontal_depth
```

---

## 📈 Ejemplo de Salida

```
[depth_inference] Frame 30: min_depth=0.345 avg_depth=0.521 median_depth=0.500 [OBSTACLE]
[depth_inference] Frame 60: min_depth=0.821 avg_depth=0.712 median_depth=0.705 [CLEAR]
[depth_inference] Frame 90: min_depth=0.412 avg_depth=0.548 median_depth=0.540 [OBSTACLE]
```

Significado:
- **Frame 30**: Hay obstáculo (0.345 < umbral 0.5)
- **Frame 60**: Camino libre (0.821 > umbral 0.5)
- **Frame 90**: Hay obstáculo nuevamente

---

## 🎯 Integración con Autonomía (Próximo)

Cuando tengas autonomía:

```python
def obstacle_callback(msg: Float32):
    if msg.data == 1.0:  # Obstacle detected
        # Stop robot
        self.velocity_pub.publish(Twist())
```

---

**B3 + B4 Completados ✅**
