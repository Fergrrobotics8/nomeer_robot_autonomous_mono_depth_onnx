# 🔌 Guía de Integración - Nodo de Profundidad con tu Pipeline RGB

## Resumen

El nodo `depth_inference_node.py` se conecta al tópico `/rgb_image` que ya está publicando en tu sistema.

**Objetivo**: Procesar cada imagen RGB y estimar la profundidad monocular en tiempo real usando MiDaS v2.1 Small (ONNX).

---

## Arquitectura del Sistema

```
┌─────────────────────────┐
│   Tu Nodo RGB           │
│ (Publica /rgb_image)    │
└──────────┬──────────────┘
           │ sensor_msgs/Image (BGR8)
           │
           ▼
┌─────────────────────────────────────┐
│  depth_inference_node.py            │
│  (Se suscribe a /rgb_image)         │
├─────────────────────────────────────┤
│ 1. Recibe imagen RGB                │
│ 2. Preprocesa (resize, normalize)   │
│ 3. Inferencia ONNX MiDaS v2.1      │
│ 4. Postprocesa (normaliza, resize)  │
│ 5. Publica resultados               │
└──────┬──────────────────────────────┘
       │
       ├─► /camera/depth_estimated (mono16)
       │   Valores: 0-65535 (normalizado 0-1)
       │
       └─► /camera/depth_colored (bgr8)
           Visualización con colormap TURBO
```

---

## 📋 Requisitos Previos

### 1. Modelo ONNX Descargado

```bash
cd ~/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx

# Verificar que existe:
ls -lh models/midas_v21_small.onnx

# Si no existe, descargar:
python3 scripts/download_midas_model.py
```

**Output esperado**: Archivo `models/midas_v21_small.onnx` (~160 MB)

### 2. Dependencias ROS 2

```bash
pip3 install onnxruntime onnxscript torch
```

### 3. Paquete Compilado

```bash
cd ~/ros2_ws
colcon build --packages-select mono_depth_onnx
source install/setup.bash
```

---

## 🚀 Ejecución Paso a Paso

### Paso 1: Verificar Tu Nodo de Imágenes

En **Terminal 1**, inicia tu nodo que publica RGB:

```bash
# Ejemplo genérico:
source ~/ros2_ws/install/setup.bash
ros2 run [tu_paquete] [tu_nodo]
```

Verifica que publica:
```bash
# En otra terminal:
ros2 topic list | grep rgb_image
ros2 topic echo /rgb_image --once
```

**Salida esperada**: Mensaje `sensor_msgs/Image` con el frame actual.

---

### Paso 2: Lanzar Nodo de Profundidad

En **Terminal 2**:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run mono_depth_onnx depth_inference_node.py
```

**Salida esperada** (logs):

```
[depth_inference] INFO: Depth Inference Node initialized
[depth_inference] INFO:   Model: midas_v21_small
[depth_inference] INFO:   Model path: models/midas_v21_small.onnx
[depth_inference] INFO:   Input size: 256x256
[depth_inference] INFO:   GPU enabled: False
[depth_inference] INFO: Loading ONNX model with providers: ['CPUExecutionProvider']
[depth_inference] INFO: Model loaded successfully
[depth_inference] INFO:   Input name: input_rgb, shape: ['1', '3', '256', '256']
[depth_inference] INFO:   Output name: output, shape: ['1', '1', '256', '256']
```

Si ves errores:
- **"Model not found"** → Ejecutar `scripts/download_midas_model.py`
- **"onnxruntime not installed"** → `pip3 install onnxruntime`
- **"Failed to initialize depth model"** → Ver logs en `install/setup.bash`

---

### Paso 3: Monitorear Ejecución

En **Terminal 3**:

```bash
# Ver FPS en tiempo real
watch -n 1 'ros2 topic hz /camera/depth_estimated'

# O ver la frecuencia:
ros2 topic hz /camera/depth_colored
```

**Salida esperada**:

```
average rate: 10.5 Hz
  min: 0.084s max: 0.124s std dev: 0.019s count: 10
```

---

## 📊 Verificar Datos Publicados

### Topic 1: Depth Estimado (16-bit)

```bash
ros2 topic echo /camera/depth_estimated --once
---
header:
  seq: 42
  stamp:
    sec: 1707861234
    nsec: 567891234
  frame_id: camera
height: 480
width: 640
encoding: mono16
is_bigendian: false
step: 1280
data: [256, 128, 512, ...]
```

- **Encoding**: `mono16` (16-bit unsigned)
- **Rango**: 0-65535 (donde 0=profundo, 65535=cerca)
- **Resolución**: Mismo tamaño que imagen entrada

### Topic 2: Depth Visualizado (BGR8)

```bash
# Para visualizar con rviz2:
rviz2

# En rviz2:
# 1. Add → Image
# 2. Topic: /camera/depth_colored
# 3. Encoder: default
```

- **Encoding**: `bgr8` (color)
- **Colormap**: TURBO (rojo=profundo, azul=cerca)

---

## ⚙️ Configuración Avanzada

### Parámetros del Nodo

Pasarlos en línea de comandos:

```bash
# Cambiar modelo dinamicamente
ros2 run mono_depth_onnx depth_inference_node.py \
  --ros-args \
  -p model_path:=models/midas_v21_small.onnx \
  -p enable_gpu:=true
```

**Parámetros disponibles**:
- `model_path`: Ruta del archivo ONNX
- `model_name`: Nombre del modelo (para logs)
- `input_height`: Altura preprocesamiento (default: 256)
- `input_width`: Ancho preprocesamiento (default: 256)
- `enable_gpu`: Usar GPU si está disponible (default: false)

### Verificar Providers Disponibles

```bash
python3 << 'EOF'
import onnxruntime as ort
print("Providers disponibles:", ort.get_available_providers())
EOF
```

**Salida esperada**:

```
Providers disponibles: ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
```

---

## 🔍 Debugging Común

### Problema: "Tópico /rgb_image no encontrado"

```bash
# Verificar que tu nodo está corriendo
ros2 node list | grep [tu_nodo]

# Ver todos los tópicos disponibles
ros2 topic list

# Subscribirse manualmente a /rgb_image
ros2 topic echo /rgb_image
```

**Solución**: Iniciar tu nodo de imágenes antes que `depth_inference_node.py`.

---

### Problema: "FPS muy bajo (< 1 Hz)"

**Causas posibles**:
1. CPU sobrecargada
2. Imágenes muy grandes (resize lento)
3. ONNX Runtime no optimizado

**Soluciones**:
```bash
# Reducir tamaño de entrada (más rápido, menos preciso)
ros2 run mono_depth_onnx depth_inference_node.py \
  --ros-args \
  -p input_height:=128 \
  -p input_width:=128

# O habilitar GPU (si está disponible)
ros2 run mono_depth_onnx depth_inference_node.py \
  --ros-args \
  -p enable_gpu:=true
```

---

### Problema: "Error: CUDA not found"

```bash
# Esto es NORMAL. El nodo fallback automáticamente a CPU.
# Solución: Usar CPU (funciona perfecto, solo más lento)
ros2 run mono_depth_onnx depth_inference_node.py \
  --ros-args \
  -p enable_gpu:=false
```

---

## 📈 Rendimiento Esperado

| Hardware | FPS (256x256) | Tiempo/Frame |
|----------|---------------|-------------|
| CPU (i7) | 5-10          | 100-200 ms  |
| CPU (i5) | 3-5           | 200-300 ms  |
| GPU (RTX) | 30-60         | 16-33 ms    |

---

## 🎯 Próximos Pasos: Usar el Depth en tu Aplicación

Una vez que tienes `/camera/depth_estimated` y `/camera/depth_colored` publicados:

### Opción 1: Visualizar en RViz2

```bash
rviz2

# Añadir:
1. Image → Topic: /camera/depth_colored
2. Image → Topic: [tu_rgb_image]
```

### Opción 2: Procesar en C++

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Procesar depth map...
}

rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub = 
    node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth_estimated", 10, depth_callback);
```

### Opción 3: Procesar en Python

```python
def depth_callback(msg):
    from cv_bridge import CvBridge
    import numpy as np
    
    bridge = CvBridge()
    depth_cv = bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
    depth_float = depth_cv.astype(np.float32) / 65535.0
    
    # Usar depth_float para tus algoritmos...
```

---

## 📝 Checklist Final

```
[ ] Modelo midas_v21_small.onnx existe en models/
[ ] Tu nodo publica en /rgb_image
[ ] depth_inference_node.py inicia sin errores
[ ] /camera/depth_estimated se publica
[ ] /camera/depth_colored se publica
[ ] FPS es razonable (> 1 Hz)
[ ] Depth values en rango 0-65535
[ ] Puedo visualizar en RViz2
```

---

## 🆘 Soporte

Si hay problemas:

1. **Revisar logs**: `ros2 run mono_depth_onnx depth_inference_node.py 2>&1 | tail -50`
2. **Verificar modelo**: `ls -lh models/midas_v21_small.onnx`
3. **Test inferencia**: `python3 -c "import onnxruntime; print(onnxruntime.__version__)"`
4. **Ver README.md**: Documentación técnica completa

---

**¡Listo!** 🎉 Tu pipeline de profundidad está integrado. Continúa con tu aplicación.
