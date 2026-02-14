# QUICK START - Mono Depth ONNX

## 🚀 5 minutos para empezar

### Paso 1: Compilar (1 minuto)

```bash
cd ~/ros2_ws
colcon build --packages-select mono_depth_onnx
source install/setup.bash
```

**Status**: ✅ Si compila sin errores

### Paso 2: Instalar Dependencias

```bash
pip3 install torch timm opencv-python onnx onnxruntime onnxscript

# IMPORTANTE: Downgrade NumPy (compatibilidad con cv_bridge)
pip install 'numpy<2'
```

**¿Por qué numpy<2?** ROS 2 Humble usa cv_bridge compilado con NumPy 1.x


**Descargar y convertir el modelo MiDaS v2.1 Small (todo automático):**
```bash
cd ~/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx
python3 scripts/download_midas_model.py
```

**Output**: `models/midas_v21_small.onnx` creado

### Paso 3: Preparar Imágenes (30 segundos)

```bash
# Si no tienes imágenes, crear carpeta
mkdir -p data/images

# Opción A: Usar imágenes de prueba (si tienes)
cp /ruta/a/tus/imágenes/*.jpg data/images/

# Opción B: Si no tienes imágenes
# Ver sección "Obtener Datos de Prueba" abajo
```

### Paso 4: Ejecutar Nodo de Inferencia (30 segundos)

El nodo de inferencia **se conecta automáticamente** al tópico `/rgb_image` que publicas.

En terminal 1 (inicia tu nodo que publica RGB):
```bash
ros2 run [tu_paquete] [tu_nodo]
```

En terminal 2 (inicia el nodo de profundidad):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run mono_depth_onnx depth_inference_node.py
```

**Esperado**: 
- Verás logs mostrando FPS, tiempo de inferencia (~100-300ms por frame en CPU)
- Topics publicados: `/camera/depth_estimated` (16-bit depth) y `/camera/depth_colored` (visualización)

### Paso 5: Verificar Salida (en otra terminal)

```bash
# Ver depth map (16-bit)
ros2 topic echo /camera/depth_estimated --once

# Ver depth visualizado (con colormap)
ros2 topic echo /camera/depth_colored --once

# Ver topics disponibles
ros2 topic list | grep depth
```

---

## 🎯 Verificación Rápida

### Checklist:

```
[ ] Paquete compiló sin errores
[ ] Modelo descargado (models/midas_v21_small.onnx existe)
[ ] Nodo de imágenes RGB en ejecución (publicando en /rgb_image)
[ ] Nodo `depth_inference_node.py` ejecutándose sin errores
[ ] Topics publicando datos (ros2 topic list | grep depth)
```

---

## 🔌 Integración Detallada

### Flujo de Datos:

1. **Tu nodo** → publica en `/rgb_image` (sensor_msgs/Image)
2. **depth_inference_node.py** → se suscribe a `/rgb_image`
3. **Procesa** → ejecuta inferencia con MiDaS v2.1 ONNX
4. **Publica**:
   - `/camera/depth_estimated`: 16-bit depth map (mono16)
   - `/camera/depth_colored`: Depth visualizado con colormap (bgr8)

### Configuración del Nodo:

```bash
# Parámetros por defecto (en el código):
- model_path: models/midas_v21_small.onnx
- model_name: midas_v21_small
- input_height: 256
- input_width: 256
- enable_gpu: false
- subscription_topic: /rgb_image
```

### Prueba Manual:

```bash
# Terminal 1: Tu nodo de imágenes
ros2 run [tu_paquete] [tu_nodo]

# Terminal 2: Nodo de profundidad
source ~/ros2_ws/install/setup.bash
ros2 run mono_depth_onnx depth_inference_node.py

# Terminal 3: Monitoreo
ros2 topic hz /camera/depth_estimated
ros2 topic echo /camera/depth_estimated --once
```

### Cambiar Parámetros (línea de comandos):

```bash
ros2 run mono_depth_onnx depth_inference_node.py \
  --ros-args \
  -p model_path:=models/midas_v21_small.onnx \
  -p enable_gpu:=false
```



### Opción 1: Descargar Dataset Online

```bash
# Crear directorio
mkdir -p mono_depth_onnx/data/images
cd mono_depth_onnx/data/images

# Descargar algunas imágenes de ejemplo (CC0 license)
# Ejemplo: COCO dataset, unsplash, etc.

# O simplemente copiar cualquier imagen:
cp ~/Pictures/*.jpg .
```

### Opción 2: Generar Imágenes desde Video

```bash
# Si tienes un video:
ffmpeg -i video.mp4 -vf fps=2 data/images/frame_%04d.jpg
```

### Opción 3: Usar Webcam Directamente

Cambiar en `config/mono_depth_config.yaml`:

```yaml
image_source:
  source_type: "webcam"
  source_path: "0"
```

Luego ejecutar: `ros2 launch mono_depth_onnx full_pipeline.launch.py`

---

## 🎯 Verificación Rápida

### Checklist:

```
[ ] Paquete compiló sin errores
[ ] Modelo descargado (models/midas_v21_small.onnx existe)
[ ] Imágenes en data/images/ (3+ imágenes)
[ ] Launch file ejecutado sin crashes
[ ] Topics publicando datos (ros2 topic list)
[ ] Métricas en results/depth_metrics.json después de unos segundos
```

---

## 🔧 Configuración Común

depth_estimator:
### Para Desarrollo Rápido
En `config/mono_depth_config.yaml`:

```yaml
image_source:
  publish_frequency: 30.0      # Más rápido
depth_estimator:
  model_name: "midas_v21_small" # Pequeño, rápido y recomendado
  enable_gpu: false            # CPUs ok
```

depth_estimator:
depth_metric:
### Para Precisión
```yaml
depth_estimator:
  model_name: "midas_v21_small" # (o el modelo ONNX que tengas)
  enable_gpu: true             # GPU recomendado si está disponible
depth_metric:
  outlier_filter_type: "iqr"   # Mejor filtrado
```

depth_estimator:
### Para GPU
```yaml
depth_estimator:
  enable_gpu: true             # Activar CUDA
```

Verificar: `python3 -c "import onnxruntime; print(onnxruntime.get_available_providers())"`

---

## 🐛 Problemas Comunes

| Problem | Solución |
|---------|----------|
| "Model not found" | Descargar manualmente el modelo y colocarlo en `models/` |
| "No images found" | `mkdir -p data/images && cp *.jpg data/images/` |
| "ONNX Runtime not found" | `pip3 install onnxruntime` |
| "Low FPS" | Usar `midas_v3_small`, reduce `publish_frequency` |
| "Import error cv_bridge" | `pip3 install cv-bridge` |
| "CUDA not found" | OK, usar CPU (más lento pero funciona) |

---

## 📊 Monitoreo

### Ver Performance

```bash
# FPS de inferencia
watch -n 1 'ros2 topic hz /camera/depth_estimated'

# FPS de métricas
watch -n 1 'ros2 topic hz /depth_metric/min_frontal_depth'

# CPU/Memory (en otra terminal)
top
```

### Ver Datos

```bash
# Profundidad mínima
ros2 topic echo /depth_metric/min_frontal_depth --once

# Ver todos los tópicos
ros2 topic list | grep camera
ros2 topic list | grep depth
```

---

## 💾 Resultados Generados

Después de ejecutar, verás:

- `results/depth_metrics.json`: Métricas guardadas
- `results/safety_events.json`: Eventos (si safety activo)

Ver contenido:
```bash
cat mono_depth_onnx/results/depth_metrics.json | python3 -m json.tool
```

---

## 🔗 Integración con Autonomía (Opcional)

Ejecutar todos los componentes:

**Terminal 1**: Autonomía
```bash
ros2 launch autonomous_patrol follow_waypoints.launch.py
```

**Terminal 2**: Profundidad
```bash
ros2 launch mono_depth_onnx with_autonomy.launch.py
```

Resultado: Robot se detiene si detecta obstáculo delante (seguridad).

---

## 📚 Más Información

- **README.md**: Documentación técnica completa
- **RESUMEN_ES.md**: Resumen en español
- **scripts/download_midas_model.py**: Script de modelo
- **config/mono_depth_config.yaml**: Todas las opciones

---

## 🚨 Testing Checklist

```bash
# Test 1: Compilación
colcon build --packages-select mono_depth_onnx

# Test 2: Modelo
python3 scripts/download_midas_model.py --model midas_v21_small

# Test 3: Nodo inferencia
ros2 run mono_depth_onnx depth_inference_node.py

# Test 4: Nodo métricas
ros2 run mono_depth_onnx depth_metric_node.py

# Test 5: Pipeline completo
ros2 launch mono_depth_onnx full_pipeline.launch.py

# Test 6: Con autonomía
# [Iniciar autonomía primero]
ros2 launch mono_depth_onnx with_autonomy.launch.py
```

---

**¡Listo!** 🎉 Ya tienes el sistema de profundidad con IA corriendo.

Para detalles avanzados, ver README.md completo.
