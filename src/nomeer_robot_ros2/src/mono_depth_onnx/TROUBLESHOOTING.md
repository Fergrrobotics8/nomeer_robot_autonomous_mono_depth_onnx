# 🔧 Troubleshooting & Dependencias

## NumPy Compatibility Issue (ROS 2 Humble)

### Problema
```
AttributeError: _ARRAY_API not found
cv_bridge compiled with NumPy 1.x but system has NumPy 2.2.6
```

### Solución
Downgrade NumPy a 1.x:
```bash
pip install 'numpy<2'
```

Este problema es **conocido en ROS 2 Humble** debido a cv_bridge compilado con NumPy 1.x.

### Verificación
```bash
python3 -c "import numpy; print(numpy.__version__)"
# Debe mostrar: 1.26.x (no 2.x)
```

---

## OpenCV Warning (Expected)
```
opencv-python 4.13.0.92 requires numpy>=2; python_version >= "3.9"
```
Este es solo un warning de dependencia. El sistema funciona perfectamente con NumPy 1.26.4.

---

## Performance Baseline (B3: Depth Inference)

### En CPU (sin GPU)
```
FPS: 5.8
Tiempo por frame: ~175ms
Modelo: MiDaS v2.1 Small
Resolución entrada: 240x320 → redimensiona a 256x256
```

### Logs Esperados
```
[INFO] Depth inference - Frame: 30, Time: 175.2ms (max: 221.7ms), FPS: 5.7, Depth range: [0.000, 0.999]
[INFO] Depth inference - Frame: 60, Time: 173.7ms (max: 231.8ms), FPS: 5.8, Depth range: [0.000, 0.999]
```

---

## Instalación Completa (Fresh Setup)

```bash
cd ~/ros2_ws

# 1. Build package
colcon build --packages-select mono_depth_onnx
source install/setup.bash

# 2. Install dependencies
pip3 install torch timm opencv-python onnx onnxruntime onnxscript

# 3. FIX: Downgrade NumPy
pip install 'numpy<2'

# 4. Download model
cd src/nomeer_robot_ros2/src/mono_depth_onnx
python3 scripts/download_midas_model.py

# 5. Ready!
cd ~/ros2_ws
```

---

## Checklist Setup

- [ ] NumPy 1.26.x instalado: `pip install 'numpy<2'`
- [ ] Modelo descargado: `ls models/midas_v21_small.onnx`
- [ ] Package compilado: `colcon build --packages-select mono_depth_onnx`
- [ ] Environment sourced: `source install/setup.bash`

---

## Después del Setup

**Terminal 1**: Tu nodo de cámara
```bash
ros2 run [tu_paquete] [tu_nodo]  # /rgb_image
```

**Terminal 2**: Depth Inference (B3)
```bash
source ~/ros2_ws/install/setup.bash
ros2 run mono_depth_onnx depth_inference_node.py
# Esperado: FPS 5-6, Depth range [0.0-1.0]
```

**Terminal 3**: Métricas (B4 - opcional)
```bash
ros2 run mono_depth_onnx depth_metric_node.py
```

**Terminal 4**: Verifica
```bash
ros2 topic list | grep depth
ros2 topic echo /camera/depth_estimated --once
```

---

**Listo. B3 operativo.**
