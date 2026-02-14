# 🚀 Setup Rápido: Robot Nomeer

## Configuración Inicial (Una sola vez)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# Instalar dependencias del sistema
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
pip3 install 'numpy<2' opencv-python onnxruntime PyYAML scipy torch timm onnx onnxscript

# Compilar paquetes
colcon build --packages-select autonomous_patrol mono_depth_onnx robot_description
source install/setup.bash

# Descargar modelo de IA
cd src/nomeer_robot_ros2/src/mono_depth_onnx
python3 scripts/download_midas_model.py
cd ~/ros2_ws
```

---

## En Cada Nueva Sesión

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Grabación de Trayectoria (Part A)

**Terminal 1**:
```bash
ros2 launch robot_description robot.launch.py
```

**Terminal 2**:
```bash
ros2 launch autonomous_patrol record_waypoints.launch.py
```

**Terminal 3**:
```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

Navega el robot, luego Ctrl+C en Terminal 2 para guardar waypoints.

---

## Reproducción de Trayectoria (Part A)

**Terminal 1**:
```bash
ros2 launch robot_description robot.launch.py
```

**Terminal 2**:
```bash
ros2 launch autonomous_patrol follow_waypoints.launch.py
```

**Terminal 3** (opcional - visualización):
```bash
ros2 launch autonomous_patrol visualize_waypoints.launch.py
```

---

## Visión: Profundidad con IA (Part B)

```bash
ros2 launch mono_depth_onnx full_pipeline.launch.py
```

---

## Integración Completa (Autonomía + Visión + Seguridad)

**Terminal 1**:
```bash
ros2 launch autonomous_patrol follow_waypoints.launch.py
```

**Terminal 2**:
```bash
ros2 run mono_depth_onnx depth_inference_node.py
```

**Terminal 3**:
```bash
ros2 run mono_depth_onnx autonomous_depth_safety_node.py
```

---

**Más detalles**: Ver `src/SETUP_GUIDE.md`
