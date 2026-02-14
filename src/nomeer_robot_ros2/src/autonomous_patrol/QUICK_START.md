# QUICK START GUIDE - Autonomous Patrol

## 1. Compilar el paquete

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_patrol
source install/setup.bash
```

## 2. Generar Waypoints de Ejemplo (Opcional)

Para pruebas rápidas sin robot real:

```bash
cd ~/ros2_ws/src/nomeer_robot_ros2/src/autonomous_patrol
python3 generate_example_waypoints.py
```

Esto creará `data/example_waypoints.yaml` con una trayectoria en cuadrado.

## 3. Workflow Completo

### Terminal 1: Iniciar Simulador
```bash
ros2 launch robot_description gazebo.launch.py
```

### Terminal 2: Grabar Waypoints
```bash
ros2 launch autonomous_patrol record_waypoints.launch.py
```

### Terminal 3: Teleoperación
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

- Usar flechas para mover robot
- Presionar **Ctrl+C** en Terminal 2 para guardar waypoints

### Terminal 2: Reproducir Waypoints
```bash
ros2 launch autonomous_patrol follow_waypoints.launch.py
```

Robot reproducirá la trayectoria automáticamente.
Métricas se guardan en `results/metrics.json`.

## 4. Comprobar Resultados

```bash
cat autonomous_patrol/results/metrics.json
```

## Parámetros Clave para Ajustar

### Más precisión:
```yaml
recorder:
  min_distance_between_waypoints: 0.05  # Más waypoints
follower:
  waypoint_tolerance: 0.1               # Más exigente
  max_linear_velocity: 0.3              # Más lento
```

### Más rápido:
```yaml
recorder:
  min_distance_between_waypoints: 0.2   # Menos waypoints
follower:
  waypoint_tolerance: 0.3               # Menos exigente
  max_linear_velocity: 0.8              # Más rápido
  control_frequency: 20.0               # Control más rápido
```

## Troubleshooting

**Robot no se mueve:**
- Verificar Gazebo ejecutándose: `ros2 topic list | grep odom`
- Verificar `/cmd_vel` existe: `ros2 topic list | grep cmd_vel`

**Waypoints no se guardan:**
- Verificar directorio `data/` existe dentro del paquete
- Comprobar permisos: `ls -la autonomous_patrol/data/`

**RViz no muestra waypoints:**
- Frame debe ser `odom` (verificar en RViz settings)
- Topics deben estar activos: mira en RViz "Add > By topic"

## Estructura de Directorios

```
autonomous_patrol/
├── data/                    # Waypoints grabados aqui
├── results/                 # Métricas de ejecución aqui
├── config/                  # Archivos configuración
└── launch/                  # Launch files
```

## Comandos Útiles

```bash
# Ver configuración actual
ros2 param list /record_waypoints
ros2 param get /record_waypoints recorder.min_distance_between_waypoints

# Cambiar parámetros en tiempo real
ros2 param set /follow_waypoints follower.waypoint_tolerance 0.15

# Monitorear topics
ros2 topic echo /waypoint_recorder/status
ros2 topic echo /waypoint_follower/status
ros2 topic echo /waypoint_follower/current_waypoint
```
