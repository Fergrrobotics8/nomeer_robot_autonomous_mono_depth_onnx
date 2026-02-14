# Autonomous Patrol - Waypoint Navigation System

Sistema de navegación autónoma por waypoints para ROS 2. Permite grabar y reproducir trayectorias del robot con métricas de desempeño integradas.

## Características

### A1: Grabación de Waypoints
- Registra waypoints desde odometría mientras el robot es teleoperado
- Dos modos de muestreo:
  - **Distancia**: Registra waypoint solo si distancia mínima es excedida
  - **Frecuencia**: Registra a frecuencia fija
- Guarda en formato YAML con metadatos
- Parámetros configurables

### A2: Seguimiento Autónomo
- Carga y reproduce waypoints secuencialmente
- Control proporcional de velocidades lineal y angular
- Cierre de lazo con realimentación de odometría
- Métricas de ejecución en tiempo real y archivo JSON

### A3: Visualización en RViz
- Waypoints visualizados como esferas con código de colores
- Trayectoria completa dibujada como línea
- Marcadores en tiempo real del waypoint objetivo actual
- Línea de conexión actual waypoint objetivo

### A4: Generación de Métricas
- Tiempo total de ejecución
- Waypoints completados / total
- Error medio y máximo respecto a waypoint objetivo
- Resumen de tiempos por transición
- Archivo JSON con datos detallados

## Estructura del Paquete

```
autonomous_patrol/
├── autonomous_patrol/
│   ├── __init__.py
│   ├── record_waypoints_node.py    # Nodo grabación (A1)
│   ├── follow_waypoints_node.py    # Nodo seguimiento (A2)
│   └── visualizer_node.py          # Visualizador (A3)
├── config/
│   ├── autonomous_patrol_config.yaml   # Configuración principal
│   └── test_config.yaml                # Configuración para pruebas
├── launch/
│   ├── record_waypoints.launch.py
│   ├── follow_waypoints.launch.py
│   └── visualize_waypoints.launch.py
├── data/                           # Directorio de almacenamiento de waypoints
├── results/                        # Directorio de resultados/métricas
└── rviz/
    └── waypoints.rviz             # Configuración RViz
```

## Instalación

### Compilar el paquete

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_patrol
source install/setup.bash
```

## Uso

### Paso 1: Grabar Waypoints

1. **Iniciar el simulador Gazebo con el robot**:
   ```bash
   ros2 launch robot_description gazebo.launch.py
   ```

2. **En otra terminal, iniciar el grabador de waypoints**:
   ```bash
   ros2 launch autonomous_patrol record_waypoints.launch.py
   ```

3. **En una tercera terminal, teleoperador el robot** (usar teleop_twist_keyboard):
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
   ```

4. **Navegar el robot** por la ruta deseada
5. **Presionar Ctrl+C** en el terminal del grabador para guardar waypoints

Los waypoints se guardarán en `autonomous_patrol/data/waypoints.yaml`.

### Paso 2: Reproducir Waypoints

1. **Iniciar el seguidor de waypoints**:
   ```bash
   ros2 launch autonomous_patrol follow_waypoints.launch.py
   ```

El robot reproducirá automáticamente la trayectoria grabada. Las métricas se guardarán en `results/metrics.json`.

### Paso 3: Visualización (Opcional)

Solo visualizar waypoints sin ejecutarlos:
```bash
ros2 launch autonomous_patrol visualize_waypoints.launch.py
```

## Configuración

### Parámetros del Grabador

En `config/autonomous_patrol_config.yaml`:

```yaml
recorder:
  sampling_mode: "distance"              # "distance" o "frequency"
  min_distance_between_waypoints: 0.1    # metros
  sampling_frequency: 5.0                # Hz (si mode = "frequency")
  output_file: "waypoints.yaml"
  data_directory: "data"
```

### Parámetros del Seguidor

```yaml
follower:
  waypoints_file: "waypoints.yaml"
  waypoint_tolerance: 0.2                # metros - umbral para considerar waypoint alcanzado
  max_linear_velocity: 0.5               # m/s
  max_angular_velocity: 1.0              # rad/s
  control_frequency: 10.0                # Hz
  use_yaw_control: false                 # true para control de orientación
```

## Formato de Waypoints (YAML)

```yaml
metadata:
  recording_date: "2026-02-12T10:30:45.123456"
  total_waypoints: 25
  sampling_mode: "distance"
  min_distance: 0.1
  sampling_frequency: 5.0

waypoints:
  - id: 1
    timestamp: 1707732645.123
    x: 0.0
    y: 0.0
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.0
    qw: 1.0
    linear_vel: 0.3
    angular_vel: 0.0
  # ... más waypoints ...
```

## Archivo de Métricas

Generado en `results/metrics.json`:

```json
{
  "execution_summary": {
    "total_execution_time": 45.23,
    "waypoints_completed": 25,
    "total_waypoints": 25,
    "success": true,
    "execution_date": "2026-02-12T10:45:30.123456"
  },
  "error_metrics": {
    "mean_error_to_waypoint": 0.087,
    "max_error_to_waypoint": 0.245,
    "min_error_to_waypoint": 0.002
  },
  "timing_metrics": {
    "mean_time_per_waypoint": 1.81,
    "max_time_per_waypoint": 3.2,
    "min_time_per_waypoint": 0.5
  },
  "per_waypoint_data": [...]
}
```

## Topics Publicados

### Grabador
- `/waypoint_recorder/status` (String): Estado del grabador

### Seguidor
- `/cmd_vel` (Twist): Comandos de velocidad
- `/waypoint_follower/status` (String): Estado de ejecución
- `/waypoint_follower/current_waypoint` (Int32): Índice waypoint actual
- `/waypoint_follower/markers` (MarkerArray): Marcadores visualización

### Visualizador
- `/waypoint_visualizer/waypoints` (MarkerArray): Todos los waypoints
- `/waypoint_visualizer/trajectory` (Marker): Línea de trayectoria

## Troubleshooting

### "Waypoints file not found"
- Asegurar que se ejecutó primero el grabador
- Verificar que el archivo existe en el directorio `data/`

### Robot no se mueve
- Verificar que Gazebo está ejecutándose
- Comprobar que `/odom` publica datos
- Revisar que `/cmd_vel` está siendo escuchado por el simulador

### Errores grandes en seguimiento
- Reducir `max_linear_velocity`
- Aumentar `waypoint_tolerance`
- Ajustar ganancia de control en `_calculate_velocity_command()`

## Notas de Diseño

- **Control**: Implementa control proporcional simple sin PID completo
- **Tolerancia**: Por defecto 0.2m - ajustar según precisión requerida
- **Frecuencia**: Control a 10Hz por defecto - aumentar para movimientos rápidos
- **Odometría**: Asume frame "odom" como referencia global

## Autor

Abdullah Nomeer (abdullahnomeer@gmail.com)

## Licencia

Apache 2.0
