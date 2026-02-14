# Monocular Depth Estimation with ONNX Runtime

**Part B**: AI-powered monocular depth estimation deployed using ONNX Runtime with MiDaS model integration in ROS 2.

## Overview

This package implements a complete pipeline for monocular depth estimation using ONNX Runtime:

1. **RGB Image Source** - From video, image folder, or webcam
2. **ONNX Inference Engine** - Fast depth estimation with MiDaS
3. **Depth Metrics** - Environment understanding metrics
4. **Safety Integration** - Optional autonomous navigation safety layer

## B1: Monocular Depth Model

### Model: MiDaS v2.1 Small (Intel Labs)

**Official Repository**: https://github.com/isl-org/MiDaS

**Selected Model**:
- `midas_v21_small`: 256×256px, rápido, pequeño (~50MB), recomendado para ROS 2 y Gazebo

### Model Download and Conversion

#### Quick Start - Automatic Download

```bash
# Install dependencies
pip3 install torch timm opencv-python onnx onnxruntime onnxscript

# IMPORTANT: Downgrade NumPy for ROS 2 Humble compatibility
pip install 'numpy<2'

# Download and convert model
cd ~/ros2_ws/src/nomeer_robot_ros2/src/mono_depth_onnx
python3 scripts/download_midas_model.py
```

**Note on NumPy**: ROS 2 Humble uses cv_bridge compiled with NumPy 1.x. 
Using `numpy<2` avoids `AttributeError: _ARRAY_API not found`.

### Model Information

| Metric | MiDaS v2.1 Small |
|--------|------------------|
| Input Size | 256×256 |
| Model Size | ~50MB |
| Inference Time* | ~30ms |
| Accuracy | Good |
| GPU Memory | Low |

*Approximate on NVIDIA GPU. CPU times ~3-5x slower.

## B2: Image Source Configuration

The package supports multiple image sources:

### Option A: External ROS 2 Node (Your RGB Publisher)

**Most flexible**: Connect to any external node that publishes RGB images to `/rgb_image`.

The `depth_inference_node.py` **automatically subscribes** to `/rgb_image` and processes each frame.

**Setup**:
1. Your node publishes `sensor_msgs/Image` to `ros2 topic pub /rgb_image`
2. Launch depth inference: `ros2 run mono_depth_onnx depth_inference_node.py`
3. Results appear on `/camera/depth_estimated` and `/camera/depth_colored`

**See**: [INTEGRATION.md](INTEGRATION.md) for step-by-step instructions.

### Option B: Image Folder (For Testing Without External Node)

```yaml
image_source:
  source_type: "folder"
  source_path: "data/images"
```

Place RGB images in `data/images/` directory. Supports:
- PNG, JPG, JPEG, BMP, TIFF formats
- Images processed in sorted order
- Loops when `loop_video: true`

### Option B: Video File

```yaml
image_source:
  source_type: "video"
  source_path: "data/video.mp4"
```

### Option C: Webcam

```yaml
image_source:
  source_type: "webcam"
  source_path: "0"  # Device ID
```

### Option D: Gazebo Camera (Optional)

Can be integrated with Gazebo camera topic with additional bridge node.

## B3: Depth Inference Pipeline

### Inference Pipeline

```
Input Image (640×480 or other)
        ↓
[Preprocessing]
  - Resize to model input (256×256 or 384×384)
  - Normalize (ImageNet stats)
  - Convert to CHW format
        ↓
[ONNX Runtime Inference]
  - Forward pass through MiDaS
        ↓
[Postprocessing]
  - Normalize output to 0-1 range
  - Resize to original image size
  - Convert to 16-bit depth image
        ↓
Output Depth Map
```

### Published Topics

- `/camera/depth_estimated` (Image/mono16): Quantized depth [0, 65535]
- `/camera/depth_colored` (Image/BGR): Colored visualization with colormap
- **Depth Units**: Relative (normalized 0-1), where:
  - 0.0 = Farthest
  - 1.0 = Closest
  - Interpretation: Inverse depth (higher = closer object)

### Performance

- **Latency**: ~30-120ms per frame (depending on model)
- **FPS**: 8-33 fps (depending on model and hardware)
- **GPU Acceleration**: Optional (set `enable_gpu: true` if CUDA available)

## B4: Depth-Based Metrics

### Metrics Calculated

1. **Minimum Frontal Depth**
   - Region: Top 30% of image, center 50% of width
   - Computation: Minimum depth value after outlier filtering
   - Use: Obstacle detection, safety

2. **Mean Depth**
   - Region: Same ROI as minimum
   - Computation: Average depth in ROI
   - Use: Environmental profiling

3. **Obstacle Detection**
   - Triggered when obstacle ratio > threshold (default 10%)
   - Used for safety decisions

### Outlier Filtering

Three filtering methods available:

- **Median**: Standard median filter (robust, smooth)
- **Percentile**: 5-95 percentile clipping + median
- **IQR**: Interquartile range filtering

Configuration:

```yaml
depth_metric:
  outlier_filter_type: "median"  # "median", "percentile", or "iqr"
  outlier_filter_size: 5          # Filter kernel size
```

### Published Metrics

- `/depth_metric/min_frontal_depth` (Float32): Minimum depth [0.0-1.0]
- `/depth_metric/mean_depth` (Float32): Mean depth [0.0-1.0]
- `/depth_metric/obstacle_detected` (Bool): Obstacle present [true/false]
- `/depth_metric/roi_visualization` (Image): ROI with overlay

## B5: Depth Visualization

### Visualization Topics

- `/camera/depth_colored` (BGR Image): Turbo colormap applied
- `/depth_metric/roi_visualization` (BGR Image): ROI with metrics overlay

### Colormaps Available

- **TURBO**: Default (blue→green→red progression)
- Can be changed in `depth_inference_node.py`

### Visualization Metrics Overlay

Shows:
- Minimum frontal depth value
- Mean depth value
- ROI boundary rectangle
- Frame count and FPS

## Optional: Safety Integration

### Autonomous Depth Safety Node

Integrates depth metrics with the autonomous patrol system:

```yaml
safety:
  enable_safety: true
  min_safe_depth: 0.2
  emergency_stop_depth: 0.1
  reduce_speed_depth: 0.3
  speed_reduction_factor: 0.5
```

### Safety Rules

1. **Emergency Stop**: If `min_depth < emergency_stop_depth` → Stop robot
2. **Reduced Speed**: If `min_depth < reduce_speed_depth` → Multiply velocity by factor
3. **Nominal**: Otherwise → Pass through velocity unchanged

### Topics for Safety

- Input: `/cmd_vel` (from autonomous patrol)
- Output: `/cmd_vel_safe` (to robot controller)
- Status: `/safety/status` (String with current state)
- Events: `results/safety_events.json` (logged events)

## Installation & Compilation

### Dependencies

```bash
# System packages
sudo apt install python3-pip python3-opencv python3-scipy

# Python packages
pip3 install onnxruntime opencv-python numpy scipy pillow

# For model conversion (optional):
pip3 install torch torchvision timm
```

### Build Package

```bash
cd ~/ros2_ws
colcon build --packages-select mono_depth_onnx
source install/setup.bash
```

## Usage Examples

### Example 1: Depth Inference Only

```bash
# Terminal 1: Publish images from folder
ros2 launch mono_depth_onnx inference.launch.py

# Terminal 2: View depth metrics
ros2 topic echo /depth_metric/min_frontal_depth
```

### Example 2: Full Pipeline with Visualization

```bash
ros2 launch mono_depth_onnx full_pipeline.launch.py
```

### Example 3: With Autonomous Navigation Safety

```bash
# Terminal 1: Autonomous patrol
ros2 launch autonomous_patrol follow_waypoints.launch.py

# Terminal 2: Depth safety layer
ros2 launch mono_depth_onnx with_autonomy.launch.py
```

## Configuration

### Main Config File

`config/mono_depth_config.yaml`:
- Image source settings
- Model selection
- Metric parameters
- Safety thresholds
- Output directories

### Parameter Tuning

**For Real-Time Performance:**
```yaml
depth_estimator:
  model_name: "midas_v3_small"  # Smallest model
  enable_gpu: true               # GPU acceleration
image_source:
  publish_frequency: 30.0        # Higher FPS
```

**For Accuracy:**
```yaml
depth_estimator:
  model_name: "dpt_large"        # Largest model
depth_metric:
  outlier_filter_type: "iqr"     # Robust filtering
  roi_height_ratio: 0.5          # Larger ROI
```

## Output and Results

### Metrics File: `results/depth_metrics.json`

```json
{
  "timestamp": "2026-02-12T...",
  "total_frames": 500,
  "min_depth_stats": {
    "current": 0.245,
    "mean": 0.312,
    "max": 0.578,
    "min": 0.102
  },
  "mean_depth_stats": {...},
  "obstacle_detections": 23,
  "obstacle_ratio": 0.046
}
```

### Safety Events: `results/safety_events.json` (if safety enabled)

```json
[
  {
    "timestamp": "2026-02-12T...",
    "event_type": "EMERGENCY_STOP",
    "min_depth": 0.085,
    "cmd_vel": {
      "linear_x": 0.5,
      "angular_z": 0.2
    }
  }
]
```

## Testing and Validation

### Test with Example Images

```bash
# Generate test folder structure
mkdir -p mono_depth_onnx/data/images

# Add some test images (3+ images recommended)
cp /path/to/images/*.jpg mono_depth_onnx/data/images/

ros2 launch mono_depth_onnx full_pipeline.launch.py
```

### Check Inference Performance

```bash
ros2 topic hz /camera/depth_estimated
# Should show ~10-30 fps depending on model
```

### Validate Output

```bash
ros2 topic echo /depth_metric/min_frontal_depth
# Should see Float32 values between 0.0 and 1.0
```

## Troubleshooting

### Problem: "Model file not found"
```
Solution:
1. Run: python3 scripts/download_midas_model.py
2. Check models/ directory exists
3. Update model_path in config file
```

### Problem: "ONNX Runtime not found"
```
Solution: pip3 install onnxruntime
```

### Problem: "No images found in folder"
```
Solution:
1. mkdir -p data/images
2. Add .jpg or .png files to data/images/
3. Use absolute paths if relative paths fail
```

### Problem: Low FPS (slow inference)
```
Solution:
1. Use smaller model: midas_v3_small
2. Enable GPU: enable_gpu: true
3. Reduce input size: input_height/width
4. Lower publish frequency
```

## Architecture

```
mono_depth_onnx/
├── mono_depth_onnx/
│   ├── image_source_node.py         # B2: Image source
│   ├── depth_inference_node.py      # B3: ONNX inference
│   ├── depth_metric_node.py         # B4: Metrics
│   ├── depth_visualizer_node.py     # B5: Visualization
│   └── autonomous_depth_safety_node.py  # Optional
├── scripts/
│   └── download_midas_model.py      # B1: Model download
├── models/                          # B1: Converted models stored here
├── config/
│   ├── mono_depth_config.yaml       # Main config
│   └── test_config.yaml             # Test config
├── launch/
│   ├── inference.launch.py          # Minimal pipeline
│   ├── full_pipeline.launch.py      # Full with viz
│   └── with_autonomy.launch.py      # With safety
├── data/                            # Input images
├── results/                         # Output metrics
└── README.md                        # This file
```

## ROS 2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/camera/image_raw` | Image | Publisher | Input RGB image |
| `/camera/depth_estimated` | Image/mono16 | Publisher | Depth map output |
| `/camera/depth_colored` | Image/BGR | Publisher | Colored visualization |
| `/depth_metric/min_frontal_depth` | Float32 | Publisher | Minimum depth metric |
| `/depth_metric/mean_depth` | Float32 | Publisher | Mean depth metric |
| `/depth_metric/obstacle_detected` | Bool | Publisher | Obstacle status |
| `/depth_metric/roi_visualization` | Image | Publisher | ROI with overlay |
| `/cmd_vel` | Twist | Subscriber | Input velocity (safety) |
| `/cmd_vel_safe` | Twist | Publisher | Safe velocity output |
| `/safety/status` | String | Publisher | Safety status |

## Performance Benchmarks

Tested on:
- CPU: Intel i7-7700K
- GPU: NVIDIA RTX 2060
- RAM: 16GB

| Model | CPU Time | GPU Time | CPU FPS | GPU FPS |
|-------|----------|----------|---------|---------|
| MiDaS v2.1 Small | 95ms | 30ms | 10.5 | 33 |

## References

- **MiDaS Paper**: https://arxiv.org/abs/2105.02542
- **MiDaS GitHub**: https://github.com/isl-org/MiDaS
- **ONNX Runtime**: https://onnxruntime.ai/
- **ROS 2 Documentation**: https://docs.ros.org/

## License

Apache License 2.0

## Author

Abdullah Nomeer (abdullahnomeer@gmail.com)

## TODO / Future Work

- [ ] Multi-model inference support
- [ ] Real-time GPU batch processing
- [ ] ROS 2 Action interface for depth requests
- [ ] Integration with Nav2 costmaps
- [ ] Depth-based point cloud generation
- [ ] Advanced filtering (Kalman, bilateral)
- [ ] Confidence map output
