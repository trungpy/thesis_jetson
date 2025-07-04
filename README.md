# ROS Advanced Video Processing & Autonomous Driving System

A comprehensive ROS-based system for real-time video processing, object detection, lane detection, and adaptive cruise control using TensorRT and OpenCV. This project implements advanced computer vision algorithms for autonomous driving applications.

## 🚀 Features

### Core Capabilities
- **Real-time Object Detection**: YOLOv12-based detection with TensorRT acceleration
- **Multi-Object Tracking**: ByteTrack algorithm for robust object tracking
- **Lane Detection**: Advanced lane detection with temporal smoothing
- **Speed Limit Recognition**: Automatic detection of traffic speed signs (30-80 km/h)
- **Adaptive Cruise Control**: Intelligent speed control based on detected objects and speed limits
- **Ego Vehicle Logic**: Sophisticated following behavior and collision avoidance

### Technical Features
- **ROS Integration**: Seamless integration with ROS ecosystem
- **GPU Acceleration**: TensorRT-powered inference for high performance
- **Modular Configuration**: JSON-based configuration system
- **Multi-Modal Input**: Support for video files, image folders, and ROS topics
- **Real-time Visualization**: Live HUD with speed, distance, and action information
- **Automatic Model Conversion**: ONNX-to-TensorRT engine conversion

## 📋 Requirements

### System Requirements
- **ROS**: Melodic or Noetic (tested and verified)
- **CUDA**: 11.0+ with compatible GPU
- **OpenCV**: 4.1+ with CUDA support
- **TensorRT**: 8.0+ for optimal performance

### Dependencies
```bash
# ROS packages
sudo apt install ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-sensor-msgs

# System dependencies
sudo apt install nlohmann-json3-dev
sudo apt install libeigen3-dev
sudo apt install libopencv-dev

# CUDA and TensorRT (install according to your system)
# Follow NVIDIA's official installation guide
```

## 🛠️ Installation & Build

### 1. Clone Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/trungpy/thesis_jetson thesis
```

### 2. Install Dependencies
```bash
# Update package list
sudo apt update

# Install ROS dependencies
sudo apt install ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-sensor-msgs

# Install system dependencies
sudo apt install nlohmann-json3-dev libeigen3-dev libopencv-dev

# Install CUDA and TensorRT (if not already installed)
# Follow NVIDIA's official documentation
```

### 3. Configure Build Paths
Edit `CMakeLists.txt` if necessary to specify paths for:
- OpenCV installation
- TensorRT installation
- CUDA toolkit

### 4. Build Project
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 🎯 Usage

### Quick Start
```bash
# Launch the complete system
roslaunch thesis thesis.launch
```

### Individual Components

#### 1. Video Publisher Node
Publishes video frames to ROS topic `/video/image`:
```bash
rosrun thesis video_publisher _video_path:=/path/to/video.mp4 _fps:=30.0
```

**Parameters:**
- `video_path`: Path to video file
- `fps`: Publishing frequency (default: 30.0)

#### 2. Video Subscriber Node
Processes video frames with full detection pipeline:
```bash
rosrun thesis video_subscriber _model_path:=/path/to/model.engine
```

**Parameters:**
- `model_path`: Path to TensorRT engine file (required)

### Configuration

The system uses `config.json` for centralized configuration:

#### Camera Settings
```json
{
  "camera": {
    "width": 1280,
    "height": 720,
    "fps": 30,
    "focalLength": 1778.0,
    "realObjectWidth": 0.7
  }
}
```

#### Detection Configuration
```json
{
  "detectConfig": {
    "confThreshold": 0.5,
    "nmsThreshold": 0.5
  }
}
```

#### Speed Control
```json
{
  "adaptiveSpeedControl": {
    "initialSpeedKph": 50.0,
    "cruiseSpeedKph": 70,
    "targetFollowingDistance": 25.0,
    "minFollowingDistance": 8.0,
    "criticalDistance": 4.0
  }
}
```

### Standalone Usage (Non-ROS)
```bash
# Video processing
./thesis --engine /path/to/model.engine --video /path/to/video.mp4

# Image folder processing
./thesis --engine /path/to/model.engine --images /path/to/image/folder
```

## 📁 Project Structure

```
thesis/
├── src/
│   ├── video_publisher.cpp      # ROS video publisher node
│   ├── video_subscriber.cpp     # Main processing node with detection/tracking
│   ├── Detect.cpp              # YOLOv12 detection implementation
│   ├── EgoVehicle.cpp          # Speed control and vehicle logic
│   ├── process.cpp             # Standalone processing utilities
│   ├── config.cpp              # Configuration loading
│   └── LaneDetector.cpp        # Lane detection implementation
├── include/
│   ├── Detect.h                # Detection class interface
│   ├── EgoVehicle.h            # Vehicle control interface
│   ├── process.h               # Processing utilities
│   ├── config.h                # Configuration structures
│   ├── LaneDetector.h          # Lane detection interface
│   └── app.h                   # CLI application interface
├── bytetrack/                  # ByteTrack tracking implementation
├── config.json                 # Main configuration file
├── thesis.launch              # ROS launch file
├── CMakeLists.txt             # Build configuration
└── package.xml                # ROS package definition
```

## 🔧 Configuration Details

### Supported Object Classes
The system detects and tracks 21 different object classes:
- **Vehicles**: car, motorcycle, bus, truck
- **Traffic Signs**: stop sign, speed limits (30-80 km/h)
- **Traffic Lights**: red, yellow, green
- **Pedestrians**: person, bicycle
- **Infrastructure**: crosswalk

### Speed Control Logic
The system implements sophisticated adaptive cruise control:

1. **Target Selection**: Automatically selects the most relevant vehicle to follow
2. **Lane Validation**: Ensures target vehicle is within detected lanes
3. **Distance Estimation**: Real-time distance calculation using camera calibration
4. **Speed Adjustment**: Smooth speed changes based on:
   - Distance to target vehicle
   - Target vehicle speed
   - Detected speed limits
   - Safety thresholds

### Lane Detection Features
- **Temporal Smoothing**: Reduces jitter using historical data
- **ROI Filtering**: Focuses on relevant road areas
- **Line Classification**: Separates left and right lane markers
- **Visualization**: Overlays detected lanes with transparency

## 🎮 ROS Topics

### Published Topics
- `/video/image` (sensor_msgs/Image): Raw video frames

### Subscribed Topics
- `/video/image` (sensor_msgs/Image): Video frames for processing

### Parameters
- `model_path`: TensorRT engine file path
- `video_path`: Input video file path
- `fps`: Publishing frequency

## 🚨 Troubleshooting

### Common Issues

1. **Model Loading Error**
   ```bash
   # Ensure TensorRT engine file exists and is accessible
   ls -la /path/to/model.engine
   ```

2. **CUDA Memory Issues**
   ```bash
   # Reduce batch size or input resolution in config.json
   # Monitor GPU memory usage
   nvidia-smi
   ```

3. **ROS Topic Issues**
   ```bash
   # Check if topics are being published
   rostopic list
   rostopic echo /video/image
   ```

4. **Build Errors**
   ```bash
   # Clean and rebuild
   cd ~/catkin_ws
   catkin_make clean
   catkin_make
   ```

### Performance Optimization
- Adjust `confThreshold` and `nmsThreshold` in `config.json`
- Modify camera resolution for speed vs. accuracy trade-off
- Use appropriate TensorRT precision (FP16/INT8) for your GPU

## 📊 Performance Metrics

Typical performance on NVIDIA RTX 3080:
- **Detection**: ~15ms per frame (640x640 input)
- **Tracking**: ~5ms per frame
- **Lane Detection**: ~10ms per frame
- **Total Pipeline**: ~30ms per frame (30+ FPS)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project includes third-party code with their respective licenses:
- **ByteTrack**: MIT License
- **TensorRT**: NVIDIA License
- **OpenCV**: BSD License

See individual license files in `bytetrack/include/logging.h` and `include/logging.h`.

## 📞 Support

For issues and questions:
- Check the troubleshooting section
- Review configuration options
- Examine source code comments
- Create an issue with detailed error information

---

**Note**: This system is designed for research and educational purposes. Always test thoroughly before deploying in real-world autonomous driving applications.