# ROS Advanced Video Processing & Autonomous Driving System

A comprehensive ROS-based system for real-time video processing, object detection, lane detection, and adaptive cruise control using TensorRT and OpenCV. This project implements advanced computer vision algorithms for autonomous driving applications with sophisticated speed control and safety features.

## 🚀 Features

### Core Capabilities
- **Real-time Object Detection**: YOLOv11-based detection with TensorRT acceleration
- **Multi-Object Tracking**: ByteTrack algorithm for robust object tracking
- **Lane Detection**: Advanced lane detection with temporal smoothing and ROI filtering
- **Speed Limit Recognition**: Automatic detection of traffic speed signs (30-80 km/h)
- **Adaptive Cruise Control**: Intelligent speed control based on detected objects and speed limits
- **Ego Vehicle Logic**: Sophisticated following behavior and collision avoidance
- **Enhanced Safety Features**: Emergency braking, distance monitoring, and speed limit compliance

### Technical Features
- **ROS Integration**: Seamless integration with ROS ecosystem
- **GPU Acceleration**: TensorRT-powered inference for high performance
- **Modular Configuration**: JSON-based configuration system
- **Multi-Modal Input**: Support for video files, image folders, and ROS topics
- **Real-time Visualization**: Live HUD with speed, distance, and action information
- **Automatic Model Conversion**: ONNX-to-TensorRT engine conversion
- **Data Logging**: Comprehensive logging and recording capabilities
- **Enhanced Video Processing**: Advanced video subscriber with safety monitoring

## 📋 Requirements

### System Requirements
- **ROS**: Melodic or Noetic (tested and verified)
- **CUDA**: 11.0+ with compatible GPU
- **OpenCV**: 4.1+ with CUDA support
- **TensorRT**: 8.0+ for optimal performance
- **Eigen3**: For mathematical computations
- **nlohmann-json**: For JSON configuration parsing

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

#### 3. Enhanced Video Subscriber
Advanced processing with safety features and data logging:
```bash
rosrun thesis video_subscriber_capture _model_path:=/path/to/model.engine
```

#### 4. Echo Nodes (for monitoring)
```bash
# Monitor ego vehicle speed
rosrun thesis echo_ego_speed

# Monitor driving actions
rosrun thesis echo_driving_action

# Monitor throttle commands
rosrun thesis echo_throttle

# Monitor brake commands
rosrun thesis echo_brake
```

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
    "initialSpeedKph": 30.0,
    "cruiseSpeedKph": 70,
    "targetFollowingDistance": 30.0,
    "minFollowingDistance": 8.0,
    "criticalDistance": 4.0
  }
}
```

#### Speed Adjustment Parameters
```json
{
  "speedAdjustment": {
    "speedUpdateInterval": 0.02,
    "maxAcceleration": 3.0,
    "maxDeceleration": -5.0,
    "comfortDeceleration": -2.0,
    "maxJerk": 2.0,
    "vehicleMass": 1500.0
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
│   ├── main.cpp                           # Main application entry point
│   ├── video_publisher.cpp                # ROS video publisher node
│   ├── video_subscriber.cpp               # Basic video processing node
│   ├── video_subscriber_capture.cpp       # Enhanced video processing with logging
│   ├── Detect.cpp                         # YOLOv12 detection implementation
│   ├── EgoVehicle.cpp                     # Speed control and vehicle logic
│   ├── LaneDetector.cpp                   # Lane detection implementation
│   ├── FrontDistanceEstimator.cpp         # Distance estimation utilities
│   ├── config.cpp                         # Configuration loading
│   ├── utils.cpp                          # Utility functions
│   ├── preprocess.cu                      # CUDA preprocessing kernels
│   ├── echo_ego_speed.cpp                 # Speed monitoring node
│   ├── echo_driving_action.cpp            # Action monitoring node
│   ├── echo_throttle.cpp                  # Throttle monitoring node
│   ├── echo_brake.cpp                     # Brake monitoring node
│   └── modules/
│       ├── EnhancedVideoSubscriber.cpp    # Advanced video processing
│       ├── HUDRenderer.cpp                # HUD rendering module
│       ├── VideoRecorder.cpp              # Video recording module
│       └── DataLogger.cpp                 # Data logging module
├── include/
│   ├── Detect.h                           # Detection class interface
│   ├── EgoVehicle.h                       # Vehicle control interface
│   ├── LaneDetector.h                     # Lane detection interface
│   ├── FrontDistanceEstimator.h           # Distance estimation interface
│   ├── config.h                           # Configuration structures
│   ├── cuda_utils.h                       # CUDA utility functions
│   ├── cxxopts.hpp                        # Command line options
│   ├── macros.h                           # Common macros
│   ├── utils.hpp                          # Utility functions
│   └── modules/
│       ├── DataLogger.h                   # Data logging interface
│       ├── EnhancedVideoSubscriber.h      # Enhanced subscriber interface
│       ├── HUDRenderer.h                  # HUD rendering interface
│       └── VideoRecorder.h                # Video recording interface
├── bytetrack/                             # ByteTrack tracking implementation
│   ├── include/
│   │   ├── BYTETracker.h                 # ByteTrack main interface
│   │   ├── STrack.h                      # Track state management
│   │   ├── kalmanFilter.h                # Kalman filter implementation
│   │   └── dataType.h                    # Data type definitions
│   └── src/
│       ├── BYTETracker.cpp               # ByteTrack implementation
│       ├── STrack.cpp                    # Track state implementation
│       └── kalmanFilter.cpp              # Kalman filter implementation
├── cmake/                                 # CMake configuration files
├── config.json                           # Main configuration file
├── thesis.launch                         # Main ROS launch file
├── terminal.launch                       # Monitoring nodes launch file
├── CMakeLists.txt                        # Build configuration
└── package.xml                           # ROS package definition
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
The system implements sophisticated adaptive cruise control with acceleration-based control:

1. **Target Selection**: Automatically selects the most relevant vehicle to follow
2. **Lane Validation**: Ensures target vehicle is within detected lanes
3. **Distance Estimation**: Real-time distance calculation using camera calibration
4. **Acceleration Control**: Smooth acceleration changes based on:
   - Distance to target vehicle
   - Target vehicle speed
   - Detected speed limits
   - Safety thresholds
   - Vehicle mass and physics constraints

### Driving States
The system implements five distinct driving states:
- **Emergency Brake**: Maximum deceleration for critical situations
- **Close Follow**: Aggressive deceleration when too close
- **Slow Traffic**: Moderate deceleration for traffic conditions
- **Normal Follow**: Proportional control for following behavior
- **Free Drive**: Cruise control towards target speed

### Lane Detection Features
- **Temporal Smoothing**: Reduces jitter using historical data
- **ROI Filtering**: Focuses on relevant road areas
- **Line Classification**: Separates left and right lane markers
- **Visualization**: Overlays detected lanes with transparency
- **Confidence Tracking**: Maintains confidence scores for lane detection

## 🎮 ROS Topics

### Published Topics
- `/video/image` (sensor_msgs/Image): Raw video frames
- `/ego_speed` (std_msgs/Float32): Current ego vehicle speed
- `/driving_action` (std_msgs/String): Current driving action
- `/control/throttle` (std_msgs/Float32): Throttle command (0.0-1.0)
- `/control/brake` (std_msgs/Float32): Brake command (0.0-1.0)

### Subscribed Topics
- `/video/image` (sensor_msgs/Image): Video frames for processing

### Parameters
- `model_path`: TensorRT engine file path
- `video_path`: Input video file path
- `fps`: Publishing frequency
- `output_dir`: Directory for logging output
- `enable_debug_output`: Enable debug output (default: true)
- `enable_data_logging`: Enable data logging (default: true)

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

5. **Configuration Issues**
   ```bash
   # Verify config.json syntax
   cat config.json | python3 -m json.tool
   ```

### Performance Optimization
- Adjust `confThreshold` and `nmsThreshold` in `config.json`
- Modify camera resolution for speed vs. accuracy trade-off
- Use appropriate TensorRT precision (FP16/INT8) for your GPU
- Tune acceleration parameters in `speedAdjustment` section

## 📊 Performance Metrics

Typical performance on NVIDIA GTX 1650:
- **Detection**: ~15ms per frame (640x640 input)
- **Tracking**: ~5ms per frame
- **Lane Detection**: ~10ms per frame
- **Total Pipeline**: ~30ms per frame (30+ FPS)

## 🔒 Safety Features

### Emergency Systems
- **Detection Timeout**: Emergency stop if no detections for extended period
- **Speed Limit Compliance**: Automatic speed reduction when limits detected
- **Critical Distance Monitoring**: Emergency braking for close objects
- **Lane Departure Warning**: Monitoring for lane boundary violations

### Data Logging
- **Video Recording**: Automatic recording of processed video
- **Performance Metrics**: Logging of processing times and detection counts
- **Safety Events**: Recording of emergency stops and warnings
- **Configuration Tracking**: Logging of all configuration parameters

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
- **Eigen3**: MPL2 License
- **nlohmann-json**: MIT License

See individual license files in `bytetrack/include/logging.h` and `include/logging.h`.

## 📞 Support

For issues and questions:
- Check the troubleshooting section
- Review configuration options
- Examine source code comments
- Create an issue with detailed error information

---

**Note**: This system is designed for research and educational purposes. Always test thoroughly before deploying in real-world autonomous driving applications. The system includes advanced safety features but should not be used as the sole control system for actual vehicles without proper validation and certification.