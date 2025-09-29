# Emergency Urban Search and Rescue Robot

A comprehensive robotic system designed to aid in urban search and rescue operations, featuring autonomous navigation, real-time sensor monitoring, and remote control capabilities.

---

## 🚀 Project Overview

This project implements a fully functional search and rescue robot capable of:
- Autonomous terrain navigation
- Real-time video streaming with object detection
- Two-way audio communication
- Environmental sensor monitoring (temperature, humidity, distance)
- LIDAR-based mapping and obstacle avoidance
- Web-based remote control interface

---

## 🏗️ System Architecture

### Hardware Components
- **Main Controller**: Raspberry Pi 4 (8GB)
- **Sensors**: 
  - RPLIDAR A3 for environmental mapping
  - HC-SR04 Ultrasonic Distance Sensors (front, left, right, back)
  - DHT11 Temperature/Humidity Sensor
  - 10 DOF Module (gyroscope, accelerometer, magnetometer, pressure)
  - Raspberry Pi Camera Module
- **Chassis**: Hiwonder Tank Car Chassis with DC brushless motors
- **Motor Control**: Arduino Uno with dual H-bridge
- **Power**: Dual battery system (5V for Pi, 14.8V for motors)

### Software Stack
- **Robot OS**: ROS2 (Robot Operating System 2)
- **Frontend**: React with Bootstrap 5
- **Backend**: Node.js with Express
- **Database**: AWS DynamoDB & S3
- **Computer Vision**: OpenCV with Flask
- **Authentication**: AWS Cognito
- **Cloud Services**: AWS Amplify

---

## 🛠️ Installation & Setup

### Prerequisites
- ROS2 Humble Hawksbill
- Node.js v16.13.0
- Python 3.8+
- AWS Account (for cloud services)

### 1. ROS Workspace Setup
cd ros_workspace/
colcon build --symlink-install
source install/setup.bash
sudo chmod 777 /dev/ttyACM0 /dev/ttyUSB0 /dev/mem /dev/gpiomem

### 2. ROS Server Setup
cd ros_server/
nvm use v16.13.0
npm install
rclnodejs-cli generate-ros-messages

### 3. Frontend Setup
cd client/
npm install
npm run dev

### 4. OpenCV Flask Application
cd OpenCV/
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install Flask opencv-python waitress
waitress-serve --host=127.0.0.1 --port=5000 PersonDetection:app

---

🎮 Usage
Starting the Complete System
### 1. Start ROS2 Core:
cd ros_workspace/
source install/setup.bash
ros2 launch robot_package all.launch.py

### 2. Start ROS Server:
cd ros_server/
ros2 launch ros_server example.launch.py

### 3. Start Video Stream:
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" -p camera_frame_id:=camera_optical_link

### 4. Start Frontend:
cd client/
npm run dev

### 5. Access Web Interface: Open http://localhost:3000

---

## Key Features Access
- Dashboard: Real-time robot control and video feed
- Manual Control: WASD keys for movement
- Sensor Monitoring: Live data from all environmental sensors
- LIDAR Mapping: Real-time point cloud visualization
- Audio Communication: Two-way audio with survivors
- Autonomous Mode: Obstacle avoidance and navigation

---

## 🔧 Configuration
### ROS2 Topics
- /motor_commands - Movement control
- /sensor_data - Environmental sensor readings
- /video_feed - Camera stream
- /lidar_scan - LIDAR mapping data
- /audio_stream - Two-way audio communication

### Environment Variables
- Create a .env file in each service directory with appropriate AWS credentials and configuration.

---

## 🧪 Testing
The project includes comprehensive test suites:

#### Hardware testing
python tests/hardware_validation.py

#### ROS2 node testing
ros2 test robot_package

#### Frontend testing
cd client/ && npm test

#### Integration testing
python tests/system_integration.py

---

## 📊 Features Demonstrated
- ✅ Real-time video streaming with OpenCV
- ✅ ROS2-based motor control and sensor integration
- ✅ React web interface with live telemetry
- ✅ LIDAR-based environmental mapping
- ✅ Object detection using HOG algorithm
- ✅ Two-way audio communication
- ✅ AWS cloud integration for data storage
- ✅ User authentication and session management
- ✅ Autonomous obstacle avoidance
- ✅ Multi-sensor data fusion
