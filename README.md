# ROS2 MAVSDK Test Package

ROS2 integration with MAVSDK for PX4-Gazebo communication and mission execution. This package provides comprehensive test nodes that combine the reliability of MAVSDK mission management with ROS2 monitoring and future precision landing capabilities.

-----

## Package Overview

### Core Philosophy
- **MAVSDK**: Handles mission upload, execution, and core flight operations
- **ROS2**: Provides system monitoring, diagnostics, and prepares for precision landing integration
- **Hybrid Design**: Best of both worlds - proven mission reliability with flexible ROS2 ecosystem

### Test Nodes

#### 1. `mavsdk_mission_diagnostics` (Primary MAVSDK Integration)
Comprehensive MAVSDK mission execution with ROS2 monitoring integration.

#### 2. `connection_test` (System Health Check)
Tests PX4-ROS2 connectivity and validates communication channels.

#### 3. `px4_msg_monitor` (PX4 Message Analysis)
Monitors PX4 message flow with detailed flight status analysis.

#### 4. `gazebo_sensor_monitor` (Simulation Data Validation)
Validates Gazebo sensor data flow via ros_gz_bridge.

#### 5. `data_collector` (Data Logging)
Collects and logs system data for analysis and validation.

-----

## Quick Start

### Prerequisites

- Ubuntu 22.04.5 LTS (or Raspberry Pi 4B with Ubuntu 22.04 arm64)
- ROS2 Humble
- PX4-Autopilot with Gazebo simulation support
- px4_msgs package built and sourced
- MAVSDK v3.x installed

### 1. Install MAVSDK

```bash
# Download and install MAVSDK for Ubuntu 22.04
wget https://github.com/mavlink/MAVSDK/releases/download/v3.10.0/libmavsdk-dev_3.10.0_ubuntu22.04_amd64.deb
sudo dpkg -i libmavsdk-dev_3.10.0_ubuntu22.04_amd64.deb
sudo apt-get install -f

# Verify installation
pkg-config --libs mavsdk
```

### 2. Build the Package

```bash
# In your ROS2 workspace
cd ~/ros2_ws/src
git clone <your-repo-url> ros2_mavsdk_test
cd ~/ros2_ws

# Build with MAVSDK support
colcon build --packages-select ros2_mavsdk_test
source install/setup.bash
```

### 3. Mission Testing Sequence

**Terminal 1 - DDS Agent (Start FIRST):**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 - PX4 SITL with Gazebo:**
```bash
cd ~/Documents/PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down_aruco

# Wait for: INFO  [uxrce_dds_client] session established
```

**Terminal 3 - Gazebo Bridge (if needed for sensor monitoring):**
```bash
cd ~/ros2_ws/src/ros2_mavsdk_test
./scripts/start_bridge.sh
```

**Terminal 4 - Run MAVSDK Mission Test:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ros2_mavsdk_test mavsdk_mission_diagnostics
```

**Terminal 5 - Additional Monitoring (Optional):**
```bash
# Monitor ROS2 connectivity
ros2 run ros2_mavsdk_test connection_test

# Monitor PX4 messages
ros2 run ros2_mavsdk_test px4_msg_monitor

# Monitor Gazebo sensors
ros2 run ros2_mavsdk_test gazebo_sensor_monitor
```

-----

## Mission Profile

The `mavsdk_mission_diagnostics` node executes a comprehensive test mission:

### **Mission Waypoints**
1. **Takeoff**: Automated takeoff to 10m altitude
2. **Waypoint 1**: Navigate to `(26.755171568042137, -80.24434557941278)`
3. **Loiter**: Hold position for 10 seconds (precision positioning test)
4. **Waypoint 2**: Navigate to `(26.755281141397138, -80.24422286838895)`
5. **Landing**: Automated landing at final waypoint

### **Expected Behavior**
```
=== MAVSDK Mission Diagnostics Node ===
🔌 Initializing MAVSDK connection...
✓ MAVSDK connection established to udp://127.0.0.1:14540
✓ Autopilot system discovered
✅ System ready! Starting mission upload...
🗺️ Creating mission plan...
📤 Uploading mission (3 waypoints)...
✅ Mission uploaded successfully!
🔫 Arming vehicle...
✅ Vehicle armed!
🚀 Starting mission...
📍 Mission progress: 1 / 3 waypoints
⏳ Starting 10-second loiter at waypoint 1...
🔄 Loitering... 5/10 seconds
✅ Loiter complete, continuing mission...
📍 Mission progress: 3 / 3 waypoints
🏁 Mission completed! Initiating landing...
🛬 Landing initiated
```

-----

## Architecture

### **System Integration**
```
┌─────────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│   ROS2 Monitoring   │    │   Micro-XRCE     │    │       PX4        │
│                     │    │   DDS Agent      │    │     Autopilot    │
│  - Vehicle Status   │◄──►│   Port 8888      │◄──►│                  │
│  - Position Data    │    │                  │    │  Mission Control │
│  - System Health    │    │                  │    │  Flight Stack    │
└─────────────────────┘    └──────────────────┘    └──────────────────┘
           │                                                 ▲
           │ ROS2 Topics                                     │ MAVLink
           │ (Status/Monitor)                                │ (Commands)
           ▼                                                 │
┌─────────────────────┐    ┌──────────────────┐            │
│   Future Features   │    │     MAVSDK       │────────────┘
│                     │    │   UDP:14540      │
│  - ArUco Landing    │    │                  │
│  - Web Interface    │    │  Mission Upload  │
│  - Path Planning    │    │  Flight Control  │
└─────────────────────┘    └──────────────────┘
```

### **Message Flow**
- **MAVSDK → PX4**: Mission upload, arming, takeoff, landing commands
- **PX4 → ROS2**: Vehicle status, position, mission progress (via DDS)
- **ROS2 → Future**: Precision landing corrections, web interface updates

-----

## Key Features

### ✅ **MAVSDK Integration**
- Robust mission upload and execution
- Comprehensive error handling and retry logic
- Real-time mission progress monitoring
- Automatic takeoff and landing sequences

### ✅ **ROS2 Compatibility**
- PX4-compatible QoS settings (`rmw_qos_profile_sensor_data`)
- Real-time vehicle status monitoring
- Position and navigation state tracking
- Mission result validation

### ✅ **Hybrid Architecture Benefits**
- **Reliability**: MAVSDK's proven mission management
- **Flexibility**: ROS2's rich ecosystem for future enhancements
- **Monitoring**: Comprehensive system diagnostics
- **Scalability**: Ready for precision landing and web integration

### ✅ **Production Ready**
- Thread-safe implementation
- Comprehensive error handling and logging
- Detailed status reporting and diagnostics
- Cross-platform compatibility (Ubuntu x86_64, Raspberry Pi arm64)

-----

## Advanced Usage

### **Custom Mission Coordinates**
Modify mission waypoints in `src/mavsdk_mission_diagnostics.cpp`:

```cpp
// Update these coordinates for your test area
mission_items.push_back(make_mission_item(
    YOUR_LATITUDE,          // Replace with your coordinates
    YOUR_LONGITUDE,         // Replace with your coordinates
    10.0f,                  // Altitude (meters AGL)
    5.0f,                   // Speed (m/s)
    false,                  // Hover at waypoint
    0.0f, 0.0f,            // Gimbal control
    Mission::MissionItem::CameraAction::None
));
```

### **Integration with Web Interface**
Future enhancement: ROS2 services for dynamic mission updates:

```cpp
// Planned ROS2 service interfaces
/ros2_mavsdk_test/upload_mission      # Upload new mission from web
/ros2_mavsdk_test/update_waypoint     # Update specific waypoint
/ros2_mavsdk_test/mission_status      # Get current mission status
```

### **ArUco Precision Landing**
Prepared integration points for precision landing:

```cpp
// Mission completion triggers precision landing mode
if (mission_completed && aruco_detected) {
    // Switch to ROS2-controlled precision landing
    trigger_precision_landing();
}
```

-----

## Troubleshooting

### **MAVSDK Connection Issues**
```bash
# Check PX4 SITL is running and accepting connections
netstat -an | grep 14540

# Verify MAVLink traffic
sudo tcpdump -i lo port 14540

# Check DDS agent connection
ps aux | grep MicroXRCEAgent
```

### **Build Issues**
```bash
# Verify MAVSDK installation
pkg-config --libs mavsdk
pkg-config --cflags mavsdk

# Clean build if needed
cd ~/ros2_ws
rm -rf build/ros2_mavsdk_test install/ros2_mavsdk_test
colcon build --packages-select ros2_mavsdk_test
```

### **Mission Upload Failures**
- Ensure vehicle is ready: Health checks must pass
- Check GPS fix: Vehicle needs valid position estimate
- Verify mission validity: Waypoints must be reasonable
- Monitor PX4 console: Look for mission-related errors

-----

## Development Roadmap

### **Phase 1: Core Integration** ✅
- [x] MAVSDK mission upload and execution
- [x] ROS2 monitoring integration
- [x] Basic waypoint navigation
- [x] Automated takeoff and landing

### **Phase 2: Web Interface** (Planned)
- [ ] ROS2 service interfaces for mission control
- [ ] Real-time mission status publishing
- [ ] Dynamic waypoint updates
- [ ] Mission validation and safety checks

### **Phase 3: Precision Landing** (Planned)
- [ ] ArUco marker detection integration
- [ ] ROS2-controlled precision landing
- [ ] Smooth handoff from MAVSDK to ROS2 control
- [ ] Landing accuracy validation

### **Phase 4: Advanced Features** (Future)
- [ ] Multi-vehicle coordination
- [ ] Complex mission patterns
- [ ] Obstacle avoidance integration
- [ ] Real-time path replanning

-----

## Files Structure

```
ros2_mavsdk_test/
├── CMakeLists.txt              # Build configuration with MAVSDK
├── package.xml                 # Package metadata
├── README.md                   # This comprehensive guide
├── scripts/
│   └── start_bridge.sh        # Gazebo bridge helper
├── src/
│   ├── mavsdk_mission_diagnostics.cpp  # Main MAVSDK integration node
│   ├── connection_test.cpp             # ROS2-PX4 connectivity test
│   ├── px4_msg_monitor.cpp            # PX4 message monitoring
│   ├── gazebo_sensor_monitor.cpp      # Gazebo sensor validation
│   └── data_collector.cpp             # Data logging node
└── .vscode/
    ├── c_cpp_properties.json   # VSCode C/C++ configuration
    └── launch.json             # VSCode debugging configuration
```

-----

## Support and Documentation

- **MAVSDK Documentation**: https://mavsdk.mavlink.io/
- **PX4 Documentation**: https://docs.px4.io/
- **ROS2 Documentation**: https://docs.ros.org/

This package demonstrates the powerful combination of MAVSDK's mission reliability with ROS2's flexibility, providing a solid foundation for advanced autonomous flight applications.

**Ready for Mission Testing! 🚁**