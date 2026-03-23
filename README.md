# ROS 2 Roomba Robot Workspace

Welcome to the ROS 2 workspace for the Roomba robot! This workspace is configured to run on a Raspberry Pi 5 running Ubuntu 24.04 and ROS 2 Jazzy. The packages within handle low-level control of the Roomba via its serial interface, stream telemetry data, process LiDAR scans, and provide a live webcam feed.

The ultimate goal of this setup is to support SLAM (Simultaneous Localization and Mapping) with the Roomba and an RPLidar C1 sensor.

---

## 📦 Packages & Nodes

### 1. `roomba`
The core package that handles all interaction with the robot's hardware.
- **`roomba_node`**: This is the main robot driver.
  - **Subscribes to:** `/cmd_vel` (`geometry_msgs/msg/Twist`) — Receives movement commands and converts them to left/right wheel velocities.
  - **Publishes to:** `/roomba/sensors` (`std_msgs/msg/String`) — Continuously publishes the latest sensor stream (Voltage, current, temperature, bumps, wheel drops) from the Roomba over an encoded JSON string.
  - **Hardware Interface:** Uses pyserial to talk to the Roomba over `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03F056-if00-port0`.
- **`roomba_teleop`**: A decoupled teleoperation node for driving the robot using a keyboard.
  - **Publishes to:** `/cmd_vel` — Sends Twist messages securely based on user input.
  - **Subscribes to:** `/roomba/sensors` — Decodes the telemetry string to display the live voltage and bump sensor status in your terminal while you drive.
- **`webcam.launch.py`**: A launch script included in the `roomba` package to bring up the `v4l2_camera` node and publish `/image_raw`.

### 2. `rplidar_ros`
The package handling the RPLidar C1 sensor.
- **`rplidar_c1_custom.launch.py`**: A launch file that brings up the `rplidar_node`.
  - **Publishes to:** `/scan` — The standard ROS 2 topic for 2D laser scans.
  - **Hardware Interface:** Uses `/dev/serial/by-id/usb-Silicon_Labs...`.

### 3. `batteries`
A support package/library designed for battery and power management.
- Contains the `INA219.py` I2C driver for measuring voltage, current, and computing power usage over the Raspberry Pi's I2C pins.

---

## 🛠 Setup & Installation

Follow these steps when SSHing into your Raspberry Pi 5 to get the workspace built and ready to rock.

1. **Activate the Python Virtual Environment** 
   If you have a Python `venv` configured at the root of this project (to safely isolate pip packages like `pyserial` or `smbus`), activate it first:
   ```bash
   cd ~/ros2_ws  # Navigate to the root of this workspace
   source venv/bin/activate  # Or whatever your venv folder is named
   ```

2. **Source ROS 2 Jazzy**
   Bring ROS 2 into your bash environment:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

3. **Build the Workspace**
   Use `colcon` to build the Python and C++ packages together:
   ```bash
   colcon build --symlink-install
   ```

4. **Source the Workspace Overlay**
   Finally, source the newly built installation so that ROS 2 knows where to find `roomba_node` and all the launch files:
   ```bash
   source install/setup.bash
   ```

---

## 🚀 How to Run the Robot

You'll need multiple terminal tabs (or a tool like `tmux`) to launch the different components of the robot. Be sure to run steps 1, 2, and 4 (activate venv + source ros2 + source install) in **every new terminal**.

### Terminal 1: Start the Roomba Base
The `roomba_node` must be running before you can drive.
```bash
ros2 run roomba roomba_node
```
*Note: The roomba might beep when the connection initializes.*

### Terminal 2: Start the Sensors (LiDAR & Webcam)
Launch the RPLidar and the V4L2 Webcam node to begin publishing `/scan` and `/image_raw`:
```bash
ros2 launch rplidar_ros rplidar_c1_custom.launch.py
ros2 launch roomba webcam.launch.py
```
*(If you get resource busy errors, try running them in separate terminals)*

### Terminal 3: Teleoperation (Drive!)
Run the newly refactored teleop script. You'll be able to use your arrow keys or `WASD` to drive the robot around while watching the live voltage output.
```bash
ros2 run roomba roomba_teleop
```

## 🗺 Next Steps: SLAM
Because the robot is successfully publishing `/scan` (from RPLidar) and subscribing to `/cmd_vel` (via `roomba_node`), the workspace is almost completely ready for map building. 

To enable SLAM (like `slam_toolbox`), the required missing link will be **Odometry (`/odom`)**, which estimates the robot's current position relative to its starting point. In the future, modifying `roomba_node` to calculate and publish `nav_msgs/msg/Odometry` based on the roomba's internal wheel encoders will complete the SLAM puzzle!
