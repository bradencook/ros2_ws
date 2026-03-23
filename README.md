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
  - **Publishes to:** `/odom` (`nav_msgs/msg/Odometry`) — Publishes high-accuracy odometry using raw wheel encoder ticks.
  - **TF:** Broadcasts the `odom` -> `base_link` transform.
  - **Hardware Interface:** Uses pyserial to talk to the Roomba over `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03F056-if00-port0`.
- **`roomba_teleop`**: A decoupled teleoperation node for driving the robot using a keyboard.
  - **Publishes to:** `/cmd_vel` — Sends Twist messages securely based on user input.
  - **Subscribes to:** `/roomba/sensors` — Decodes the telemetry string to display the live voltage and bump sensor status in your terminal while you drive.
- **`webcam.launch.py`**: A launch script included in the `roomba` package to bring up the `v4l2_camera` node (publishing `/image_raw`) and a `web_video_server` to stream the video to a web browser.
- **`mapping.launch.py`**: The definitive launch file for SLAM. Brings up the Roomba driver, LiDAR, webcam, `slam_toolbox`, and `foxglove_bridge` all at once to enable map building and debugging.

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

## 🚀 How to Create a 3D Map (SLAM)

Now that the robot publishes odometry, you can launch everything required for Foxglove debugging and SLAM mapping with a single command! Note: you will need multiple terminal tabs (or a tool like `tmux`). Be sure to activate your `venv` and source ROS 2 and your workspace in **every new terminal**.

### Terminal 1: Launch the Mapping Stack
This launch file handles bringing up the Roomba base, LiDAR, webcam, `slam_toolbox`, and the `foxglove_bridge`:
```bash
ros2 launch roomba mapping.launch.py
```

### Terminal 2: Teleoperation (Drive!)
Run the teleop script in a separate window so you can intercept keyboard input. You'll use your arrow keys or `WASD` to drive the robot around a room to build the map:
```bash
ros2 run roomba roomba_teleop
```

### Visualizing in Foxglove Studio
1. Turn on [Foxglove Studio](https://foxglove.dev/).
2. Open a new connection and select **Foxglove WebSocket**. Use the default `ws://localhost:8765` if on the Pi directly, or `ws://roomba.local:8765` / `ws://<pi-ip-address>:8765` if on your Mac.
3. Open a **3D Panel** to visualize:
   - The `/map` and `/odom` TF transform frames.
   - The `/scan` LaserScan data showing the LiDAR hitting walls.
   - The live `slam_toolbox` map building actively as the robot drives.
4. Open an **Image Panel** and subscribe to `/image_raw` to see your webcam feed!
