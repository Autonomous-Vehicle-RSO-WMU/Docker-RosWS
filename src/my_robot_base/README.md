# my_robot_base package

For low level hardware drivers & base controller nodes for the **physical** robot.

**Build Type:** `ament_python`

**Environment:** Physical

## System Interface
### Required Frames (from URDF)
* **`imu_link`**: Hardcoded into imu/magnetometer msg headers. - must exist in TF tree
* **`gps_link`**: Hardcoded into the NavSatFix msg header. - must exist in TF tree
### Subscription Topics
### Publishing Topics
* **`/imu/data`** (`sensor_msgs/Imu`): Publishes absolute orientation (quaternion), linear acceleration, and angular velocity.
* **`/imu/mag`** (`sensor_msgs/MagneticField`): Publishes raw magnetic field data from the Arduino.
* **`/gps/fix`** (`sensor_msgs/NavSatFix`): Publishes global positioning data (Lat, Long, & Alt scaled to meters/degrees).
### TF Transforms

## Package Structure

### Launch Files
* `base_launch.py`
    * Launches arduino node & loads hardware specific parameters from the config file.

### Config Files
* `base_config.yaml`
    * Defines the physical parameters for the serial connection (`port`, `baudrate`) and the Arduino polling rate (`poll_rate_hz`).

### Source Files
* `nodes/arduiino.py`
    * Establish serial connection w/ arduino, parse raw IMU & GPS str data, publishes as standard ROS2 msg.
* `utils/quarternion_calc.py`
    * Mathematical utility module to convert raw accelerometer & magnetometer Euler angles inot a Quarternion array.

### Simple Auxiliary Scripts (Python/Bash)

### Interfaces (msg/, srv/, action/)

### Assets (urdf/, meshes/, maps/)

## Executables & Nodes

### Nodes
* `arduino_node_ros`
    * **Source File:** `nodes/arduiino.py`
    * **Executable:** `arduino_node`