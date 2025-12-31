# my_robot_localization package

Hosts sensor fusion & localization nodes (& their parameter files) that are responsible for estimating the robots state.

**Build Type:** `ament_python`

**Environment:** Physical & Simulation

## Package Structure

### Launch Files

### Config Files

### Source Files
* `my_robot_localization/nodes/`
    * `sensor_fusion.py`
        * dependencies not connected in `package.xml`
        * not registered in `setup.py`
        * legacy (uses CARLA)

### Scripts (Python/Bash)

### Interfaces (msg/, srv/, action/)

### Assets (urdf/, meshes/, maps/)

## Executables & Nodes

### Nodes
* `sensor_fusion_node`
    * **Source File:** `my_robot_localization/nodes/sensor_fusion.py`
    * **Desc:** Fuses lidar & radar messages from carla topics, and publishes to a carla topic.

### Executables