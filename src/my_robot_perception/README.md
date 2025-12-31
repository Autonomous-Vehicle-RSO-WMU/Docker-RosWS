# my_robot_perception package

Responsible for turning raw sensor data into high-level detections/flags.
Implements perception nodes.

**Build Type:** `ament_python`

**Environment:** Physical & Simulation

## Package Structure

### Launch Files

### Config Files

### Source Files
* `my_robot_perception/nodes/`
    * `vision_detector_node.py`
        * not registered in `setup.py`
        * legacy (uses CARLA)

### Simple Auxiliary Scripts (Python/Bash)

### Interfaces (msg/, srv/, action/)

### Assets (urdf/, meshes/, maps/)
* `models/`
    * `yolov8n.pt`

## Executables & Nodes

### Nodes
* `vision_detector`
    * **Source File:** `my_robot_perception/nodes/vision_detector_node.py`
    * **Desc:** Identifies stop sign from carla camera topic using yolov8n. When identified, publishes a stop flag to carla.
    * legacy (uses CARLA)

### Executables