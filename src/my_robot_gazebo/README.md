# my_robot_gazebo package

Contains Gazebo worlds, robot models, sim helper scripts, & launch files for simulation.

**Note:** Src and include folder currently removed (no C++ executables or libraries.)

## Package Structure

**Build Type:** `ament_cmake`

**Environment:** Simulation

### Launch Files
* `gazebo.launch.py`
    * Placeholder

### Config Files
* `controllers.yaml`
    * Placeholder

### Source Files
> **Note:** May not use local source files?
* `<example.cpp>`

### Simple Auxiliary Scripts (Python/Bash)
* `scripts/`
    * `spawn_actors.py`
        * dependencies not connected in `package.xml`
        * not registered in `CMakeLists.txt`
        * legacy (uses CARLA)
* `models/`
* `worlds/`

### Interfaces (msg/, srv/, action/)

### Assets (urdf/, meshes/, maps/)
* `models/`
* `worlds/`

## Executables & Nodes

### Nodes
> **Note:** May not use local nodes?
* `<example_node>`
    * **Source File:** `<example/source.file>`
    * **Desc:** 

### Executables