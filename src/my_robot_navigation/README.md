# my_robot_navigation package

Contains Nav2 specific config & launch files. Provides Navigation behavior.

**Note:** Src and include folder currently removed (no C++ executables or libraries.)

**Build Type:** `ament_cmake`

**Environment:** Physical & Simulation

## Package Structure

### Launch Files  
* `nav2.launch.py`
    * Not properly setup for new workspace
    * Dependencies partially connected in `package.xml` 
        * (missing `av_sim_core` dependency)

### Config Files  
* `nav2_params.yaml`

### Source Files
> **Note:** Won't use local source files?

### Scripts (Python/Bash)

### Interfaces (msg/, srv/, action/)

### Assets (urdf/, meshes/, maps/)
* `data/`
    * `waypoints.csv`
        * placeholder

## Executables & Nodes

### Nodes
> **Note:** Won't use local nodes?
* `<example_node>`
    * **Source File:** `<example/source.file>`
    * **Desc:** 

### Executables
> **Note:** Won't use local executables?