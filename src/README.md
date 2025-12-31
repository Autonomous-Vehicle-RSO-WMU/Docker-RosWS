<a id="readme-top"></a>

# my_robot_workspace

<p style="color: yellow; font-size: 40px; text-align: center;">
   WAIT
</p>
<div style="border: 2px solid yellow; padding: 10px; text-align: center;">
    <p> 
        I know there's a lot, but please take the time to at least skim through this README. 
    </p>
    <p>
        There's a lot of important info that took real human hours to write and put together (not AI slop).
    </p>
    <p>
        With that being said, there may be mistakes...
    </p>
    <p>
        Please message me on teams if you have any feedback or questions!
    </p>
    <p>
        Thank u,
    </p>
    <p>
        Charles &nbsp; •‿•
    </p>
</div>

&nbsp;

<!-- TABLE OF CONTENTS -->
<details>
  <summary style="font-size:18px">Table of Contents</summary>
  <ol>
    <li>
      <a href="#general-overview">General Overview</a>
    </li>
    <li>
      <a href="#high-level-structure">High Level Structure</a>
    </li>
    <li>
        <a href="#quick-introduction-to-build-types">Quick Introduction to Build Types</a>
        <ul>
            <li><a href="#1-ament_cmake">1. ament_cmake</a></li>
        </ul>
        <ul>
            <li><a href="#2-ament_python">2. ament_python</a></li>
        </ul>
        <ul>
            <li><a href="#3-ament_cmake_python">3. ament_cmake_python</a></li>
        </ul>
        <ul>
            <li><a href="#creating-a-package">Creating a Package</a></li>
        </ul>
    </li>
    <li>
        <a href="#after-creating-a-package">After Creating a Package</a></li>
        <ul>
            <li><a href="#a-any">Any Build Type</a></li>
        </ul>
        <ul>
            <li><a href="#a-cmake">CMake Build Type</a></li>
        </ul>
        <ul>
            <li><a href="#a-python">Python Build Type</a></li>
        </ul>
    </li>
    <li>
        <a href="#package-folder-structure">Package Folder Structure</a></li>
        <ul>
            <li><a href="#b-any">Any Build Type</a></li>
        </ul>
        <ul>
            <li><a href="#b-cmake">CMake Build Type</a></li>
        </ul>
        <ul>
            <li><a href="#b-python">Python Build Type</a></li>
        </ul>
    </li>
    <li>
        <a href="#helpful-resources">Helpful Resources</a>
    </li>
  </ol>
</details>

&nbsp;  
Before adding/editing a package please review the following sections:
* <a href="#quick-introduction-to-build-types">Quick Introduction to Build Types</a>
* <a href="#after-creating-a-package">After Creating a Package</a>
* <a href="#package-folder-structure">Package Folder Structure</a>

For specific package information, please see its respective README.
> **IMPORTANT:** <span style="color: orange;">Currently, packages contain mostly placeholder/legacy content, and are not in a working state.</span>

> **IMPORTANT:** <span style="color: orange;">Docker in this workspace is also legacy.</span>

## General Overview

The goal of this workspace structure is to provide clarity and modularity for our project.

### Some important design choices

* **Bringup Package**
    * A bringup package to contain high-level launch files to start the robot either physically or in simulation.

* **Environment Agnostic Nodes**
    * Core logic nodes should be environment agnostic.
        * using logical topics. 
        * physical/simulation specific launch files in `my_robot_bringup` will remap logical topics to their specific physical/simulation topics.

* All packages have their own `config/` and `launch/` folders.

* All packages have their own README to keep documentation clean.
    * Please update as changes are made.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## High Level Structure
``` text
└── my_robot_workspace/
    └── src/
        ├── my_robot_base          (ament_cmake)
        ├── my_robot_bringup       (ament_cmake)
        ├── my_robot_description   (ament_cmake)
        ├── my_robot_gazebo        (ament_cmake)
        ├── my_robot_localization  (ament_python)
        ├── my_robot_navigation    (ament_cmake)
        ├── my_robot_perception    (ament_python)
        └── my_robot_teleop        (ament_python)
```

> **Note:** <span style="color: yellow;">This package list is **NOT** exhaustive, please update as changes are made.</span>

> **Note:** <span style="color: yellow;">The Chosen Build Types are also open to change.</span>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Quick Introduction to Build Types

### 1. ament_cmake  
---
**When to use:** When you compile any C++ code.
* Can include Python, but recommended only for simple auxiliary scripts managed via CMake.

### 2. ament_python
---
**When to use:** When code is mainly Python, and you **DON'T** need to compile C++.  

**Non-Python code:** Some launch files (XML/YAML), some config files (YAML,JSON,TOML), data/resources used by python code, etc... You get the idea.

### 3. ament_cmake_python
---
**When to use:** When you want a combination of C++ and full Python packaging in a package.
* Recommended to use if Python code is more than simple auxiliary scripts
* Allows for Python packaging (`setup.py`/`setup.cfg`) to coexist with CMake packaging.

### Creating a package
---
`ros2 pkg create --build-type <build_type> --license MIT <package_name>`

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## After Creating a Package

<h3 id="a-any">Any Build Type</h3>

---

> #### In `package.xml`
Here, you will add any **ROS/ament** dependencies (packages) used.
1. **Including Runtime Dependencies**  
    Dependencies used in nodes, launch files, Python modules, etc...
    * `<exec_depend>...</exec_depend>`
2. **Build Dependencies**  
    e.g., header files/libraries from other ROS/ament packages
    * `<build_depend>...</build_depend>`
3. **Exported Header Dependencies**  
    If headers in current package include headers from another package.
    * `<build_export_depend>...</build_export_depend>`
4. **Testing Dependencies**  
    Dependencies used only when building/running tests.
    * `<test_depend>...</test_depend>`
5. **Build Tool Dependencies**  
    Usually already included from `ros2 pkg create` command.
    * `<build_tool_depend>...</build_tool_depend>`
6. **Documentation Dependencies**  
    If including documentation tools like `Doxygen` (TBD, but may include this)
    * `<doc_depend>...</doc_depend>`

>**Note:** <span style="color: yellow;">Currently, non-ROS third party **Python** libraries are managed via `requirements-ros.txt`, not `package.xml`.</span>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<h3 id="a-cmake">CMake Build Type</h3>

---

> #### In `CMakeLists.txt`
1. **Including Dependencies**
    * Any dependencies (e.g., ROS/ament packages, third party C++ libraries, etc...) used in C++ code go in
    ``` CMake
    find_package(<dependency> REQUIRED)
    ```
    * & likewise, testing dependencies go in
    ``` CMake
    if(BUILD_TESTING)

      find_package(<dependency> REQUIRED)

    endif()
    ```

2. **Including Non-Executable Data Files**
    * Add directories containing non-executable files to `install(DIRECTORY...)` section
    * For data files in directories such as `launch/`, `config/`, `maps/`, etc...
        ``` CMake
        install(DIRECTORY
          <directory1>
          <directory2>
          DESTINATION share/${PROJECT_NAME}
        )
        ```

3. **Including C++ Executables**
    1. **Add & name executable** (so can run executable w/ `ros2 run`), after `find_package()` function/s.
        ``` CMake
        add_executable(<chosen_name> 
          src/<path/executable_1>.cpp
          src/<path/executable_2>.cpp
        )
        ```
    2. **Declare ROS/ament dependencies (packages) used in executable**
        ``` CMake
        ament_target_dependencies(<chosen_name>
          <ros_dependency_1>
          <ros_dependency_2>
        )
        ```
    3. **Add executable to `install(TARGETS...)` section** (so `ros2 run` can find executable)
        ``` CMake
        install(TARGETS
          <chosen_name>
          DESTINATION lib/${PROJECT_NAME}
        )
        ```

4. **Including Custom Libraries**
    * For non-executable code shared by multiple executables in a package.
    1. **Add & name library**, after `find_package()` function/s.

        ``` CMake
        add_library(<chosen_name>
          src/<path/file_1>.cpp
          src/<path/file_2>.cpp
        )
        ```
    2. **Declare ROS/ament dependencies (packages) used in library**
        ``` CMake
        ament_target_dependencies(<library_name>
          <ros_dependency_1>
          <ros_dependency_2>
        )
        ```  

    3. **Link library in `target_link_libraries(...)` section** (To link it to the executable/s that use it).
        ``` CMake
        target_link_libraries(<chosen_executable>
          <library_name>
        )
        ```

5. **Including Third Party Libraries**  
    >**IMPORTANT:** <span style="color: orange;">Don't forget `find_package()` step mentioned previously.</span>

    1. **Link library in `target_link_libraries(...)` section**
        ``` CMake
        target_link_libraries(<chosen_name>
          PRIVATE <lib_name::lib_name>  # e.g., TensorFlow::TensorFlow 
        )
        ```

6. **Including Python Executables**
    1. **Make file executable**  
    Only need to do this **ONCE** per script.  
        ``` bash
            chmod +x <path/executable_name.py> 
        ```
    2. **Add executable to `install(PROGRAMS...)` section**
        ``` CMake
        install(PROGRAMS
            <path/executable_name.py>
            DESTINATION lib/${PROJECT_NAME}
        )
        ```

7. **No Executables?**
    * If package has no C++ executables or libraries, remove default `src` & `include` folders

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<h3 id="a-python">Python Build Type</h3>

---

> #### In `setup.cfg`
* Most likely won't ever need to edit.
* You'd edit only if you need to adjust Python package metadata.

> #### In `setup.py` 
1. **Mirror `package.xml`**
    * `maintainer`, `maintainer_email`, and `description` lines must match corresponding fields in `package.xml`.
2. **Including Non-Executable Files**  
    > In `data_files`
    * (IF applicable) Add **launch**, **config**, **description**, and **map/world files** paths here **using `os` and `glob`**.  
    * **Ex:** Copy all launch files (ending in launch.py) to package's share directory  
    ``` py
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))
    ```
    > **Note:** <span style="color: yellow;">Please keep in mind, `ament_python` does not automatically grab all files in a folder when installing files like `ament_cmake` does. It's explicit.</span>
3. **Including Executables**
    > In `entry_points`
    * Add an entry point for any **executable node** in package so can run with `ros2 run`

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Package Folder Structure

<h3 id="b-any">Any Build Type</h3>

---

All to be included in top level folders.

* **Launch Files:** In `launch/`

* **Config Files:** In `config/`

* **Interfaces:** In relevantly named folder, e.g., `msg/`, `srv/`, `action/`

* **Assets:** In relevantly named folder, e.g., `urdf/`, `meshes/`, `maps/`

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<h3 id="b-cmake">CMake Build Type</h3>

---

All C++ code will be inside either the `src/` or `include/` folder.

For clarity, we can use subfolders inside `src/` and `include/` as seen below.

* **Source Files:**

    * **Implementation Files:** In `src/`, in a relevantly named subfolder, e.g., `src/nodes/`, `src/lib`.

    * **Header Files (.hpp):** In `include/` OR `src/`
        * `include/` for public headers.
        * `src/` for private headers, in a relevantly named subfolder, e.g., `src/headers`.

        >**Note:** <span style="color: yellow;">May or may not be necessary to use subfolders inside of `include/`. They're all gonna be headers anyways...</span>

* **Simple Auxiliary Scripts (Py/Bash):** In top level `scripts/` folder.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<h3 id="b-python">Python Build Type</h3>

---

* **Source Files:**  
In the **Python package directory**  
    >*(directory with same name as package, e.g, `my_robot_package/`, where `__init__.py` is contained.)*  

    in a relevantly named subfolder  
    >(e.g., `my_robot_package/nodes`, `my_robot_package/utils`).

* **Simple Auxiliary Scripts (Py/Bash):** In top level `scripts/` folder.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Helpful Resources

> **Note:** <span style="color: yellow;">Obviously, none of these will match our workspace exactly.</span>

* Official ROS documentation: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html  

* Easy to follow overview: https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/  

* Practical Example: https://github.com/linorobot/linorobot2  

* Video if you really wanna get crazy: https://www.youtube.com/watch?v=Gg25GfA456o&t=2071s  

<p align="right">(<a href="#readme-top">back to top</a>)</p>