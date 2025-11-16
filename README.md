# X500 Drone Simulation and Altitude Controller

This repository contains a ROS 2 workspace for simulating an X500 drone in Gazebo and controlling its altitude with a Python-based PID controller.



---

## Packages

This workspace includes the following packages:
* **x500_description**: Contains the URDF/XACRO models for the X500 drone and its sensors (Lidar, cameras, etc.).
* **x500_gazebo**: Provides the Gazebo world files for the simulation.
* **x500_bringup**: Contains the main launch files to start the Gazebo simulation, spawn the drone, and launch RViz.
* **drone_altitude_controller**: A Python package containing the PID altitude controller node (`altitude_pid.py`) and a test node (`test.py`).

---

## Getting Started

### Prerequisites
* ROS 2 (e.g., Humble, Iron)
* Gazebo (usually included with `ros-distro-desktop-full`)
* Colcon (ROS 2 build tool)
* `ros-distro-ros-gz-bridge` (for communication between ROS 2 and Gazebo)

### Installation & Building

1.  **Clone the Repository**:
    Place all the packages (`drone_altitude_controller`, `x500_bringup`, `x500_description`, `x500_gazebo`) into the `src/` directory of your ROS 2 workspace.

    ```bash
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    # ... (Clone or copy your packages here)
    ```

2.  **Install Dependencies**:
    Navigate to the root of your workspace (`ros2_ws`) and install any missing dependencies.
    ```bash
    cd .. 
    rosdep install -i --from-path src -y --rosdistro $ROS_DISTRO
    ```

3.  **Build the Workspace**:
    Build all the packages using `colcon`.
    ```bash
    colcon build --symlink-install
    ```

---

## 🎮 Usage

You will need two terminals for this. **Remember to source your workspace in each new terminal!**

```bash
# In the root of your workspace (e.g., ~/ros2_ws)
source install/setup.bash
````

### 1\. Terminal 1: Launch the Simulation

In your first terminal, launch the main bringup file. This will start Gazebo, spawn the X500 drone, and open RViz.

```bash
ros2 launch x500_bringup x500.launch.py
```

### 2\. Terminal 2: Run the Controller

In your second terminal (after sourcing `install/setup.bash`), you can run your altitude controller node.

#### To run the main PID controller:

This command runs the `altitude_pid.py` script.

```bash
ros2 run drone_altitude_controller pid
```

#### To run the test node:

Alternatively, you can run the `test.py` script. This si to test pid for x y and z axis.

```bash
ros2 run drone_altitude_controller test
```
