# Box Bot ROS 2 - Mobile Robot Simulation & Autonomous Navigation

## Project Description

Developed a complete ROS 2-based mobile robot simulation with autonomous obstacle avoidance and navigation capabilities. Designed and deployed a differential-drive robot in Gazebo simulator equipped with LIDAR sensors, integrated with Nav2 autonomous navigation stack for path planning and obstacle avoidance.

## Key Technologies & Skills Demonstrated

- **Languages:** Python, XML (URDF), YAML
- **Framework:** ROS 2 (Kilted/Jazzy), Gazebo Physics Engine, Nav2 Navigation Stack
- **Core Competencies:**
  - **Robot Modeling:** Created complete URDF specifications (box chassis, differential wheels, caster wheels, LIDAR sensor)
  - **Simulation & Integration:** Configured Gazebo environment with custom world files, GPS/odometry bridging, and sensor simulation
  - **Autonomous Navigation:** Implemented Nav2 configuration including AMCL localization, path planning (NavFn planner), and motion control (DWB controller)
  - **Obstacle Avoidance:** Built state-machine based obstacle avoider node with real-time LIDAR processing (laser scan filtering, distance evaluation)
  - **Launch System:** Developed Python-based launch files for multi-node orchestration and parameter management
  - **ROS 2 Architecture:** Publishers/subscribers pattern, node lifecycle management, sensor QoS profiles

## Project Structure

```
box_bot_ros2/
├── src/
│   └── box_bot_description/           # Main ROS 2 package
│       ├── urdf/
│       │   └── box_bot.urdf           # Robot model definition (links, joints, sensors)
│       ├── world/
│       │   └── my_world.world         # Gazebo simulation environment
│       ├── launch/
│       │   ├── sim.launch.py          # Simulation launch configuration
│       │   └── navigation.launch.py   # Nav2 navigation stack launch
│       ├── config/
│       │   └── nav2_params.yaml       # Navigation & planning parameters
│       ├── box_bot_description/
│       │   └── obstacleavoider.py     # Obstacle avoidance node
│       ├── package.xml                # ROS 2 package manifest
│       └── setup.py                   # Package installation configuration
└── project_creation_roadmap.ipynb     # Development documentation & setup guide
```

### How It Fits Together

The system operates as a complete autonomous navigation pipeline:

1. **Simulation Layer:** Gazebo simulator runs the robot physics with custom world environment
2. **Perception:** LIDAR sensor provides 640-sample laser scans at 10 Hz (0.1-10m range)
3. **Localization:** AMCL node performs Monte Carlo Localization against pre-built map
4. **Planning:** NavFn planner generates collision-free paths on occupancy grid
5. **Control:** DWB (Dynamic Window Approach) controller tracks planned path via velocity commands
6. **Autonomy:** Obstacle avoider node provides reactive obstacle avoidance using state-machine (FORWARD → BACKING → TURNING)
7. **Integration:** ros_gz_bridge translates between ROS 2 and Gazebo topics in real-time

## How to Run It

### Prerequisites

- Ubuntu 24.04 LTS
- ROS 2 Jazzy/Kilted installed
- Gazebo Sim (Harmonic or compatible)
- Nav2 stack

### Quick Start

**1. Clone and Setup Workspace**
```bash
git clone https://github.com/Abdel505/box_bot_ros2.git
cd box_bot_ros2
colcon build
source install/setup.bash
```

**2. Run Simulation**
```bash
ros2 launch box_bot_description sim.launch.py
```
This starts Gazebo with the custom world and spawns the robot with all sensor bridges.

**3. Run Navigation Stack**
```bash
ros2 launch box_bot_description navigation.launch.py
```
This launches the full Nav2 stack: map server, AMCL, planner, controller, and BT navigator.

**4. Run Obstacle Avoidance (Optional)**
```bash
ros2 run box_bot_description obstacle_avoider
```
Enables autonomous obstacle avoidance using LIDAR feedback.

### Testing Navigation

Use ROS 2 CLI to send navigation goals:
```bash
# Send a goal to coordinates (5.0, 5.0)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0}, orientation: {w: 1.0}}}}"
```

Or use the default RViz visualization interface (part of Nav2 stack).

## Try Asking

- **How does the obstacle avoider state machine work?** - Check `obstacleavoider.py` for FORWARD/BACKING/TURNING state transitions
- **What are the Nav2 controller parameters optimized for?** - Review `config/nav2_params.yaml` for tuned DWB critic scales
- **How is the LIDAR sensor integrated into Gazebo?** - See URDF GPU lidar plugin definition in `urdf/box_bot.urdf`
- **What ROS 2 topics are bridged between Gazebo and ROS?** - Inspect topic remappings in `launch/sim.launch.py`

---

**Project Creation:** December 29, 2025 | **Last Updated:** January 14, 2026
