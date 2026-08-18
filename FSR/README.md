# FSR Workspace

FSR is a ROS 2 workspace for an internal mobile robot controller. It brings
together simulation, mapping, localization, navigation, task execution,
waypoint tools, emergency-stop handling, and battery/docking behavior.

The source tree lives under `src/FSR`. Generated colcon outputs such as
`build/`, `install/`, and `log/` are workspace artifacts, not source packages.

## System Architecture

At a system level, FSR is organized as a set of ROS 2 packages that cooperate
through launch files, topics, services, and actions:

```text
Operator / higher-level client
        |
        | services/actions
        v
+----------------------+        +----------------------+
| Control managers     |        | Task manager         |
| - simulation control |        | - task files         |
| - mapper control     |        | - pause/stop/shutdown|
| - navigator control  |        | - Nav2 action client |
+----------+-----------+        +----------+-----------+
           |                               |
           | launches                      | NavigateToPose /
           v                               | NavigateThroughPoses
+----------------------+                   v
| Runtime stacks       |        +----------------------+
| - Gazebo/RViz        |        | Nav2 navigation      |
| - SLAM Toolbox       |<------>| map, AMCL, planner,  |
| - RTAB-Map ICP       |        | controller, BT, dock |
+----------+-----------+        +----------+-----------+
           |                               |
           | sensor, tf, map, cmd_vel       | cmd_vel, goals
           v                               v
+------------------------------------------------------+
| Robot model / simulated robot / physical integration |
+------------------------------------------------------+

Cross-cutting services:
  - Robot state manager publishes `/robot_state`
  - Emergency stop publishes `/emergency_stop`
  - Battery manager monitors `/battery_status` and drives docking
  - Planner switcher updates Nav2 planner/controller plugins at runtime
```

## Source Layout

```text
src/FSR/
  config/                         Shared Nav2, controller, docking config
  simulator/                      Gazebo, RViz, robot descriptions, teleop
  mapper/                         2D SLAM Toolbox and 3D RTAB-Map mapping
  localization_3d/                RTAB-Map localization manager/launch
  navigator/                      Nav2 launch files and navigation managers
  features/
    task_manager_server/          Custom ROS interfaces
    task_manager/                 Task execution and emergency-stop nodes
    waypoint_recorder/            RViz clicked-point waypoint recording
    waypoint_navigator/           Waypoint-file navigation
    battery_manager/              Battery simulation and docking behavior
```

## Package Responsibilities

### `simulator`

Provides the robot simulation environment.

- `2D.simulator.launch.py` starts Gazebo with the 2D differential-drive robot
  description, ROS 2 control, controller spawners, teleop, and RViz.
- `3D.simulator.launch.py` starts Gazebo server/client with the 3D robot
  description, teleop, and RViz.
- Robot descriptions are stored in `2D_description/` and `3D_description/`.
- Gazebo world and RViz configs are stored in `worlds/` and `rviz/`.
- `general.simulator` exposes services to start/stop 2D or 3D simulation.
- `zone_recorder` and `zone_detector` support keepout/zone workflows.

### `mapper`

Owns mapping workflows.

- `2D.mapper.launch.py` starts `slam_toolbox` in online async mode.
- `3D.mapper.launch.py` starts RTAB-Map LiDAR mapping using ICP odometry and
  `/velodyne_points2` as the point-cloud input.
- `general_mapper` exposes services to start/stop 2D or 3D mapping and
  publishes `/mapping_active`.

### `localization_3d`

Owns 3D localization control.

- `3D.localization.launch.py` starts RTAB-Map ICP odometry and RTAB-Map in
  localization mode against `/home/panu/.ros/rtabmap.db`.
- `localization_3D_manager` exposes `localization_3D_control` and blocks
  start/stop requests while emergency stop is active.

### `navigator`

Owns the Nav2 runtime and navigation control helpers.

- `2D.navigator.launch.py` starts Nav2 map server, AMCL, controller, planner,
  behavior server, BT navigator, collision monitor, smoother, waypoint
  follower, OpenNav docking, planner switcher, and lifecycle manager.
- `3D.navigator.launch.py` starts the Nav2 navigation stack without AMCL/map
  server, intended to pair with the 3D localization stack.
- `planner_switcher` exposes `change_planners` and updates Nav2 global/local
  planner plugins through `/planner_server/set_parameters` and
  `/controller_server/set_parameters`.
- `general_navigator` exposes services to start/stop 2D or 3D navigation and
  publishes `/navigation_active`.
- `state_manager` combines `/mapping_active`, `/navigation_active`, and
  `/emergency_stop` into `/robot_state`.

### `task_manager_server`

Defines project-specific interfaces used by the Python nodes.

- `action/RunTaskFile.action` runs a task file and reports subtask feedback.
- `srv/ChangePlanners.srv` requests global/local planner plugin changes.
- `srv/SelectFile.srv` selects a named file.
- `msg/RobotStatus.msg` describes emergency, pause, navigation, task, and
  state information.

### `task_manager`

Executes task files and coordinates safety controls.

- `task_manager` provides the `run_task_file` action server.
- It drives Nav2 through `navigate_to_pose` and `navigate_through_poses`.
- It exposes `/pause_navigation`, `/stop_navigation`, and `/shutdown_node`.
- It listens to `/amcl_pose` for ETA/distance calculations and
  `/emergency_stop` for safety interruption.
- Task definitions live in `features/task_manager/tasks/`.
- `E_stop` exposes `emergency_stop_toggle` and publishes `/emergency_stop`.

### `waypoint_recorder`

Records waypoints from RViz.

- Subscribes to `/clicked_point`.
- Publishes markers on `/waypoint_recorder/markers`.
- Uses `start_recording` and `stop_recording` services.
- Saves YAML waypoint files under
  `features/waypoint_recorder/points/`.

### `waypoint_navigator`

Loads saved waypoint YAML files and sends Nav2 goals.

- Exposes `/load_paths` and `/stop_navigation`.
- Publishes selected waypoint markers on `/navigator/markers`.
- Sends either `NavigateToPose` or `NavigateThroughPoses` goals.
- Cancels active goals when `/emergency_stop` is activated.

### `battery_manager`

Simulates battery state and handles low-battery docking behavior.

- `battery_pub` publishes `/battery_status`, drains while active, charges
  while docking status is `charging`, and publishes `/battery/low_warning`.
- `battery_manager` watches `/battery_status`, cancels navigation when the
  battery drops below the threshold, navigates to the configured dock, calls
  OpenNav docking/undocking actions, and publishes `/robot/status`.

## Important Runtime Interfaces

### Topics

- `/robot_state` - high-level state from `state_manager`.
- `/mapping_active` - mapping controller state.
- `/navigation_active` - navigation controller state.
- `/emergency_stop` - emergency stop latch.
- `/cmd_vel` - robot velocity command.
- `/amcl_pose` - 2D localization pose used by task execution.
- `/battery_status` - simulated battery percentage.
- `/robot/status` - battery/docking state string.
- `/clicked_point` - RViz clicked points for waypoint recording.
- `/waypoint_recorder/markers` and `/navigator/markers` - RViz markers.

### Services

- `control_simulation_2d`, `control_simulation_3d`
- `control_mapper_2d`, `control_mapper_3d`
- `control_navigation_2d`, `control_navigation_3d`
- `localization_3D_control`
- `change_planners`
- `emergency_stop_toggle`
- `/pause_navigation`, `/stop_navigation`, `/shutdown_node`
- `start_recording`, `stop_recording`, `/load_paths`

### Actions

- `run_task_file` from `task_manager_server/action/RunTaskFile.action`
- `navigate_to_pose` from Nav2
- `navigate_through_poses` from Nav2
- `dock_robot` and `undock_robot` from OpenNav docking

## Configuration

Shared configuration is stored in `src/FSR/config/`.

- `2D_params.yaml` - Nav2 parameters for 2D navigation.
- `3D_params.yaml` - Nav2 parameters for 3D navigation.
- `controllers.yaml` - ROS 2 control configuration for the simulated robot.
- `software.yaml` - map server, docking database, and behavior-tree paths.
- `dock_database.yaml` - OpenNav dock definitions.
- `_params.yaml` - additional parameter set kept with the workspace.

Navigation assets are stored in `src/FSR/navigator/`.

- `maps/new_map.yaml` and `maps/new_map.pgm` are used by the 2D map server.
- `behavior_tree/bt1.xml` and `behavior_tree/bt2.xml` are used by Nav2.

Mapping/localization database files are present at the workspace root as
`rtabmap.db` and `rtabmap.db.back`; the 3D localization launch currently points
to `/home/panu/.ros/rtabmap.db`.

## Common Commands

Build the workspace:

```bash
colcon build
source install/setup.bash
```

Start simulation directly:

```bash
ros2 launch simulator 2D.simulator.launch.py
ros2 launch simulator 3D.simulator.launch.py
```

Start mapping directly:

```bash
ros2 launch mapper 2D.mapper.launch.py
ros2 launch mapper 3D.mapper.launch.py
```

Start navigation directly:

```bash
ros2 launch navigator 2D.navigator.launch.py
ros2 launch navigator 3D.navigator.launch.py
```

Start 3D localization:

```bash
ros2 launch localization_3d 3D.localization.launch.py
```

Run manager nodes:

```bash
ros2 run simulator general.simulator
ros2 run mapper general_mapper
ros2 run navigator general_navigator
ros2 run navigator state_manager
ros2 run task_manager task_manager
ros2 run task_manager E_stop
```

Example service calls:

```bash
ros2 service call /control_simulation_2d std_srvs/srv/SetBool "{data: true}"
ros2 service call /control_mapper_2d std_srvs/srv/SetBool "{data: true}"
ros2 service call /control_navigation_2d std_srvs/srv/SetBool "{data: true}"
ros2 service call /emergency_stop_toggle std_srvs/srv/SetBool "{data: true}"
ros2 service call /pause_navigation std_srvs/srv/SetBool "{data: true}"
```

## Typical Operating Flows

### 2D simulation and navigation

1. Start the 2D simulator.
2. Start 2D mapping if a map needs to be built, or start 2D navigation if
   `navigator/maps/new_map.yaml` is ready.
3. Use waypoint, task-manager, or Nav2 action clients to send goals.
4. Use emergency-stop and pause/stop services for safety control.

### 3D mapping/localization and navigation

1. Start the 3D simulator or connect the 3D sensor stack.
2. Run `3D.mapper.launch.py` to create/update the RTAB-Map database.
3. Run `3D.localization.launch.py` for localization mode.
4. Start `3D.navigator.launch.py` for Nav2 planning and control.

### Task execution

1. Start Nav2 and `task_manager`.
2. Send a `run_task_file` action goal with the task filename.
3. The task manager parses the selected YAML/JSON task file, sends Nav2 goals,
   publishes `task_feedback`, and handles pause, stop, shutdown, retry, and
   emergency-stop state.

### Battery docking

1. Start `battery_pub` and `battery_manager`.
2. `battery_pub` publishes simulated battery percentage.
3. When the battery is below threshold, `battery_manager` cancels active
   navigation, navigates to `flex_dock1`, runs docking, waits for full charge,
   and undocks.

## Development Notes

- This workspace targets ROS 2 with Python packages built by `ament_python` and
  custom interfaces built by `ament_cmake`.
- Several launch files and nodes currently use absolute paths under
  `/home/panu/FSR_WS`; update those paths if the workspace is moved.
- Some generated or local runtime artifacts are present in the workspace root
  (`build/`, `install/`, `log/`, `rtabmap.db`). Keep source changes focused
  under `src/FSR/` and this root README.
- Package test folders contain the default lint/test scaffolding. Add
  behavior-level tests around task execution, emergency stop, planner switching,
  and docking before using this stack in higher-risk deployments.
