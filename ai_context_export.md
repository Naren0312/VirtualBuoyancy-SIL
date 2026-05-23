# AI Agent Context Export: Hammerhead ROV Migration

**To the receiving AI Agent**: The user is currently migrating a legacy ROS1 AUV (Autonomous Underwater Vehicle) system into a ROS2 (Humble/Jazzy) Remote Operated Vehicle (ROV) workspace. Below is the full technical state of the workspace to help you transition seamlessly.

## 1. Environment & Directories
* **Workspace Path**: `/home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil`
* **OS/Distribution**: Linux (ROS2 Humble/Jazzy)
* **Project Name**: Hammerhead Master

### Package Architecture
The `vibu_sil/src` directory contains 4 primary packages:
1. **`rov_bringup`**: Contains launch files (`rov_launch.py`) and the primary parameter configuration (`config/rov_params.yaml`).
2. **`rov_msgs`**: Contains custom `.msg` definitions (`SensorData.msg`, `Setpoint.msg`, `Depth.msg`, `IMUData.msg`, `PIDGains.msg`, `ThrusterCommand.msg`).
3. **`rov_hardware`**: Contains the nodes for interfacing with hardware.
4. **`rov_control`**: Contains nodes for joystick mapping and PID calculation.

## 2. Code Breakdown & Algorithms

### `rov_hardware`
1. **`serial_bridge_node`** (`src/serial_bridge_node.cpp`)
   * **Purpose**: Talks to an Arduino over UART (`/dev/nuc_nano` or similar).
   * **Algorithm**: Implements asynchronous polling and custom string parsing. It reads lines matching `D<depth>#Y<yaw>#P<pitch>#R<roll>#` and publishes to `/depth` and `/imu_data`.
2. **`sensor_sync_node`** (`src/sensor_sync_node.cpp`)
   * **Purpose**: Fuses decoupled depth and IMU messages into a unified packet.
   * **Algorithm**: Uses ROS2 `message_filters::Synchronizer` with an `ApproximateTime` policy to match timestamps of incoming `/depth` and `/imu_data` topics, emitting a unified `rov_msgs/msg/SensorData` packet on `/sensor_data` (Topic is vital for the PID loop).
3. **`thruster_driver_node`** (`src/thruster_driver_node.cpp`)
   * **Purpose**: Interacts with the Pololu Maestro board controlling ESCs.
   * **Algorithm**: Converts `int16_t` PWM ranges (1250-1750) to the Pololu Maestro quarter-microsecond binary protocol (`0x84, channel, target_low, target_high`).

### `rov_control`
1. **`control_node`** (`src/control_node.cpp`)
   * **Purpose**: Core compute node for movement.
   * **Algorithm**: Implements a discrete 6-DOF Proportional-Integral-Derivative (PID) loop. Triggers *only* when new `/sensor_data` is received. Uses a Control Allocation Matrix to proportionally map the Surge, Sway, Heave, Pitch, Roll, and Yaw outputs to the 6 physical thrusters. Prevents winding on angular error by using `atan2(sin, cos)`. Subscribes to `/set_mode` (`0`=SURFACE, `1`=TELEOP).
2. **`teleop_node`** (`src/teleop_node.cpp`)
   * **Purpose**: Human Interface.
   * **Algorithm**: Maps incoming `/joy` topic messages (axes and button states) to limits (e.g. `surge_limit_`, `sway_limit_`) to incrementally build `rov_msgs/msg/Setpoint` arrays.

## 3. Current User Status & Focus
* The user is a **beginner in ROS2**.
* We recently created a `rov_user_manual.md` inside `vibu_sil` so the user knows how to configure the stack safely using the YAML parameters rather than modifying C++ files directly.
* We actively discussed CLI testing commands (bypassing logic) using `ros2 topic pub` for topics like `/thruster_command` (injecting PWM directly into hardware) and `/setpoints` (testing the PID controller mathematically).

## 4. Message Signatures You Should Know
* **`ThrusterCommand.msg`**: `int16[6] pwm`, `bool[6] reverse`
* **`Setpoint.msg`**: `float32[6] setpoints`
* **`SensorData.msg`**: `std_msgs/Header header`, `float32 depth`, `float32[3] orientation`, `float32[3] acceleration`

---
**Agent Instruction**: Use this context to continue helping the user debug, test, or expand features in their ROV build!
