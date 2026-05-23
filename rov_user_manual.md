# Hammerhead ROV: ROS2 Software Architecture & User Manual

Welcome to the software stack for the Hammerhead ROV! This manual provides an overview of the algorithms, program logic, and how to modify the ROS2 workspace (`vibu_sil`). 

As a ROS2 beginner, it's helpful to realize that the ROV acts as a distributed system: tiny programs called **Nodes** run simultaneously and talk to each other by publishing and subscribing to **Topics**.

---

## 1. Directory & Package Structure

The `vibu_sil/src` directory contains four main packages:
1. **`rov_bringup`**: Contains start-up (launch) scripts and the master configuration YAML file (`rov_params.yaml`).
2. **`rov_hardware`**: Nodes that interact directly with physical hardware (sensors, ESCs, microcontrollers). 
3. **`rov_control`**: The "brain" computing the vehicle's movement and accepting joystick commands.
4. **`rov_msgs`**: Custom message definitions (the data structures sent over topics).

---

## 2. Core Nodes & Algorithms

### 2.1 `serial_bridge_node`
**Location:** `rov_hardware/src/serial_bridge_node.cpp`
* **Purpose:** Serves as the communication bridge between the ROV's onboard PC and an Arduino reading depth/IMU sensors via USB Serial.
* **Algorithm/Logic:** 
  * Async Polling: Runs a timer ticking every 33 milliseconds (~30 Hz) to poll a POSIX file descriptor linked to a `/dev/tty*` serial port.
  * Custom String Protocol Parser: As data streams in, it buffers characters until a newline. It reads strings formatted like `D<depth>#Y<yaw>#P<pitch>#R<roll>#` and casts the values to floats.
* **How to Modify:** If you wire up a new temperature sensor to the Arduino, you would send `T<temp>#` over serial. In `parse_serial_line(const std::string &line)`, add `{'T', 1}` to `key_map` and capture the token to publish it on a new topic.

### 2.2 `sensor_sync_node`
**Location:** `rov_hardware/src/sensor_sync_node.cpp`
* **Purpose:** Combines disparate sensor messages into a unified packet.
* **Algorithm/Logic:** 
  * Uses the ROS2 `message_filters::Synchronizer` with an `ApproximateTime` policy. Because depth sensors and IMUs operate at different frequencies, this algorithm looks at the timestamps on `/depth` and `/imu_data` messages. Once it finds two messages close together in time, it fuses them into a single `/sensor_data` packet.
* **How to Modify:** If you want to add pressure data to the sync, you would declare another `message_filters::Subscriber` in the constructor, increase the `SyncPolicy` template arguments, and fuse it inside `sync_callback`.

### 2.3 `control_node`
**Location:** `rov_control/src/control_node.cpp`
* **Purpose:** Calculates the exact physical push needed from thrusters to reach the desired state. 
* **Algorithm/Logic:** 
  * 1. **Proportional-Integral-Derivative (PID) Controller:** In `compute_pid()`, error is measured as `target_setpoint - current_sensor_reading`. The PID formula outputs a correction curve: `Output = (Kp * error) + (Kd * error_rate_of_change) + (Ki * accumulated_error)`. It uses trigonometric clamping to prevent spinning when crossing 360/0 degrees.
  * 2. **Control Allocation Matrix:** In `compute_forces()`, the ROV maps the 6 Degrees-Of-Freedom (Surge, Sway, Heave, Pitch, Roll, Yaw) PID corrections onto the 6 physical thrusters using proportional math.
  * 3. **PWM Conversion:** In `forces_to_pwm()`, thrust forces are converted to pulse-width modulation signals (typically between 1250µs and 1750µs).
* **How to Modify:** If the ROV feels jittery or sluggish, **do not modify this C++ file!** Instead, tune the PID values in the parameter config file. Only change this C++ code if you are physically changing where thrusters sit on the ROV chassis (which changes the allocation matrix).

### 2.4 `teleop_node`
**Location:** `rov_control/src/teleop_node.cpp`
* **Purpose:** Converts human remote control commands (via `/joy`) into vehicle targets (`/setpoints`).
* **Algorithm/Logic:**
  * Checks gamepad thresholds: Maps joystick axes directly to yaw/surge setpoints, and buttons to step-wise increases/decreases in depth/pitch. 
  * Enforces limits using basic clamping blocks (`surge_limit_`, `sway_limit_`) prior to sending the commands so the human cannot tell the ROV to go to unsafe depths.
* **How to Modify:** If using an Xbox vs PlayStation controller, button IDs might change. Update the parameters like `axis_yaw_coarse` or `btn_surge_fwd` to alter control scheme bindings.

### 2.5 `thruster_driver_node`
**Location:** `rov_hardware/src/thruster_driver_node.cpp`
* **Purpose:** Sends the low-level thrust signals to the Pololu Maestro board (which powers the ESCs).
* **Algorithm/Logic:** 
  * Iterates over a command array and converts PWM intervals to Pololu Maestro's required "quarter-microsecond" binary serial data payloads (`0x84, channel, target_low, target_high`).
* **How to Modify:** If a thruster is plugged into the wrong pin on the Maestro board, update the `port_mapping` parameter list. If a thruster spins backwards, swap the `thruster_reverse` boolean parameter.

---

## 3. How to Configure the ROV Safely

As a ROS2 beginner, it's highly recommended to **avoid manually modifying the C++ files** unless adding massive new features. ROS2 uses a Parameter system that allows changing the code's behavior at runtime without recompiling.

All crucial variables live in:
`vibu_sil/src/rov_bringup/config/rov_params.yaml`

Within this file you can alter:
* **`pid.kp, pid.ki, pid.kd`**: Fine-tune the ROV's stability.
* **`surge_limit, heave_max`**: Change how fast or deep it is allowed to travel.
* **`port_mapping`**: Fix thruster wiring mixups.
* **`btn_*, axis_*`**: Bind gamepad actions.

---

## 4. Building and Launching

Whenever you modify a `.cpp` file, you must rebuild the code:
```bash
cd ~/Documents/hammerhead-master/hammerhead-master/vibu_sil
colcon build --symlink-install
```

After configuring or building, Source your environment and run the main launch script to bring up the entire ROV stack:
```bash
source install/setup.bash
ros2 launch rov_bringup rov_launch.py
```
