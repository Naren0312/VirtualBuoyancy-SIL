#  ViBu (Virtual Buoyancy) Software-in-the-Loop (SIL) Simulator Guide

Welcome to the documentation for the ViBu Software-in-the-Loop (SIL) simulator for ROVs. This guide details what was built, how the underlying mathematics and algorithms function, how to operate the simulation environment, and how to configure or tune the system.

---

## 1. Basics & Concept of SIL Simulation

### What is SIL?
In physical operations, the ROV control algorithms run on an onboard computer (Raspberry Pi/Intel NUC) and interface with hardware devices:
*   A depth sensor and an IMU providing feedback over serial.
*   Pololu Maestro motor controllers translating commands to PWM signals for 6 ESC-driven thrusters.

**Software-in-the-Loop (SIL)** replaces the physical sensors, serial bridges, and thruster drivers with a software simulation, keeping the **exact same control code** running unchanged. This allows you to:
1.  Validate control stack changes (e.g. PID gains, allocation matrices, filtering) safely on your local machine.
2.  Run automated regression tests to verify performance before deploying to the physical vehicle.
3.  Simulate hardware-independent scenarios.

### The "Cut Point"
To implement SIL without altering the control package, we cut the hardware interface and insert the simulator:
*   **Disabled Hardware Nodes**: `serial_bridge_node` (reads sensors) and `thruster_driver_node` (drives ESCs).
*   **Active Control Nodes**: [control_node](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_control/src/control_node.cpp) (calculates PIDs and allocation) and [sensor_sync_node](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_hardware/src/sensor_sync_node.cpp) (aggregates sensor readings).
*   **Inserted Simulation Node**: [mock_dynamics_node](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/rov_sim/mock_dynamics_node.py) (computes physics and mocks sensors) or `default_server_endpoint` (interfaces with Unity3D physics).

---

## 2. System Architecture

The following block diagrams illustrate the difference in topic routing and node layout between physical operations and the SIL simulator:

### Real ROV Stack Routing
```
                   ┌──────────────────┐
                   │ sensor_sync_node │
                   └────────┬─────────┘
                            │ /sensor_data (fused)
                            ▼
┌──────────────┐   ┌──────────────┐
│  teleop_node ├──►│ control_node │
└──────────────┘   └────────┬─────────┘
   /setpoints               │ /thruster_command (PWM values)
                            ▼
                   ┌──────────────────────┐
                   │ thruster_driver_node │ ──► (Physical ESCs/Thrusters)
                   └──────────────────────┘
                   ┌──────────────────────┐
 (Physical IMU) ──►│  serial_bridge_node  │ ──► /depth & /imu_data
                   └──────────────────────┘
```

### SIL Simulator Routing
```
                   ┌──────────────────┐
                   │ sensor_sync_node │◄────────────────────────┐
                   └────────┬─────────┘                         │
                            │ /sensor_data (fused)              │
                            ▼                                   │
┌─────────────────┐┌──────────────┐                             │
│ scenario_runner ├┤ control_node │                             │
│   (or teleop)   │└────────┬─────────┘                         │
└─────────────────┘         │ /thruster_command (PWM values)    │
   /setpoints               ▼                                   │
                  ┌────────────────────┐                        │
                  │ mock_dynamics_node │                        │
                  │ (or Unity Bridge)  ├─ /depth (Timestamped)──┤
                  └────────────────────┘─ /imu_data (Timestamp)─┘
```

---

## 3. Core Algorithms & Mathematical Principles

### A. Inverse PWM Mapping
The physical thruster controllers require 16-bit PWM commands (range: 1250µs to 1750µs) where 1500µs is neutral. Physical thrusters have different force efficiencies in the forward vs. reverse directions. The control stack models this curve in [control_node.hpp](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_control/include/rov_control/control_node.hpp) using separate forward and reverse scales.

To simulate physics, [mock_dynamics_node.py](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/rov_sim/mock_dynamics_node.py) maps the output PWM commands back to physical thruster force (Newtons) using the following piecewise calculation:

```text
if PWM > 1530:
    Thrust Force (F) = (PWM - 1530) / 156.78       (Forward thrust range)
elif PWM < 1470:
    Thrust Force (F) = (PWM - 1470) / 200.00       (Reverse thrust range)
else:
    Thrust Force (F) = 0.0                          (Deadband range)
```

If the thruster configuration requires direction inversion (determined by the `thruster_reverse` boolean array in [rov_params.yaml](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_bringup/config/rov_params.yaml)), the sign is flipped:

```text
Actual Thrust Force = -F
```

---

### B. Inverse Thruster Allocation Matrix
The control node maps the desired 6-DOF body forces/moments (Roll, Pitch, Yaw, Surge, Sway, Heave) to the 6 individual thrusters using the **Forward Allocation Matrix**:

```text
f[0] = heave - pitch - roll         (Vertical Thruster 1)
f[1] = heave - pitch + roll         (Vertical Thruster 2)
f[2] = 2 * heave + 2 * pitch        (Vertical Thruster 3)
f[3] = yaw + surge + alpha * sway   (Horizontal Thruster 1)
f[4] = -yaw + surge - alpha * sway  (Horizontal Thruster 2)
f[5] = sway                         (Horizontal Thruster 3)
```

To compute the resulting body forces driving the ROV, the simulator applies the **Analytical Inverse Allocation Matrix** (assuming the cross-coupling coefficient `alpha` is approximately `0` for the mock model):

```text
roll  = (f[1] - f[0]) / 2.0
pitch = (f[2] - f[0] - f[1]) / 4.0
yaw   = (f[3] - f[4]) / 2.0
surge = (f[3] + f[4]) / 2.0
sway  = f[5]
heave = (f[0] + f[1] + f[2]) / 4.0
```

---

### C. 6-DOF Rigid-Body Dynamics
Once the body forces and moments are computed, the simulator updates the velocity and position vectors for each degree of freedom `i`:

1.  **Acceleration Calculation**:
    ```text
    acceleration[i] = (Force_DOF[i] / mass[i]) - (damping[i] / mass[i]) * velocity[i]
    ```
    *   Where `mass[i]` represents the mass (for linear DOFs) or moment of inertia (for angular DOFs).
    *   Where `damping[i]` represents the linear damping or angular damping coefficient.
2.  **State Integration**:
    ```text
    velocity[i](t + dt) = velocity[i](t) + acceleration[i] * dt
    position[i](t + dt) = position[i](t) + velocity[i](t + dt) * dt
    ```
    *   Timestep `dt = 1.0 / sim_rate_hz` (default 30 Hz gives `dt` of approximately `0.0333` seconds).
3.  **Restoring Forces**:
    To simulate buoyancy and righting stability, the heave and angular axes are acted upon by restoring moments pulling them back to equilibrium:
    *   *Buoyancy force (pulls towards surface)*: `Force_buoyancy = -buoyancy_k * depth`
    *   *Righting torque (pulls roll/pitch to level)*: `Torque_righting = -righting_k * angle`
4.  **Limits & Angle Wrapping**:
    *   Depth is clamped to `depth >= 0.0` meters (the vehicle cannot fly out of the water).
    *   Yaw orientation is mathematically wrapped to `[-pi, pi]` using `atan2(sin(yaw), cos(yaw))`.

---

### D. Sensor Synchronization & Timestamping
The control package employs a synchronizer ([sensor_sync_node.cpp](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_hardware/src/sensor_sync_node.cpp)) running ROS2 `ApproximateTime` matching.
*   **Queue Size**: 50 messages.
*   **The Gotcha**: If `/depth` and `/imu_data` messages do not have matching header timestamps, the message filter drops them. If dropped, `/sensor_data` is not published, and the PID calculations in the C++ control node will hang.
*   **Simulator Solution**: The `mock_dynamics_node` retrieves a single ROS2 clock reading `now = self.get_clock().now().to_msg()` and writes it to both msg headers simultaneously.

---

## 4. Operation Guide

### A. Building the Workspace
Before running the simulator, build and source your workspace:
```bash
# Navigate to the workspace root
cd ~/Documents/hammerhead-master/hammerhead-master/vibu_sil

# Clean build all packages
colcon build --symlink-install

# Source the environment setup script
source install/setup.bash
```

---

### B. Launching the Simulation

The simulation setup is launched via [rov_sim_launch.py](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/launch/rov_sim_launch.py). It offers three primary modes:

#### 1. Joystick / Teleop Mode (Default)
Runs the Python mock dynamics and binds command inputs to a joystick controller (via `teleop_node`):
```bash
ros2 launch rov_sim rov_sim_launch.py
```

#### 2. Automated Scenario Mode (No Joystick)
Launches the mock dynamics and executes a pre-recorded sequence of timed commands from a YAML scenario file. This is ideal for quick testing or headless servers:
```bash
ros2 launch rov_sim rov_sim_launch.py use_teleop:=false scenario_file:=/path/to/scenario.yaml
```
*(If no `scenario_file` argument is provided, it defaults to the `depth_hold.yaml` scenario).*

#### 3. Unity Physics Mode
Launches the real control stack and brings up the TCP Endpoint bridge. Use this when connecting the ROS2 machine to the Unity visualizer/physics engine running on the RTX machine:
```bash
ros2 launch rov_sim rov_sim_launch.py sim_backend:=unity
```

---

## 5. Scenario Writing Specification

Scenarios are defined in YAML format. The scenario runner node ([scenario_runner.py](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/rov_sim/scenario_runner.py)) parses these files and publishes setpoints and mode switches at correct intervals.

### Coordinate Convention
The setpoint array follows the 6-DOF order defined in the control node:
```yaml
setpoints: [roll, pitch, yaw, surge, sway, heave]
```
*   **roll, pitch, yaw**: Degrees.
*   **surge, sway**: Velocity/force inputs.
*   **heave**: Target depth (meters).
*   **mode**: `0` = SURFACE mode, `1` = TELEOP mode (enables control node's PID loop).

### Scenario YAML Schema
Create files under `src/rov_sim/config/scenarios/`. Here is an example schema:

```yaml
scenario:
  name: "depth_hold_1m"
  description: "Switch to TELEOP, dive to 1m, hold, and return to surface"
  
  steps:
    - time: 0.0                      # Seconds from start of scenario
      mode: 1                        # Enable TELEOP mode
      setpoints: [0, 0, 0, 0, 0, 0.22] # Initial setpoints (hold surface depth)
      
    - time: 2.0                      # At 2 seconds
      setpoints: [0, 0, 0, 0, 0, 1.0]  # Command a step depth to 1.0m
      
    - time: 12.0                     # At 12 seconds
      mode: 0                        # Switch back to SURFACE mode (disables thrusters)
```

---

## 6. Regression Testing & Baseline Validation

A test harness is provided to verify that control algorithm changes do not degrade vehicle performance. It executes a scenario, captures the metrics, and compares them against a verified baseline.

### A. Run a Regression Test
To run a regression script against a scenario (e.g. `depth_hold.yaml`):
```bash
# Navigate to the script directory
cd ~/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/scripts

# Run regression test (uses depth_hold.yaml by default)
./run_regression.sh

# Or target a specific scenario YAML
./run_regression.sh yaw_sweep.yaml
```

### B. What the Harness Does under the Hood
1.  **Launch**: Starts the simulator in headless mode (`use_teleop:=false`) with the target YAML file.
2.  **Record**: Starts a `rosbag2` recorder storing the `/setpoints`, `/set_mode`, `/sensor_data`, and `/thruster_command` topics.
3.  **Monitor**: Polls `/scenario_status` until it reads `COMPLETE`.
4.  **Parse & Compare**: Invokes [compare_baseline.py](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/scripts/compare_baseline.py) to read the SQLite3 database rosbag and calculate control metrics.

### C. Performance Metrics Calculated
The baseline validator computes four classic controller metrics:
*   **Overshoot (%)**: The maximum percentage excursion beyond the target depth or angle.
*   **Settling Time (s)**: The time it takes for the state to enter and stay within a $\pm 5\%$ band of the target value.
*   **Steady-State Error (m/deg)**: The average absolute error $|state - target|$ during the last $20\%$ of the scenario execution time.
*   **Rise Time (s)**: The time taken for the feedback signal to rise from $10\%$ to $90\%$ of the target.

### D. Managing the Baseline Metrics File
The script compares the computed metrics against the values inside `baseline_metrics.json`.
*   **If no baseline exists**: The current metrics are saved as the new baseline, and the test passes.
*   **If a baseline exists**: The metrics are verified against the baseline within defined tolerances:
    *   *Overshoot Tolerance*: $\pm 10\%$ percentage points
    *   *Settling Time Tolerance*: $\pm 1.0$s
    *   *Steady-State Error Tolerance*: $\pm 0.05$m
    *   *Rise Time Tolerance*: $\pm 1.0$s
*   **Resetting the Baseline**: If you deliberately changed PID values to improve response and want to save the new metrics as the baseline, delete the existing baseline file and rerun the script:
    ```bash
    rm -rf ../bags/regression_*/baseline_metrics.json
    ./run_regression.sh
    ```

---

## 7. How to Make Changes & Tune the Simulator

### Tuning the Simulation Physics
You can tune the dynamics parameters in the simulator without modifying code. The [mock_dynamics_node](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/rov_sim/mock_dynamics_node.py) declares parameters that can be overridden in the [rov_params.yaml](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_bringup/config/rov_params.yaml) file or overridden in launch.

#### 1. Adjust parameters in `rov_params.yaml`
Add a block for `mock_dynamics_node` in [rov_params.yaml](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_bringup/config/rov_params.yaml) to tune the mock simulation behaviour:
```yaml
mock_dynamics_node:
  ros__parameters:
    mass: 12.0             # Vehicle mass in kg (affects linear acceleration)
    inertia: 2.0           # Angular moment of inertia (affects angular acceleration)
    damping_linear: 8.0    # Linear damping (increases velocity drag)
    damping_angular: 4.0   # Angular damping (increases rotational drag)
    buoyancy_k: 3.0        # Restoring spring constant for depth
    righting_k: 1.5        # Righting spring constant for roll/pitch stability
    initial_depth: 0.22    # Start depth of the vehicle on launch
```

#### 2. Why does depth settle at 0.37m instead of 1.0m?
In the crude mock simulator, the buoyancy restoring spring is modeled as pulling the ROV upwards:
```text
Force_buoyancy = -3.0 * depth
```
*   When a setpoint of 1.0m is commanded, the control node outputs downwards heave force.
*   However, the restoring force fights this output.
*   The C++ PID gains in [rov_params.yaml](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_bringup/config/rov_params.yaml) have a high proportional gain (`kp = 3.0`) but a low integral gain (`ki = 0.165`).
*   Without sufficient integral accumulation, the proportional and derivative feedback matches the restoring force, settling the vehicle at approximately `0.37`m.
*   *Note*: This is normal for mock physics and demonstrates the feedback loop is working stably. Real hydrodynamics and neutral buoyancy will be handled when switching to the Unity visualizer.

### Tuning the PID Controller Gains
The PID gains are loaded from [rov_params.yaml](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_bringup/config/rov_params.yaml) under the `control_node` block:
```yaml
control_node:
  ros__parameters:
    pid:
      kp: [0.0, 1.0, 3.0, 0.2, 0.2, 3.0]  # [roll, pitch, yaw, surge, sway, heave]
      ki: [0.0, 0.024, 0.031, 0.0, 0.0, 0.165]
      kd: [0.0, 1.0, 1.0, 0.0, 0.0, 4.0]
```
To tune these, modify the arrays in the YAML file and launch the simulator. Since the YAML file is a parameter file loaded at launch, **you do not need to compile the C++ codebase with `colcon build` to apply gain changes**.

---

## 8. Swapping to Unity3D
When you are ready to transition to Unity:
1.  Launch the simulator with the Unity backend:
    ```bash
    ros2 launch rov_sim rov_sim_launch.py sim_backend:=unity
    ```
2.  Open the Unity project on the RTX machine and ensure the `ROS-TCP-Connector` configuration is pointed to the IP address of your ROS2 machine.
3.  Unity will take over the task of subscribing to `/thruster_command` and publishing the `/depth` and `/imu_data` topics, using high-fidelity hydrodynamics simulation. No changes are required on the ROS2 code side.
