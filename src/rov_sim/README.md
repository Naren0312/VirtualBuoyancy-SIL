# rov_sim

This ROS2 package contains the Software-in-the-Loop (SIL) simulation stack for the Hammerhead ROV.

## Directory Structure

*   [config/scenarios/](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/config/scenarios/): YAML scenario configurations for repeatable tests.
*   [launch/](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/launch/): Startup launch scripts.
*   [rov_sim/](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/rov_sim/): Python source nodes:
    *   [mock_dynamics_node.py](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/rov_sim/mock_dynamics_node.py): Temporary standalone 6-DOF physics simulator.
    *   [scenario_runner.py](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/rov_sim/scenario_runner.py): YAML scenario executor.
*   [scripts/](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/scripts/): Automated regression testing scripts.

## Detailed Documentation

For a full explanation of the SIL concept, system architecture, core equations, operation instructions, and tuning guides, please refer to the master documentation at the root of the workspace:
*   [SIMULATOR_GUIDE.md](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/SIMULATOR_GUIDE.md)
