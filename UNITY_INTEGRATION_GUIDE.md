# Hammerhead ROV — Unity3D Integration Guide

This document outlines the next steps to transition from the Python-based mock dynamics simulator to a high-fidelity **Unity3D visualizer and physics engine**.

When you run the ROS2 launch file with the `sim_backend:=unity` parameter, the C++ control stack runs on your ROS2 machine and communicates over TCP with Unity running on another machine (e.g. your RTX visualizer rig).

---

## 1. Prerequisites & Unity Setup

### A. Install Unity Hub & Unity Editor
*   **Recommended Version**: Unity 2022.3 LTS (Long Term Support).
*   Add the **Linux Build Support** and **Windows Build Support** modules if you plan to build standalone simulation executables later.

### B. Add the ROS-TCP-Connector package
Unity communicates with the ROS2 workspace using the official **Unity Robotics Hub** packages.
1.  Open your Unity Project.
2.  Go to `Window` ➔ `Package Manager`.
3.  Click the `+` button in the top-left corner and select **Add package from git URL...**
4.  Enter the URL:
    ```
    https://github.com/Unity-Technologies/URDF-Importer.git
    ```
5.  Click `+` again and add the ROS-TCP-Connector:
    ```
    https://github.com/Unity-Technologies/ROS-TCP-Connector.git
    ```

### C. Configure Connection Settings
1.  In Unity, navigate to the top menu: `Robotics` ➔ `ROS Settings`.
2.  Configure the settings in the Inspector panel:
    *   **ROS IP Address**: The IP address of your ROS2 machine. (If running Unity on the same machine/WSL, use `127.0.0.1`).
    *   **ROS Port**: `10000` (this matches the port exposed by [src/ROS-TCP-Endpoint/](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/ROS-TCP-Endpoint)).
    *   **Protocol**: `ROS2`.

---

## 2. Importing Custom ROS2 Messages

Unity needs to understand the custom message structures defined in your workspace. The ROS-TCP-Connector includes a tool to compile `.msg` files into C# classes.

1.  In the Unity menu, go to `Robotics` ➔ `Generate Messages...`.
2.  Set the **Source Paths** to your workspace package:
    *   Path: `~/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_msgs/`
3.  The window will display the available messages:
    *   [Depth.msg](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_msgs/msg/Depth.msg)
    *   [IMUData.msg](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_msgs/msg/IMUData.msg)
    *   [ThrusterCommand.msg](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_msgs/msg/ThrusterCommand.msg)
4.  Click **Build** next to the messages (or click **Build All**).
5.  Unity will generate C# message classes inside your project asset folder under `Assets/RosMessages/RovMsgs/`.

---

## 3. Creating the ROS2 Communication Scripts in Unity

You will need a script in Unity to subscribe to control commands and publish sensor feedback. Below are template code structures for the C# scripts.

### A. Subscriber: Reading Thruster Commands (`ROSThrusterSubscriber.cs`)
Attach this script to your main ROV GameObject. It listens to `/thruster_command` and stores the incoming PWM and direction values:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.RovMsgs; // Custom namespace built by the message generator

public class ROSThrusterSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "/thruster_command";

    // Cache latest command data
    public short[] latestPWM = new short[6] { 1500, 1500, 1500, 1500, 1500, 1500 };
    public bool[] latestReverse = new bool[6] { false, false, false, false, false, false };

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ThrusterCommandMsg>(topicName, OnThrusterCommandReceived);
    }

    void OnThrusterCommandReceived(ThrusterCommandMsg msg)
    {
        for (int i = 0; i < 6; i++)
        {
            latestPWM[i] = msg.pwm[i];
            latestReverse[i] = msg.reverse[i];
        }
    }
}
```

### B. Publisher: Writing Sensor Feedback (`ROSSensorPublisher.cs`)
Attach this script to publish `/depth` and `/imu_data` to the ROS2 stack.

> [!IMPORTANT]
> **ApproximateTime Stamp Synchronization**: Both messages must share the **exact same header timestamp** to satisfy the `ApproximateTime` message filter inside [sensor_sync_node.cpp](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_hardware/src/sensor_sync_node.cpp). If their timestamps differ, the synchronizer will drop them, and the controller will hang.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.RovMsgs;
using RosMessageTypes.Std; // For Header and Time messages

public class ROSSensorPublisher : MonoBehaviour
{
    private ROSConnection ros;
    public string depthTopic = "/depth";
    public string imuTopic = "/imu_data";
    public string frameId = "base_link";

    public Rigidbody rovRigidbody;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<DepthMsg>(depthTopic);
        ros.RegisterPublisher<IMUDataMsg>(imuTopic);
    }

    void FixedUpdate()
    {
        // 1. Generate a single synchronized timestamp
        double timeSec = Time.timeAsDouble;
        uint sec = (uint)System.Math.Truncate(timeSec);
        uint nanosec = (uint)((timeSec - sec) * 1e9);

        HeaderMsg commonHeader = new HeaderMsg
        {
            stamp = new TimeMsg { sec = sec, nanosec = nanosec },
            frame_id = frameId
        };

        // 2. Publish Depth (Assume Unity Y-axis or Z-axis is depth below water surface)
        // Adjust coordinate transformations based on your scene setup.
        float currentDepth = Mathf.Max(0.0f, -transform.position.y); 

        DepthMsg depthMsg = new DepthMsg
        {
            header = commonHeader,
            depth = currentDepth
        };
        ros.Publish(depthTopic, depthMsg);

        // 3. Publish IMU Data
        // Convert Unity orientation (Euler degrees) to match the real IMU expectations.
        // Euler angles in degrees: [Roll, Pitch, Yaw]
        Vector3 rot = transform.rotation.eulerAngles;
        
        // Wrap Yaw to [-180, 180] or [0, 360] to match physical sensor output
        float yaw = rot.y > 180.0f ? rot.y - 360.0f : rot.y;
        float pitch = rot.x > 180.0f ? rot.x - 360.0f : rot.x;
        float roll = rot.z > 180.0f ? rot.z - 360.0f : rot.z;

        // Extract linear acceleration in local coordinates
        Vector3 localAccel = transform.InverseTransformDirection(rovRigidbody.linearAcceleration);

        IMUDataMsg imuMsg = new IMUDataMsg
        {
            header = commonHeader,
            orientation = new float[3] { roll, pitch, yaw },
            acceleration = new float[3] { localAccel.x, localAccel.y, localAccel.z }
        };
        ros.Publish(imuTopic, imuMsg);
    }
}
```

---

## 4. Physics: Force Allocation & Thruster Mechanics

In your main Unity physics script (e.g. `ROVPhysics.cs`), you must map the PWM values back to physical forces and apply them to the ROV Rigidbody.

### A. Implement PWM-to-Thrust Forces
Each thruster is represented by a local position and direction vector in the ROV's coordinate frame. Retrieve the constants from [rov_params.yaml](file:///home/naren/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_bringup/config/rov_params.yaml) to ensure parity:

```csharp
public class ROVPhysics : MonoBehaviour
{
    public Rigidbody rb;
    public ROSThrusterSubscriber commandSub;

    // Thruster transform references inside Unity
    public Transform[] thrusterAnchors = new Transform[6]; 

    // Mapping Constants
    private const float FwdScale = 370.0f / 2.36f;
    private const int FwdOffset = 1530;
    private const float RevScale = 370.0f / 1.85f;
    private const int RevOffset = 1470;

    // Reverse flags from rov_params.yaml
    public bool[] thrusterReverseFlags = new bool[6] { false, false, false, true, true, false };

    void FixedUpdate()
    {
        for (int i = 0; i < 6; i++)
        {
            int pwm = commandSub.latestPWM[i];
            bool driverReverse = commandSub.latestReverse[i];

            // 1. Calculate force magnitude from PWM
            float forceMag = 0.0f;
            if (pwm > FwdOffset)
            {
                forceMag = (pwm - FwdOffset) / FwdScale;
            }
            else if (pwm < RevOffset)
            {
                forceMag = (pwm - RevOffset) / RevScale;
            }

            // 2. Apply direction inversions
            if (thrusterReverseFlags[i]) forceMag = -forceMag;
            if (driverReverse) forceMag = -forceMag;

            // 3. Apply force vector to the Rigidbody at the thruster location
            // The force is applied along the forward axis of the thruster anchor
            Vector3 forceVector = thrusterAnchors[i].forward * forceMag;
            rb.AddForceAtPosition(forceVector, thrusterAnchors[i].position, ForceMode.Force);
        }
    }
}
```

---

## 5. Simulating Hydrodynamics in Unity

To replace the crude mass-damping physics used by the mock node, Unity should simulate realistic water mechanics:

### A. Buoyancy
1.  Define the water level (e.g. $Y = 0.0\text{m}$).
2.  If the ROV's depth goes below the water level, apply an upward force:
    $$F_{\text{buoyancy}} = \rho \cdot V \cdot g$$
    *   Where $\rho$ is the water density.
    *   Where $V$ is the submerged volume.
3.  **Stability tip**: Apply buoyancy at the **Center of Buoyancy (CoB)** (usually slightly above the Center of Mass (CoM)). This naturally creates righting moments that keep the ROV level, eliminating the need for the artificial spring stiffness (`righting_k`) used in the mock node.

### B. Drag (Fluid Resistance)
Use Unity's built-in Rigidbody Drag properties or write a simple drag script to oppose motion:
*   **Linear Drag**: $\vec{F}_{\text{drag}} = -C_d \cdot \vec{v} \cdot |\vec{v}|$ (quadratic drag) or simple linear damping.
*   **Angular Drag**: $\vec{\tau}_{\text{drag}} = -C_{\omega} \cdot \vec{\omega} \cdot |\vec{\omega}|$ to damp rotation.

---

## 6. How to Test & Verify the Connection

Once the Unity scripts are ready, verify the loop is closed:

1.  **Launch ROS2 in Unity Mode**:
    ```bash
    ros2 launch rov_sim rov_sim_launch.py sim_backend:=unity use_teleop:=false
    ```
2.  **Start Unity**:
    Press **Play** in the Unity Editor.
3.  **Monitor the Console**:
    *   Verify the ROS-TCP-Connector icon in the Unity HUD shows a active, green connection.
    *   Open a new terminal on the ROS2 machine and run:
        ```bash
        ros2 topic hz /sensor_data
        ```
        If `/sensor_data` is publishing at ~30 Hz, the Unity sensors are being successfully received and synchronized with the control loop.
4.  **Execute a Scenario / Regression Test**:
    Run a test scenario to witness the ROV move in 3D:
    ```bash
    cd ~/Documents/hammerhead-master/hammerhead-master/vibu_sil/src/rov_sim/scripts
    ./run_regression.sh depth_hold.yaml
    ```
    Watch the ROV visualizer drop down, stabilize at the target depth, and return to the surface.
