# final_version_no_slam Branch Description

This branch contains the code for the nodes that were planned to be used for autonomous navigation **without SLAM**, as well as some work related to the integration of **visual navigation with SLAM**.

The objective and functionality of each node are described below.

---

# Nodes Description

## aruco_control.py

**Objective**

Move the drone toward the ArUco marker of the whiteboard mission to a safe distance in order to later draw on the whiteboard.

**Operation**

ArUco detections are read and the errors are calculated to move to a specified distance from the marker. These distances are defined in the `__init__` function of the `ArucoControlNode` class as:
self.x_distance
self.y_distance
self.z_distance


**Navigation Type**

Visual navigation.

---

## cross_tunnel.py

**Objective**

Cross the selected tunnel.

**Operation**

The error from the `/tunnel_error` topic is read to center the drone in the tunnel and the required velocities are sent to cross it.

**Navigation Type**

Visual navigation or SLAM + visual navigation.

---

## cross_tunnel_smach.py

**Objective**

Same objective as `cross_tunnel.py` but implemented as a SMACH state.

**Operation**

Same operation as `cross_tunnel.py`, but including the exit condition of the state.

**Navigation Type**

Visual navigation or SLAM + visual navigation.

---

## fly_drone.py

**Objective**

Move the drone at a specified velocity in **x, y, z, and yaw** for a defined amount of time.

**Operation**

The desired velocity and flight time are specified and then published.

**Note**

This node was created for the version that was planned to be presented at the last minute for **IMAV25**, mainly to advance a specified distance in a simple but easy-to-implement way.

**Navigation Type**

Visual navigation.

---

## goto_whiteboard.py

**Objective**

Approach the whiteboard using a neural network.

**Operation**

The error from the `/whiteboard_error` topic is read to center the drone in **x** and **y** relative to the whiteboard, and the required velocities are sent to center the drone while advancing forward at a constant speed.

**Navigation Type**

Visual navigation.

---

## goto_zone.py

**Objective**

Move toward a specific zone of the flight arena.

**Operation**

**Summary sequence**

1. Nav2 is activated  
2. Three parameters are received: `x`, `y`, `yaw`  
3. The generated map is used to navigate toward the specified **x-y position** with the orientation given by the received parameters.

**Navigation Type**

SLAM + visual navigation.

**Note**

This node was intended to be used as a state to move to a specific zone of the arena using the generated map and then switch to mission-specific states.

---

## indoor_smach.py

**Objective**

State sequence structure used to execute the missions.

**Navigation Type**

Visual navigation or SLAM + visual navigation.

**Note**

The current code represents the initial structure planned for **SLAM + visual navigation**.

---

## landing_platform.py

**Objective**

Landing on the platform (without platform movement).

**Operation**

The error from the `/platform_error` topic is read to center the drone on the platform and the required velocities are sent to perform the landing.

**Navigation Type**

Visual navigation or SLAM + visual navigation.

---

## move_drone.py

**Objective**

Move the drone in simulation using a **PS4 controller**.

**Navigation Type**

Manual control in simulation.

**Note**

To use this node, it is necessary to configure which buttons control **roll, pitch, yaw, and z movement**.

To do this, check the values published in the `/joy` topic when the controller is connected and the joysticks are moved.

---

## platform_detect.py

**Objective**

Platform detection using a neural network.

**Operation**

The neural network is used to detect the platform and calculate the error, which is then published to the `/platform_error` topic.

This node works in **simulation**. To run it on the real drone, modifications are required to execute the network using the **OAK camera or a Raspberry Pi**.

**Navigation Type**

Visual navigation or SLAM + visual navigation, but currently only functional in simulation.

---

## start_msg.py

**Objective**

Initial state where nothing happens but the state machine is started.

**Operation**

The node subscribes to the `/wait_start_msg` topic and waits for an **Empty message** to exit the initial state and begin the sequence.

**Navigation Type**

SLAM, visual, or visual + SLAM navigation.

---

## take_photos.py

**Objective**

Capture images.

**Operation**

Frames from the selected camera (check the subscriber) are saved at a defined interval specified by `save_interval`.

**Navigation Type**

Manual navigation in simulation or real drone operation.

Note: The subscriber may change depending on the camera used or whether it is running in simulation or on the real drone.

---

## tunnel_detect.py

**Objective**

Tunnel detection using a neural network.

**Operation**

The neural network detects the desired tunnel and calculates the error, which is then published to the `/tunnel_error` topic.

This node works in **simulation**. To run it on the real drone, modifications are required to execute the network using the **OAK camera or a Raspberry Pi**.

**Navigation Type**

Visual navigation or SLAM + visual navigation, but currently only functional in simulation.

---

## whiteboard_detect.py

**Objective**

Whiteboard detection using a neural network.

**Operation**

The neural network detects the whiteboard and calculates the error, which is then published to the `/whiteboard_error` topic.

This node works in **simulation**. To run it on the real drone, modifications are required to execute the network using the **OAK camera or a Raspberry Pi**.

**Navigation Type**

Visual navigation, currently only functional in simulation.

---

# Important Notes

- The nodes `platform_detect`, `tunnel_detect`, and `whiteboard_detect` must be executed in the directory where the **ONNX neural network files** are located.
- Therefore, `simulator.launch.py` must also be executed in the folder containing the ONNX files.
- The simulation ONNX files can be found in the **final_version_no_slam** branch of this repository.
- Some changes in this branch may not be correct due to the context in which the code was developed.
- For any questions, contact **Lucy Carmona** or **Jair Aguilar**.