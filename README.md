# Flightmare Policy Inference in PX4–Gazebo

**ROS 2 and C++/Python implementations for running Flightmare-trained quadrotor reinforcement-learning policies in a separate PX4–Gazebo simulation.**

This repository is part of the [Robust Goal-Directed Aerial Manipulation project](https://github.com/1shit4/aerial-manipulation-rl-control). The associated [Flightmare training repository](https://github.com/1shit4/flightmare) contains quadrotor RL environments and policy-training code. Here, the focus is **sim-to-sim policy execution**: reconstructing a policy's observations from PX4 feedback, running an exported model, and converting its actions into motor commands for the simulated vehicle.

## Flightmare-to-PX4 Control Pipeline

A policy trained in Flightmare cannot be transferred to PX4–Gazebo simply by loading its weights. Its observations must retain the training convention (coordinate frames, scaling, and ordering), and its actions must be converted into commands accepted by the PX4-equipped vehicle.

The primary C++ implementation follows this pipeline:

```text
PX4–Gazebo vehicle state (position, attitude, velocity, body rates)
                       |
                       v
        Frame conversion and observation assembly
                       |
                       v
           Flightmare-trained ONNX policy
                       |
                       v
         Collective-thrust + body-rate references
                       |
                       v
           Angular-rate feedback controller
                       |
                       v
             Desired body torques + thrust
                       |
                       v
          Quadrotor motor-allocation model
                       |
                       v
             Four normalized motor commands
                       |
                       v
              PX4 direct-actuator interface
                       |
                       v
                 Gazebo quadrotor
```
The Flightmare policy outputs four control values: one collective-thrust reference and three desired body rates. These are converted into motor commands through the implemented rate controller and motor-allocation model.
The external controller handles rate tracking and motor allocation, so the **C++ direct-actuator path bypasses PX4's normal angular-rate controller and control allocator**. PX4 supplies state feedback, offboard-mode handling, and the vehicle/simulator interface.

## Policy Observations and Actions

### Reconstructing the Flightmare observation

The main C++ pipeline uses the **19-dimensional observation** defined in the Flightmare quadrotor environment:

| Observation | Dimensions |
| --- | ---: |
| Vehicle position | 3 |
| Attitude as a flattened 3 × 3 rotation matrix | 9 |
| Linear velocity | 3 |
| Body angular velocity | 3 |
| Accumulated yaw-error term | 1 |
| **Total** | **19** |

The position and linear-velocity components are scaled to match the training environment. Vehicle pose and angular-rate measurements from PX4 are converted into the corresponding observation convention before inference. Matching the attitude representation, axis conventions, and scaling is essential when moving the policy between simulators.

## Implementations

### C++ inference and motor control

The [`drone_test_cpp`](src/drone_test_cpp/) package contains the main C++ inference pipeline:

- [`test_dron_node.cpp`](src/drone_test_cpp/src/test_dron_node.cpp): assembles observations, runs the policy, computes rate-feedback torque, and maps thrust/torque demands to motor commands.
- [`model.cpp`](src/drone_test_cpp/src/model.cpp): performs ONNX Runtime inference using an exported Flightmare policy.
- [`vehicle.cpp`](src/drone_test_cpp/src/vehicle.cpp): handles PX4 state feedback, offboard control, vehicle arming, and motor-command publication.

The source uses separate timers for policy updates (**20 ms**) and the motor-control loop (**1 ms**).

### Python inference experiments

The [`test_drone`](src/test_drone/) package contains earlier and alternative inference experiments, including a **direct-motor policy interface**. Its active direct-motor function reconstructs an **18-dimensional state observation**, runs an ONNX model, and maps four policy outputs to normalized motor commands. This is a different policy/action configuration from the 19-observation, thrust-and-body-rate C++ pipeline.

Relevant files include [`test_drone.py`](src/test_drone/scripts/test_drone.py), [`model.py`](src/test_drone/scripts/model.py), and [`vehicle.py`](src/test_drone/scripts/vehicle.py). Additional scripts cover coordinate transformations and actuator-allocation checks.

## Evaluation

This workspace was used to test **Flightmare-trained quadrotor policies in PX4–Gazebo**, including hovering and goal-directed position control. The purpose was to assess execution in a different simulator and flight-control interface from the environment used for training.

The repository preserves the policy-inference and control code rather than a consolidated quantitative benchmark. Policy training, reward design, and trajectory-generation experiments are documented in the [Flightmare repository](https://github.com/1shit4/flightmare).

## Repository Structure

| Path | Contents |
| --- | --- |
| [`src/drone_test_cpp/`](src/drone_test_cpp/) | C++ ONNX inference, rate-feedback control, motor allocation, and PX4 interface |
| [`src/test_drone/`](src/test_drone/) | Python ONNX inference, alternative direct-motor experiments, and frame-conversion utilities |
| [`src/px4_msgs/`](src/px4_msgs/) | PX4 ROS 2 message definitions (upstream dependency) |
| [`environment.yml`](environment.yml) | Python environment specification |

## Running the C++ Interface

The code uses **ROS 2, PX4 SITL, Gazebo, Eigen, ONNX Runtime, and `px4_msgs`**. A compatible PX4–Gazebo vehicle and PX4-to-ROS 2 bridge are required.

Before building, update the local ONNX Runtime location in [`CMakeLists.txt`](src/drone_test_cpp/CMakeLists.txt) and the exported policy path in [`test_dron_node.cpp`](src/drone_test_cpp/src/test_dron_node.cpp). The supplied source retains machine-specific paths from the development setup.

With the ROS 2 and PX4 dependencies configured, build the package from the workspace root:

```bash
colcon build --packages-up-to drone_test_cpp
source install/setup.bash
```

Start the matching PX4–Gazebo simulation and ROS 2 bridge, then launch the inference node:

```bash
ros2 run drone_test_cpp drone_test
```

Vehicle parameters, motor ordering, controller gains, and model normalization should match the simulator and exported policy used for evaluation.

## Related Repositories

- [Aerial Manipulation — Project Overview](https://github.com/1shit4/aerial-manipulation-rl-control): research objective and complete repository index.
- [Flightmare Training](https://github.com/1shit4/flightmare): quadrotor RL environment, PPO training, policy evaluation, and ONNX export.
- [Quadrotor Inner-Loop Control](https://github.com/1shit4/quadcopter_inner_loop_control): separate disturbance-estimating, ASMC, and Adaptive INDI control experiments for aerial manipulation.

*Research conducted at the Robotics Research Center, IIIT Hyderabad. PX4 message definitions and the underlying Flightmare/PX4/Gazebo frameworks are credited to their respective upstream projects.*
