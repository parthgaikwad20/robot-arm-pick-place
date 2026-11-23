# Robot Arm Pick-and-Place (ROS 2 Humble)

A simulated 4-DOF robot arm with a two-finger gripper that picks up a cube and
places it at another spot, then carries it back, on a loop. It runs entirely in
the browser via GitHub Codespaces — nothing to install.

![idea](https://img.shields.io/badge/ROS%202-Humble-blue) ![lang](https://img.shields.io/badge/Python-3.10-green)

## What it shows

| Concept | Where |
|---|---|
| Robot modelled in URDF (4 revolute + 2 prismatic joints) | `src/arm_pick_place/urdf/arm.urdf` |
| Closed-form **inverse kinematics** (base yaw + planar 2-link, gripper kept vertical) | `arm_pick_place/kinematics.py` |
| Straight-line **Cartesian motion** with smooth (S-curve) velocity, IK solved at 50 Hz | `pick_place_node.py` |
| Pick-and-place **state machine**: approach → descend → grasp → lift → carry → release → retreat | `pick_place_node.py` |
| ROS 2 topics, parameters, launch files, TF via `robot_state_publisher`, RViz markers | `launch/`, `config/` |
| Unit tests for the IK (FK round-trip, reachability) | `test/test_kinematics.py` |

```
 pick_place node ──/joint_states──► robot_state_publisher ──/tf──► RViz2
        │                                                           ▲
        └────────────── /pick_place/markers (cube, pads) ───────────┘
        └────────────── /pick_place/status  (current phase)
```

## Run it online (GitHub Codespaces)

1. Create a new GitHub repository and upload this whole folder (keep the `.devcontainer` folder).
2. On the repo page: **Code → Codespaces → Create codespace on main**. The first build takes ~5–10 minutes.
3. When it's ready, open the **Ports** tab and click the globe icon on port **6080** (“Desktop (RViz)”). Click *Connect*, password **`ros`**.
4. In the VS Code terminal in the Codespace:

   ```bash
   source install/setup.bash
   ros2 launch arm_pick_place demo.launch.py
   ```

   RViz opens in the browser desktop tab and the arm starts moving.

Free GitHub accounts include monthly Codespaces hours — stop the Codespace when you're done.

## Things to try live (good for a demo)

```bash
# watch the state machine
ros2 topic echo /pick_place/status

# see the joint angles the IK produces
ros2 topic echo /joint_states

# move the cube to different spots
ros2 launch arm_pick_place demo.launch.py   # after editing config/params.yaml
# or run the node directly with new targets:
ros2 run arm_pick_place pick_place --ros-args \
    -p pick_xyz:="[0.35, 0.0, 0.02]" -p place_xyz:="[0.0, 0.30, 0.02]"

# run the IK tests
colcon test --packages-select arm_pick_place && colcon test-result --verbose
```

Reach limits: the grasp point can reach roughly 0.15–0.42 m from the base at table height.

## Running locally instead

On Ubuntu 22.04 with ROS 2 Humble installed:

```bash
mkdir -p ~/arm_ws && cd ~/arm_ws && cp -r <this-repo>/src .
colcon build --symlink-install && source install/setup.bash
ros2 launch arm_pick_place demo.launch.py
```

## Ideas for the next step

- Replace the hand-written IK with **MoveIt 2** (`moveit_setup_assistant` on this URDF) for collision-aware planning.
- Spawn the arm in **Gazebo** with `ros2_control` and a real physics grasp.
- Add a camera and pick the cube where **OpenCV** detects it instead of a fixed position.
- Drive a real hobby-servo arm by sending `/joint_states` to an Arduino/Teensy over serial.
