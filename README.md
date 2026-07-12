# tb3_vio_ekf

EKF-fused Visual-Inertial Odometry for TurtleBot3 in Gazebo simulation.

**Architecture (loosely-coupled VIO):**

```
Gazebo (stock TurtleBot3 "waffle" model — built-in R200 RGB-D camera + IMU)
   ├─ /imu ───────────────► imu_covariance_relay (rclpy) ──► /imu/data ─┐
   ├─ /camera/rgb/*                                                    │
   └─ /camera/depth/* ──► rgbd_odometry (rtabmap_ros) ──► /rtabmap/odom │
                                │                                       │
                                ▼                                       ▼
                        vo_odom_relay (rclpy) ──► /vo/odom ──► ekf_node (robot_localization)
                                                                        │
                                                                        ▼
                                                          /odometry/filtered + TF odom->base_footprint
```

Two rclpy nodes do the "gluing" work instead of a hand-written filter:
- **imu_covariance_relay**: fills in non-zero IMU covariance if Gazebo publishes zeros (which some estimators otherwise treat as "perfectly known").
- **vo_odom_relay**: attaches realistic covariance to the visual odometry output and detects likely tracking loss (large jumps / snap-to-origin), inflating covariance on those samples so a bad VO estimate doesn't yank the filter around.

The actual VO front-end (rtabmap `rgbd_odometry`) and the EKF (`robot_localization` `ekf_node`) are both existing, battle-tested packages — no visual tracking or Kalman math had to be reimplemented.

## Assumptions

- **ROS 2 Humble** on Ubuntu 22.04, with **Gazebo Classic** (`gazebo_ros` plugins) — this is still the most common TurtleBot3-simulation setup.
- Uses the **stock `turtlebot3_waffle` model** (`TURTLEBOT3_MODEL=waffle`). The apt-packaged `turtlebot3_description` on Humble ships **pre-generated flat `.urdf` files** (not `.xacro` sources) at `/opt/ros/humble/share/turtlebot3_description/urdf/` — `turtlebot3_burger.urdf`, `turtlebot3_waffle.urdf`, `turtlebot3_waffle_pi.urdf`. **These files contain an unresolved `${namespace}` placeholder on every link/frame name** (normally string-substituted by ROBOTIS's own bringup launch scripts for multi-robot setups). This package's launch file reads the URDF directly in Python and strips that placeholder before publishing it as `robot_description` — if you see log lines like `got segment ${namespace}base_link` from `robot_state_publisher`, that substitution didn't happen and every downstream frame name (including the camera frames) will be broken. `waffle` was chosen over `waffle_pi` because it has a built-in Intel R200 RGB-D camera already publishing `/camera/rgb/...` and `/camera/depth/...` — exactly what `rgbd_odometry` needs. `waffle_pi`'s stock camera is RGB-only (Raspberry Pi camera), so it won't work with this RGB-D pipeline without modification.
- If you're on a newer distro using Gazebo Sim (Harmonic) with `ros_gz` instead of Gazebo Classic, the `gazebo.launch.py` include and `spawn_entity.py` node in the launch file will need to be swapped for the `ros_gz` equivalents.

## Install dependencies

```bash
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-msgs ros-humble-turtlebot3-simulations \
                  ros-humble-robot-localization \
                  ros-humble-rtabmap-ros \
                  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-gazebo-ros-pkgs
```

`rtabmap_ros` package naming has changed across releases. Check which executable name your install provides:

```bash
ros2 pkg executables | grep rgbd_odometry
```

If it reports `rtabmap_ros rgbd_odometry` instead of `rtabmap_odom rgbd_odometry`, edit the `package=` field for the `rgbd_odometry` Node in `launch/vio_ekf_sim.launch.py` accordingly.

## Build

```bash
cd ~/ros2_ws/src
# copy/clone this tb3_vio_ekf/ folder here
cd ~/ros2_ws
colcon build --packages-select tb3_vio_ekf
source install/setup.bash
```

## Run

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch tb3_vio_ekf vio_ekf_sim.launch.py
```

Add `use_rviz:=false` to skip launching RViz.

Once Gazebo is up, drive the robot around (e.g. `ros2 run turtlebot3_teleop teleop_keyboard`) and watch `/odometry/filtered` in RViz (add an Odometry display) or:

```bash
ros2 topic echo /odometry/filtered
```


## Tuning notes

- **`config/ekf.yaml`**: `odom0_config`/`imu0_config` and `process_noise_covariance` are reasonable starting points, not final values — tune against your actual sim behavior. In particular, watch for drift when the robot spins in place (yaw fusion) vs. drift when driving straight (x/y fusion) and adjust the corresponding variances.
- **`vo_odom_relay.py`**: `POS_VAR_DEFAULT`, `YAW_VAR_DEFAULT`, and `MAX_JUMP_M` are heuristics. A more principled version would derive per-message covariance from rtabmap's own odometry info (inlier count, reprojection error) rather than static values — rtabmap publishes this on `/rtabmap/odom_info` if you want to wire it in.
- **`two_d_mode: true`** in `ekf.yaml` constrains the filter to planar motion, which is appropriate for TurtleBot3 on flat ground. Set to `false` only if you actually need z/roll/pitch estimates (e.g. ramps).


