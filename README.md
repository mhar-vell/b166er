# b1-66er — Mobile Manipulation Robot

Pioneer 3-AT base + Mitsubishi RV-M2 arm, controlled via ROS Noetic (RoboStack) on Linux.

## Hardware

| Component | Interface | Device |
|---|---|---|
| Pioneer 3-AT base | USB-serial | `/dev/ttyPioneer` |
| Mitsubishi RV-M2 | Arduino via rosserial | `/dev/ttyArduino` |
| Sparton AHRS-8 IMU | USB-serial | `/dev/ttyAHRS` |
| Intel RealSense T265 | USB | câmera `t265` |
| Hokuyo UST-05LX (Smart URG) | Ethernet | IP a definir |
| Intel NUC 5i5RYH | ROS master | `b166er-nuc.local` |

## Packages

| Package | Description |
|---|---|
| `b166er_robot` | URDF, launch files, hardware drivers |
| `b166er_whole_body_control` | Whole-body controller (8-DOF: Pioneer + RV-M2) |
| `movemaster_control` | RV-M2 driver (serial) |
| `sparton_ahrs8_driver` | IMU driver |
| `rosaria` | Pioneer driver (AriaCoda) |

## Build

ROS Noetic is provided by [RoboStack](https://robostack.github.io/) (conda-based).
The `rosaria` driver requires [AriaCoda](https://github.com/reedhedges/AriaCoda); the repo includes it at `src/AriaCoda/`.

```bash
# one-time: build rosaria against AriaCoda
cd ~/b166er
ARIA=~/b166er/src/AriaCoda catkin build rosaria

# full workspace build
catkin build
```

> **Note:** `src/Aria/` contains the original ARIA headers (macOS ARM64 binaries).
> Always use `src/AriaCoda/` when building on Linux x86_64.

## Machines

| Machine | Shell | Role |
|---|---|---|
| **Shiroi** (laptop) | `zsh` | operação, RViz, desenvolvimento |
| **NUC** (`b166er-nuc.local`) | `bash` | ROS master, hardware embarcado |

Comandos na NUC são sempre `bash -c "..."`. Comandos no Shiroi usam `zsh`.

## Git workflow

`local-state` is the persistent working branch — not a one-off feature
branch. Day-to-day work accumulates there across sessions; it's pushed
and opened as a PR to `main` whenever a batch of work is ready for
review (usually once per session, sometimes more). After merge,
`local-state` stays alive: pull `main` back into it (or fast-forward)
before starting the next round of work, rather than deleting and
recreating it. `main` has no branch protection configured, but the
convention holds anyway: **only Marco approves and merges PRs**, never
via `gh pr merge`/`gh pr review --approve`.

## Environment setup

```bash
# Shiroi — aponta ROS para a NUC como master
source ~/miniforge3/envs/ros_env/setup.zsh
source ~/b166er/devel/setup.zsh
export ROS_MASTER_URI=http://b166er-nuc.local:11311
export ROS_HOSTNAME=shiroi.local

# NUC (via SSH)
source ~/miniforge3/envs/ros_env/setup.bash
source ~/b166er/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=b166er-nuc.local
```

## NUC initial setup (one-time)

```bash
# copy to a USB stick and run on the NUC
bash setup/nuc_setup.sh        # SSH, avahi, hostname
bash setup/nuc_ros_setup.sh    # Miniforge3 + ros_env conda env
bash setup/udev_pioneer.sh     # /dev/ttyPioneer udev rule
```

## Launch

### Whole-body control (unified launch)

```bash
source devel/setup.zsh

# Gazebo simulation (recommended for development)
roslaunch b166er_whole_body_control b166er_wb.launch mode:=gazebo gui:=true rviz:=true

# Hardware (Pioneer + T265 + RV-M2 + IMU)
roslaunch b166er_whole_body_control b166er_wb.launch mode:=hardware rviz:=true

# Kinematic preview only (RViz + joint_state_publisher_gui)
roslaunch b166er_whole_body_control b166er_wb.launch mode:=sim
```

### Send an EE target (Gazebo, safe workspace: z ≥ 0.60 m in odom frame)

```bash
rostopic pub -r 5 /b166er/ee_target geometry_msgs/PoseStamped \
  '{header: {frame_id: odom}, pose: {position: {x: 0.55, y: 0.10, z: 0.70}, orientation: {w: 1.0}}}'
```

### Legacy launches

```bash
# RViz preview (no hardware)
roslaunch b166er_robot b166er_base.launch

# Pioneer base only
roslaunch b166er_robot pioneer_hardware.launch
```

## Whole-body control architecture

The RV-M2 arm has no accessible joint encoders. Control is performed in
**task space** using the T265 camera on the end-effector as a 6-DOF position sensor,
with an 8-DOF Mamdani fuzzy controller (Pioneer base + arm joints):

```
T265 (EE pose) ──► state_estimator (IK) ──► fuzzy_wb_controller ──► cmd_vel + arm_vel_cmd
Pioneer odom  ──►                                                 └──► arm_vel_integrator
```

See [`src/b166er_whole_body_control/docs/controle_sem_encoders.md`](src/b166er_whole_body_control/docs/controle_sem_encoders.md)
for the full technical description.
