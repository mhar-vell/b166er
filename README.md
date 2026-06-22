# b1-66er — Mobile Manipulation Robot

Pioneer 3-AT base + Mitsubishi RV-M2 arm, controlled via ROS Noetic (RoboStack) on Linux.

## Hardware

| Component | Interface | Device |
|---|---|---|
| Pioneer 3-AT base | USB-serial | `/dev/ttyPioneer` |
| Mitsubishi RV-M2 | Arduino via rosserial | `/dev/ttyArduino` |
| Sparton AHRS-8 IMU | USB-serial | `/dev/ttyAHRS` |
| Intel NUC 5i5RYH | ROS master | `b166er-nuc.local` |

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

## Environment setup

```bash
# on the laptop — points ROS at the NUC as master
source ~/b166er/setup/ros_env.sh
```

## NUC initial setup (one-time)

```bash
# copy to a USB stick and run on the NUC
bash setup/nuc_setup.sh        # SSH, avahi, hostname
bash setup/nuc_ros_setup.sh    # Miniforge3 + ros_env conda env
bash setup/udev_pioneer.sh     # /dev/ttyPioneer udev rule
```

## Launch

```bash
# RViz preview (no hardware)
roslaunch b166er_robot b166er_base.launch

# Pioneer base
roslaunch b166er_robot pioneer_hardware.launch

# Full hardware stack
roslaunch b166er_robot pioneer_hardware.launch &
roslaunch b166er_robot imu_hardware.launch &
roslaunch b166er_robot movemaster_hardware.launch
```
