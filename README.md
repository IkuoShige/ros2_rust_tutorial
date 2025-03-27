# ros2_rust_tutorial

This repository is a tutorial packages for ros2_rust.

## dependency

[ros2_rust](https://github.com/ros2-rust/ros2_rust.git)

Read this link for more information.

```sh
sudo apt install -y git libclang-dev python3-pip python3-vcstool
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
mkdir -p rust_ws/src && cd rust_ws/
git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
vcs import src < src/ros2_rust/ros2_rust_humble.repos
source /opt/ros/humble/setup.sh
colcon build
```

## Set up for tf2_msgs

Read [this link](https://github.com/ros2-rust/ros2_rust/discussions/201) for more information.

Follow to `ros2_rust/ros2_rust_humble.repos`

```
ros2/geometry2:
  type: git
  url: https://github.com/ros2/geometry2.git
  version: humble
```

Re-import the repos

```sh
vcs import src < src/ros2_rust/ros2_rust_humble.repos
source /opt/ros/humble/setup.sh
colcon build
```

Add `tf2_lmsgs` to Cargo.toml and package.xml

## how to use

```sh
cd rust_ws/ && rust_ws
git clone https://github.com/IkuoShige/ros2_rust_tutorial.git src/ros2_rust_tutorial
git clone https://github.com/IkuoShige/ros2_rust_tutorial_interfaces.git src/ros2_rust_tutorial_interfaces
colcon build
source install/setup.bash
ros2 launch ros2_rust_tutorial ros2_rust_tutorial.launch.py
```
