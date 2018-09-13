# ROS STorM32 Gimbal Driver

[master]: https://dev.mcgillrobotics.com/buildStatus/icon?job=ros-storm32-gimbal/master
[master url]: https://dev.mcgillrobotics.com/job/ros-storm32-gimbal/job/master
[![master]][master url]

This is a driver to communicate with STorM32 gimbal controllers.

**You MUST configure your gimbal with the Windows app before using this
package.**

*This package has been tested on ROS Kinetic Kame on Ubuntu 16.04.*

## Setting up

You must clone this repository as `storm32_gimbal` into your catkin workspace:

```bash
git clone https://github.com/mcgill-robotics/ros-storm32-gimbal storm32_gimbal
```

## Dependencies

Before proceeding, make sure to install all dependencies by running:

```bash
rosdep update
rosdep install storm32_gimbal
```

## Compiling

You **must** compile this package before being able to run it. You can do so
by running:

```bash
catkin_make
```

from the root of your `catkin` workspace.

## Running

To run, simply launch the `storm32_node.py` node as such:

```bash
rosrun storm32_gimbal storm32_node.py
```

You can change the port and TF frame ID by passing them as parameters

```bash
rosrun storm32_gimbal storm32_node.py port:=<device_path> frame:=<frame_name>
```

A sample launch file is available in the `launch` directory.

## Interfacing

### The `gimbal_ref` frame

All orientations used by this package are relative to a global reference frame
called `gimbal_ref`. This reference frame is attached to the gimbal, and has a
z axis always pointing away from the center of the earth (i.e. it does not
pitch or roll, but can yaw).

### Topics

This package publishes to two topics:

- `~camera_orientation`: The IMU1 readings as a `QuaternionStamped` message
  (i.e. the orientation of the gimballed link relative to the global
  `gimbal_ref` frame). Since the link is stabilized, this should always be
  approximately the target orientation of the gimbal.
- `~controller_orientation`: The IMU2 readings as a `QuaternionStamped` message
  (i.e. the orientation of the STorM32 board relative to the global
  `gimbal_ref` frame). This should represent the orientation of the gimbal and
  is not stabilized.

You can also set a new target orientation relative to the `gimbal_ref` frame by
publishing a `GimbalOrientation` message to the `~target_orientation` topic.
The `orientation` field is expected to be relative to the `gimbal_ref` frame,
and the `unlimited` field defines whether the controller should attempt to
limit its rotation to hard set limits on the STorM32 controller.

## Contributing

Contributions are welcome. Simply open an issue or pull request on the matter,
and it will be accepted as long as it does not complicate the code base too
much.

As for style guides, we follow the ROS Python Style Guide for ROS-specifics and
the Google Python Style Guide for everything else.

### Linting

We use [YAPF](https://github.com/google/yapf) for all Python formatting needs.
You can auto-format your changes with the following command:

```bash
yapf --recursive --in-place --parallel .
```

We also use [catkin_lint](https://github.com/fkie/catkin_lint) for all `catkin`
specifics. You can lint your changes as follows:

```bash
catkin lint --explain -W2 .
```

## License

See [LICENSE](LICENSE).
