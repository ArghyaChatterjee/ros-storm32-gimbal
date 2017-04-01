# ROS STorM32 Gimbal Driver

[master]: https://dev.mcgillrobotics.com/buildStatus/icon?job=ros-storm32-gimbal_master
[master url]: https://dev.mcgillrobotics.com/job/ros-storm32-gimbal_master
[![master]][master url]

This is a driver to communicate with STorm32 gimbal controllers.

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
rosdep install interop
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
