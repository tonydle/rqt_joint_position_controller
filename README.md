# `rqt_joint_position_controller`
A graphical frontend designed for quick interaction with `JointPositionController` and `JointGroupPositionController` for ROS 1 Noetic (Ubuntu 20.04)

## Modifications
- Work with either `JointPositionController` (sends [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html) commands) or `JointGroupPositionController` (sends [std_msgs/Float64MultiArray](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64MultiArray.html) commands)
- In idle states, the UI reads from `/joint_states` and updates the latest joint position values.

## Installation
1. Clone this repository into your ROS workspace's `src` directory.
2. Navigate to your ROS workspace directory:
   ```bash
   cd /path/to/your/ros/workspace
   ```
3. Use `rosdep` to install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. Build the workspace using `catkin_make`:
   ```bash
   catkin_make
   ```

## Usage
To run the `rqt_joint_position_controller`, use the following command:
```bash
rosrun rqt_joint_position_controller rqt_joint_position_controller
```

## Acknowledgements
This package is based on the original [`rqt_joint_trajectory_controller`](http://wiki.ros.org/rqt_joint_trajectory_controller) by PAL Robotics S.L.

## License
This package is distributed under the BSD license. Please see the `LICENSE` file for more details.

## Contributors
- Original work by PAL Robotics S.L.
- Modifications by Tony Le ([tonyle98@outlook.com](mailto:tonyle98@outlook.com))