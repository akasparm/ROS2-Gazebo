# ROS2-Gazebo
Working with Gazebo using ROS2

## Dependencies

- rclcpp
- sensor_msgs
- geometry_msgs
- OS: Ubuntu Linux 22.04
- ROS Version: ROS2 Humble Hawksbill

## Build Instructions

Navigate to the src folder of the ROS2 workspace

```sh
cd ~/ros2_ws/src
```

Clone the GitHub repository

```sh
git clone git@github.com:akasparm/ROS2-Gazebo.git
```

Now to build the package go to the root of the ROS2 workspace

```sh
cd ..
# check the dependencies
rosdep install -i --from-path src --rosdistro humble -y
# Source ros2
source /opt/ros/humble/setup.bash
# Building the package
colcon build --packages-select ros2_gazebo
```

## Run Instructions
Sourcing the package
```
source install/setup.bash
```

### Using the launch file

To run the launch file and start the ros bag file recording,

```sh
export TURTLEBOT3_MODEL=burger
```
```sh
cd src/ros2_gazebo/bag\ files
```
```sh
ros2 launch ros2_gazebo launch.py bag_record:=True
```


## References

[1] <http://docs.ros.org/en/humble/index.html>