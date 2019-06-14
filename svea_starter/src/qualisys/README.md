# Qualisys ROS driver

## License
Apache 2.0 wherever not specified

## Usage

- Launch main connection
```
roslaunch qualisys qualisys.launch
```
- Convert the message to `nav_msgs/Odometry.msg`
```
roslaunch qualisys qualisys_odom.launch model:=<name-of-your-model-on-qualisys>
```
