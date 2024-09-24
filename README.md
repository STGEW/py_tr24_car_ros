# py_tr24_car_ros

Activate ROS env variables
```
source /opt/ros/humble/setup.bash
source /opt/ros/foxy/setup.bash
```


```
cd ~/ros2_ws/src
colcon build
```

```
cd ~/ros2_ws/src/src
source ros2_venv/bin/activate
```


```
ros2 run py_tr24_car_ros cmd_uart
ros2 run py_tr24_car_ros car_logger
```

# Send something once to the ROS topic
```
ros2 topic pub -1 /to_car_new_coords tr24_ros_interfaces/msg/Point2D "{x: 4.02, y: 56.4}"
```

```
ros2 topic pub -1 /to_car_cmd_stop std_msgs/msg/String "{data: 'stop'}"
```


# How to create two virtual serial ports with socat
```
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```


# Connect RPI and ext laptop together

On the Raspberry Pi:
```
export ROS_DOMAIN_ID=0  # or your chosen domain ID
export ROS_IP=<Raspberry_Pi_IP>
```

On Laptop:
```
export ROS_DOMAIN_ID=0  # or the same domain ID as Raspberry Pi
export ROS_IP=192.168.0.105
```