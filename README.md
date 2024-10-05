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
ros2 topic pub -1 /to_car_new_coords tr24_ros_interfaces/msg/Point2D "{x: 4.02, y: 3.4}"
```

```
ros2 topic pub -1 /to_car_cmd_stop std_msgs/msg/String "{data: 'stop'}"
```

```
ros2 topic pub -1 /from_car_pos tr24_ros_interfaces/msg/Position2D "{x: 1.0, y: 1.5, phi: 0.0}"
```

# How to create two virtual serial ports with socat
```
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```


# Connect RPI and ext laptop together

On the Raspberry Pi:
```
export ROS_DOMAIN_ID=0  # or your chosen domain ID
export ROS_IP=192.168.0.118
```

On Laptop:
```
export ROS_DOMAIN_ID=0  # or the same domain ID as Raspberry Pi
export ROS_IP=192.168.0.106
```

# Run visualization
```
cd ~/ros2_ws/src
ros2 launch py_tr24_car_ros demo.launch.py
```

```
cd ~/ros2_ws
rviz2 -d ./src/install/py_tr24_car_ros/share/py_tr24_car_ros/tr24_car.rviz 
```