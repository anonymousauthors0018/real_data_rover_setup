# Realtime Data Capture using Intel Realsense L515 and TI mmWave Radar 

## 1. Setup 

### Without Docker 

1. Install LibRealSense (https://github.com/realsenseai/librealsense/blob/master/doc/installation.md) - L515 uses v2.50
2. Install ROS2 (https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
3. Install ROS1 (https://wiki.ros.org/noetic/Installation/Ubuntu) for robot control 
4. Install pyRadar Library 
``` 
git clone https://github.com/anonymousauthors0018/pyRadar.git 
cd pyRadar
sudo python3 -m pip install --upgrade pip
sudo python3 -m pip install --upgrade setuptools
sudo python3 -m pip install ./fpga_udp
sudo python3 -m pip install . 

sudo python3 -m pip install -r requirements.txt
``` 

### With Docker
``` 
docker pull anonymousauthors0018/real_data_rover_setup:latest
sudo docker run -it \
  --network host \
  --privileged \
  -v /dev:/dev \
  real_data_rover_setup:latest
```


## 2. Build Data Collection Package  

``` 
git clone https://github.com/anonymousauthors0018/real_data_rover_setup.git 

# Initialize ROS2 ENV 
source /opt/ros/foxy/setup.bash

#Build Sensor Collection Package
cd real_data_rover_setup/ros_ws
colcon build --symlink-install 
source install/setup.bash
``` 

## 3. Build Robot Package 

```
# Initialize ROS1 ENV 
source /opt/ros/noetic/setup.bash

cd real_data_rover_setup/wheeltec

#Build Robot Control package
catkin_make 
source devel/setup.bash
```

## 4. Data Collection 

1. Ensure [Setup](#1-setup) and [Installation](#2-build-data-collection-package) steps are completed 
2. Update Config files in ``` /configs ``` based on actual Radar configurations
3. Ensure to update the config file path, storage path, and radar ports in ```/ros_ws/src/sensor_bringup/launch```
3. To execute Data collection with a Real Time Dashboard 
``` 
cd real_data_rover_setup
source /opt/ros/foxy/setup.bash
source ros_ws/install/setup.bash
python3 scripts/realtime_dashboard_launch.py

# Open the browser and access the dashboard at localhost:8080
``` 
4. To execute the data collection from CLI 
``` 
cd real_data_rover_setup
source /opt/ros/foxy/setup.bash
source ros_ws/install/setup.bash
ros2 launch sensor_bringup sensors.launch.py 
```

## 5. Robot Control 

1. Ensure [ROS1 wheeltec package](#3-build-robot-package) is built 
2. Ensure robot chassis is connected 
3. Run python script 
``` 
source /opt/ros/noetic/setup.bash
cd real_data_rover_setup/ 
source wheeltec/devel/setup.bash 
roslaunch turn_on_wheeltec_robot base_serial.launch

#Open another terminal 
source /opt/ros/noetic/setup.bash
cd real_data_rover_setup/ 
source wheeltec/devel/setup.bash 
python3 scripts/robot_control_cycles.py
```

## References 
1. RealSense SDK 2.0 - https://github.com/realsenseai/librealsense 
2. pyRadar - https://github.com/gaoweifan/pyRadar 
3. Wheeltec Robot Control 









