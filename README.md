# Trilobot_brain

## Build the dockerfile in the Dockerfile/ directory:
```
docker build -t ros2-humbe-img .
```
## Run the container in the Trilobot_brain/ directory:
```
docker run -it --privileged $(stat -c '--group-add %g' /dev/dri/* | sort -u)  -v $PWD:/project   -w /project   -u $UID   -e HOME=/tmp  --name ros2-humble-cont --net=host ros2-humbe-img
```
## Run the container:
```
docker start ros2-humble-cont
```
## Run the container in one or more terminals:
```
docker exec -it ros2-humble-cont bash
```
## Build the ROS 2 workspace:
```
cd trilobot_brain_ws
```
```
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
## Source the ROS 2 enviroment and the workspace:
```
source /opt/ros/humble/setup.bash
```
```
source install/setup.bash
```
## Ros2 commands to run in the container
## Give permissions for the Lidar and esp32 usb ports
```
sudo chmod 666 /dev/ttyACM1
```
```
sudo chmod 777 /dev/ttyUSB0
```
## Start the robots operation using the diffbot.launch.py lauchfile:
```
ros2 launch diffdrive_arduino diffbot.launch.py
```
