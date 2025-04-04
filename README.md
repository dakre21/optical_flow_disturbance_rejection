# Optical Flow Disturbance Rejection (BPRL)

This repository contains software for the following:
- Optical flow estimation on 8 on-board cameras
- Flow aggregation that gets propagated to mavros velocity observer plugin
- Special odometry mavlink message from velocity observer to ardupilot

TODO:
- Handle incoming odometry for inner loop control (stubbed out)
- Need estimated C matrix for projecting flows into velocity estimate (stubbed out)

## Architecture

![alt text](OpticalFlowStack.png)

## Instructions (Build & Run)
**install dependencies**
1. ```./setup.sh```
1. Luxonis udev control
    1. ```sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash```
    1. ```sudo cp host/80-movidius.rules /etc/udev/rules.d```
    1. ```sudo udevadm control --reload-rules```
    1. ```sudo udevadm trigger```

**optical_flow processes running in docker**
1. ```docker build --no-cache -t bprl/optical-flow-ros:latest .```
1. ```docker compose up``` and ```docker compose down``` to shut down
    1. On the raspberry pi it should be ```docker-compose up -d``` followed by ```docker container ls``` get the container id then shell into it ```docker exec -it <container-id> bash```
1. In the container build the ros2 source simply by running ```colcon build --symlink-install``` in the cwd of docker on pi add ```--packages-skip mocap4r2_optitrack_driver``` as this isn't supported on arm
    1. Ensure that dependencies have been imported by ```vcs import src < dependency.repos``` either on host or in docker container
1. To run do the following:
    1. ```ros2 launch optical_flow_bringup bringup.launch.py``` will launch everything including mavros and mocap related nodes
    1. ```ros2 run optical_flow optical_flow_node --ros-args --remap __ns:=/<rpi1/2>```
    1. ```ros2 run optical_flow flow_aggregator_node``` on the raspberrypi1 host connected to the pixhawk
    1. Currently topics and stuff are static, could improve by parameterizing them as ros2 parameters later on

**optical_flow processes running on debian**
1. Clone depthai-core and build locally [depthai-core](https://github.com/luxonis/depthai-core/tree/main) (v2.29.0)
    1. ```cmake -S. -Bbuild -D'CMAKE_INSTALL_PREFIX=/usr/local'```
    1. ```cmake --build build --target install```
1. Host code ```mkdir host/build & cd host/build```
1. ```cmake .. & make -j```
1. ```./optical_flow``` with all 4 cameras connected (2x camera module 3 and 2x luxonis oak)
    1. To test rpi cameras ```libcamera-hello --list-cameras```
    1. To test luxonis cameras run the debug exe ```./luxonis_debug```

**SITL ardupilot simulation**
1. Clone ardupilot from [ardupilot](https://github.com/ArduPilot/ardupilot)
1. ```cd ardupilot && git submodule update --init --recursive``` 
1. Do all of the approriate setup in the README.md in ardupilot
1. ```./Tools/autotest/sim_vehicle.py -v copter --console --map -w``` will run the SITL simulation with mavproxy and map
    1. Alternatively run ```build/sitl/bin/arducopter -w --model + --speedup 1 --slave 0 --defaults Tools/autotest/default_params/copter.parm --sim-address=127.0.0.1 -I0```
    and ```mavproxy.py --retries 5 --out 127.0.0.1:14550 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --map --console```
1. In a separate terminal run ```ros2 launch optical_flow_bringup bringup.launch.py fcu_url:="udp://127.0.0.1:14550@145550" use_optical_flow:=False use_mocap:=False```
