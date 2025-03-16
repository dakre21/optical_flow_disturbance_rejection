# Optical Flow (BPRL)

## Instructions (Build & Run)
**install dependencies**
1. ```./setup.sh```

**processes running in docker**
1. ```docker build --no-cache -t bprl/optical-flow-ros:latest .```
1. ```docker compose up``` and ```docker compose down``` to shut down
    1. On the raspberry pi it should be ```docker-compose up -d``` followed by ```docker container ls``` get the container id then shell into it ```docker exec -it <container-id> bash```
1. In the container build the ros2 source simply by running ```colcon build --symlink-install``` in the current directory "/workspaces"
1. Run the following:
    1. ```ros2 run optical_flow optical_flow_node --ros-args --remap __ns:=/<rpi1/2>```
    1. ```ros2 run optical_flow flow_aggregator_node``` on the raspberrypi1 host connected to the pixhawk
    1. Currently topics and stuff are static, could improve by parameterizing them as ros2 parameters later on

**processes running on debian**
1. Clone depthai-core and build locally [depthai-core](https://github.com/luxonis/depthai-core/tree/main) (v2.29.0)
    1. ```cmake -S. -Bbuild -D'CMAKE_INSTALL_PREFIX=/usr/local'```
    1. ```cmake --build build --target install```
1. Host code ```mkdir host/build & cd host/build```
1. ```cmake .. & make -j```
1. ```./optical_flow``` with all 4 cameras connected (2x camera module 3 and 2x luxonis oak)
