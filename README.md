# Optical Flow (BPRL)

## Build
**install dependencies**
1. ```./install_deps.sh```

**docker**
1. ```docker build --no-cache -t bprl/optical-flow-ros:latest .```
1. ```docker compose up```
    1. ```docker compose down``` to shut down

**host**
1. Clone depthai-core and build locally [depthai-core](https://github.com/luxonis/depthai-core/tree/main) (v2.29.0)
    1. ```cmake -S. -Bbuild -D'CMAKE_INSTALL_PREFIX=/usr/local'```
    1. ```cmake --build build --target install```
. ```mkdir host/build & cd host/build```
1. ```cmake .. & make```
