#!/bin/bash

. install/setup.bash

#ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'GPS_TYPE', value: {integer: 0}}"

#ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'GPS1_TYPE', value: {integer: 0}}"
#ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_ENABLE', value: {integer: 1}}"
#ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_SRC1_POSXY', value: {integer: 3}}"
#ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_SRC1_POSZ', value: {integer: 3}}"
#ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_SRC1_VELXY', value: {integer: 3}}"
#ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_SRC1_VELZ', value: {integer: 3}}"
#ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_SRC1_YAW', value: {integer: 3}}"
#ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_GPS_CHECK', value: {integer: 0}}"
#ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'VISO_TYPE', value: {integer: 0}}"
#ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'AHRS_EKF_TYPE', value: {integer: 3}}"

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"

ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

ros2 topic pub --once /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 10.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# ros2 topic pub /mavros/vision_pose/pose geometry_msgs/msg/Pose "{position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
