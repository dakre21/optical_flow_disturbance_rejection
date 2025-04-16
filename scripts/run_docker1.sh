#!/bin/bash

echo "Starting Docker Container"

cd /home/pi/optical_flow_disturbance_rejection
docker-compose down
docker-compose up

#cd ..
#
#while true; do
#	if [ pgrep $1 ]; then
#    echo "ROS Docker Container is not running, restarting it"
#		cd /home/pi/optical_flow_disturbance_rejection
#		docker-compose up &
#  fi
#	sleep 5
#done
