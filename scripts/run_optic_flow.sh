#!/bin/bash

echo "Starting Optical Flow"

while true; do
	if [ pgrep $1 ]; then
    echo "Optical Flow is not running, restarting it"
		/home/pi/optical_flow_disturbance_rejection/src/host/build/optical_flow & 
  fi
	sleep 5
done
