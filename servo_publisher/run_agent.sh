#!/bin/bash

PORT=8888

if lsof -i UDP:$PORT >/dev/null 2>&1; then
  echo "Port $PORT is already in use. Please close the existing process first."
  exit 1
fi

echo "Starting micro-ROS Agent on UDP port $PORT..."

docker run -it --rm \
  -p ${PORT}:${PORT}/udp \
  microros/micro-ros-agent:humble \
  udp4 --port $PORT -v6