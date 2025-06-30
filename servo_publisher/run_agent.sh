#!/bin/bash

echo "Starting micro-ROS Agent on UDP port 8888..."

docker run -it --rm \
  -p 8888:8888/udp \
  microros/micro-ros-agent:humble \
  udp4 --port 8888 -v6