#!/bin/bash
docker run --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy udp4 --port 8888
docker run -it --rm --net=host --volume="`pwd`/mnt:/mnt:rw"  osrf/ros:jazzy-desktop-full
