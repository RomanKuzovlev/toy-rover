SHELL := /bin/bash

ROS_DISTRO ?= jazzy
PACKAGE := toy_rover
ROS_SETUP := /opt/ros/$(ROS_DISTRO)/setup.bash
LOCAL_SETUP := install/setup.bash

.PHONY: help build test launch launch-core clean

help:
	@echo "toy-rover commands"
	@echo "  make build       Build the ROS 2 package"
	@echo "  make test        Run unit tests"
	@echo "  make launch      Start Gazebo, bridge, and spawn the rover"
	@echo "  make launch-core Start Gazebo plus mapping/planner/controller nodes"
	@echo "  make clean       Remove colcon output"

build:
	source $(ROS_SETUP) && colcon build --packages-select $(PACKAGE)

test:
	source $(ROS_SETUP) && colcon test --packages-select $(PACKAGE) --event-handlers console_direct+

launch:
	source $(ROS_SETUP) && source $(LOCAL_SETUP) && \
		ros2 launch $(PACKAGE) sim.launch.py start_core_nodes:=false

launch-core:
	source $(ROS_SETUP) && source $(LOCAL_SETUP) && \
		ros2 launch $(PACKAGE) sim.launch.py start_core_nodes:=true

clean:
	rm -rf build install log
