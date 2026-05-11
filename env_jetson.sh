#!/usr/bin/env bash
# Source on the Jetson Nano before launching any ROS2 node.
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CYCLONEDDS_URI="file://${REPO_DIR}/cyclonedds_jetson.xml"
echo "ROS2 env → Domain=${ROS_DOMAIN_ID}  RMW=${RMW_IMPLEMENTATION}"
echo "CycloneDDS URI → ${CYCLONEDDS_URI}"
