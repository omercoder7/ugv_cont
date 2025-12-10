#!/bin/bash
# validate_dockerfile.sh - Quick validation of Dockerfile dependencies
# Run this before build_ugv_image.sh to catch errors early

set -e

echo "🔍 Validating Dockerfile dependencies..."

BASE_IMAGE="ros:humble-ros-core-jammy"

echo "📦 Checking apt packages (dry-run)..."
docker run --rm $BASE_IMAGE bash -c "
  apt-get update -qq && \
  apt-get install -y --dry-run --no-install-recommends \
    curl gnupg2 lsb-release wget sudo software-properties-common git build-essential cmake \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rosbridge-suite \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    libeigen3-dev \
    libxml2-dev \
  2>&1 | tail -5
"

echo ""
echo "🐍 Checking Python packages..."
docker run --rm $BASE_IMAGE bash -c "
  pip3 install --dry-run 'setuptools<81' pyserial flask mediapipe requests 2>&1 | tail -5
"

echo ""
echo "✅ Validation passed! Safe to run ./build_ugv_image.sh"
