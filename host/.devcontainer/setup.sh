#!/bin/bash
set -e

echo "Changing directory to /workspace..."
cd /workspace

echo "Removing old build artifacts (build, install, log)..."
rm -rf build install log

echo "Updating rosdep..."
rosdep update

echo "Installing rosdep dependencies for packages in src..."
rosdep install --from-paths src --ignore-src -y --rosdistro humble

echo "Building the workspace with colcon..."
colcon build --symlink-install

echo "Setup complete."
