#!/bin/bash
set -e

echo "Ensuring that /workspace/install/setup.bash is sourced in new shells..."

# Append the source command to /root/.bashrc if it isn’t already there.
if ! grep -q "source /workspace/install/setup.bash" /root/.bashrc; then
  echo "source /workspace/install/setup.bash" >> /root/.bashrc
  echo "Added 'source /workspace/install/setup.bash' to /root/.bashrc"
else
  echo "Sourcing command already exists in /root/.bashrc"
fi

# Create a file in /etc/profile.d/ to help ensure it is sourced.
cat << 'EOF' > /etc/profile.d/ros_setup.sh
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi
EOF
chmod +x /etc/profile.d/ros_setup.sh

echo "Post-start script complete."
