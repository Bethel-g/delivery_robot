#!/bin/bash
# ============================================================
#  INSTALL.sh — One-script setup for delivery_robot
#  Run this ONCE after cloning the repo.
#  Usage: bash INSTALL.sh
# ============================================================
set -e

YELLOW='\033[1;33m'; GREEN='\033[0;32m'; RED='\033[0;31m'; NC='\033[0m'
info()    { echo -e "${YELLOW}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[OK]${NC}   $1"; }
error()   { echo -e "${RED}[ERR]${NC}  $1"; exit 1; }

echo ""
echo "=================================================="
echo "  delivery_robot — automated setup"
echo "=================================================="
echo ""

# 1. Check Ubuntu version
info "Checking Ubuntu version..."
UBUNTU=$(lsb_release -rs)
if [[ "$UBUNTU" != "22.04" ]]; then
  error "Ubuntu 22.04 required. You have $UBUNTU. See README."
fi
success "Ubuntu 22.04 confirmed"

# 2. Check ROS 2 is sourced
info "Checking ROS 2 Humble..."
if ! command -v ros2 &>/dev/null; then
  error "ROS 2 not found. Run: source /opt/ros/humble/setup.bash first"
fi
success "ROS 2 found"

# 3. Install system dependencies
info "Installing apt dependencies (may take a few minutes)..."
sudo apt update -qq
sudo apt install -y -qq \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  git wget kazam ffmpeg 2>/dev/null || true
success "Dependencies installed"

# 4. rosdep
info "Running rosdep..."
if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init 2>/dev/null || true
fi
rosdep update --quiet 2>/dev/null || true

WS="$HOME/delivery_ws"
PKG="$WS/src/delivery_robot"

# Detect if script is being run from inside the package
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/package.xml" ]; then
  PKG="$SCRIPT_DIR"
fi

cd "$WS" 2>/dev/null || { mkdir -p "$WS/src"; cd "$WS"; }

rosdep install --from-paths src --ignore-src -r -y -q 2>/dev/null || true
success "rosdep done"

# 5. Build
info "Building with colcon (this takes 1-2 minutes)..."
cd "$WS"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_cohesion+ 2>&1 | tail -5
success "Build complete"

# 6. Source lines in .bashrc
ROS_SOURCE="source /opt/ros/humble/setup.bash"
WS_SOURCE="source $WS/install/setup.bash"
grep -qxF "$ROS_SOURCE" ~/.bashrc || echo "$ROS_SOURCE" >> ~/.bashrc
grep -qxF "$WS_SOURCE"  ~/.bashrc || echo "$WS_SOURCE"  >> ~/.bashrc
source ~/.bashrc 2>/dev/null || true
success "~/.bashrc updated — every new terminal will auto-source"

echo ""
echo "=================================================="
echo -e "${GREEN}  Setup complete!${NC}"
echo "=================================================="
echo ""
echo "  NEXT STEPS:"
echo "  1. Close this terminal and open a fresh one"
echo "  2. Phase 1 (mapping):"
echo "     ros2 launch delivery_robot slam_launch.py"
echo ""
echo "  3. Phase 2 (navigation):"
echo "     ros2 launch delivery_robot navigation_launch.py"
echo ""
echo "  4. Send a mission (new terminal):"
echo "     ros2 run delivery_robot delivery_mission room1 room2"
echo ""
