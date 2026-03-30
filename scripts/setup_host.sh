#!/bin/bash
# =============================================================================
# OPEN-01 Laptop Setup Script
# Target: Ubuntu 24.04 LTS (amd64) + ROS2 Jazzy
# Usage: bash setup_laptop.sh
# =============================================================================

set -e

# ── Config ────────────────────────────────────────────────────────────────────
DEV_USER=${SUDO_USER:-$(whoami)}
DEV_HOME=$(eval echo "~$DEV_USER")
REPO_URL="https://github.com/RonishNadar/open-01.git"
REPO_DIR="$DEV_HOME/open-01"
ROS_WS="$REPO_DIR/robot_ros2/ros2_ws"
ROS_DISTRO="jazzy"

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[OPEN-01]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
err()  { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# ── Root check ────────────────────────────────────────────────────────────────
if [[ $EUID -ne 0 ]]; then
    err "Run as root: sudo bash setup_laptop.sh"
fi

log "Setting up OPEN-01 on laptop"
log "User: $DEV_USER | Home: $DEV_HOME | Repo: $REPO_DIR"

# =============================================================================
# STEP 1 — System dependencies
# =============================================================================
log "Step 1/5 — System dependencies"

apt-get update -qq
apt-get install -y \
    curl wget git nano \
    python3 python3-pip python3-venv \
    build-essential \
    2>/dev/null

# =============================================================================
# STEP 2 — ROS2 Jazzy
# =============================================================================
log "Step 2/5 — ROS2 Jazzy"

if [ -f /opt/ros/jazzy/setup.bash ]; then
    log "  ROS2 Jazzy already installed, skipping"
else
    log "  Installing ROS2 Jazzy..."

    # Locale
    apt-get install -y locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # ROS2 apt repo
    apt-get install -y software-properties-common
    add-apt-repository universe -y
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list

    apt-get update -qq
    apt-get install -y ros-jazzy-desktop
    apt-get install -y python3-colcon-common-extensions python3-rosdep
fi

# ROS2 additional packages
log "  Installing ROS2 extra packages..."
apt-get install -y \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-rviz-imu-plugin \
    2>/dev/null

# rosdep init
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log "  Initializing rosdep..."
    rosdep init
fi

sudo -u $DEV_USER rosdep update 2>/dev/null || true

# =============================================================================
# STEP 3 — Clone repo
# =============================================================================
log "Step 3/5 — Repository"

if [ -d "$REPO_DIR/.git" ]; then
    log "  Repo already cloned, pulling latest..."
    sudo -u $DEV_USER git -C "$REPO_DIR" pull
else
    log "  Cloning OPEN-01 repo..."
    sudo -u $DEV_USER git clone "$REPO_URL" "$REPO_DIR"
fi

# =============================================================================
# STEP 4 — Build ROS2 workspace
# =============================================================================
log "Step 4/5 — Building ROS2 workspace"

source /opt/ros/jazzy/setup.bash

sudo -u $DEV_USER bash -c "
    source /opt/ros/jazzy/setup.bash
    cd $ROS_WS
    colcon build --symlink-install
"

# Add ROS2 source to .bashrc
BASHRC="$DEV_HOME/.bashrc"
if ! grep -q "source /opt/ros/jazzy/setup.bash" "$BASHRC"; then
    log "  Adding ROS2 to .bashrc..."
    echo "" >> "$BASHRC"
    echo "# ROS2 Jazzy" >> "$BASHRC"
    echo "source /opt/ros/jazzy/setup.bash" >> "$BASHRC"
    echo "source $ROS_WS/install/setup.bash" >> "$BASHRC"
    echo "export ROS_DOMAIN_ID=0" >> "$BASHRC"
fi

# =============================================================================
# STEP 5 — Verify
# =============================================================================
log "Step 5/5 — Verifying installation"

source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"

PACKAGES=("open01_description" "open01_viz")
for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg list 2>/dev/null | grep -q "^$pkg$"; then
        log "  ✓ $pkg found"
    else
        warn "  ✗ $pkg not found — check build output"
    fi
done

# =============================================================================
# Done
# =============================================================================
echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  OPEN-01 Laptop setup complete!${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "Usage:"
echo "  Visualize robot:  ros2 launch open01_viz viz.launch.py"
echo "  View TF tree:     ros2 run tf2_tools view_frames"
echo "  List topics:      ros2 topic list"
echo ""
echo "Make sure ROS_DOMAIN_ID=0 matches the RPi."
echo "Open a new terminal or run: source ~/.bashrc"
echo ""