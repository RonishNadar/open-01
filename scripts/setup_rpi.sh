#!/bin/bash
# =============================================================================
# OPEN-01 Raspberry Pi Setup Script
# Target: Ubuntu 22.04.5 LTS (arm64) + ROS2 Humble
# Usage: bash setup_rpi.sh
# =============================================================================

set -e  # Exit on error

# ── Config ────────────────────────────────────────────────────────────────────
ROBOT_USER=${SUDO_USER:-$(whoami)}
ROBOT_HOME=$(eval echo "~$ROBOT_USER")
REPO_URL="https://github.com/RonishNadar/open-01.git"
REPO_DIR="$ROBOT_HOME/open-01"
ROS_WS="$REPO_DIR/robot_ros2/ros2_ws"
ROS_DISTRO="humble"
VENV_DIR="$ROBOT_HOME/.venv"

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
    err "Run as root: sudo bash setup_rpi.sh"
fi

log "Setting up OPEN-01 on Raspberry Pi 3B"
log "User: $ROBOT_USER | Home: $ROBOT_HOME | Repo: $REPO_DIR"

# =============================================================================
# STEP 1 — System prep
# =============================================================================
log "Step 1/8 — System prep"

# Disable cloud-init
if [ ! -f /etc/cloud/cloud-init.disabled ]; then
    log "  Disabling cloud-init..."
    touch /etc/cloud/cloud-init.disabled
    systemctl disable cloud-init cloud-init-local cloud-config cloud-final 2>/dev/null || true
else
    log "  cloud-init already disabled, skipping"
fi

# Disable snapd
if systemctl is-enabled snapd 2>/dev/null | grep -q "enabled"; then
    log "  Disabling snapd..."
    systemctl disable snapd snapd.socket snapd.seeded 2>/dev/null || true
    apt-get purge -y snapd 2>/dev/null || true
else
    log "  snapd already disabled, skipping"
fi

# Mask serial getty on ttyAMA0 (conflicts with ESP32 UART)
if ! systemctl is-masked serial-getty@ttyAMA0 2>/dev/null | grep -q "masked"; then
    log "  Masking serial-getty@ttyAMA0..."
    systemctl mask serial-getty@ttyAMA0
else
    log "  serial-getty@ttyAMA0 already masked, skipping"
fi

# =============================================================================
# STEP 2 — System dependencies
# =============================================================================
log "Step 2/8 — Installing system dependencies"

apt-get update -qq
apt-get install -y \
    curl wget git nano \
    python3 python3-pip python3-venv \
    i2c-tools \
    udev \
    build-essential \
    2>/dev/null

# =============================================================================
# STEP 3 — ROS2 Humble
# =============================================================================
log "Step 3/8 — ROS2 Humble"

if [ -f /opt/ros/humble/setup.bash ]; then
    log "  ROS2 Humble already installed, skipping"
else
    log "  Installing ROS2 Humble..."

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
    apt-get install -y ros-humble-desktop
    apt-get install -y python3-colcon-common-extensions python3-rosdep
fi

# ROS2 additional packages
log "  Installing ROS2 extra packages..."
apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    2>/dev/null

# rosdep init
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    log "  Initializing rosdep..."
    rosdep init
fi

sudo -u $ROBOT_USER rosdep update 2>/dev/null || true

# =============================================================================
# STEP 4 — Python venv
# =============================================================================
log "Step 4/8 — Python virtual environment"

if [ -d "$VENV_DIR" ]; then
    log "  venv already exists, skipping"
else
    log "  Creating venv at $VENV_DIR..."
    sudo -u $ROBOT_USER python3 -m venv "$VENV_DIR" --system-site-packages
fi

# Install Python dependencies
log "  Installing Python packages..."
sudo -u $ROBOT_USER "$VENV_DIR/bin/pip" install --quiet \
    pyserial \
    smbus2 \
    luma.oled \
    RPi.GPIO 2>/dev/null || true

# =============================================================================
# STEP 5 — Clone repo
# =============================================================================
log "Step 5/8 — Repository"

if [ -d "$REPO_DIR/.git" ]; then
    log "  Repo already cloned, pulling latest..."
    sudo -u $ROBOT_USER git -C "$REPO_DIR" pull
else
    log "  Cloning OPEN-01 repo..."
    sudo -u $ROBOT_USER git clone "$REPO_URL" "$REPO_DIR"
fi

# =============================================================================
# STEP 6 — Build ROS2 workspace
# =============================================================================
log "Step 6/8 — Building ROS2 workspace"

# Source ROS2
source /opt/ros/humble/setup.bash

cd "$ROS_WS"
sudo -u $ROBOT_USER bash -c "
    source /opt/ros/humble/setup.bash
    cd $ROS_WS
    colcon build --symlink-install
"

# Add ROS2 source to .bashrc
BASHRC="$ROBOT_HOME/.bashrc"
if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC"; then
    log "  Adding ROS2 to .bashrc..."
    echo "" >> "$BASHRC"
    echo "# ROS2 Humble" >> "$BASHRC"
    echo "source /opt/ros/humble/setup.bash" >> "$BASHRC"
    echo "source $ROS_WS/install/setup.bash" >> "$BASHRC"
    echo "export ROS_DOMAIN_ID=0" >> "$BASHRC"
fi

# =============================================================================
# STEP 7 — udev rules
# =============================================================================
log "Step 7/8 — udev rules"

UDEV_RULE='/etc/udev/rules.d/99-open01.rules'
if [ -f "$UDEV_RULE" ]; then
    log "  udev rules already exist, skipping"
else
    log "  Installing udev rules..."
    cat > "$UDEV_RULE" << 'EOF'
# OPEN-01 udev rules
# LDS-02 lidar via USB2LDS adapter (CP210x)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyLIDAR", MODE="0666"
EOF
    udevadm control --reload-rules
    udevadm trigger
fi

# Add user to dialout group
if ! groups "$ROBOT_USER" | grep -q "dialout"; then
    log "  Adding $ROBOT_USER to dialout group..."
    usermod -aG dialout "$ROBOT_USER"
    warn "  Log out and back in for dialout group to take effect"
else
    log "  User already in dialout group, skipping"
fi

# =============================================================================
# STEP 8 — systemd service
# =============================================================================
log "Step 8/8 — systemd service"

SERVICE_FILE='/etc/systemd/system/open01.service'
if [ -f "$SERVICE_FILE" ]; then
    log "  open01.service already exists, skipping"
else
    log "  Installing open01.service..."
    cat > "$SERVICE_FILE" << EOF
[Unit]
Description=OPEN-01 Robot
After=network.target

[Service]
User=$ROBOT_USER
WorkingDirectory=$REPO_DIR
Environment="HOME=$ROBOT_HOME"
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source $ROS_WS/install/setup.bash && ros2 launch open01_bringup bringup.launch.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload
    systemctl enable open01.service
fi

# =============================================================================
# Done
# =============================================================================
echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}  OPEN-01 RPi setup complete!${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "Next steps:"
echo "  1. Reboot: sudo reboot"
echo "  2. Start robot: sudo systemctl start open01.service"
echo "  3. Check status: sudo systemctl status open01.service"
echo ""