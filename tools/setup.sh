#!/bin/bash
# =============================================================
#  Robot Firmware - ESP-IDF Setup Script
#  Run once after cloning the repo
#  Supports: Ubuntu / Debian / Fedora / Arch
# =============================================================

set -e

ESP_IDF_VERSION="v5.4"
ESP_IDF_DIR="$HOME/esp/esp-idf"
TOOLS_DIR="$HOME/.espressif"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[SETUP]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $1"; }
err()  { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# ── Detect distro ────────────────────────────────────────────
detect_distro() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        echo "$ID"
    else
        echo "unknown"
    fi
}

# ── Install system dependencies ──────────────────────────────
install_deps() {
    DISTRO=$(detect_distro)
    log "Detected distro: $DISTRO"

    case "$DISTRO" in
        ubuntu|debian|linuxmint|pop)
            log "Installing apt dependencies..."
            sudo apt update -q
            sudo apt install -y \
                git wget curl flex bison gperf \
                python3 python3-pip python3-venv \
                cmake ninja-build ccache \
                libffi-dev libssl-dev libusb-1.0-0-dev \
                dfu-util pkg-config
            ;;
        fedora)
            log "Installing dnf dependencies..."
            sudo dnf install -y \
                git wget curl flex bison gperf \
                python3 python3-pip \
                cmake ninja-build ccache \
                libffi-devel openssl-devel libusb1-devel \
                dfu-util
            ;;
        arch|manjaro)
            log "Installing pacman dependencies..."
            sudo pacman -Sy --noconfirm \
                git wget curl flex bison gperf \
                python python-pip \
                cmake ninja ccache \
                libffi openssl libusb \
                dfu-util
            ;;
        *)
            warn "Unknown distro. Please install manually:"
            warn "git wget curl flex bison gperf python3 python3-pip cmake ninja libffi-dev libusb-1.0-0-dev"
            ;;
    esac
}

# ── Install ESP-IDF ──────────────────────────────────────────
install_espidf() {
    if [ -d "$ESP_IDF_DIR" ]; then
        warn "ESP-IDF already exists at $ESP_IDF_DIR"
        read -p "Re-install? (y/N): " REPLY
        [[ "$REPLY" =~ ^[Yy]$ ]] || { log "Skipping ESP-IDF install."; return; }
        rm -rf "$ESP_IDF_DIR"
    fi

    log "Cloning ESP-IDF $ESP_IDF_VERSION..."
    mkdir -p "$HOME/esp"
    git clone --branch "$ESP_IDF_VERSION" --depth 1 \
        https://github.com/espressif/esp-idf.git "$ESP_IDF_DIR"

    log "Installing ESP-IDF tools (this takes ~5 minutes)..."
    cd "$ESP_IDF_DIR"
    ./install.sh esp32s2
}

# ── Add to shell rc ──────────────────────────────────────────
setup_shell() {
    EXPORT_LINE=". $ESP_IDF_DIR/export.sh"
    ALIAS_LINE="alias idf='. $ESP_IDF_DIR/export.sh'"

    for RC in "$HOME/.bashrc" "$HOME/.zshrc"; do
        [ -f "$RC" ] || continue
        if ! grep -q "esp-idf/export.sh" "$RC"; then
            log "Adding ESP-IDF alias to $RC"
            echo "" >> "$RC"
            echo "# ESP-IDF" >> "$RC"
            echo "$ALIAS_LINE" >> "$RC"
        fi
    done

    log "Run 'idf' in any terminal to activate ESP-IDF"
}

# ── Add user to dialout ───────────────────────────────────────
setup_usb() {
    if ! groups "$USER" | grep -q dialout; then
        log "Adding $USER to dialout group (required for USB flash)..."
        sudo usermod -aG dialout "$USER"
        warn "Log out and back in for USB permissions to take effect."
    else
        log "USB permissions: OK"
    fi
}

# ── Install Python GUI deps ───────────────────────────────────
install_gui_deps() {
    log "Installing GUI dependencies..."
    DISTRO=$(detect_distro)
    case "$DISTRO" in
        ubuntu|debian|linuxmint|pop)
            sudo apt install -y python3-tk python3-serial
            ;;
        fedora)
            sudo dnf install -y python3-tkinter python3-pyserial
            ;;
        arch|manjaro)
            sudo pacman -Sy --noconfirm tk python-pyserial
            ;;
    esac
    pip3 install pyserial --quiet 2>/dev/null || true
}

# ── Main ──────────────────────────────────────────────────────
main() {
    echo ""
    echo "================================================"
    echo "  Robot Firmware - ESP-IDF Setup"
    echo "================================================"
    echo ""

    install_deps
    install_espidf
    setup_shell
    setup_usb
    install_gui_deps

    echo ""
    echo "================================================"
    log "Setup complete!"
    echo ""
    echo "  Next steps:"
    echo "  1. Open a new terminal (or run: source ~/.bashrc)"
    echo "  2. Run the GUI launcher:  python3 tools/launcher.py"
    echo "  3. Or use CLI directly:   idf && idf.py build flash monitor"
    echo "================================================"
    echo ""
}

main
