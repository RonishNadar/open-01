# Robot Firmware Tools

## First Time Setup (run once after cloning)

```bash
chmod +x tools/setup.sh
./tools/setup.sh
```

This will:
- Install all system dependencies (apt/dnf/pacman auto-detected)
- Clone and install ESP-IDF v5.4
- Add `idf` alias to your shell
- Add your user to the `dialout` group for USB access
- Install Python GUI dependencies

## GUI Launcher

```bash
python3 tools/launcher.py
```

| Button | What it does |
|---|---|
| Build | Compile the firmware |
| Flash | Write to ESP32-S2 |
| Monitor | Open serial monitor (Ctrl+C to stop) |
| Build+Flash+Monitor | All three in sequence |
| Clean | Full clean build |
| Stop | Kill the running command |

## CLI (alternative)

```bash
# Activate ESP-IDF in any terminal
idf

# Then use idf.py directly
cd firmware
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## Requirements

- Linux (Ubuntu/Debian/Fedora/Arch)
- Python 3.8+
- python3-tk (installed by setup.sh)
