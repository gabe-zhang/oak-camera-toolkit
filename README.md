# OAK Camera Toolkit

[![Release](https://img.shields.io/github/v/release/gabe-zhang/oak-camera-toolkit)](https://github.com/gabe-zhang/oak-camera-toolkit/releases) [![Python](https://img.shields.io/badge/python-%3E%3D3.10-blue)](https://www.python.org/) [![OpenCV](https://img.shields.io/badge/OpenCV-27338e?logo=OpenCV&logoColor=white)](https://opencv.org/) [![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

A toolkit for [OAK-D](https://docs.luxonis.com/) stereo cameras featuring real-time collision avoidance, object detection, and device management. Built with [DepthAI v3](https://docs.luxonis.com/software/depthai/).

## Tools

| Script | Description |
|--------|-------------|
| `collision_avoidance.py` | Real-time collision avoidance using stereo depth with grid-based spatial analysis |
| `detection_network.py` | YOLOv6-nano object detection with live bounding boxes and FPS tracking |
| `device_info.py` | Device diagnostics — sensors, calibration, intrinsics, and bootloader info |
| `device_manager.py` | GUI-based device management for configuration, flashing, and network setup |

## Hardware

**OAK-D Camera** (PoE or USB) — tested with OAK-D-W-POE

## Installation

Requires Python 3.10+ and [uv](https://github.com/astral-sh/uv).

```bash
git clone https://github.com/gabe-zhang/oak-camera-toolkit.git
cd oak-camera-toolkit
uv sync
```

## Usage

Set your camera's IP address:

```bash
cp .env.example .env # Edit .env with your camera's IP address
# Or
export OAK_DEVICE_IP=<your-camera-ip>
```

Then run:

```bash
# Collision avoidance
uv run collision_avoidance.py

# Object detection
uv run detection_network.py

# Device info
uv run device_info.py

# Device manager GUI
uv run device_manager.py
```

### Collision Avoidance

Divides the camera's field of view into a 15x9 grid (135 regions) and calculates real-time distances using stereo depth. Objects within the critical distance are highlighted with red overlays.


### Detection Network

Runs YOLOv6-nano inference on the RGB camera with real-time bounding boxes, class labels, and confidence scores.

## Configuration

Default settings in `collision_avoidance.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| Critical Distance | 2.0m | Adjustable via slider (0.5m–5m) |
| Depth Range | 0.2m–10m | SpatialLocationCalculator thresholds |
| Grid Size | 15 x 9 | Spatial analysis regions |
| Stereo Preset | ROBOTICS | Optimized for outdoor/mobile use |

## Third-Party Code

`device_manager.py` is from the [DepthAI Python repository](https://github.com/luxonis/depthai-python/blob/main/utilities/device_manager.py) by Luxonis, licensed under the MIT License.

## License

[MIT](LICENSE)
