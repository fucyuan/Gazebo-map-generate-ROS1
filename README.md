# Gazebo Map Creator

[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![ROS](https://img.shields.io/badge/ROS-Noetic-34aec5.svg)](https://wiki.ros.org/noetic)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange.svg)](http://gazebosim.org/)

A ROS plugin and toolset for generating 2D/3D maps from Gazebo world files. This tool allows you to scan Gazebo simulations and export the results as point clouds, occupancy grids, and various navigation-compatible formats.

## Features

- **2D/3D Map Generation** - Generate both 2D occupancy grids and 3D point clouds from Gazebo worlds
- **One-Click Script** - Automated script that launches Gazebo, waits for services, and exports maps
- **Multiple Output Formats** - Exports to PCD, PGM, PNG, YAML, and OctoMap (.bt) formats
- **Customizable Resolution** - Adjustable scanning resolution for detail vs. performance tradeoffs
- **Progress Estimation** - Real-time progress tracking with calibration support
- **Example Terrains** - Included sample terrain models (desert) for testing

## Installation

### Prerequisites

- Ubuntu 20.04
- ROS 1 Noetic
- Gazebo 11 (Classic)

### Dependencies

```bash
sudo apt update
sudo apt install -y libboost-dev libpcl-dev liboctomap-dev
```

### Build from Source

```bash
# Clone the repository
git clone <repository-url> GM
cd GM

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Build the workspace
catkin_make
source ./devel/setup.bash
```

## Quick Start

```bash
# Generate a map with default parameters
./generate_map.sh
```

This will:
1. Launch Gazebo with the default world file
2. Scan the specified area
3. Export the map to `kongdi_map.{pcd,bt,pgm,png,yaml}`

## Usage

### Generate Map Script

The `generate_map.sh` script provides a one-click solution for map generation.

```bash
# Default parameters
./generate_map.sh

# Custom world and scan area
./generate_map.sh src/gazebo_map_creator/kongdi.world -30 -30 -1 30 30 8

# Custom resolution and output prefix
./generate_map.sh src/gazebo_map_creator/kongdi.world -25 -25 -3 25 25 10 0.1 maps/kongdi_map
```

脚本行为：
- 启动前清理旧的 Gazebo 进程
- 等待 `save_map` 服务
- 显示估算进度（可通过 `EST_STEPS_PER_SEC` 调整）
- 结束后自动关闭 Gazebo

进度估算调优：

```bash
EST_STEPS_PER_SEC=380000 ./generate_map.sh
```

#### Parameters

| Argument | Description | Default |
|----------|-------------|---------|
| `world_file` | Path to Gazebo world file | `src/gazebo_map_creator/kongdi.world` |
| `x_min`, `y_min`, `z_min` | Lower-left corner coordinates | `-25, -25, -3` |
| `x_max`, `y_max`, `z_max` | Upper-right corner coordinates | `25, 25, 10` |
| `resolution` | Scan resolution in meters | `0.05` |
| `output_file` | Output file prefix | `kongdi_map` |

#### Environment Variables

```bash
# Adjust progress estimation speed (default: 380000)
EST_STEPS_PER_SEC=500000 ./generate_map.sh
```

### Progress Calibration Script

Use `estimate_steps.sh` to auto-calibrate progress estimation for your system.

```bash
# Default calibration
./estimate_steps.sh

# Custom parameters
./estimate_steps.sh src/gazebo_map_creator/kongdi.world -30 -30 -1 30 30 8 0.1 maps/kongdi_map

# Adjust calibration fraction (default: 0.5)
CALIBRATION_FRACTION=0.3 ./estimate_steps.sh
```

### Using Custom Terrains

Example world with desert terrain:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:models
gazebo worlds/desert_mesh.world
```

Terrain customization:
- **Scale**: Edit `models/desert_mesh/model.sdf` (current: `3 3 3`)
- **Position**: Edit `worlds/desert_mesh.world`
- **Texture**: Replace `models/desert_mesh/materials/textures/sand.png`

## Output Formats

The map generation produces multiple files:

| Format | Extension | Description |
|--------|-----------|-------------|
| Point Cloud | `.pcd` | PCL point cloud format |
| OctoMap | `.bt` | Binary OctoMap for 3D navigation |
| Occupancy Grid | `.pgm` | Portable Gray Map format |
| Image | `.png` | PNG visualization |
| Metadata | `.yaml` | Navigation stack configuration |

## Project Structure

```
GM/
├── README.md
├── generate_map.sh           # Main map generation script
├── estimate_steps.sh         # Progress calibration script
├── models/
│   └── desert_mesh/          # Example terrain model
├── worlds/
│   └── desert_mesh.world     # Example world file
├── src/
│   └── gazebo_map_creator/
│       ├── gazebo_map_creator/
│       │   └── src/gazbeo_map_creator.cpp  # Core plugin
│       └── gazebo_map_creator_interface/
│           └── srv/MapRequest.srv          # ROS service definition
├── build/
└── devel/
```

## Troubleshooting

**Q: Gazebo fails to start?**

Check for existing Gazebo processes:
```bash
killall gzserver gzclient gazebo
```

**Q: Progress estimation is inaccurate?**

Run the calibration script:
```bash
./estimate_steps.sh
```

Or manually adjust:
```bash
EST_STEPS_PER_SEC=380000 ./generate_map.sh
```

**Q: Compilation errors?**

Ensure all dependencies are installed:
```bash
sudo apt install -y libboost-dev libpcl-dev liboctomap-dev
source /opt/ros/noetic/setup.bash
catkin_make clean && catkin_make
```

## Acknowledgments

This project is inspired by and builds upon ideas from [arshadlab/gazebo_map_creator](https://github.com/arshadlab/gazebo_map_creator), which provides a ROS 2 implementation.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](src/gazebo_map_creator/LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request
