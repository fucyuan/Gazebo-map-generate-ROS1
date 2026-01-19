# Gazebo 地图生成工具

从 Gazebo 世界生成 2D/3D 地图的 ROS 插件与脚本工具。

注：本项目思路借鉴自 https://github.com/arshadlab/gazebo_map_creator.git
（其为 ROS 2 实现）。

## 功能概览

- Gazebo world 扫描生成 2D 栅格图与 3D 点云
- 一键脚本自动启动 Gazebo、等待服务并导出地图
- 示例地形模型与最小 world（`desert_mesh`）

## 目录结构

```
GM/
├── README.md
├── generate_map.sh        # 一键生成地图
├── estimate_steps.sh      # 估算进度参数并调用 generate_map.sh
├── desert.pcd             # 示例点云
├── forest.pcd             # 示例点云
├── kongdi_map.*           # 示例输出（pcd/pgm/png/yaml/bt）
├── models/
│   └── desert_mesh/
│       ├── model.config
│       ├── model.sdf
│       ├── meshes/terrain.dae
│       └── materials/
│           ├── scripts/desert.material
│           └── textures/sand.png
├── worlds/
│   └── desert_mesh.world
├── build/
├── devel/
└── src/
```

## 快速开始

```bash
# 1) 进入工作区根目录
cd GM

# 2) 加载 ROS 环境（替换为你本机的 setup.bash）
source <ros-setup.bash>

# 3) 编译
catkin_make
source ./devel/setup.bash

# 4) 一键生成地图（默认参数）
./generate_map.sh
```

## 一键生成地图

**generate_map.sh** 默认参数：
- world: `src/gazebo_map_creator/kongdi.world`
- 扫描区域：`(-25,-25,-3)` 到 `(25,25,10)`
- 分辨率：`0.05`
- 输出前缀：`kongdi_map`

使用示例：

```bash
# 默认参数
./generate_map.sh

# 自定义 world 与扫描区域
./generate_map.sh src/gazebo_map_creator/kongdi.world -30 -30 -1 30 30 8

# 自定义分辨率与输出前缀
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

## 估算进度参数脚本

**estimate_steps.sh** 会先用较小区域做一次校准，计算 `EST_STEPS_PER_SEC` 并自动调用 `generate_map.sh`。

```bash
# 使用默认参数估算并生成地图
./estimate_steps.sh

# 自定义 world 与扫描区域
./estimate_steps.sh src/gazebo_map_creator/kongdi.world -30 -30 -1 30 30 8 0.1 maps/kongdi_map

# 调整校准比例（默认 0.5）
CALIBRATION_FRACTION=0.3 ./estimate_steps.sh
```

## 使用 desert_mesh 示例地形

示例 world：`worlds/desert_mesh.world`（包含 sun + desert_mesh）。

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:models
gazebo worlds/desert_mesh.world
```

地形缩放与位置：
- `models/desert_mesh/model.sdf` 中的 `scale`（当前为 `3 3 3`）
- `worlds/desert_mesh.world` 中的 `pose`

纹理替换：
- 纹理图片：`models/desert_mesh/materials/textures/sand.png`
- 材质脚本：`models/desert_mesh/materials/scripts/desert.material`

## 环境要求

- ROS1 Noetic
- Gazebo 11 (Classic)
- 依赖库：
  - libboost-dev
  - libpcl-dev
  - liboctomap-dev

## 常见问题

**Q: Gazebo 启动失败？**
A: 检查是否已有 Gazebo 进程占用 11345 端口，必要时 `killall gzserver gzclient gazebo`。

**Q: 估算进度不准确？**
A: 调整 `EST_STEPS_PER_SEC` 或使用 `estimate_steps.sh` 做一次校准。
