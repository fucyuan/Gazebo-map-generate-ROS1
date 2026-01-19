# Gazebo 地图生成工具

从 Gazebo 世界生成 2D 和 3D 地图的 ROS 插件与工具脚本。

注：本项目思路借鉴自 https://github.com/arshadlab/gazebo_map_creator.git（其为 ROS 2 实现）。

## 功能概览

- 从 Gazebo world 扫描生成 2D 栅格地图与 3D 点云地图
- 提供一键启动脚本：自动启动 Gazebo、等待服务、导出地图
- 支持自定义扫描区域与分辨率

## 目录结构

```
GM/
├── README.md
├── .gitignore       # Git忽略文件
├── generate_map.sh   # 一键启动脚本
├── build/          # 编译目录
├── devel/          # 编译输出
│   └── lib/
│       └── libgazebo_map_creator.so   # Gazebo插件
├── kongdi.world    # 示例world文件
└── src/
    └── gazebo_map_creator/
        └── scripts/
            └── request_map.py   # 地图请求脚本
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

## 编译

```bash
source <ros-setup.bash>   # 例如: ROS 安装目录中的 setup.bash
cd GM                     # 或进入你的仓库根目录
catkin_make
source ./devel/setup.bash
```

## 一键启动脚本

**generate_map.sh** - 一键启动脚本，自动完成以下步骤：
1. 检测并停止旧的 Gazebo 进程（gzserver/gzclient）
2. 启动 Gazebo（带地图生成插件）
3. 等待 ROS 服务就绪
4. 生成地图
5. 自动关闭 Gazebo

**使用方法：**

```bash
# 默认参数生成（区域: -25,-25,-3 到 25,25,10，分辨率: 0.1）
./generate_map.sh

# 自定义 world 文件（相对路径）
./generate_map.sh src/gazebo_map_creator/kongdi.world

# 自定义扫描区域
./generate_map.sh src/gazebo_map_creator/kongdi.world -30 -30 -1 30 30 8

# 自定义分辨率（0.05 = 高精度，0.1 = 推荐，0.2 = 快速）
./generate_map.sh src/gazebo_map_creator/kongdi.world -25 -25 -3 25 25 10 0.05

# 自定义输出前缀（会生成 .pcd/.pgm/.png/.yaml/.bt）
./generate_map.sh src/gazebo_map_creator/kongdi.world -25 -25 -3 25 25 10 0.1 maps/kongdi_map
```

## 估算进度参数脚本

**estimate_steps.sh** - 估算 `EST_STEPS_PER_SEC` 并自动调用 `generate_map.sh`：

```bash
# 使用默认参数估算并生成地图
./estimate_steps.sh

# 自定义 world 与扫描区域
./estimate_steps.sh src/gazebo_map_creator/kongdi.world -30 -30 -1 30 30 8 0.1 maps/kongdi_map
```

**说明：**

- 默认在更小的扫描区域进行一次校准，以减少耗时
- 可通过 `CALIBRATION_FRACTION` 调整校准区域比例（默认 `0.5`）
- 估算完成后会把 `EST_STEPS_PER_SEC` 传给 `generate_map.sh`

```bash
CALIBRATION_FRACTION=0.3 ./estimate_steps.sh
```

## PCD 转 DAE 网格脚本

**pcd_to_dae.sh** - 从 PCD 生成 mesh，并尽量导出 DAE（用于 Gazebo mesh）：

```bash
# 使用当前目录下的 desert.pcd
./pcd_to_dae.sh

# 指定输入与输出前缀
./pcd_to_dae.sh desert.pcd out/desert_mesh
```

**说明：**

- 依赖 PCL 工具链（`pcl_voxel_grid`/`pcl_normal_estimation`/`pcl_poisson_reconstruction` 等）
- 若系统中存在 `meshlabserver`，会自动输出 `.dae`
- 可通过环境变量调优：
  - `VOXEL_SIZE`（默认 `0.05`）
  - `NORMAL_RADIUS`（默认 `0.1`）
  - `POISSON_DEPTH`（默认 `9`）
 - 若 `meshlabserver` 报 OpenGL/GLEW 错误，可安装 `xvfb` 后重试

## 自定义纹理（desert_mesh）

已为 `desert_mesh` 配置示例沙地纹理：

- 纹理：`models/desert_mesh/materials/textures/sand.png`
- 材质脚本：`models/desert_mesh/materials/scripts/desert.material`

如需替换纹理，直接替换 `sand.png`，或修改 `desert.material` 中的材质名称与贴图路径。


### 脚本参数（位置参数）

| 位置 | 参数 | 说明 | 默认值 |
|------|------|------|--------|
| 1 | world | world 文件路径 | `src/gazebo_map_creator/kongdi.world` |
| 2-4 | x_min y_min z_min | 扫描区域下界 | `-25 -25 -3` |
| 5-7 | x_max y_max z_max | 扫描区域上界 | `25 25 10` |
| 8 | resolution | 分辨率（米） | `0.1` |
| 9 | output | 输出文件前缀 | `kongdi_map` |

### 脚本行为说明

- 检测并清理旧的 Gazebo 进程，避免服务冲突
- Gazebo 启动失败时会直接退出（可查看 Gazebo 日志目录）
- 脚本完成后自动关闭 Gazebo
- 生成地图时会显示估算进度（基于扫描体积与分辨率）

#### 进度估算调优

脚本使用 `EST_STEPS_PER_SEC` 估算扫描速度（默认 `380000`），你可以通过环境变量调大或调小：

```bash
EST_STEPS_PER_SEC=1500000 ./generate_map.sh
```

## 手动启动

### 终端 1 - 启动 Gazebo（带地图生成插件）

```bash
source <ros-setup.bash>
source devel/setup.bash
gazebo -s devel/lib/libgazebo_map_creator.so src/gazebo_map_creator/kongdi.world
```

### 终端 2 - 生成地图

```bash
source <ros-setup.bash>
source devel/setup.bash

# 基本用法（区域 50x50x13米，分辨率 0.1）
python3 devel/lib/gazebo_map_creator/request_map.py \
    -c '(-25,-25,-3)(25,25,10)' \
    -r 0.1 \
    -f kongdi_map
```

## request_map 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-c` | 扫描区域 (x_min,y_min,z_min)(x_max,y_max,z_max) | (-10,-10,0.05)(10,10,10) |
| `-r` | 分辨率（单位：米） | 0.01 |
| `-d` | 碰撞距离倍数 | 0.55 |
| `--skip-vertical-scan` | 跳过垂直扫描（快速模式，适合2D地图） | - |
| `-t` | 2D地图像素阈值 | 255 |
| `-f` | 输出文件前缀 | map |

## 生成的文件

| 文件 | 说明 |
|------|------|
| `kongdi_map.pcd` | 3D点云地图 |
| `kongdi_map.pgm` | 2D栅格地图 |
| `kongdi_map.yaml` | 地图配置文件 |
| `kongdi_map.bt` | OctoMap八叉树地图 |
| `kongdi_map.png` | PNG格式图像 |

## 查看结果

```bash
# 查看 2D 地图
xdg-open kongdi_map.pgm

# 查看 3D 点云（需安装 pcl-tools）
pcl_viewer kongdi_map.pcd
```

## 环境要求

- ROS1 Noetic
- Gazebo 11 (Classic)
- 依赖库：
  - libboost-dev
  - libpcl-dev
  - liboctomap-dev

## 注意事项

1. **分辨率选择**：
   - `0.1`：推荐，点云大小约9MB，扫描速度快
   - `0.05`：高分辨率，点云大小约27MB，扫描时间长
   - `0.01`：超高分辨率，扫描非常慢

2. **快速模式**：`--skip-vertical-scan` 适合仅生成2D地图，3D点云可能不完整

3. **Gazebo问题**：如果扫描结果异常，重启Gazebo后再试

## 常见问题

**Q: 生成的点云文件很小或为空？**
A: 重启Gazebo进程，确保插件正确加载。

**Q: 扫描速度很慢？**
A: 降低分辨率参数 `-r`，或使用快速模式 `--skip-vertical-scan`。
