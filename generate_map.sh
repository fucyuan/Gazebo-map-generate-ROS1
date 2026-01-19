#!/bin/bash

# Gazebo 地图生成器一键启动脚本
# 用法: ./generate_map.sh [world文件] [x_min] [y_min] [z_min] [x_max] [y_max] [z_max] [resolution]

set -e

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认参数
WORLD_FILE="${1:-$SCRIPT_DIR/src/gazebo_map_creator/kongdi.world}"
X_MIN="${2:--25}"
Y_MIN="${3:--25}"
Z_MIN="${4:--3}"
X_MAX="${5:-25}"
Y_MAX="${6:-25}"
Z_MAX="${7:-10}"
RESOLUTION="${8:-0.05}"
OUTPUT_FILE="${9:-$SCRIPT_DIR/kongdi_map}"

GM_DIR="$SCRIPT_DIR"
PLUGIN_PATH="$GM_DIR/devel/lib/libgazebo_map_creator.so"

GAZEBO_PID=""

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

echo_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

echo_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

cleanup_gazebo() {
    killall gzserver gzclient gazebo 2>/dev/null || true
}

# 基础依赖检查
if ! command -v gazebo >/dev/null 2>&1; then
    echo_error "未找到gazebo命令，请先安装Gazebo或检查环境变量"
    exit 1
fi

# 检查文件是否存在
if [ ! -f "$WORLD_FILE" ]; then
    echo_error "World文件不存在: $WORLD_FILE"
    exit 1
fi

if [ ! -f "$PLUGIN_PATH" ]; then
    echo_error "插件不存在，请先编译项目:"
    echo "  source /opt/ros/noetic/setup.bash"
    echo "  cd $GM_DIR"
    echo "  catkin_make"
    exit 1
fi

# 显示配置
echo_info "========================================="
echo_info "地图生成配置"
echo_info "========================================="
echo "  工作目录: $GM_DIR"
echo "  World文件: $WORLD_FILE"
echo "  扫描区域: ($X_MIN, $Y_MIN, $Z_MIN) 到 ($X_MAX, $Y_MAX, $Z_MAX)"
echo "  分辨率: ${RESOLUTION}m"
echo "  输出文件: $OUTPUT_FILE"
echo_info "========================================="
echo ""

# 检查并杀掉旧的Gazebo进程（包含 gzserver/gzclient）
if pgrep -f "gzserver|gzclient|gazebo" > /dev/null 2>&1; then
    echo_warn "检测到Gazebo正在运行，正在停止..."
    cleanup_gazebo
    sleep 3
fi

# 启动Gazebo
echo_info "启动Gazebo..."

# 检测ROS环境
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
else
    echo_warn "未找到ROS Noetic，尝试使用其他ROS版本..."
    # 尝试从系统路径加载
    source /opt/ros/*/setup.bash 2>/dev/null || true
fi

if [ -f "$GM_DIR/devel/setup.bash" ]; then
    source "$GM_DIR/devel/setup.bash"
else
    echo_warn "未找到devel/setup.bash，可能尚未编译或未在工作区根目录运行"
fi

# 后台启动Gazebo
gazebo -s "$PLUGIN_PATH" "$WORLD_FILE" > /dev/null 2>&1 &
GAZEBO_PID=$!

echo_info "等待Gazebo加载（约10秒）..."
sleep 10

# 检查Gazebo是否启动成功
if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
    echo_error "Gazebo启动失败（可检查 ~/.gazebo 日志获取更多信息）"
    exit 1
fi

# 检查ROS服务是否就绪
echo_info "检查ROS服务..."
MAX_RETRIES=10
RETRY_COUNT=0
while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    if rosservice list 2>&1 | grep -q "save_map"; then
        echo_info "save_map服务已就绪"
        break
    fi
    RETRY_COUNT=$((RETRY_COUNT + 1))
    echo "等待服务... ($RETRY_COUNT/$MAX_RETRIES)"
    sleep 2
done

if [ $RETRY_COUNT -eq $MAX_RETRIES ]; then
    echo_error "save_map服务未找到"
    exit 1
fi

# 生成地图（后台 + 估算进度）
echo ""
echo_info "开始生成地图（耗时可能较长，进度为估算值）..."
TMP_LOG="$(mktemp /tmp/map_request.XXXXXX)"
python3 "$GM_DIR/devel/lib/gazebo_map_creator/request_map.py" \
    -c "($X_MIN,$Y_MIN,$Z_MIN)($X_MAX,$Y_MAX,$Z_MAX)" \
    -r "$RESOLUTION" \
    -f "$OUTPUT_FILE" > "$TMP_LOG" 2>&1 &
MAP_PID=$!

# 估算步数与耗时（可通过环境变量调优速率）
EST_STEPS_PER_SEC="${EST_STEPS_PER_SEC:-380000}"
EST_TOTAL_SEC="$(awk -v xmin="$X_MIN" -v ymin="$Y_MIN" -v zmin="$Z_MIN" \
    -v xmax="$X_MAX" -v ymax="$Y_MAX" -v zmax="$Z_MAX" -v res="$RESOLUTION" \
    -v rate="$EST_STEPS_PER_SEC" 'BEGIN{
        dx=(xmax-xmin)/res; dy=(ymax-ymin)/res; dz=(zmax-zmin)/res;
        if (dx<1) dx=1; if (dy<1) dy=1; if (dz<1) dz=1;
        steps=dx*dy*dz; sec=steps/rate;
        if (sec<5) sec=5;
        printf "%.0f", sec;
    }')"

START_TS="$(date +%s)"
while kill -0 "$MAP_PID" 2>/dev/null; do
    NOW_TS="$(date +%s)"
    ELAPSED="$((NOW_TS - START_TS))"
    PCT="$(awk -v e="$ELAPSED" -v t="$EST_TOTAL_SEC" 'BEGIN{
        p=int((e/t)*100); if (p>99) p=99; if (p<0) p=0; print p;
    }')"
    printf "\r[INFO] 生成地图中... %3d%%" "$PCT"
    sleep 0.5
done
printf "\r[INFO] 生成地图中... 100%%\n"

set +e
wait "$MAP_PID"
STATUS=$?
set -e

cat "$TMP_LOG"
rm -f "$TMP_LOG"

# 检查生成结果
if [ $STATUS -eq 0 ]; then
    echo ""
    echo_info "========================================="
    echo_info "地图生成成功！"
    echo_info "========================================="
    ls -lh "$OUTPUT_FILE".*
else
    echo_error "地图生成失败"
fi

# 脚本完成后统一关闭Gazebo
echo_info "脚本完成，关闭Gazebo..."
cleanup_gazebo
