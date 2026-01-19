#!/bin/bash

# 估计EST_STEPS_PER_SEC并自动调用generate_map.sh
# 用法: ./estimate_steps.sh [world文件] [x_min] [y_min] [z_min] [x_max] [y_max] [z_max] [resolution] [output]

set -e

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

CALIBRATION_FRACTION="${CALIBRATION_FRACTION:-0.5}"

cleanup_gazebo() {
    killall gzserver gzclient gazebo 2>/dev/null || true
}

if ! command -v gazebo >/dev/null 2>&1; then
    echo "Error: 未找到gazebo命令"
    exit 1
fi

if [ ! -f "$WORLD_FILE" ]; then
    echo "Error: World文件不存在: $WORLD_FILE"
    exit 1
fi

if [ ! -f "$PLUGIN_PATH" ]; then
    echo "Error: 插件不存在，请先编译项目"
    exit 1
fi

# 尝试加载ROS环境
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
else
    source /opt/ros/*/setup.bash 2>/dev/null || true
fi

if [ -f "$GM_DIR/devel/setup.bash" ]; then
    source "$GM_DIR/devel/setup.bash"
fi

# 计算校准区域（以中心为基准缩放）
read -r CAL_X_MIN CAL_Y_MIN CAL_Z_MIN CAL_X_MAX CAL_Y_MAX CAL_Z_MAX < <(
    awk -v xmin="$X_MIN" -v ymin="$Y_MIN" -v zmin="$Z_MIN" \
        -v xmax="$X_MAX" -v ymax="$Y_MAX" -v zmax="$Z_MAX" \
        -v f="$CALIBRATION_FRACTION" 'BEGIN{
            cx=(xmin+xmax)/2; cy=(ymin+ymax)/2; cz=(zmin+zmax)/2;
            rx=(xmax-xmin)*f/2; ry=(ymax-ymin)*f/2; rz=(zmax-zmin)*f/2;
            print cx-rx, cy-ry, cz-rz, cx+rx, cy+ry, cz+rz;
        }'
)

cleanup_gazebo
gazebo -s "$PLUGIN_PATH" "$WORLD_FILE" > /dev/null 2>&1 &
GZ_PID=$!
sleep 10

if ! ps -p "$GZ_PID" > /dev/null 2>&1; then
    echo "Error: Gazebo启动失败"
    exit 1
fi

TMP_DIR="$(mktemp -d /tmp/map_calib.XXXXXX)"
CALIB_OUT="$TMP_DIR/calib_map"
TIME_LOG="$TMP_DIR/time.log"
REQ_LOG="$TMP_DIR/request.log"

/usr/bin/time -p python3 "$GM_DIR/devel/lib/gazebo_map_creator/request_map.py" \
    -c "($CAL_X_MIN,$CAL_Y_MIN,$CAL_Z_MIN)($CAL_X_MAX,$CAL_Y_MAX,$CAL_Z_MAX)" \
    -r "$RESOLUTION" \
    -f "$CALIB_OUT" > "$REQ_LOG" 2> "$TIME_LOG"

cleanup_gazebo

REAL_SEC="$(awk '/^real/ {print $2}' "$TIME_LOG")"
if [ -z "$REAL_SEC" ]; then
    echo "Error: 无法获取耗时"
    rm -rf "$TMP_DIR"
    exit 1
fi

EST_STEPS_PER_SEC="$(awk -v xmin="$CAL_X_MIN" -v ymin="$CAL_Y_MIN" -v zmin="$CAL_Z_MIN" \
    -v xmax="$CAL_X_MAX" -v ymax="$CAL_Y_MAX" -v zmax="$CAL_Z_MAX" -v res="$RESOLUTION" \
    -v t="$REAL_SEC" 'BEGIN{
        dx=(xmax-xmin)/res; dy=(ymax-ymin)/res; dz=(zmax-zmin)/res;
        if (dx<1) dx=1; if (dy<1) dy=1; if (dz<1) dz=1;
        steps=dx*dy*dz; rate=steps/t;
        printf "%.0f", rate;
    }')"

rm -rf "$TMP_DIR"

echo "估算EST_STEPS_PER_SEC=$EST_STEPS_PER_SEC"

EST_STEPS_PER_SEC="$EST_STEPS_PER_SEC" "$GM_DIR/generate_map.sh" \
    "$WORLD_FILE" "$X_MIN" "$Y_MIN" "$Z_MIN" "$X_MAX" "$Y_MAX" "$Z_MAX" "$RESOLUTION" "$OUTPUT_FILE"
