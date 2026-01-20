#!/bin/bash

# 引入 TMUX 会话管理模块
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
source "${SCRIPT_DIR}/auto_tmux.sh"

# ===================== 配置区域 =====================
SESSION_NAME=sunray_tmux
FIRST_WINDOW="main.1"
LAYOUT="even-horizontal"

declare -A TMUX_CONFIG=(
    ["main"]="
        roslaunch mavros px4.launch
        sleep 5 && roslaunch vins vins_rviz.launch
        sleep 5 && roslaunch vins realsense_d415.launch
        sleep 5 && roslaunch realsense2_camera rs_camera_2.launch
    "
)
# ===================== 配置结束 =====================
# roslaunch sunray_uav_control sunray_mavros_exp.launch
# 创建会话
create_tmux_session

# 可选：附加到会话
attach_to_tmux_session
