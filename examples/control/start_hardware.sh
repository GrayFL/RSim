#!/usr/bin/env bash
launch_entry=$(realpath -- "${BASH_SOURCE[0]}")
source "$(dirname -- "$launch_entry")/../_launch.sh"
launch_group=1
LAUNCH_WINDOWS=(stm32 imu bluesea)
launch_help() { echo 'Local native ROS2 hardware only. Select windows: start [stm32 imu bluesea].'; }
launch_command() {
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    case $1 in
        stm32)
            CMD=(ros2 run rsim_stm32 stm32_node --ros-args
                -r __ns:=/rsim/chassis -r __node:=stm32_driver
                -p "port:=${RSIM_STM32_PORT:?Set RSIM_STM32_PORT}"
                -p "motion_enabled:=${RSIM_HARDWARE_MOTION:-false}" "${STM32_ROS_ARGS[@]}") ;;
        imu)
            CMD=(ros2 run rsim_hipnuc serial_node --ros-args
                -p "port:=${RSIM_IMU_PORT:?Set RSIM_IMU_PORT}" -p baudrate:=460800
                -p frame_id:=imu_frame -p navigation_frame:=enu
                -r imu/data:=/rsim/chassis/imu/data "${IMU_ROS_ARGS[@]}") ;;
        bluesea)
            CMD=(ros2 run bluesea2 bluesea2_node --ros-args
                -p type:=uart -p "port:=$(realpath -- "${RSIM_BLUESEA_PORT:?Set RSIM_BLUESEA_PORT}")"
                -p baud_rate:=500000 -p frame_id:=laser_frame -p raw_bytes:=3
                -p output_360:=true -p output_scan:=true -p output_cloud:=false -p output_cloud2:=false
                -p with_angle_filter:=false -p max_dist:=50.0 -p inverted:=true
                -p scan_topic:=/rsim/chassis/scan_raw "${BLUESEA_ROS_ARGS[@]}") ;;
    esac
}
launch_main hardware "$@"
