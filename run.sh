#!/bin/bash

echo "正在执行比赛程序"

# 在这里添加您的比赛程序代码运行指令

# Source your workspace's setup.bash file
source ~/mtuav-competition-2024/devel/setup.bash  # 根据你的工作空间路径修改

# Run the ROS node
rosrun race_demo demo.py

echo "程序执行完毕"
