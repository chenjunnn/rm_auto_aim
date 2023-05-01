# rm_auto_aim

## Overview

RoboMaster 装甲板自瞄算法模块

<img src="docs/rm_vision.svg" alt="rm_vision" width="200" height="200">

该项目为 [rm_vision](https://github.com/chenjunnn/rm_vision) 的子模块

若有帮助请Star这个项目，感谢~

### License

The source code is released under a [MIT license](rm_auto_aim/LICENSE).

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

Author: Chen Jun

运行环境：Ubuntu 22.04 / ROS2 Humble (未在其他环境下测试)

![Build Status](https://github.com/chenjunnn/rm_auto_aim/actions/workflows/ros_ci.yml/badge.svg)

## Building from Source

### Building

在 Ubuntu 22.04 环境下安装 [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

创建 ROS 工作空间后 clone 项目，使用 rosdep 安装依赖后编译代码

	cd ros_ws/src
	git clone https://github.com/chenjunnn/rm_auto_aim.git
	cd ..
	rosdep install --from-paths src --ignore-src -r -y
	colcon build --symlink-install --packages-up-to auto_aim_bringup

### Testing

Run the tests with

	colcon test --packages-up-to auto_aim_bringup

## Packages

- [armor_detector](armor_detector)

	订阅相机参数及图像流进行装甲板的识别并解算三维位置，输出识别到的装甲板在输入frame下的三维位置 (一般是以相机光心为原点的相机坐标系)

- [armor_processor](armor_processor)

	订阅识别节点发布的装甲板三维位置及机器人的坐标转换信息，将装甲板三维位置变换到指定惯性系下，然后将目标送入跟踪器中得到跟踪目标在指定惯性系下（一般是以云台中心为原点，IMU 上电时的 Yaw 朝向为 X 轴的惯性系）的位置及速度，再经过小陀螺状态检测器的处理后，发布最终的目标位置和速度

- auto_aim_interfaces

	定义了识别节点和处理节点的接口以及定义了用于 Debug 的信息

- auto_aim_bringup

	包含启动识别节点和处理节点的默认参数文件及 launch 文件
