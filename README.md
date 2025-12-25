# SJTU工程实践课程 - 无人船项目

## 项目简介

这是一个上海交通大学（SJTU）工程实践课程的项目，旨在设计和实现一个基于Arduino的无人船系统。项目通过超声波传感器和电机控制，实现小船在水池中的自主导航，包括直线行驶和绕圈行驶。

## 功能特性

- **直线行驶任务** (`task_straight.ino`): 控制无人船在水池中保持直线前进，避免障碍物。
- **绕圈行驶任务** (`task_turning.ino`): 控制无人船在水池中绕圈行驶。
- **标准测试程序** (`test.ino`): 用于测试硬件和基本功能。
- 使用NewPing库进行超声波测距。
- 双电机驱动系统，支持前进、转向和停止。

## 文件结构

```
ship/
├── README.md                 # 项目说明文档
├── lib/
│   └── README.md            # 库文件说明
├── task_straight/
│   └── task_straight.ino    # 直线行驶任务代码
├── task_turning/
│   └── task_turning.ino     # 绕圈行驶任务代码
└── test/
    └── test.ino             # 标准测试程序
```

## 环境配置

### 系统环境
- **操作系统**: Ubuntu 24.04.01 LTS

### Arduino环境
- **Arduino CLI**: 版本 1.3.1
- **依赖库**: NewPing.h (用于超声波传感器)
  - 安装命令: `arduino-cli lib install NewPing`

### 编辑器环境
- **Visual Studio Code**: 推荐安装Arduino扩展
- **Arduino扩展**: 用于代码编辑、编译和上传

## 安装和使用

### 1. 克隆项目
```bash
git clone <repository-url>
cd ship
```

### 2. 配置Arduino环境
确保Arduino CLI已安装并配置好开发板（例如Arduino Uno）。

### 3. 编译和上传
选择相应的.ino文件，使用Arduino IDE或VS Code Arduino扩展编译并上传到开发板。

#### 示例命令（使用Arduino CLI）:
```bash
# 编译直线行驶任务
arduino-cli compile --fqbn arduino:avr:uno task_straight/task_straight.ino

# 上传到开发板
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno task_straight/task_straight.ino
```

### 4. 硬件连接
- 连接左右电机到指定引脚（见代码注释）
- 连接超声波传感器到指定引脚
- 确保电源供应稳定

## 测试

1. 首先运行 `test/test.ino` 验证硬件连接和基本功能。
2. 在水池中测试 `task_straight/task_straight.ino` 直线行驶。
3. 测试 `task_turning/task_turning.ino` 绕圈行驶。

## 注意事项

- 确保水池环境安全，无其他障碍物。
- 监控电池电量，避免电机过载。
- 根据实际硬件调整传感器参数和电机速度。

## 贡献

欢迎提交Issue和Pull Request来改进项目。

## 许可证

本项目仅用于SJTU工程实践课程学习目的。