
# Ego-Planner (早期版本 / Early Version)

> [!IMPORTANT]
> **版本重要更新与迁移提示：**
> *   **本项目状态**：这是早期版本，其核心优化器仍为 **3D 结构**。
> *   **实现方式**：在本项目中，我们通过从地图维度限制 Z 轴优化，使 3D 算法勉强适配地面 2D 移动机器人场景。
> *   **最新推荐**：为了获得更高的计算效率和真正的 2D 适配（算法层降维），请访问最新的重构版本：
> 👉 **[Ego-Planner-2D-ROS2 (最新 2D 原生版)](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2)**
> 
> **新版优势**：从数学底层将优化器从 3D 降维至 2D，彻底消除 Z 轴冗余计算，更适合算力受限的嵌入式平台。

---

# EgoPlanner: 2D Real-time Trajectory Optimizer

[![C++](https://img.shields.io/badge/Language-C++-blue.svg)](https://isocpp.org/)
[![Build](https://img.shields.io/badge/Build-CMake-brightgreen.svg)](https://cmake.org/)
[![License](https://img.shields.io/badge/License-MIT-orange.svg)](./LICENSE)

## 📖 项目简介

本项目基于 **Ego-Planner** 后端核心代码进行剥离与封装，实现了一个**纯净版、轻量级**的 B 样条（B-Spline）轨迹优化库。

针对**地面移动机器人**（AGV/AMR）的应用场景，我们将原有的三维轨迹规划算法**降维适配为二维平面**，专注于实现高效、平滑的实时局部路径规划。

**核心特性：**
*   **纯 C++ 实现**：无繁杂依赖，易于集成。
*   **2D 平面适配**：专为地面移动机器人优化。
*   **B 样条优化**：保证轨迹的平滑性与动态可行性。

---

## 🚀 编译与运行 (How to use)

请确保您的环境中已安装 `CMake` 和 `C++` 编译器。

```bash
# 1. 创建构建目录
mkdir build 
cd build

# 2. 编译项目
cmake ..
make

# 3. 运行示例
./EgoPlanCore 
```

---

## 🖼️ 运行效果 (Demo)

下图展示了算法规划出的二维平滑轨迹：

<p align="center">
  <!-- 注意：建议将图片链接替换为 raw 格式以确保在某些网络环境下正常显示，或者使用相对路径 -->
  <img src="https://github.com/JackJu-HIT/EgoPlanner/raw/master/results.png" alt="Trajectory Optimization Result" width="600" />
</p>

---

## 🔗 参考项目

本项目核心算法源自以下优秀的开源项目，特此致谢：

*   **Ego-Planner**: [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner.git)

---

## 📢 更多技术细节

如果您对代码实现细节、算法原理感兴趣，欢迎关注我们的媒体渠道获取更多教程：

*   **微信公众号**：`机器人规划与控制研究所`
*   **Bilibili 视频演示**：[点击观看视频教程](https://www.bilibili.com/video/BV1RgsNz7ETm/)

---

## ⚖️ 开源协议
本项目遵循 [MIT](LICENSE) 开源协议。如果您觉得本项目对您的研究有帮助，欢迎给最新的 [Ego-Planner-2D-ROS2](https://github.com/JackJu-HIT/Ego-Planner-2D-ROS2) 一个 **Star** ⭐！

*Created by JackJu-HIT*

