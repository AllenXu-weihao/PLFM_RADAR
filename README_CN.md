# AERIS-10：开源脉冲线性调频（PLFM）相控阵雷达系统

[![GitHub stars](https://img.shields.io/github/stars/NawfalMotii79/PLFM_RADAR?style=social)](https://github.com/NawfalMotii79/PLFM_RADAR/stargazers)
[![Hardware: CERN-OHL-P](https://img.shields.io/badge/Hardware-CERN--OHL--P-blue.svg)](https://ohwr.org/cern_ohl_p_v2.txt)
[![Software: MIT](https://img.shields.io/badge/Software-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Status: Alpha](https://img.shields.io/badge/Status-Alpha-orange)](https://github.com/NawfalMotii79/PLFM_RADAR)
[![Frequency: 10.5GHz](https://img.shields.io/badge/Frequency-10.5GHz-blue)](https://github.com/NawfalMotii79/PLFM_RADAR)

> **原项目作者**：Nawfal Motii (ABAC INDUSTRY, 摩洛哥) | **GitHub ⭐ 21.7k+**
>
> 本文档为 **AERIS-10 相控阵雷达系统的中文版完整说明文档**，基于原始项目整理翻译，结合硬件架构、信号处理流水线与代码仓库三方面深度解析。

---

## 目录

- [一、项目概述](#一项目概述)
- [二、系统版本对比](#二系统版本对比)
- [三、硬件架构详解](#三硬件架构详解)
  - [3.1 系统总体框图](#31-系统总体框图)
  - [3.2 三大核心子系统](#32-三大核心子系统)
  - [3.3 主板详细组成](#33-主板详细组成)
  - [3.4 天线阵列设计](#34-天线阵列设计)
  - [3.5 外设与传感器系统](#35-外设与传感器系统)
- [四、信号处理流水线](#四信号处理流水线)
  - [4.1 完整六步流程总览](#41-完整六步流程总览)
  - [4.2 发射链路（TX Chain）](#42-发射链路tx-chain)
  - [4.3 接收链路（RX Chain）](#43-接收链路rx-chain)
  - [4.4 FPGA 数字信号处理详解](#44-fpga-数字信号处理详解)
  - [4.5 系统管理与数据融合](#45-系统管理与数据融合)
  - [4.6 Python GUI 可视化](#46-python-gui-可视化)
- [五、FPGA 固件架构深度解析](#五-fpga-固件架构深度解析)
  - [5.1 顶层模块与接口](#51-顶层模块与接口)
  - [5.2 核心模块清单](#52-核心模块清单)
  - [5.3 关键设计技术](#53-关键设计技术)
  - [5.4 时钟域与跨时钟处理](#54-时钟域与跨时钟处理)
  - [5.5 主机命令集](#55-主机命令集)
- [六、软件系统与代码仓库](#六软件系统与代码仓库)
  - [6.1 固件代码结构](#61-固件代码结构)
  - [6.2 仿真与验证体系](#62-仿真与验证体系)
  - [6.3 PCB 制造与装配](#63-pcb-制造与装配)
- [七、快速开始](#七快速开始)
- [八、开源许可协议](#八开源许可协议)
- [九、目录结构总览](#九目录结构总览)

---

## 一、项目概述

### 1.1 什么是 AERIS-10？

**AERIS-10** 是一个**开源、低成本、X 波段（10.5 GHz）相控阵雷达系统**，采用 **脉冲线性调频（Pulse LFM / PLFM）** 调制技术。项目提供完整的硬件设计（原理图、PCB 布局、Gerber 文件）、固件代码（STM32 + FPGA）和上位机软件（Python GUI），面向研究人员、无人机开发者和高阶 SDR 爱好者。

> **核心理念**：使雷达技术民主化——让工程师能够**构建**，而不仅仅是**购买**。

![AERIS-10 天线阵列](8_Utils/Antenna_Array.jpg)

### 1.2 核心特性

| 特性 | 说明 |
|------|------|
| **工作频率** | X 波段 10.5 GHz |
| **完全开源** | 硬件（CERN-OHL-P）+ 软件（MIT）双许可 |
| **双版本配置** | Nexus 3km 短距 / Extended 20km 远距 |
| **电子波束扫描** | 方位角 + 俯仰角 ±45° 电子扫描 + 360° 机械扫描 |
| **FPGA 信号处理** | 脉冲压缩、多普勒 FFT、MTI、CFAR 全链路 |
| **Python GUI** | 实时目标显示 + 地图集成 + 雷达控制 |
| **GPS/IMU 集成** | 实时位置标记 + 姿态校正 |
| **模块化设计** | 电源管理、频率合成、射频前端独立分板 |

### 1.3 PLFM（脉冲线性调频）调制

AERIS-10 的核心技术是 **PLFM（Pulse Linear Frequency Modulation）**：

```
传统 CW 雷达：连续波 → 峰值功率低 → 探测距离近
传统 FMCW 雷达：连续调频 → 测距测速好 → 峰值功率低
AERIS-10 PLFM：脉冲 + 线性调频 → 峰值功率高 + 脉冲压缩增益
                              → 作用距离远 + 抗干扰能力强
```

**LFM 信号特点**：
- 发射短时宽带宽的 Chirp（线性啁啾）信号——频率随时间线性增加或减少
- 通过**匹配滤波（Matched Filtering）**实现**脉冲压缩**——将长脉冲的能量"压缩"到窄脉冲
- 效果：既保持长脉冲的高能量（远距离），又获得短脉冲的高距离分辨率
- 波形参数（周期、时间间隔、频率斜率）可通过 FPGA 完全自定义

---

## 二、系统版本对比

AERIS-10 提供两种配置版本，从实验室到专业应用全覆盖：

| 参数 | **AERIS-10N（Nexus 近程版）** | **AERIS-10E（Extended 远程版）** |
|------|------|------|
| **最大探测距离** | ~3 km | ~20 km |
| **天线阵列** | 8×16 贴片天线阵 (128 单元) | 32×16 介质填充开槽波导阵 (512 单元) |
| **输出功率** | ~1W × 16（ADTR1107 内置功放） | 10W × 16（QPA2962 GaN 功放） |
| **峰值总功率** | ~16 W | **160 W** |
| **功放板** | 不需要（ADTR1107 直接驱动） | 16 块独立 PA 板 |
| **天线增益** | 中等 | 高（波导结构损耗低、方向性强） |
| **适用场景** | 实验室教学、近距离探测、无人机入门 | 远距离监控、无人机防御、专业科研 |
| **成本** | 较低 | 较高 |

### 关键差异说明

- **天线选择**：贴片天线成本低、易加工；开槽波导天线增益高但工艺复杂
- **功率放大**：远程版的 QPA2962 GaN HEMT 提供 10W/通道，是近程版 10 倍
- **共享核心**：两版本使用**完全相同**的频率合成板、主板（FPGA+MCU）、信号处理链路和软件生态——区别仅在前端射频功率和天线

---

## 三、硬件架构详解

### 3.1 系统总体框图

![AERIS-10 系统架构图](8_Utils/RADAR_V6_V2.png)

AERIS-10 采用**高度模块化的三层架构**，三大子系统各司其职：

```
┌─────────────────────────────────────────────────────────────┐
│                    AERIS-10 系统架构                         │
├──────────┬──────────────────┬──────────────────────────────┤
│ 电源管理板 │   频率合成器板    │         主板 (Main Board)     │
│   (PWR)  │   (Freq Synth)   │                              │
│          │                  │  ┌─────┐ ┌──────┐ ┌────────┐  │
│ 电压分配  │ AD9523-1 时钟    │  │DAC  │ │Mixer │ │ADAR1000│  │
│ 滤波降噪  │   ADF4382 ×2    │  │     │ │×2   │ │  ×4    │  │
│ 上电时序  │   (RX/TX PLL)   │  └──┬──┘ └──┬───┘ └───┬────┘  │
│          │                  │     │      │        │        │
│ STM32控制│ STM32 配置       │  ┌──▼──┐ ┌──▼───────▼──┐    │
│ 时序保护  │                  │  │FPGA │ │ ADTR1107 ×16 │    │
│          │                  │  │50T  │ │ (LNA/PA)    │    │
│          │                  │  └──┬──┘ └──────────────┘    │
│          │                  │     │           ↑            │
│          │                  │  ┌──▼───────────┐            │
│          │                  │  │  STM32F746xx │            │
│          │                  │  │ (系统管理)    │            │
│          │                  │  └──────────────┘            │
└──────────┴──────────────────┴──────────────────────────────┘
                    ↓              ↓               ↓
              天线阵列 ←——— RF 开关 ———→ GPS / IMU / 步进电机
```

### 3.2 三大核心子系统

#### ① 电源管理板（Power Management Board）

| 能力 | 说明 |
|------|------|
| **电压供给** | 为所有电子元器件提供所需电压等级 |
| **滤波机制** | 对噪声敏感的外设（GPS/IMU）提供干净电源 |
| **上电/掉电时序** | 由 MCU 严格控制，防止瞬态电压损坏昂贵射频器件 |
| **大功率支撑** | Extended 版需支撑 160W 峰值输出 + 冷却风扇 |

> 时序管理是整个系统的**第一步**——其优先级甚至高于外设配置。

#### ② 频率合成器板（Frequency Synthesizer Board）

**核心芯片**：**AD9523-1** 低抖动时钟发生器 —— 系统的「同步心脏」

为以下模块提供**相位对齐（Phase-Aligned）** 的时钟参考：

```
AD9523-1 (低抖动时钟发生器)
    ├──→ ADF4382 #1  （TX 频率综合器 — 上变频本振）
    ├──→ ADF4382 #2  （RX 频率综合器 — 下变频本振）
    ├──→ DAC (AD9708)  （波形生成采样时钟）
    ├──→ ADC (AD9484)  （数据采集采样时钟）
    └──→ FPGA (XC7A50T)（数字处理系统时钟）
```

**关键价值**：
- 16 通道之间的精确相位对齐是实现 ±45° 电子波束指向的前提
- 全链路相干性确保 PLFM 信号在发射到接收的全过程中保持高度一致
- 支持两种性能版本的统一时序基准

#### ③ 主板（Main Board）—— 系统枢纽

主板上集成了雷达系统的绝大多数关键组件（详见 3.3 节）。

### 3.3 主板详细组成

| 组件 | 型号 | 数量 | 功能 |
|------|------|------|------|
| **DAC** | AD9708 | 1 | 生成 PLFM 啁啾波形（Chirp） |
| **ADC** | AD9484 | 1 | 500 MSPS 高速模数转换 |
| **微波混频器** | LTC5552 | 2 | 上变频(TX) + IF下变频(RX) |
| **4通道移相器** | ADAR1000 | 4 | RX/TX 波束赋形（共16通道） |
| **前端芯片** | ADTR1107 | 16 | RX: LNA / TX: PA（~1W/通道） |
| **FPGA** | XC7A50T-2FTG256I | 1 | 雷达全数字信号处理 |
| **MCU** | STM32F746xx | 1 | 系统管理与外设控制中枢 |
| **I²C ADC** | ADS7830 | 3 | 16路 Idq监测 + 8路温度传感 |
| **I²C DAC** | DAC5578 | 2 | 16路 PA Vg 栅极电压闭环校准 |
| **RF开关** | M3SWA2-34DR+ | 多 | TX/RX 切换与保护 |

##### FPGA 功能全景（XC7A50T）

FPGA 是整个系统的**计算引擎**，位于上游 FTG256 载板上：

```
┌──────────────────────────────────────────────────────────────┐
│                XC7A50T FPGA 信号处理流水线                     │
├──────────────────────────────────────────────────────────────┤
│ 【发射路径】                                                    │
│   PLFM Chirp 数字合成 → DAC 接口 → AD9708                      │
│                                                                │
│ 【接收路径】                                                    │
│   AD9484 400MHz LVDS 数据采集                                  │
│       ↓                                                        │
│   I/Q 正交下变频（NCO 400MHz 数字混频）                         │
│       ↓  400MHz                                                │
│   CIC 5级 4× 抽取滤波（DSP48E1 级联优化）                      │
│       ↓  100MHz (CDC Gray-code 同步)                           │
│   FIR 32 抽头低通滤波（多相 FIR，加法树）                       │
│       ↓                                                        │
│   匹配滤波 / 脉冲压缩（重叠保留法，4段拼接）                   │
│       ↓                                                        │
│   距离单元抽取（1024 → 64 bin）                                 │
│       ↓                                                        │
│   MTI 杂波抑制（可选 2/3 脉冲对消）                            │
│       ↓                                                        │
│   Doppler 处理器（Staggered PRF, 双16点FFT）                   │
│       ↓                                                        │
│   CFAR 恒虚警检测（CA/GO/SO 三种模式自适应阈值）               │
│       ↓                                                        │
│   USB 数据接口（FT601 USB3.0 / FT2232H USB2.0）                │
│                                                                │
│ 【跨层协作】                                                    │
│   混合 AGC 控制（FPGA ↔ STM32 ↔ GUI 闭环）                     │
└──────────────────────────────────────────────────────────────┘
```

##### STM32 微控制器功能全景（STM32F746xx）

MCU 是系统的**「系统管理员」**，协调一切外设：

```
┌──────────────────────────────────────────────────────┐
│              STM32F746xx 功能矩阵                     │
├──────────────────────────────────────────────────────┤
│ ▸ 电源上电 / 掉电严格时序控制                          │
│ ▸ 与 FPGA 高速通信                                   │
│ ▸ AD9523-1 时钟发生器初始化与配置                     │
│ ▸ ADF4382 频率综合器配置（×2，TX/RX 各一）             │
│ ▸ ADAR1000 移相器配置（×4，雷达脉冲序列控制）          │
│ ▸ PA 栅极电压 Vg 校准（DAC5578，启动闭环标定到目标Idq）│
│ ▸ PA 静态电流 Idq 监测（ADS7830 + INA241A x50）      │
│ ▸ GPS 定位（UM982，GUI 地图中心 + 目标位置打标签）     │
│ ▸ IMU 姿态感知（GY-85，Pitch/Roll 坐标修正）           │
│ ▸ BMP180 气压计（环境参考高度）                        │
│ ▸ 步进电机驱动（360° 机械方位扫描）                    │
│ ▸ 温度监控 + 散热风扇自动控制（EN_DIS_COOLING GPIO）  │
│ ▸ RF 开关路由管理                                     │
│ ▸ USB 协议桥接                                        │
└──────────────────────────────────────────────────────┘
```

### 3.4 天线阵列设计

| 版本 | 类型 | 规模 | 特点 | 适用 |
|------|------|------|------|------|
| **Nexus (10N)** | 微带贴片天线 | 8×16 = 128 单元 | 成本低、易加工、紧凑 | 入门实验 |
| **Extended (10E)** | 介质填充开槽波导 | 32×16 = 512 单元 | 高增益、低损耗、定向性强 | 专业应用 |

**天线仿真资源**：
- openEMS（开源电磁仿真）：`5_Simulations/Antenna/`
- MATLAB 方向图计算：`5_Simulations/Matlab/Antenna_array.m`
- KiCad 设计文件：`4_Schematics and Boards Layout/4_6_Schematics/Antennas/`
- 波导规格说明：`4_Schematics and Boards Layout/4_6_Schematics/Antennas/Waveguide/Dielectric_Filled_Waveguide_Array_Spec.docx`

### 3.5 外设与传感器系统

外设赋予雷达**环境意识**、**物理覆盖**和**运行安全性**：

#### 定位与姿态校正层

| 外设 | 型号 | 功能 | 在流水线中的角色 |
|------|------|------|-----------------|
| **GPS 模块** | UM982 | 实时经纬度定位 | GUI 地图自动对中 + 每次检测位置标签 |
| **IMU** | GY-85 | 俯仰(Pitch)/翻滚(Roll)姿态 | 目标坐标物理偏移校正 |
| **气压计** | BMP180 | 环境气压/高度参考 | 多维环境感知补充 |

#### 机械运动层

| 外设 | 功能 |
|------|------|
| **步进电机 + 驱动器** | 补充电子扫描的角度限制，提供 360° 全方位机械扫描 |
| **滑环（Slip-Ring）** | 允许雷达头部无限旋转而不缠绕电缆 |

#### 安全监控层

| 机制 | 说明 |
|------|------|
| **热敏电阻 × 8** | ADS7830 读取射频区域温度 |
| **冷却风扇** | 任一通道超温时 GPIO 自动开启（EN_DIS_COOLING） |
| **RF 开关** | 信号路由管理与 TX/RX 保护 |

---

## 四、信号处理流水线

### 4.1 完整六步流程总览

```
Step 1          Step 2          Step 3          Step 4           Step 5          Step 6
波形生成          上/下变频        波束指向         FPGA 处理        系统管理         可视化
(Waveform Gen)  (Mixing)       (Beam Steer)   (DSP)           (Mgmt)          (GUI)
  │               │               │               │                │               │
  ▼               ▼               ▼               ▼                ▼               ▼
DAC产生         LTC5552        ADAR1000        ADC原始数据      STM32外设        Python GUI
PLFM Chirp      频率变换        16通道相位控制   捕获             协调融合         实时绘图
                                               I/Q下变频        GPS/IMU         地图集成
                                               CIC/FIR滤波     AGC增益         雷达控制
                                               脉冲压缩        电机驱动        跨层反馈
                                               Doppler FFT
                                               MTI对消
                                               CFAR检测
```

**本质**：这是一个跨硬件层的协同过程——模拟信号生成 → 射频前端处理 → 高速数字算法 → 传感器融合 → 人机交互的完整闭环。

### 4.2 发射链路（TX Chain）

```
FPGA (数字合成)  →  DAC (AD9708)  →  LPF (重构滤波)
                                            ↓
                    LTC5552 (上变频至 10.5GHz)
                                            ↓
                    ADAR1000 (移相器, ±45° 波束赋形)
                                            ↓
                    ADTR1107 (PA 功率放大, ~1W 或 10W×GaN)
                                            ↓
                    天线阵列 → 自由空间传播
```

**DAC 是整个探测循环的「发令枪」**：
- 物理产生 LFM 啁啾信号的唯一出口
- 与 ADC 共享 AD9523-1 提供的**相位对齐时钟基准**
- 这种同步是后续 FPGA 执行**脉冲压缩**的物理前提

### 4.3 接收链路（RX Chain）

```
天线回波 → ADTR1107 (LNA 低噪声放大)
                ↓
          ADAR1000 (移相器, 波束合成)
                ↓
          LTC5552 (IF 下变频至中频)
                ↓
          AD9484 (ADC 500 MSPS 数字化)
                ↓
          FPGA (全链路数字信号处理)
```

**ADC 数据捕获是「模拟→数字」的关键转折点**：
- 将物理回波转化为信息流
- 同样受 AD9523-1 精密时序驱动
- 确保发射波形与接收回波的**相位相干性**

### 4.4 FPGA 数字信号处理详解

这是流水线中**计算量最大**的部分，也是 AERIS-10 区别于普通 SDR 雷达的核心竞争力：

#### 4.4.1 数据速率变化

| 阶段 | 采样率 | 数据宽度 | 说明 |
|------|---------|----------|------|
| ADC 原始输入 | 400 MHz | 8 bit | LVDS DDR 双沿采样 |
| DDC 后（I/Q下变频+CIC+FIR） | 100 MHz | 18 bit | 4×抽取 + 通道整形 |
| 匹配滤波后 | 100 MHz | 18 bit | 脉冲压缩输出 |
| 距离抽取后 | 100 MHz | 18 bit | 1024 → 64 bin |
| Doppler FFT 后 | 100 MHz | 10 bit | 含 sub_frame + bin 信息 |
| CFAR 输出 | 事件驱动 | 检测报告 | 仅超过门限的目标 |

#### 4.4.2 各阶段算法详解

| 处理阶段 | 算法/技术 | 核心作用 | 工程实现要点 |
|----------|-----------|----------|-------------|
| **I/Q 下变频** | NCO 400MHz 数字正交混频 | IF → 基带，提取相位/幅度 | 6级流水线 DSP48E1，LUTRAM 正弦表 |
| **CIC 抽取** | 5级级联积分梳状滤波 | 400MHz → 100MHz，降速率 | PCOUT→PCIN 级联布线，零乘法器 |
| **FIR 低通** | 32抽头多相 FIR | 抗混叠 + 通道整形 | 加法树 5级缩减，USE_DSP="no" |
| **脉冲压缩** | 重叠保留法匹配滤波 | 长脉冲→窄脉冲，提高距离分辨率 | 3000样本分4段1024点FFT+IFFT |
| **Doppler FFT** | Staggered PRF 双16点FFT | 提取目标速度，解决速度模糊 | 中国余数定理解算真实速度 |
| **MTI 对消** | 2/3脉冲对消滤波器 | 抑制静止地面杂波 | H(z)=1-z⁻¹ / H(z)=1-2z⁻¹+z⁻² |
| **CFAR 检测** | CA/GO/SO-CFAR | 自适应阈值，自动识别真实目标 | max+min/2 幅值近似（误差~5%） |
| **混合 AGC** | FPGA/STM32/GUI 跨层闭环 | 动态优化接收增益，防饱和防过弱 | 饱和计数 + 峰值幅度联合决策 |

#### 4.4.3 脉冲压缩——距离分辨率的灵魂

**为什么需要脉冲压缩？**

```
问题：长脉冲 = 高能量（远距离），但距离分辨率差
      短脉冲 = 好分辨率，但能量不足（近距离）

解决方案：LFM 啁啾 + 匹配滤波 = 脉冲压缩
         长脉冲发射 → 接收后相关处理 → "压缩"成窄脉冲
         结果：同时拥有长脉冲的能量 + 窄脉冲的分辨率！
```

**AERIS-10 的实现方案**：

```
3000 样本长啁啾（长距离模式）
    │
    ├──→ 分割为 4 个 1024 点 Segment（重叠保留法）
    │       Seg0 | Seg1 | Seg2 | Seg3
    │       (1024)(1024)(1024)(1024)
    │
    ├──→ 每个 Segment:
    │       零填充 → 1024点 FFT → 频域相乘(×参考Chirp) → IFFT
    │
    └──→ 去除前 128 点（重叠区）→ 保留 896 点 → 拼接完整距离像
        SEGMENT_ADVANCE = 896, OVERLAP = 128
```

#### 4.4.4 CFAR——恒虚警率的智能门卫

CFAR 通过在检测单元两侧设置**保护单元**和**参考窗**来估计局部噪声功率，自适应设置检测门限：

```
  参考窗    保护    检测    保护    参考窗
  ████████  ████   [☆]   ████  ███████
  ← 左半窗 → ←保→ ←检→ ←保→ → 右半窗 →

三种模式：
  CA-CFAR (平均):    取前后窗噪声均值 → 通用场景
  GO-CFAR (选大):    取大的一侧均值    → 多目标环境
  SO-CFAR (选小):    取小的一侧均值    → 边带干扰环境
```

### 4.5 系统管理与数据融合

STM32 承担第五步的系统管理任务，将**射频数据**与**地理数据**最终融合：

```
┌─────────────────────────────────────────────────┐
│           STM32 数据融合枢纽                      │
├─────────────────────────────────────────────────┤
│                                                  │
│  雷达探测数据 ──┐                                │
│  (FPGA处理后)   │                                │
│                 ├──→ 坐标校正(IMU Pitch/Roll)      │
│                 │       ↓                        │
│  GPS (UM982) ──┼──→ 位置打标签(Lat/Lon)           │
│                 │       ↓                        │
│  IMU (GY-85) ──┘       ↓                        │
│                                                   │
│              输出: 带地理位置的真实目标坐标          │
│                                                   │
│  AGC 闭环: FPGA(饱和计数)↔STM32(增益决策)↔GUI(显示)│
└─────────────────────────────────────────────────┘
```

### 4.6 Python GUI 可视化

作为流水线的终端环节，GUI 不仅仅是显示器——它是**交互控制中心**：

| 功能 | 说明 |
|------|------|
| **实时 PPI 显示** | 平面位置指示器，极坐标系目标展示 |
| **RDM 可视化** | 距离-多普勒地图（Range-Doppler Map） |
| **AGC 分析仪表盘** | 显示混合 AGC 的工作状态 |
| **地图集成** | GPS 自动居中 + 目标地理位置标注 |
| **雷达控制台** | 参数调整、波束控制、扫描逻辑修改 |
| **跨层 AGC 反馈** | 用户可观察并影响接收链路的增益策略 |

**GUI 版本演进**：

| 版本 | 文件 | UI 框架 | 状态 |
|------|------|---------|------|
| V6 | `GUI_V6.py` | Tkinter | 已弃用 |
| **V65** | `GUI_V65_Tk.py` | **Tkinter** | **当前稳定版（推荐）** |
| V7 | `GUI_V7_PyQt.py` | PyQt6 | 开发中（模块化新架构） |

---

## 五、FPGA 固件架构深度解析

> 📖 更完整的技术细节参见 [`9_Firmware/9_2_FPGA/AERIS-10_FPGA架构说明.md`](9_Firmware/9_2_FPGA/AERIS-10_FPGA架构说明.md)

### 5.1 顶层模块与接口

系统顶层文件 `radar_system_top.v`（1078 行）集成所有子系统：

```
                    radar_system_top
  100MHz ── clk_100m     ┌────────────────────┐
  120MHz ── clk_120m_dac → radar_transmitter → DAC_data
  FT601_CLK ─ ft601_clk   └────────┬───────────┘
                                │ detect_valid
                       ┌────────▼──────────┐
                       │ radar_receiver_final │  (接收机顶层)
                       └────────┬───────────┘
                                │
                       ┌────────▼──────────┐
                       │     cfar_ca         │  (CFAR检测器)
                       └────────┬───────────┘
                                │
                       ┌────────▼──────────┐
                       │ usb_data_interface  │  (USB传输)
                       └─────────────────────┘
```

### 5.2 核心模块清单

| Verilog 模块文件 | 行数 | 功能 |
|-----------------|------|------|
| `radar_system_top.v` | 1078 | 系统顶层（单一事实来源） |
| `radar_receiver_final.v` | 501 | 接收链路顶层集成 |
| `radar_transmitter.v` | 249 | 发射链路（DAC Chirp 生成） |
| `ad9484_interface_400m.v` | 169 | AD9484 LVDS 400MSPS 接口 |
| `ddc_400m.v` | 787 | 数字下变频 400MHz（NCO+CIC+FIR） |
| `nco_400m_enhanced.v` | 368 | 增强型 NCO 数控振荡器 |
| `cic_decimator_4x_enhanced.v` | 903 | 5级 CIC 4×抽取滤波器 |
| `fir_lowpass.v` | 318 | 32抽头 FIR 低通滤波器 |
| `matched_filter_multi_segment.v` | 541 | 多段匹配滤波（长距离脉压） |
| `matched_filter_processing_chain.v` | - | 匹配滤波处理链 |
| `frequency_matched_filter.v` | - | 频域匹配滤波 |
| `range_bin_decimator.v` | - | 距离单元抽取 |
| `doppler_processor.v` | 536 | Staggered-PRF Doppler 处理 |
| `mti_canceller.v` | 216 | MTI 动目标对消（2/3脉冲） |
| `cfar_ca.v` | 561 | CA/GO/SO-CFAR 检测器 |
| `rx_gain_control.v` | 283 | 接收增益 / AGC 控制 |
| `radar_mode_controller.v` | 394 | 雷达工作模式状态机 |
| `usb_data_interface.v` | - | FT601 USB3.0 接口 |
| `usb_data_interface_ft2232h.v` | - | FT2232H USB2.0 接口 |
| `cdc_modules.v` | 271 | CDC 跨时钟域处理库 |
| `plfm_chirp_controller.v` | - | PLFM 波形控制器 |
| `dac_interface_single.v` | - | DAC 接口 |
| `fpga_self_test.v` | - | FPGA 自检逻辑 |
| `edge_detector.v` | - | 边沿检测器 |
| `xfft_16.v` | - | 16点 FFT 引擎封装 |
| `latency_buffer.v` | - | 延迟缓冲 |

### 5.3 关键设计技术

#### DSP48E1 原语级优化

本项目大量使用 **Xilinx DSP48E1** 硬件原语的显式实例化（而非依赖综合器推断）：

| 技术 | 应用 | 收益 |
|------|------|------|
| **PCOUT→PCIN 级联** | CIC 5级积分器 | 避免 fabric 布线延迟，保证 400MHz |
| **AREG/BREG/MREG/PREG** | 混频器、NCO | 4级流水打散关键路径 |
| **CREG=1** | Comb 滤波器 | 消除 0.643ns 组合逻辑延迟 |
| **USE_DSP="no"** | FIR 加法树 | 节省 DSP 给 FFT 使用 |
| **LUTRAM 强制** | NCO 正弦表 | 满足 400MHz 时序（不用 BRAM） |

#### 400MHz 时序收敛策略

- IDDR 原语：`SAME_EDGE_PIPELINED` 模式，单沿稳定输出
- 复位同步器：2级异步断言+同步释放（抑制去断言时序违例）
- CDC：Gray-code + Toggle 同步跨 400MHz→100MHz

### 5.4 时钟域与跨时钟处理

| 时钟域 | 频率 | 来源 | 用途 |
|--------|------|------|------|
| `clk_400m` | 400 MHz | ADC Clock | ADC接口、DDC、CIC |
| `clk_120m_dac` | 120 MHz | DAC Clock | 发射机、NCO |
| `clk_100m` | 100 MHz | 系统时钟 | 处理后级、USB接口 |
| `ft601_clk` | 可变 | USB Clock | USB数据传输 |

### 5.5 主机命令集

主机通过 SPI/USB 发送命令（1字节命令码 + N字节参数）：

| 命令码 | 名称 | 参数 | 描述 |
|--------|------|------|------|
| `0x01` | SET_RADAR_MODE | 1B | 设置工作模式 |
| `0x02` | SET_CHIRP_PARAM | 4B | 啁啾参数（长度/带宽/斜率等） |
| `0x03` | SET_CFR_CONFIG | 2B | CFAR 配置（窗长/保护单元/模式） |
| `0x04` | SET_GAIN | 1B | 接收增益设置 |
| `0x05` | SET_MTI_ENABLE | 1B | MTI 使能与脉冲数选择 |
| `0x06` | SET_AGC_ENABLE | 1B | AGC 使能 |
| `0x07` | SET_SCAN_MODE | 1B | 扫描模式 |
| `0x08` | SET_BEAM_ANGLE | 2B | 波束角度设置 |
| `0x20` | START_SELF_TEST | - | 启动自检 |
| `0x21` | CALIBRATE_DC_OFFSET | - | DC 偏移校准 |
| `0x22` | CALIBRATE_IQ_IMBALANCE | - | IQ 不平衡校准 |

---

## 六、软件系统与代码仓库

### 6.1 固件代码结构

```
9_Firmware/
├── 9_1_Microcontroller/           # STM32 固件 (C/C++)
│   ├── 9_1_1_C_Cpp_Libraries/     # 外设驱动库 (SPI/I²C/UART/GPIO)
│   ├── 9_1_2_C_Cpp_Algorithms/    # 信号处理算法文档
│   ├── 9_1_3_C_Cpp_Code/         # 主程序 main.cpp
│   └── tests/                    # MCU 单元测试 (CppUTest)
│
├── 9_2_FPGA/                     # FPGA 信号处理 (Verilog)
│   ├── *.v                       # RTL 源码 (~30个模块)
│   ├── *.mem                     # 存储器初始化文件 (Chirp LUT 等)
│   ├── constraints/              # XDC 时序约束文件
│   ├── scripts/                  # Vivado 构建脚本
│   │   ├── build_200t.tcl        # XC7A200T 构建
│   │   ├── build_50t.tcl         # XC7A50T 构建
│   │   ├── build_te0712.tcl      # TE0712 FMC 开发板
│   │   └── build_te0713.tcl      # TE0713 FMC 开发板
│   ├── formal/                   # 形式化验证 (SymbiYosys)
│   ├── tb/                       # Testbench (iverilog + xSim)
│   │   └── run_regression.sh     # 5阶段回归测试
│   └── reports/                  # 本地编译报告 (gitignored)
│
├── 9_3_GUI/                      # Python 上位机
│   ├── GUI_V5.py ~ GUI_V7_PyQt.py # 多版本界面实现
│   ├── test_GUI_V65_Tk.py        # V65 测试套件
│   ├── test_v7.py                # V7 测试套件
│   ├── radar_protocol.py         # 雷达通信协议
│   ├── adi_agc_analysis.py       # AGC 分析工具
│   ├── smoke_test.py             # 冒烟测试脚本
│   ├── requirements*.txt         # 依赖声明
│   └── v7/                       # V7 PyQt6 模块化包
│
├── tests/cross_layer/            # 跨层契约测试 (FPGA-MCU-GUI)
│
└── tools/                        # 工具脚本
    └── uart_capture.py           # UART 数据抓包工具
```

**语言分布**：Verilog 18.6% / Python 17.3% / C/C++ 15.2% / Tcl 14.1%

### 6.2 仿真与验证体系

#### 射频/电磁仿真（`5_Simulations/`）

| 类别 | 工具 | 内容 |
|------|------|------|
| **天线仿真** | openEMS / MATLAB | 介质填充开槽波导、贴片天线方向图 |
| **AAV 孔径天线** | openEMS | 孔径天线辐射特性 |
| **DAC 重构滤波器** | QucsStudio | DAC 输出平滑滤波 |
| **IF 带通滤波器** | QucsStudio | 平衡/非平衡 BPF 设计 |
| **枝节 BPF** | QucsStudio | TE 模式枝节滤波器 |
| **波导** | Sonnet EM / openEMS | 氧化铝基板波导传输 |
| **过孔围栏** | openEMS / QucsStudio | 过孔隔离效果 |
| **GaN 功放** | QucsStudio | QPA2962 功放电路仿真 |
| **RF 开关** | QucsStudio + S参数 | 开关阻抗分析 |
| **阵列方向图** | MATLAB | Kaiser窗加权方向图计算 |

#### FPGA 验证（`9_Firmware/9_2_FPGA/tb/`）

- **单元测试**：每个 Verilog 模块都有独立 testbench
- **回归测试**：`run_regression.sh` 自动运行 5 阶段
  ```
  Lint检查 → 模块级TB → 集成TB → 信号处理TB → P0对抗测试
  ```
- **形式验证**：`formal/` 目录包含 SymbiYosys 属性检查
- **跨层测试**：`tests/cross_layer/` 验证 FPGA-MCU-GUI 协议一致性

### 6.3 PCB 制造与装配

| 文件夹 | 内容 | 层数 |
|--------|------|------|
| `Gerber_Main_Board` | 主板 Gerber + BOM/CPL | **10层** |
| `Gerber_freq_synth` | 频率综合器板 | 6层 |
| `Gerber_PA` | 功率放大器板（Extended专用） | 4层 |
| `Gerber_Patch_Antenna` | 贴片天线板（Nexus专用） | 4层 |
| `Gerber_PowerBoard` | 电源管理板 | 2层 |

**板材**：Rogers RO4350B（高频板材），阻抗控制参考已包含在 `4_7_Production Files/PCBWay_Impedance_Note_RO4350B_h0p102mm.pdf`

**EDA 工具**：
- 原理图/PCB：**Eagle**（`.sch` / `.brd` 格式）
- FPGA 综合：**Xilinx Vivado**（支持 XC7A50T / XC7A200T）
- 开发板兼容：TE0712 / TE0713（FMC接口）、UMFT601X（HSDIO）

---

## 七、快速开始

### 7.1 前置条件

- 基础雷达原理知识
- PCB 焊接组装经验（如需自制硬件）
- **Python 3.12+**（运行 GUI）
- **Xilinx Vivado**（修改 FPGA 逻辑时需要）

### 7.2 硬件装配步骤

```bash
# 1. 订购 PCB
# 生产文件位于: 4_Schematics and Boards Layout/4_7_Production Files/

# 2. 采购元器件
# BOM/CPL 文件与 Gerber 同目录

# 3. 参考原理图焊接
# 原理图位于: 4_Schematics and Boards Layout/4_6_Schematics/

# 4. 选择对应版本的天线
# Nexus:  Antennas/Patch/
# Extended: Antennas/Waveguide/

# 5. 机械加工外壳
# 图纸位于: 8_Utils/Mechanical_Drawings/
```

### 7.3 运行软件

```bash
# 安装 GUI 依赖（V65 Tkinter 版本）
cd 9_Firmware/9_3_GUI
pip install -r requirements_dashboard.txt

# 启动雷达 GUI
python GUI_V65_Tk.py

# 或运行 V7 PyQt6 版本
pip install -r requirements_v7.txt
python GUI_V7_PyQt.py
```

### 7.4 运行测试

```bash
# 代码风格检查
uv run ruff check .

# Python 测试
cd 9_Firmware/9_3_GUI && uv run pytest test_GUI_V65_Tk.py test_v7.py -v

# FPGA 回归测试（5阶段）
cd 9_Firmware/9_2_FPGA && bash run_regression.sh

# MCU 单元测试
cd 9_Firmware/9_1_Microcontroller/tests && make clean && make

# 跨层契约测试
uv run pytest 9_Firmware/tests/cross_layer/test_cross_layer_contract.py -v
```

---

## 八、开源许可协议

本项目采用**硬件/软件分离许可**策略，在保持开源精神的同时提供法律保障：

### 硬件部分 — CERN-OHL-P v2

适用于：原理图、PCB布局、BOM表、Gerber文件、机械图纸

> **为什么选择 CERN-OHL-P？**
>
> 项目最初全部采用 MIT 许可证，但社区指出 MIT 缺乏对物理硬件的法律保护。
> CERN-OHL-P 提供：
> - ✅ 明确定义 "Hardware"、"Documentation"、"Product"
> - ✅ 显式的专利保护（贡献者和用户双方）
> - ✅ 更强的责任限制（对高功率射频设备至关重要）
> - ✅ 符合 CERN / OSHWA 专业标准

### 软件/固件部分 — MIT License

适用于：FPGA 代码（Verilog）、STM32 固件（C/C++）、Python GUI 及所有工具脚本

> 选择 MIT 是为了给开发者提供**最大的灵活性**——方便算法修改、SDK 集成和二次开发。

### 许可演变历程

```
最初：全部 MIT → 社区反馈(gmaynez) → 硬件缺法律保护
                                          ↓
现在：硬件 CERN-OHL-P + 软件 MIT → 专业级开源工程
```

---

## 九、目录结构总览

```
PLFM_RADAR/
├── 1_Project_Description/                    # 项目描述文档
├── 2_Functional Diagram & Interconnection Matrices/  # 功能框图 & 连接矩阵
├── 3_Power Management/                      # 电源管理方案 (Excel)
├── 4_Schematics and Boards Layout/          # ★ 原理图 + PCB + Gerber + BOM
│   ├── 4_4_Board Stack-up/                  # 板材堆叠 (RO4350B)
│   ├── 4_6_Schematics/                      # Eagle 原理图 (5类板卡)
│   └── 4_7_Production Files/                # 生产文件 (Gerber/BOM/CPL)
├── 5_Simulations/                           # ★ 射频/电磁/算法仿真
├── 6_Application Notes/                     # 应用笔记 (PDF, 如 UG-290)
├── 7_Components Datasheets and Application notes/  # 元器件数据手册
│   ├── AD9484/                               # ADC 详细资料 + 链路预算
│   ├── ADF4382A/                             # 频率综合器评估板资料
│   └── QPA2962/                              # GaN 功放资料
├── 8_Utils/
│   ├── Antenna_Array.jpg                     # 天线实物照片
│   ├── RADAR_V6_V2.png                      # 系统架构图
│   ├── GUI_V6.gif                            # GUI 演示动图
│   ├── Eagle_CAD_Libs/                       # Eagle 元器件库
│   ├── Mechanical_Drawings/                 # 3D 机械图纸
│   └── Python/                               # 雷达方程/波形生成工具
├── 9_Firmware/                              # ★★ 核心固件代码
│   ├── 9_1_Microcontroller/                  # STM32 固件
│   ├── 9_2_FPGA/                             # FPGA 信号处理 (Verilog)
│   │   ├── AERIS-10_FPGA架构说明.md          # ★ 中文架构深度解析
│   │   ├── OPTIMIZATION_CN.md                # 优化笔记 (中文)
│   │   └── adc_clk_mmcm_integration.md       # MMCM 时钟集成文档
│   ├── 9_3_GUI/                              # Python 上位机
│   ├── tests/cross_layer/                    # 跨层契约测试
│   └── tools/                                # 辅助工具
├── docs/                                    # GitHub Pages 文档站
│   ├── artifacts/                            # 已发布 Bitstream
│   └── assets/img/                           # 文档图片资源
├── .github/workflows/ci-tests.yml            # CI/CD
├── README.md                                 # 英文原始 README
├── README_CN.md                              # ★ 中文说明文档（本文档）
├── CONTRIBUTING.md                           # 贡献指南
├── Licence                                   # CERN-OHL-P 许可证全文
└── pyproject.toml                            # Python 项目配置 (Ruff linting)
```

---

## 致谢

> *"这个项目始于摩洛哥的一个小工作室。今天，19,000 名工程师在 GitHub 上为它点亮了星星。"*
> —— Nawfal Motii, ABAC INDUSTRY

| 角色 | 信息 |
|------|------|
| **原作者** | Nawfal Motii ([GitHub](https://github.com/NawfalMotii79)) |
| **机构** | ABAC INDUSTRY ([www.abacindustry.com](http://www.abacindustry.com)) |
| **赞助商** | PCBWay（PCB 打样制造） |
| **中文整理** | 基于原始开源项目 + 学习笔记整理 |

---

⭐ 如果你对开源雷达技术感兴趣，欢迎 Star 这个项目！

*注：本项目处于活跃开发中，部分功能仍在完善中。查看 [Issues](https://github.com/NawfalMotii79/PLFM_RADAR/issues) 了解已知限制和即将发布的特性。*

*文档版本：v2.0 | 整合笔记：6篇学习文档 + FPGA架构深度解析 | 更新日期：2026-06-18*
