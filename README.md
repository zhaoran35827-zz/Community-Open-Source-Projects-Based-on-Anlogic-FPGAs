# 基于安路 PH1P35 的 720P 智能视觉一体系统（红目标 + 运动检测 + 交通灯与双舵机云台） 

> 单芯片 SoC 风格的视觉参考设计：MIPI 摄像头采集 → ISP 图像处理 → 双算法并行检测 → DDR2 帧缓存 → HDMI 显示，
> 并集成红绿灯状态机、四键双舵机云台、板载数码管倒计时与通道占用报警、实时延迟（LAT）显示。

![Device](https://img.shields.io/badge/Device-PH1P35MDG324--3-blue)
![Toolchain](https://img.shields.io/badge/Toolchain-Anlogic%20TD%206.2.1-lightgrey)
![Video](https://img.shields.io/badge/Video-1280%C3%97720%4060-informational)
![License](https://img.shields.io/badge/License-See%20repo%20LICENSE-yellow)

---

## 目录

- [1. 项目简介](#1-项目简介)
- [2. 功能特性](#2-功能特性)
- [3. 系统架构](#3-系统架构)
- [4. 仓库结构](#4-仓库结构)
- [5. 硬件接口与引脚](#5-硬件接口与引脚)
- [6. 使用说明](#6-使用说明)
- [7. 算法说明](#7-算法说明)
- [8. 资源占用与时序（参考实现）](#8-资源占用与时序参考实现)
- [9. 构建与下载](#9-构建与下载)
- [10. 验证范围与已知限制](#10-验证范围与已知限制)
- [11. 版权与许可](#11-版权与许可)
- [English Summary](#english-summary)

---

## 1. 项目简介

本工程面向**安路（Anlogic）PH1P35 系列 FPGA**，在一颗器件内完成从图像采集到显示的完整链路，并叠加多种视觉算法与外设控制：

| 环节 | 实现 |
|---|---|
| 图像采集 | SC500 系列图像传感器，MIPI D-PHY **2 Lane**，RAW10，**1280×720@60** |
| 图像处理 | 黑电平校正、去马赛克（Demosaic）、自动白平衡（AWB）、空间降噪、卡通、边缘检测 |
| 智能分析 | **红色目标检测**（最多 6 个目标）与**运动目标检测**（最多 3 个框）双算法分支 |
| 帧缓存与传输 | PH1P35 DDR2 控制器 + 自定义 96/128 bit 打包解包、4 Bank 读写的帧缓冲 |
| 显示输出 | HDMI 1.4b（TMDS），1280×720，像素时钟 75 MHz（1650×750，≈60.6 Hz） |
| 外设控制 | 两组交通灯状态机、四键双舵机云台（PWM）、板载 8 位数码管、蜂鸣器报警 |
| 交互 | 4 个拨码开关选择算法模式，OSD 实时显示 LAT 延迟、CNT、框与标签 |

设计目标是**可复用、可裁剪的教学/竞赛级参考工程**：所有图像处理与检测逻辑均以纯 Verilog 实现，不使用软核 CPU，也不含神经网络加速器。

---

## 2. 功能特性

### 2.1 四种图像算法模式（拨码开关）

| 开关 | 引脚 | 功能 | 说明 |
|---|---|---|---|
| SW1 | A4 | 卡通画效果 | 可与其他算法叠加 |
| SW2 | B4 | 红色目标检测 / 联动 | 红色目标最多 6 个，显示 ID、坐标、`TRK` 标记 |
| SW3 | E11 | 运动目标检测 | 运动框最多 3 个，不做静止目标的持续追踪 |
| SW4 | D11 | 中心 1:1 分屏 | 左右同中心裁剪、无缩放；其他算法只作用于彩色侧 |

**开关先后顺序决定模式主导权**（同一帧内生效，不从中间状态拼接半帧）：

- 只开 SW3：纯运动检测，显示当前运动框。
- 运动先开 → 再开 SW2：**运动主导、目标辅助**；物体停止后启用外观保留与 `TRK`/XY 显示。
- SW2 先开 → 再开 SW3：**红目标主导**，运动检测仅用于生成红色 ROI。
- 两路同时从关到开：默认红色主导。
- 模式切换时清空上一模式残留结果，避免异帧/异角色结果混用。

### 2.2 交通灯与四键双舵机云台

- **交通灯**：外接 A/B 两组红黄绿（3.3 V 高有效）。
  - 绿灯时长按该方向车辆数：1 辆 6 s、2 辆 10 s、3 辆 14 s、4 辆及以上 18 s；
  - 黄灯 2 s；全红至少 2 s，且需共用通道确认清空才允许下一方向进入；
  - 等待超过 25 s 的方向拥有优先权（安全清空条件仍然优先）。
- **通道占用报警**：交通开启且感知健康时，中间区域**连续占用 30 秒**即锁存故障——双红、数码管 `AErr/bErr`、蜂鸣器 256 ms 鸣 / 256 ms 停循环；移开物体不会自动解除，需交通开关 OFF→ON 复位。
- **双舵机云台**：四键手动控制（左/右/上/下，按下接地、松开保持）。
  - 中心 1500 μs，周期 20 ms，步进 10 μs / 40 ms；
  - 水平 1100～1900 μs，俯仰 800～2400 μs；
  - 交通 ON 时锁定角度但持续输出 PWM 保持位置，OFF 且释放四键后可继续调节。
- **板载显示**：SW6 拨上选择 8 位数码管（左四位 A 端、右四位 b 端；绿/黄显示剩余秒数，红灯显示 `--`，故障显示 `AErr/bErr`）；SW5 拨下接通蜂鸣器。
  - 注：SW5/SW6 是开发板上的**物理选择开关**，不占用 FPGA 引脚。

### 2.3 实时延迟（LAT）显示

- OSD 顶部显示 `LAT:xxxMS`，按采集帧龄换算：`(帧差 × 1000 + 30) / 60`，固定 60 fps 下为精确整数毫秒。
- 该数值的分辨率约为 16.67 ms（一个采集帧间隔），属于**帧龄统计**，不是逐帧硬件时间戳。
- 超量程显示橙色 `999MS`，元数据无效/陈旧显示橙色 `XXXMS`。

---

## 3. 系统架构

### 3.1 数据流

```
MIPI D-PHY（2 lane，RAW10）  ←  SC500 传感器 @1280×720p60
        │
        ▼
raw10_unpacket → BLC → Demosaic → AWB
        │
        ├─► 主视频支路
        │     空间降噪 / 卡通 / 边缘检测 → 框与标签叠加 → 中心分屏缓存 → 算法 OSD
        │     → 96/128 bit 打包 → DDR2（4 bank 帧缓存）→ 128/24 bit 解包
        │     → Logo / CNT / LAT OSD → HDMI 1.4b TMDS @75 MHz
        │
        └─► 分析支路（2×2 降采样 → 640×360）
              ├─ 红色分割 / 去噪 / 连通域(CCL) → 最多 6 个红目标（数量 / ID / 坐标）
              └─ 运动网格 → 160×90 RGB555 双网格 → 32 个区域提案 → 串行模板匹配
                    → 相机运动补偿 → 时间确认 → 相干碎片合并 → 最多 3 个运动框

分析结果（坐标 / 角色 / 模式 / 文字就绪）在 SOF 时刻整体提交给显示，不驱动主视频 SOF。
```

- **主视频支路**与**分析支路**解耦：分析结果不驱动主视频 SOF，主流不等待分析完成。
- 全分辨率 RGB888 流水线与分析支路共享同一份像素流；分析结果（坐标、角色、模式、文字就绪）在 SOF 时刻作为一个整体快照提交给显示，保证一拍对齐。
- 显示侧：box 后一拍同步延迟原图 RGB、处理 RGB、边缘、valid、SOF、EOL 与标签坐标，因此 OSD 与画面不错位。

### 3.2 关键模块

| 模块 | 位置 | 职责 |
|---|---|---|
| `design_top_wrapper.v` | `hdl_source/` | 顶层：时钟/PLL、MIPI RX、DDR2、HDMI TX、外设与算法参数配置 |
| `isp_top.v` | `hdl_source/isp/` | ISP 与视觉算法总装、显示/分析双支路调度 |
| `fusion_hd_96.v` / `fusion_vision_96.v` | `hdl_source/vision/` | 高清混合处理与视觉融合（降噪/增强/主控） |
| `red_components_96.v` / `red_bbox_96.v` / `red_ids_eram.v` | `hdl_source/vision/` | 红色分割、连通域、目标框与 ID 存储 |
| `motion_*.v`（15 个模块） | `hdl_source/vision/` | 运动检测后端：网格采集、区域 worker、相机补偿、模板匹配、轨迹与合框 |
| `edge_split_96.v` / `multi_target_overlay_96.v` / `algorithm_osd_96.v` | `hdl_source/vision/` | 边缘分屏、多目标叠加、算法 OSD |
| `traffic_controller.v` / `traffic_board_io.v` / `manual_gimbal.v` | `hdl_source/traffic/` | 交通状态机、数码管/蜂鸣器、四键舵机 |
| `hdmi_mixer.v` / `hdmi_tx.v` / `osd_char_lib.v` | `hdl_source/` | 像素混合、TMDS 发送、字库与 OSD 合成 |
| `uicfgcs500_720p.v` / `uii2c.v` | `hdl_source/uics500_cfg/` | 传感器 I2C 配置（720p60 模式与 AE 寄存器表） |
| `ph1p35_ddr/`、`PLL/`、`MCU/`、`*_fifo/` | `ip_source/`、`hdl_source/` | 厂商 IP：DDR2 控制器、PLL、FIFO 等 |

---

## 4. 仓库结构

```
.
├── user_source/
│   ├── hdl_source/                # 187 个文件（144 个 .v 源码 + 厂商 IP 描述文件）
│   │   ├── design_top_wrapper.v   # 顶层模块（TOP = design_top_wrapper）
│   │   ├── hdmi_*.v / video_*.v   # 采集、混合、TMDS 发送
│   │   ├── osd_char_lib.v         # 字库与 OSD 合成
│   │   ├── isp/                   # ISP：BLC、Demosaic、AWB、空间/边缘处理
│   │   ├── vision/                # 视觉算法：红目标、运动检测、叠加与 OSD
│   │   ├── traffic/               # 交通灯、数码管/蜂鸣器、手动云台
│   │   ├── dsi_tx_packet/         # DSI 打包（可选输出路径）
│   │   ├── mipi_dphy_rx/          # MIPI D-PHY 接收
│   │   ├── ph1p35_ddr/            # DDR2 PHY / 控制器
│   │   ├── uics500_cfg/           # 传感器配置与 I2C
│   │   └── uiisp_beta/ uial2axis/ vtc/ …   # 厂商参考模块
│   ├── ip_source/                 # 53 个 IP 文件：PLL、FIFO、MCU、BRAM 等
│   └── constraints_source/
│       ├── pin.adc                # 引脚约束（FPGA 球号 / 电平 / 上下拉）
│       └── timing.sdc             # 时钟与 CDC 约束
└── td_project/
    ├── camera_to_dsi_display.al   # TD（Anlogic）工程文件
    ├── camera_to_dsi_display.sdc  # 工程级时序约束
    └── verify_gui_strategy.tcl    # TD 物理实现参数回读校验脚本
```

**顶层模块**：`design_top_wrapper`（器件 `PH1P35MDG324`，速度等级 `-3`）。

---

## 5. 硬件接口与引脚

### 5.1 板级开关与调试

| 信号 | 引脚 | 说明 |
|---|---|---|
| 系统时钟 `I_sys_clk` | C4 | 50 MHz |
| 复位 `I_rst_n` | R10 | 低有效 |
| SW1 / SW2 / SW3 / SW4 | A4 / B4 / E11 / D11 | 算法模式选择 |
| 板载 LED[3:0] | D10 / E10 / C6 / D6 | 状态指示 |
| 数码管 SEG/DIG[15:0] | C9,C11,B11,E7,E6,B6,B7,E5,G15,G14,A17,A18,B16,A15,B15,A14 | 低有效 |
| 蜂鸣器 | C12 | **低有效** |

### 5.2 摄像头（MIPI）

| 信号 | 引脚 | 说明 |
|---|---|---|
| `O_cam_scl` / `IO_cam_sda` | K3 / J3 | SCCB/I2C 配置 |
| `O_cam_24m` | L3 | 24 MHz 主时钟 |
| `O_cam_rst` | L2 | 传感器复位 |
| MIPI 差分对 | `IO_rx_clk_pad_p/n`、`IO_rx_data_pad_p/n[3:0]` | 本设计使用 **2 Lane** |

### 5.3 HDMI 输出

| 信号 | 引脚 | 说明 |
|---|---|---|
| TMDS CH0 / CH1 / CH2 | G16 / F17 / H14 | TMDS33 |
| TMDS CLK | C17 | TMDS33 |

### 5.4 外接交通灯与云台（J2 排针）

| 功能 | 排针 | FPGA 球位 | 说明 |
|---|---|---|---|
| A 组 红 / 黄 / 绿 | J2-1 / 2 / 3 | C18 / D15 / E12 | 3.3 V 高有效 |
| B 组 红 / 黄 / 绿 | J2-4 / 5 / 6 | E16 / E17 / F13 | 3.3 V 高有效 |
| 交通使能开关 | J2-7（接 GND 开启） | F14 | 低有效、内部上拉 |
| 水平舵机 PWM | J2-8 | F18 | 1100～1900 μs |
| 俯仰舵机 PWM | J2-9 | G17 | 800～2400 μs |
| 舵机电源 | J2-11 | — | 板卡 5 V 直接分配 |
| 公共地 | J2-12 / 30 | — | — |
| 云台按键 左/右/上/下 | J2-16 / 17 / 18 / 19 | J15 / J16 / J18 / D9 | 常开按键，另一端接 GND |

> 接线前务必断电核对公共地与插头针序；灯组禁止接 5 V；板卡 5 V 带双舵机的电源余量需实测确认。

---

## 6. 使用说明

### 6.1 上电与基本画面

1. 断电接线，确认摄像头、HDMI、交通灯、舵机与按键接线无误，交通开关保持 OFF。
2. 上电后 HDMI 应输出 720P 原始画面，OSD 显示 `CNT` 计数递增、`LAT:xxxMS` 与底部状态栏。
3. 通过 SW1～SW4 切换算法；建议先在全关状态确认画面稳定，再逐一开启。

### 6.2 运动检测模式（单开 SW3）

1. 固定相机，用手、鼠标、灯管等物体做慢移、快移、转动、靠近/远离。
2. 观察是否出现**完整覆盖物体**的框；扩到背景的大框、重复框、离场残框都属异常，不应视为成功。
3. 加入不发光的灰白/肤色物体再测，避免只验证高亮点。
4. 物体停止后应退出持续静止输出；背景静止时不应出现长期追踪框。

### 6.3 目标检测与联动

- SW2 单开：红卡、红衣物、邻接皮肤的弱肤色物体，检查红框不扩到非红区域，红目标数量上限 6。
- 运动先开后 SW2：物体停止后检查外观保留（`TRK`/XY），短时遮挡允许有限次预测，确认离场后清除。
- SW2 先开后 SW3：红色主导，运动 ROI 不得把非红物体当作红色目标。

### 6.4 交通灯与云台

1. 交通开关闭合前，先松开所有云台按键，避免上电后持续移动。
2. 云台按住移动、松开保持；交通 ON 时锁定角度但 PWM 持续输出。
3. 交通 OFF 后需释放一次全部按键才能恢复云台调节。
4. 观察数码管倒计时与 HDMI 显示的绿/黄阶段秒数是否一致；切换灯态时左右两组的归零关系应正确。
5. 在中间区域放置可识别目标并保持连续占用 30 s，检查双红 + `AErr/bErr` + 间歇蜂鸣是否锁存，以及 OFF→ON 能否复位。

---

## 7. 算法说明

### 7.1 红色目标检测

- 输入 640×360 分析图（原图 2×2 降采样），进行颜色分割 + 严格种子筛选 + 弱连接收窄，抑制黄灯与弱肤色误检。
- 掩码去噪 → 连通域标记（CCL）→ 最多 6 个目标 → ID 与描述符存入 ERAM。
- 目标的“数量 / ID / 坐标”就绪信号与“形状分类就绪”解耦，避免分类未完成时坐标面板被挡住。
- 局限性：依赖 RGB 颜色分离，极端光照、强反光或深红色场景可能漏检/误检；本设计**不含**语义分类或学习模型。

### 7.2 运动目标检测（本轮重点）

```
640×360 输入 → 4×4 块平均 → 160×90 RGB555 双网格（两份 ERAM 交替采集/处理）
            → 区域颜色一致性生长 → 32 个完整候选提案
            → 相机四区域一致性 + 插值估计全局平移/小旋转 → 背景补偿
            → 同一个串行 patch matcher：24 个背景模板 + 12 个临时物体模板
            → 独立运动证据 + 时间确认 → 相干碎片合并 → 最多 3 个运动框
            → 坐标 ×2 映射回 720P，按 SOF 原子提交显示
```

设计要点：

- **单一匹配器分时复用**：背景搜索与物体搜索共用一个 RGB555 模板匹配核（复用 3 个 DSP），不额外增加匹配引擎。
- **背景质量缓存**：无纹理区域跳过无效搜索，但仍逐帧学习。
- **相对对比门限**：模板轮廓准入使用相对阈值（所选对比通道峰值 ≤16 用 2、≤24 用 3，其余用 4），使弱对比物体也能学成模板，同时拒绝单级噪声与平坦背景。
- **缓慢运动确认**：相机补偿后的累计残差门为 32（两次）/ 48（三次），允许平均 1 个分析像素/更新的慢速运动通过，同时保留时间投票机制。
- **独立运动判据**：最佳匹配需比相机运动假设的评分改善 ≥8 才建立独立运动证据，防止背景被误判为运动。
- **几何防背景规则**：新出现且近乎填满画面贴边的候选在学习前被过滤；该规则是几何性的，**不是语义分类**，也会连带抑制“几乎充满镜头的新近物体”。
- **短时遮挡**：前两次失配允许预测外推，此后停止外推；连续约 8 次失配清除目标。
- **输出上限**：运动框 ≤3、红目标 ≤6；运动与红色两条支线的结果不互相注入。

### 7.3 图像增强

- 空间降噪与边缘检测共享 Sobel/Gaussian 列和算术，逐拍等价；
- 卡通、边缘检测、中心分屏按开关独立运行，仅彩色侧叠加效果；
- 所有增强都在原视频流水线内完成，不增加帧缓存或额外延迟级。

---

## 8. 资源占用与时序（参考实现）

参考实现（器件 `PH1P35MDG324-3`，TD 6.2.1，seed 0）的实际物理结果：

| 资源 | 用量 / 容量 | 占用率 |
|---|---:|---:|
| Slice | 20901 / 21216 | 98.52% |
| LUT6 | 20521 / 21216 | 96.72% |
| Register | 17037 / 42432 | 40.15% |
| ERAM | 83 / 108 | 76.85% |
| DSP | 23 / 40 | 57.50% |
| PLL | 3 / 6 | 50.00% |

- 最终 STA：Setup WNS **+0.112 ns**、Hold WNS **+0.020 ns**，Setup/Hold TNS 均为 0。
- 参考 bit 生成于布局布线后，未包含在本仓库中（仓库仅含源码与约束）。
- 参考 bit 校验值（SHA256）：`FB55DBEA7911188E979247A2BA515E49157D3DFAB7D7B1279AA3DEC8C194C114`

> ⚠️ Slice 占用已接近上限（98.5%），重新实现时请使用下方给出的物理策略，并留出余量，否则可能出现布线拥塞或时序不收敛。

---

## 9. 构建与下载

### 9.1 环境

- **Anlogic TD 6.2.1**（工程版本 `6.2.168116`）
- 目标器件：`PH1P35MDG324`，速度等级 `-3`
- 建议充足内存（资源占用接近满载，布局布线较慢）；布局布线耗时与机器性能相关。

### 9.2 打开工程

1. 用 TD 打开 `td_project/camera_to_dsi_display.al`。
2. 源文件以**相对路径**引用（`../user_source/...`），因此仓库保持 `user_source/` 与 `td_project/` 同级即可直接编译。
3. 若因仓库克隆到不同盘符导致工程提示原始路径不存在（`.al` 中的 `<Project Path>` 记录的是作者机器上的绝对路径），在 TD 中重新指定工程位置/另存即可，**不要修改源文件相对引用**。
4. 引脚约束 `user_source/constraints_source/pin.adc`、时钟约束 `timing.sdc` 与 `td_project/camera_to_dsi_display.sdc` 已在工程中登记。

### 9.3 综合与实现

```
TD: Run Synthesis → Run Place & Route → Generate Bitstream
```

推荐的布局布线参数（可用 `td_project/verify_gui_strategy.tcl` 在实现后回读校验）：

| 阶段 | 参数 | 建议值 |
|---|---|---|
| place | `opt_timing` | high |
| place | `ofv` / `opt_max_density` | 70 / 50 |
| place | `target_density` / `target_density_pp` | 0.6 / 0.7 |
| place | `aggressive_opt` / `enhanced_beeline_opt` | 20 / 2 |
| place | `enhanced_gp` / `non_crit_opt` / `post_place_opt` | on / on / high |
| place | `enable_congestion_recovery` | on |
| place | `seed` | 0 |
| route | `post_route_opt` | high |
| bitgen | `compress` | on |

回读校验示例（TD 命令行）：

```tcl
# 在 td_commands_prompt.exe 中执行
source td_project/verify_gui_strategy.tcl
```

### 9.4 下载

- 生成 bit 后通过 TD Programmer 下载到开发板；**注意核对开发板器件型号与速度等级**。
- 本工程不包含任何上电自动加载（Flash 固化）流程。

---

## 10. 验证范围与已知限制

### 10.1 已完成的验证（仿真/实现层面）

- **图像链路**：720P 完整帧（2 764 800 像素）的 Bayer→Demosaic→AWB→空间处理→分屏→OSD→打包一拍对齐；实际 VTC/mixer 921 600 像素精确 RGB 比对。
- **红目标**：102 019 个色值的亮度斜坡严格种子不变性，收窄黄/弱肤色连接；完整 720P 红卡邻接弱肤色场景（红框不扩到手部）。
- **运动检测**：真实 640×360 连续 RGB、三物体完整框（IoU ≥0.8）；68 帧复杂场景（停止/遮挡/离场，背景随相机平移、小旋转、光强变化）；160 帧新场景 50 项整框真值 + 45 个负例帧；独立匹配器 9 组实际 RGB555 与 10 次精确位移对照。
- **传输与缓存**：4 显示帧 3 686 400 像素精确 RGB、真实 MC 适配器与写通道停顿、FIFO 信用边界。
- **外设**：毫秒换算全 0～63 帧、999/XXX 字模 6912 像素；舵机 PWM 中心/行程/步进/锁角；交通启动清空、6→5 倒计时与超时双红/蜂鸣联动。

### 10.2 已知限制（请勿把“仿真通过”当作“实拍准确”）

- 主视频为固定 720P 采集（分析支路取其 2×2 降采样的 640×360）；改变分辨率或帧率需同步修改参数、LAT 换算与约束。
- 运动检测为 160×90 稀疏模板后端，**不擅长**任意快速旋转、强视差、手部剧烈形变、大面积遮挡与强反光场景；框可能不完整。
- 不含语义分类，因此“新生近全幅贴边”几何规则会压制几乎占满画面的新物体。
- 红色检测基于 RGB 颜色分离，深红/弱肤色/极端白平衡下存在漏检风险。
- 板上曾观察到偶发紫/黄像素相位错位现象，该问题未在仿真中复现；物理 DDR PHY、PCB 信号完整性与电源余量均需实板确认。
- 5 V 双舵机带载、灯组与舵机插头极性、数码管/蜂鸣器实际极性均需按实物核对。

---

## 11. 版权与许可

- 本工程面向**安路（Anlogic）FPGA 社区开源项目**提交，设计代码可按仓库统一的许可证（见仓库根目录 `LICENSE`）使用与二次分发。
- 工程中包含**安路官方 IP 与厂商参考代码**（例如 `ph1p35_ddr/`、`PLL/`、`hdmi_1_4b_transmitter_core_wrapper.enc.v` 等加密或参考实现），其版权归安路科技及原始作者所有，使用须遵守安路授权条款与 EDA 许可，不得单独再分发。
- 摄像头配置与部分 ISP 参考模块来自 PH1P35 开发板厂商示例工程（相关文件头注释已注明来源），二次分发请保留原始版权声明。
- 若本仓库需要明确的开源许可证，建议在仓库根目录补充 `LICENSE`（如 Apache-2.0 / MIT）并在提交说明中注明适用文件范围。

---

## English Summary

A self-contained 720P smart-vision reference design on **Anlogic PH1P35MDG324-3** (TD 6.2.1), written in pure Verilog — no soft CPU, no neural-network accelerator.

**Pipeline:** MIPI 2-lane SC500 sensor @1280×720p60 → BLC/Demosaic/AWB → spatial denoise / cartoon / edge → box & label overlay → split-screen cache → algorithm OSD → 96/128-bit packing → DDR2 (4 banks) → unpack → Logo/CNT/LAT OSD → HDMI 1.4b TMDS @75 MHz (1650×750, ≈60.6 Hz).

**Algorithms:** red-target detection (≤6 objects, ID / XY / `TRK`) and a motion-detection backend (640×360 analysis → 160×90 dual RGB555 grids → 32 region proposals → shared patch matcher with 24 background + 12 object templates → camera-motion compensation → temporal confirmation → ≤3 motion boxes). Four DIP switches select cartoon / red-target / motion / center split-screen; switch order decides which algorithm dominates, and results are committed atomically per frame onset.

**Peripherals:** two-way traffic-light state machine (vehicle-count-based green times, 2 s yellow, guarded all-red), 4-key dual-servo manual gimbal (1500 μs center, 20 ms period, pan 1100–1900 μs, tilt 800–2400 μs), onboard 8-digit countdown display, buzzer alarm latched after 30 s of continuous region occupancy, and an on-screen `LAT:xxxMS` frame-age readout.

**Reference implementation:** 20901/21216 slices (98.52%), 20521 LUT6, 17037 registers, 83/108 ERAM, 23/40 DSP, 3/6 PLL; setup WNS +0.112 ns, hold WNS +0.020 ns, both TNS zero.

**Build:** open `td_project/camera_to_dsi_display.al` in Anlogic TD 6.2.1, synthesize, place & route with the documented high-effort density strategy (seed 0), then generate the bitstream. Because slice utilization is near 99 %, keep the recommended physical-implementation parameters or expect congestion/timing failures.

**Limitations:** fixed 720P input, no semantic classification, motion detection degrades under fast rotation, strong parallax, heavy deformation or large occlusion, and red detection depends on RGB separation. Board-level behaviour (servo power, display polarity, occasional pixel-phase artefacts) must be verified on hardware.

---

*本 README 描述的仓库内容为硬件源码与约束（`user_source/` + `td_project/`）；bit 文件、仿真日志与内部验收脚本不在本仓库中。*
