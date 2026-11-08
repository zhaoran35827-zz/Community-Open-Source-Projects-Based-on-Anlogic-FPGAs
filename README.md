# Vision_MotionRefine_20261103

安路（Anlogic）PH1P35 / TD 6.2.1 平台的 720P 相机 → DSI/HDMI 视觉工程，在用户输入版
`Vision_BoardFix_20261102`（已含四键双舵机、交通灯、板载数码管/蜂鸣器报警、LAT 毫秒源码）之上，
**只修订运动检测**：弱对比模板准入、缓慢运动确认、弱慢组合的独立匹配分数门。

本轮 23 项有范围功能证据、原生布局布线、最终 STA 与器件容量均已核验通过；**实板效果仍待上板检查**。
本轮没有下载 FPGA、没有写 Flash。

> 旧 `VERIFY_BOARDFIX*.py` / `VERIFY_DISPLAY_ALARM.py` / `VERIFY_TRAFFIC_GIMBAL.py`、旧
> `stable_source_reference`、`reference/refine_attempt_de4` 及旧 bit 都属于历史，**不能用于验收本轮**。

---

## 1. 当前交付身份

| 项 | 值 |
|---|---|
| 器件 | `PH1P35MDG324`（TD `6.2.168116`） |
| 工程入口 | `camera_to_dsi_display.al`（GUI，仅重定向工程路径） |
| 交付 bit | `output/vision_motion_refine_20261103.bit`（1,789,539 字节） |
| bit SHA256 | `FB55DBEA7911188E979247A2BA515E49157D3DFAB7D7B1279AA3DEC8C194C114` |
| 源码冻结 | `motion_refine_source_reference.json` |
| 冻结 SHA256 | `CD620165D599997235EDF8FC118DEC1099ECBE346DE58CC693FCBF85D6A46C3E`（245 个非日志源） |
| 功能证据 | `output/motion_refine_test_index.json`（23 项） |
| 物理种子 | seed 0（脚本与原生日志显式记录；clean 包中 GUI 不覆写该字段） |

### 资源与时序（实际物理结果，非综合估算）

| 资源 | 用量 / 容量 | 比例 |
|---|---:|---:|
| slice | 20901 / 21216 | 98.52% |
| lut6 | 20521 / 21216 | 96.72% |
| reg | 17037 / 42432 | 40.15% |
| eram | 83 / 108 | 76.85% |
| dsp | 23 / 40 | 57.50% |
| pll | 3 / 6 | 50.00% |

最终 STA：`SWNS +0.112 ns`、`STNS 0`、`HWNS +0.020 ns`、`HTNS 0`（详见 `RESOURCE_REPORT.md`、
`output/motion_refine.routed_timing.txt`）。

---

## 2. 目录结构

```
Vision_MotionRefine_20261103/
├─ camera_to_dsi_display.al          # 当前 GUI 工程入口（canonical，源清单权威）
├─ motion_refine_gui_origin.al       # 冻结时的原始 GUI 入口（对照用）
├─ project/
│  ├─ user_source/                   # 242 个冻结硬件源：hdl_source / ip_source / constraints_source
│  │  └─ hdl_source/vision/          # 运动与视觉算法 RTL（motion_*.v、red_*.v、fusion_*.v …）
│  └─ td_project/                    # TD 工程与运行目录（由 prepare_build.py 重建运行输入）
├─ output/                           # 当前 bit、原生报告、功能证据索引、GUI 绑定
├─ tests/                            # 测试平台 tb_*.v、向量生成/检查脚本、RUN_*.ps1
│  ├─ motion_refine_vectors/         # 本轮 160 帧/50 整框检查的输入与真值
│  ├─ motion_complex_vectors/        # 旧 68 帧复杂场景真值
│  └─ vendor/                        # 厂商仿真模型（逐字节复制，可移植）
├─ experiments/                      # 诊断与候选用：拍屏采样、LAT 查表候选及其验证
├─ reference/                        # 输入清单（incoming_20261103）与被拒尝试归档
├─ build/                            # 构建期安全/监控脚本与 TD 批处理模板
├─ motion_refine_common.py           # 源/证据/原生身份检查公共库
├─ FREEZE_MOTION_REFINE.py           # 显式冻结当前源
├─ VERIFY_MOTION_REFINE.py           # 当前验收入口
├─ RECORD_MOTION_REFINE_TESTS.py     # 汇总并写入 23 项功能证据
├─ BUILD_MOTION_REFINE.ps1           # 独立综合 + seed0 物理实现
├─ PACK_MOTION_REFINE.py             # 严格验收后生成干净交付包
├─ VERIFY_MOTION_REFINE_PACKAGE.py   # 可移植包的只读独立验收
├─ motion_refine_selfcheck.py        # 验收器的正/负向自检（隔离合成树）
└─ prepare_build.py                  # 重建 td_project 运行输入
```

---

## 3. 快速开始

### 3.1 环境

- Windows + PowerShell（构建脚本为 `.ps1`）
- Python 3（含标准库即可；建议 `-X utf8`）
- 安路 TD `6.2.1`，默认路径 `F:/Anlogic/TD_6.2.1_Engineer_6.2.168.116`（可用 `-TdInstall` 覆盖）
- 仿真回归需 ModelSim（历史脚本默认 `D:/altera/14.1/modelsim_ase/win32aloem`）

### 3.2 验收（只读，不启动综合、不碰硬件）

```powershell
python -X utf8 VERIFY_MOTION_REFINE.py --source-only      # 仅源/夹具/输入/真值/日志哈希
python -X utf8 VERIFY_MOTION_REFINE.py --functional       # 源 + 23 项有范围功能证据
python -X utf8 VERIFY_MOTION_REFINE.py --native           # 最终验收：源 + 功能 + 原生 bit/STA
python -X utf8 VERIFY_MOTION_REFINE.py --native-only      # 只查原生身份与 STA
```

可移植交付包另有独立入口：

```powershell
python -X utf8 VERIFY_MOTION_REFINE_PACKAGE.py            # 包树只读核验
python -X utf8 motion_refine_selfcheck.py                 # 验收器正向/负向自检
```

### 3.3 重新构建（仅在有意改动 RTL 之后）

```powershell
python -X utf8 FREEZE_MOTION_REFINE.py --replace          # 先建立新的、经审阅的源冻结
powershell -File BUILD_MOTION_REFINE.ps1 -TdInstall F:/Anlogic/TD_6.2.1_Engineer_6.2.168.116
```

- 流程固定为 **fresh 综合 → seed0 原生物理实现 → 容量/ setup / hold 全部通过**；有界监控会在路线长时间冲突时停止自有 TD 进程。
- `-SynthesisOnly` / `-PhysicalOnly` 二选一；`PhysicalOnly` 需要与新源精确绑定的本机综合检查点，干净包中没有该检查点。
- 构建状态只看 `output/motion_refine_job_status.json`。
- 源码改动后必须重新冻结并重跑受影响测试，**既有 bit 与证据不覆盖新源**。

### 3.4 证据汇总与打包

```powershell
python -X utf8 RECORD_MOTION_REFINE_TESTS.py --check-ready   # 只读预检
python -X utf8 RECORD_MOTION_REFINE_TESTS.py                 # 写入 23 项索引（需全部生产者状态通过）
python -X utf8 PACK_MOTION_REFINE.py --destination <目标目录>
```

---

## 4. 功能与开关

### 4.1 四个图像开关（先开者主导）

| 开关 | 功能 |
|---|---|
| SW1 | 卡通效果 |
| SW2 | 红色目标检测 / 联动（最多 6 个目标，含 TRK、XY） |
| SW3 | 运动检测（最多 3 个框，不显示 MOV 数量面板） |
| SW4 | 中心 1:1 分屏（左右同中心裁剪无缩放，其他算法只作用于彩色半屏） |

- 运动单开：只显示当前运动，**不做持续静止追踪**。
- 运动先开 → 再 SW2：运动主导、目标辅助，启用外观保留与 TRK/XY。
- SW2 先开 → 再 SW3：红色主导，运动协助 ROI。
- 同时从关到开：默认红色主导。TRK 是显示模式标志，不是持久身份 ID。

### 4.2 四键双舵机、交通灯与板载报警

- 四键按下接地，松开保持；水平 1100～1900 μs，俯仰 800～2400 μs，中心 1500 μs，周期 20 ms，步进 10 μs / 40 ms。
- 交通开关 J2-7 接地开启；交通 ON 锁角但仍输出 PWM，OFF 并释放四键后恢复可调。
- 灯控：绿 6/10/14/18 秒（1/2/3/4+ 辆），黄 2 秒，全红 ≥2 秒且需清空确认；等待 ≥25 秒有优先权。
- 板载：SW6 上选数码管，SW5 下开蜂鸣器；感知健康时中间通道**连续占用 30 秒**锁存双红 + `AErr/bErr` + 间歇鸣叫，交通 OFF→ON 复位。
- 接线与操作详见 `TRAFFIC_GIMBAL.md`、`BOARD_DISPLAY_ALARM.md`。

### 4.3 LAT 毫秒显示

- 沿用输入的帧龄换算 `(帧差 × 1000 + 30) / 60`，显示 `LAT:xxxMS`；分辨率仍约 16.67 ms，**不是独立时间戳测量**。
- 本轮把该组合乘除换成逐值精确查表（6912 字模像素、921600 像素 VTC/mixer 对齐验证通过），999MS / XXXMS 行为不变。
- 若将来更改采集帧率，查表与对应验证必须同步更新。

---

## 5. 运动检测后端（本轮修订摘要）

640×360 分析输入 → 4×4 块平均 → 160×90 RGB555 双网格（两份 ERAM 交替采集/处理）→
区域 worker 颜色一致性生长得到 32 个提案 → 相机四区域一致性/插值补偿背景 →
**同一个**串行 patch matcher 处理 24 个背景模板与 12 个临时物体模板 → 时间确认 → 相干碎片合并 →
最多 3 个运动框 → 结果/文字/模式按 SOF 原子提交到全分辨率显示。

| 修订项 | 原行为 | 本轮行为 |
|---|---|---|
| 普通物体轮廓 | RGB555 内外色差固定 ≥4，弱对比候选学不成模板 | native 改相对门限：所选对比通道峰值 ≤16 用 2、≤24 用 3，否则 4；噪声/平坦背景仍拒绝 |
| 缓慢运动 | 相机补偿后两次 64 / 三次 96 累计门 | native 门 32 / 48，允许平均 1 分析像素/更新，时间投票保留 |
| 弱慢组合 | 12 vs 20 的改善被 `>8` 严格门拒绝 | 改为 `>=8`，放行恰 8 边界；仍拒绝改善 5 的背景与 20 vs 24 负例 |

- `motion_region_worker.v` 未修改，几何重测与统一轨迹层**尚未实施**。
- `legacy CONTOUR_GUARD=0 / ROBUST_MATCH=0` 规则保留，未通过全局增亮或改白平衡掩盖门槛问题。
- 架构细节见 `MOTION_OBJECTS_ARCHITECTURE.md`、`DATAFLOW_REVIEW.md`。

---

## 6. 验证与证据

`output/motion_refine_test_index.json` 的 23 项按范围绑定源码、夹具、日志与依赖哈希：

- **核心运动 5 项**：`core_contour_support`、`core_motion_evidence`、`core_birth_guard`、`core_native_three`、`core_complex`
- **独立匹配对照 1 项**：`native_contrast`（9 场景 / 10 次精确位移 / 4 项拒绝保护）
- **排名复用 2 项**：`rank_fast`、`rank_vendor`（52 帧 / 744 任务，含真实 ERAM 原语）
- **RGB 场景 1 项**：`rgb_scenes`（`production_ge8`：160 帧、50 项整框真值、45 个负例，IoU 门 0.55）
- **LAT / 外设 7 项**：`latency_ms`、`manual_gimbal`、`manual_gimbal_50mhz`、`gimbal_limits`、`actual_vtc_mixer`、`board_outputs`、`display_alarm_integration`
- **接口 7 项**：`interface_red_hand`、`interface_visual`、`interface_patch_default`、`interface_motion_hud`、`interface_readiness`、`interface_transport`、`interface_traffic`

其中 21 项按真实依赖闭包从父 epoch `DE4EB0EE…` 保留（该 epoch 只差 `hdmi_mixer.v`），2 项为新候选实际执行日志；
索引规定 `output/motion_refine_test_index.json` 及相关 proof **冻结后不再改动**。

仿真回归入口：

```powershell
tests/RUN_MOTION_REFINE_CONTRAST.ps1
tests/RUN_MOTION_REFINE_INTERFACES.ps1
tests/RUN_MOTION_REFINE_RANK.ps1
```

---

## 7. 上板检查

按 `ON_BOARD_TEST_MOTION_REFINE.md` 执行：全关基线 → 单开 SW3 的运动/停止/相机移动 → SW3→SW2 与 SW2→SW3 两种顺序 → SW4 分屏叠加 → 四键云台/交通锁角/数码管/蜂鸣器。
测试视频、回传要求与"不能当成功"的典型现象（扩到背景的大框、离场残框、重复框）都写在该文档中。
上板报告请标明 bit 文件名、开关顺序与测试对象。

---

## 8. 已知局限（不得用仿真 PASS 掩盖）

- 三段视频是**拍屏**输入，只能辅助观察，不等于 RAW 逐帧精度真值。
- 任意快速旋转、强视差、手部形变、反光与大面积遮挡下的框完整性**未保证**；当前仍是 160×90 候选/稀疏模板后端，没有高清 ROI 整框重测或语义分割。
- 新生近全幅贴边的几何防背景规则会连带抑制"几乎充满镜头的新近物体"，这是几何规则而非语义分类。
- 历史偶发**紫黄错位**未在仿真复现；物理 DDR PHY、电气完整性与板上 5 V 双舵机电源余量均需实板确认。
- `tb_board_rank_reuse` 在 frame 21 失败：检测策略有意改变，旧"逐拍完全等价"验收不再适用，失败日志保留未隐藏。
- 实拍检测准确率、舵机动作与报警电气行为一律以上板结果为准。

---

## 9. 文档索引

| 文档 | 内容 |
|---|---|
| `MOTION_REFINE_STATE.md` | 当前状态与过程记录（状态以此为准） |
| `MOTION_REFINE_DELIVERY.md` | 交付包范围与两条验收入口 |
| `MOTION_REFINE_CHANGELOG.md` | 本轮修订、对照验证与局限 |
| `MOTION_REFINE_ERROR_LOG.md` / `REWORK_ERRORS.md` | 失败尝试、被拒候选与撤回记录 |
| `RESOURCE_REPORT.md` | 实际物理资源与最终 STA |
| `MOTION_OBJECTS_ARCHITECTURE.md` | 运动后端架构、合框与开关语义 |
| `DATAFLOW_REVIEW.md` | 主/分析支路数据流与传输复核 |
| `VALIDATION_MATRIX.md` | 历史 20 项覆盖矩阵与"不能据此声称"边界 |
| `TRAFFIC_GIMBAL.md` / `BOARD_DISPLAY_ALARM.md` | 接线、灯控规则、数码管/蜂鸣器 |
| `LATENCY_TILT_UPDATE.md` | 毫秒显示与俯仰 800～2400 μs 修改 |
| `ON_BOARD_TEST_MOTION_REFINE.md` | 上板对照检查清单 |
| `ENGINEERING_MEMORY.md`、`CURRENT_HANDOFF.md`、`SESSION_STATE_CURRENT.md`、`BOARD_*`、`*_TILT_*` 等 | 历史版本交接与追溯，非本轮验收入口 |

---

## 10. 约定

1. 不覆盖、不删除 `reference/incoming_20261103` 中的原始输入与旧 bit。
2. 运动模块之外的 top、引脚、SDC、舵机、交通、相机、AWB、DDR/HDMI、720P 与开关规则均与输入逐字节对照。
3. 禁止用旧 bit 改名交付、忽略负保持时间、放松真值门或拿综合估算替代物理资源。
4. 修改 RTL 后：重新冻结 → 重跑受影响测试 → 原生综合/布局布线/STA → 重新绑定 bit 与证据。
5. 本工程不做 YOLO 或学习型语义分类；不下载 FPGA、不写 Flash 的规则保持不变。
6. `project/` 下的源集合必须与冻结清单**完全一致**。若在其中新增任何非瞬态文件（例如在 `project/`
   里执行 `git init` 产生的 `.git`），`VERIFY_MOTION_REFINE.py --source-only` 会以
   `Non-transient source set changed` 失败；请把版本库放在工程根目录（用 `.gitignore` 排除
   `output/`、`*_Runs/`、日志）或将 `project/.git` 移出后再验收。
