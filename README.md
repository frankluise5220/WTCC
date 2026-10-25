# WTCC 换水控制器固件

基于 ESP32 的鱼缸换水控制器固件，硬件平台为 **WTCC-ESP32-RLY4-V1**（四路继电器 + 四路光耦输入）。本固件将成品板原厂 Modbus 从机功能与换水控制器逻辑合并，支持网页控制、MQTT、Home Assistant 自动发现，以及单键遥控（光耦#4/IN4：1 击排水、2 击上水、3 击换水、长按急停）。

当前版本：**v5.25**

## 功能特性

### 核心控制

- **4 路继电器 + 4 路光耦输入**：继电器干接点可直接串接 220V 交流水泵、电磁阀线圈；光耦为无源输入（低电平有效），带电气隔离与每路状态灯。
- **换水流程**：单次执行、开机启动、定时计划（10 条）、执行对象、超时保护（T0~T5）、手动互斥三选一。
- **传感器冗余**：上下双浮球水位检测，另有 GPIO36/39 作冗余备用浮球。
- **单键遥控**：光耦#4（IN4）以击数区分动作 —— 1 击排水、2 击上水、3 击换水、长按急停。
- **浸水安全保护**：光耦#3（IN3）接浸水传感器，为最高优先级安全信号；确认浸水后强制全关并**自锁**，传感器变干也不会自动解除，须人工复位（网页按钮或 MQTT）。

### 连接与控制

- **网页控制**：内置 Web 界面（控制 / 设置 / 信息 / 系统 / 日志五个页签），支持局域网内直接操作与配置。
- **MQTT**：完整主题订阅/发布，支持远程控制与状态上报。
- **Home Assistant 自动发现**：接入 MQTT 后自动注册实体，无需手动配置。
- **Modbus RTU 从机**（功能码 01/02/05/0F，地址 1 / 115200 / 8N1）：保留成品板原厂 RS485 能力，DAM 调试软件、RS485 上位机、HA Modbus 接入继续可用。
- **WiFi 自动切换**：热点 / 家网自动切换，断网自愈重连。
- **OTA 无线烧录**：Arduino IDE 端口列表中显示 `WTCC at 192.168.x.x`，支持无线更新固件。

### Home Assistant 自动发现实体

| 类型 | 实体名 | 说明 |
|---|---|---|
| switch | 开机启动 | 上电自动运行开关 |
| switch | 泵正转 / 泵反转 / 电磁阀 / 备用继电器 | 4 路继电器 |
| button | 换水 / 排水 / 上水 | 触发对应流程 |
| button | 复位漏水保护 | 解除浸水自锁 |
| binary_sensor | 下水位 / 上水位 | 双浮球水位 |
| binary_sensor | 漏水报警 | 浸水检测（device_class=moisture） |
| binary_sensor | 流程完成 | 最近一次流程正常跑完 |
| sensor | 上次结果 | 如「换水完成」「上水超时（T5）」 |
| number | T0~T5 | 各阶段超时/延时参数 |

### MQTT 主题

基础前缀 `esp-pump/`：

| 主题 | 方向 | 说明 |
|---|---|---|
| `esp-pump/relay1..4` | 状态 | 4 路继电器状态 |
| `esp-pump/relay1..4/set` | 命令 | 控制继电器 |
| `esp-pump/floatA` `floatB` | 状态 | 上下水位 |
| `esp-pump/leak` | 状态 | 漏水报警 |
| `esp-pump/boot` | 状态/命令 | 开机启动 |
| `esp-pump/t0..t5` | 状态/命令 | 定时参数 |
| `esp-pump/progress` | 状态 | 流程进度（网页/MQTT 共用） |
| `esp-pump/cmd` | 命令 | `start` / `drain` / `fill` / `leak_reset` |
| `esp-pump/ip` `diag` | 状态 | 设备 IP、诊断信息 |

## 软件界面

<table>
  <tr>
    <td align="center"><b>控制页</b><br><img src="docs/screenshots/shot_1_control.png" width="360"></td>
    <td align="center"><b>设置页</b><br><img src="docs/screenshots/shot_2_settings.png" width="360"></td>
  </tr>
  <tr>
    <td align="center"><b>信息页</b><br><img src="docs/screenshots/shot_3_info.png" width="360"></td>
    <td align="center"><b>系统页</b><br><img src="docs/screenshots/shot_4_system.png" width="360"></td>
  </tr>
  <tr>
    <td align="center" colspan="2"><b>日志页</b><br><img src="docs/screenshots/shot_5_log.png" width="360"></td>
  </tr>
</table>

## 目录结构

```
WTCC/
├── WTCC_firmware/
│   └── WTCC_firmware.ino    # 固件源码
├── .github/                 # GitHub Actions 云编译配置
└── docs/screenshots/        # 软件界面截图
```

## 编译与烧录

固件使用 Arduino IDE 编译，目标芯片为 ESP32。仓库已配置 GitHub Actions 云编译，可直接在 Actions 中产出固件，也可本地编译后烧录。

烧录方式：
- **OTA 无线烧录**：设备联网后在 Arduino IDE 端口列表中会显示 `WTCC at 192.168.x.x`。
- **串口烧录**：全片合并固件见 `_bin/` 目录（未纳入版本库）。

## 版本历史

| 版本 | 说明 |
|---|---|
| v5.25 | 修 HA 实体全消失（报文少一个 `}`）；恢复出厂设置为真全清；BOOT 键表三列 |
| v5.24 | 修「下次执行时间」指向过去 |
| v5.23 | 事件日志时间改成真实时钟 |
| v5.22 | 修 HA 发现报文超 256 字节发不出去（v5.21 回归） |
| v5.21 | 固件源码 + GitHub Actions 云编译配置 |
