# navbot_chassis —— STM32 底盘固件「骨架版」

> 对应 P3 项目（ROS2 自主导航移动机器人）真机路线的 **S19 固件**。
> 这是**骨架，不是成品**：核心逻辑 25 个 TODO 留空，等你填（铁律：只给需求+判据，不给成品）。

---

## 一、它在整条链路里的位置

```
PC(chassis_bridge.py)  ──V帧(速度指令)──▶  USART1(PA9/PA10)  ──▶  固件解析
                                          USART1 ◀──  O帧(里程计)  ──  固件组帧
固件：TIM2/3 读编码器 ─▶ 算轮速 ─▶ 累加里程计 ─▶ 组 O 帧
固件：V帧 ─▶ 目标速度 ─▶ 限幅 ─▶ PWM(TIM4) + DIR(PA4/PA5) ─▶ 电机驱动板
```

## 二、引脚映射（默认方案，按实际接线改）

| 功能 | 引脚 | 复用 |
|---|---|---|
| USART1 TX / RX | PA9 / PA10 | 接 USB-TTL（交叉） |
| 左轮编码器 A/B | PA0 / PA1 | TIM2_CH1/CH2 |
| 右轮编码器 A/B | PA6 / PA7 | TIM3_CH1/CH2 |
| 左轮 / 右轮 PWM | PB6 / PB7 | TIM4_CH1/CH2 |
| 左轮 / 右轮 DIR | PA4 / PA5 | GPIO 输出 |
| 电池电压 | PA2 | ADC1_IN2 |
| LED | PC13 | 心跳 |

## 三、编译与烧录

```bash
cd ~/bridge/stm32_ws/navbot_chassis
make -j$(nproc)          # 产物在 build/
make size                # 看 Flash/RAM 占用
make flash               # st-flash 烧录（ST-Link）
make clean
```

## 四、25 个 TODO 分布

| 文件 | TODO | 内容 |
|---|---|---|
| `Core/Src/main.c` | A1 A2 B1 B3 B4 B5 B6 C1 C2 | 核心逻辑（全部留空） |
| `Core/Src/stm32f1xx_hal_msp.c` | — | 外设底层初始化（已写死） |

每个 TODO 后面都带「判据」，写完就自测。完整作战计划、推进顺序、验收判据见：
`桌面/具身智能学习路线/13_ROS2自主导航移动机器人/02_阶段文档/S19_STM32底盘固件_硬件层作战计划.md`

## 五、目录结构

```
navbot_chassis/
├── Makefile                    ← 手写构建，TARGET=navbot_chassis
├── STM32F103C8Tx_FLASH.ld      ← 64K Flash / 20K RAM 链接脚本
├── Core/
│   ├── Inc/main.h
│   ├── Inc/stm32f1xx_hal_conf.h    ← 已启用 HAL_TIM_MODULE_ENABLED
│   └── Src/
│       ├── main.c              ← 底盘固件骨架（25 TODO）
│       ├── stm32f1xx_hal_msp.c ← TIM2/3 编码器 + TIM4 PWM + ADC + UART
│       ├── stm32f1xx_it.c      ← SysTick
│       └── syscalls.c          ← printf 重定向
└── Drivers/
    ├── CMSIS/                  ← 内核头 + 启动文件 + system_stm32f1xx.c
    └── STM32F1xx_HAL_Driver/   ← ST 官方 HAL（含补上的 hal_tim.c / hal_tim_ex.c）
```

## 六、三个必踩的坑

1. **printf 不出数据**：nano.specs 下 stdout 全缓冲，必须 `setvbuf(stdout, NULL, _IONBF, 0)`（见 TODO-B6【1】）。
2. **上电狂转**：PWM 初始化（置 0）要在编码器初始化之前（判据 C3）。
3. **拔线撞墙**：没写失联保护（500ms 超时停车）会一直跑（判据 B2）。
