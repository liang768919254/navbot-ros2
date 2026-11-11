# H1 · ESP8266 接线确认与 AT 透传方案

> 2026-10-31。起因：底盘 PCB 上的 WiFi 是焊死的 ESP8266，`check_hw.py --wifi-module` 要先确认它到底接在哪。

## 一、已知事实（核过代码，不是推测）

| 事实 | 证据位置 |
|---|---|
| USART1（PA9/PA10）接 USB-TTL，跑 printf 和命令接收 | `Drivers/SYSTEM/usart/usart.h:39-49`，`USART_UX` = `USART1` |
| USART3（PB10/PB11）接 RPLIDAR A1 | `Drivers/BSP/RADAR/rplidar.h:34`，`RPLIDAR_USART` = `USART3` |
| 全工程没有 USART2 初始化 | grep `USART2` 只在 CMSIS 头文件命中，业务代码 0 处 |
| 全工程没有 ESP8266 / AT 指令代码 | grep `ESP8266|AT+CWJAP|AT+CIFSR` 业务代码 0 处 |

F103C8T6 一共三路 USART，前两路已被占满。**ESP8266 只能落在 USART2（PA2/PA3），或者根本不经过 STM32。**

## 二、三种接线，靠现象区分

| | 接线方式 | 插上电后能看到什么 | `--wifi-module` 的表现 |
|---|---|---|---|
| **A** | ESP 的 UART 直连板载 USB-TTL（独立一颗 CH340/CP210x） | `lsusb` 出现 **2 颗**转串口芯片，`/dev/ttyUSB0` + `/dev/ttyUSB1` | 能探到，出 SSID/信号/IP |
| **B** | ESP 接 STM32 的 USART2，固件没做透传 | 只有 **1 颗** CH340 | 串口能开，但 AT 无应答 |
| **C** | ESP 接 STM32 的 USART2，固件做了透传 | 只有 **1 颗** CH340 | 同上（脚本还不支持走 STM32 转发） |

## 三、第一步：数一转串口芯片有几颗

```bash
lsusb | grep -E "1a86|10c4|0403"
```

或者让脚本自己报：

```bash
python3 ~/navbot_ws/tools/env/check_hw.py --json - \
  | python3 -c "import json,sys; [print(d['vid_pid'], d.get('known','?'), d.get('product','')) for d in json.load(sys.stdin)['usb']['devices']]"
```

- **2 颗** → 大概率是 A，直接 `--wifi-module` 就能读状态
- **1 颗** → B 或 C，ESP 挂在 STM32 后面，往下走

## 四、第二步：让脚本自己说「这个口上挂的是什么」

比数芯片更直接的办法——插上板子直接探：

```bash
python3 ~/navbot_ws/tools/env/check_hw.py --wifi-module
```

脚本会先**静听 0.8 秒不发任何字节**，再发 `AT`。两种结果对应两种结论：

| 输出 | 含义 |
|---|---|
| `对面是: STM32 底盘（正在发 O/V 帧）` | 这个口是 USART1，ESP 不在上面 → 走第五步量 PA2/PA3 |
| `对面是: 乱码（波特率不匹配）` | 波特率不对，或 ESP 没烧 AT 固件 |
| `静默（没有 AT 应答，也没有主动数据）` | 对面没供电 / EN 没拉高 / 透传没做 |
| 出 SSID、信号、IP | ESP 直连 USB-TTL，恭喜，情况 A |

「静听」这招能成立，是因为 navbot 的 STM32 每 50 Hz 主动吐一帧 O 帧，
而 ESP8266 是纯被动的——不问不说话。**有没有主动数据流，直接区分了两者。**

> 看到 `STM32 底盘` 其实是好消息：说明 USART1 链路是通的、底盘固件在跑。
> 只是 ESP 不在这条线上。

### 如果扫不到可读内容：固件不持续输出

两次实测把这件事摊开了：

**第一次**（`--wifi-module`）：`@115200` 静默、`@9600` 收到乱码。翻固件源码找到答案：

```c
/* User/main.c:64 */
usart_init(115200);   /* 调试串口波特率, 复位一上电就能看到日志 */
```

波特率就是 115200，没错。但 R4 固件**只在复位时打一次 banner**，之后沉默。
盲试 AT 永远试不出来。

**第二次**（`--baudscan`，六档各听 2 秒）：

```
115200 : 0 字节          9600   : 0 字节
57600  : 1210 字节 ← 乱码  38400  : 0 字节
230400 : 0 字节          460800 : 0 字节
```

只有一档撞到数据，而且是错的波特率——这恰好证明**数据是间歇发的**：
2 秒的窗口太小，谁撞上谁有，跟波特率对不对关系不大。

看 hex 也能佐证：`28 0c 6d 93 39 2d a1 88 ...` 一半字节高位为 1、一半是可打印字符，
正是「115200 的数据被 57600 采样」的典型样子（差 2 倍，丢一半 bit）。

**正确做法：在已知波特率上开长窗口，并按复位键让固件重新说话。**

```bash
python3 ~/navbot_ws/tools/env/check_hw.py --listen --baud 115200 --sec 30
```

跑起来之后**按一下板子的复位键**。抓到的话会直接 dump 出固件打印的内容：

```
【静听 /dev/ttyUSB0 @ 115200】
  时长   : 30.0 秒
  收到   : 864 字节（约 28.8 B/s）  →  可读文本，但不是 AT 协议
  内容   : R4 radar link self-test start
           USART3 ready
           RPLIDAR A1 fw 1.28
```

如果 30 秒还是 0 字节，那才需要怀疑：固件没在跑、板子没上电、或者 TX 线断了。

> 判读技巧：`6b 6b 6b 6b`（连续四个相同字节）是过采样特征，说明线路上有数据、
> 只是波特率没对上。每一档都是 0 字节，才是真的没数据。

## 五、第三步：确认 PA2/PA3 有没有连到 ESP

需要看原理图或用万用表量通断。两个必查点：

1. **PA2（USART2_TX）→ ESP 的 RX**，PA3（USART2_RX）← ESP 的 TX
   → 串口是**交叉接**，TX 接 TX 是最常见的接错方式，表现为「完全没反应」
2. **ESP 的 EN/CH_PD 有没有上拉**
   → 没拉高时模块根本不启动，串口也是死的

## 六、第四步：如果要走固件透传（情况 B/C）

### 设计约束（先定清楚，不然改到一半会推倒重来）

1. **命令要能和现有帧区分**。现有 O 帧/V 帧以 `O,` / `V,` 开头，透传用 `E,` 前缀，解析器才不用大改
2. **不得抢中断优先级**。USART3（雷达）已经是抢占优先级 1，透传不能更高，否则雷达点会丢
3. **必须有超时**。ESP 掉电时 `esp_send_at` 要能自己返回，绝不能让主循环卡在等 `OK`
4. **进得去也要出得来**。透传是「一问一答」，不能进了一个模式就再也回不来

### 骨架（结构给到，实现自己填）

```c
/* usart2_esp.c —— 骨架，TODO 自己补 */

/* 初始化 USART2，波特率按 ESP 固件定（1.7.x 默认 115200） */
void esp_init(uint32_t baud);                                    /* TODO */

/* 透传一条 AT 指令
 *   1. 清 USART2 接收缓冲
 *   2. 发 cmd + "\r\n"
 *   3. 最多等 timeout_ms，把收到的字节攒进 out
 *   4. 返回实际字节数
 * 关键：等待期间不能让 PID 停摆，用非阻塞轮询或状态机，别用死等
 */
uint16_t esp_send_at(const char *cmd, char *out,
                     uint16_t out_max, uint32_t timeout_ms);     /* TODO */

/* USART1 收到 "E,<at指令>" 时调这个：把 ESP 响应包成 "E,<响应>" 回上位机 */
void esp_handle_cmd(const char *at);                             /* TODO */
```

### 验收判据（写完拿这几条卡，缺一条都不算完）

| # | 操作 | 期望 |
|---|---|---|
| 1 | 上位机发 `E,AT` | 回含 `OK` 的响应 |
| 2 | 发 `E,AT+GMR` | 响应含 `AT version:` |
| 3 | 发 `E,AT+CWJAP?` | 响应含 `+CWJAP:` 或 `No AP` |
| 4 | **把 ESP 断电**，发 `E,AT` | `timeout_ms` 后返回超时，**主循环不卡死** |
| 5 | **透传进行中**发 `V,1,0.2,0.2` | 电机照常转（说明没抢中断） |
| 6 | 连发 100 条 `E,AT` | 不丢帧、不溢出、不死机 |

> 4 和 5 是重点。透传最容易引入的两个 bug 就是「ESP 掉电把整车主循环拖死」和「为了等响应把电机控制中断饿死」。

## 七、脚本当前的边界

`--wifi-module` **只覆盖情况 A**（ESP 直连 USB-TTL）。

- 情况 B：先按第五节加固件透传
- 情况 C：脚本再加一个 `--esp-via-stm32` 模式，把 AT 指令包成 `E,` 帧从 USART1 转发，解析同 `_parse_esp_responses`

先把第三节那一步做了，确认是 A 还是 B/C，再决定要不要动固件。
