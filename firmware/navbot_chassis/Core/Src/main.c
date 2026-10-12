/* ============================================================================
 * main.c —— navbot 底盘固件「骨架版」（STM32F103C8T6）
 * ============================================================================
 * ★★ 这是骨架，不是成品。核心逻辑全部留空（TODO），只给「需求 + 判据」。
 *
 *    为什么这么给：2026-09-22 立的铁律（见 代码回收清单.md §6）——
 *    核心逻辑（协议解析、编码器统计、速度换算、电机控制、失联保护）
 *    只给「需求 + 骨架 + 验收判据」，不给整份可编译成品。
 *    成品代码把决策点全藏起来了；记忆存的是推理链，不是结论。
 *
 *    每个 TODO 后面都跟了「判据」，写完用它验证，不是"跑通即可"。
 *    卡住先自己查 20 分钟（查手册、查报错），再来问，带上「试了什么 + 报错原文」。
 *
 * ============================================================================
 * 与外设的对应关系（相对 adc_serial_nocube 新增的用 ★ 标出）
 * ============================================================================
 *   USART1   PA9/PA10  115200 8N1 → PC（上行 O 帧 / 下行 V 帧）   [已有]
 *   TIM2     PA0/PA1   编码器接口模式 → 左轮编码器                 ★ 新增
 *   TIM3     PA6/PA7   编码器接口模式 → 右轮编码器                 ★ 新增
 *   TIM4     PB6/PB7   PWM 输出 → 左/右电机驱动板                  ★ 新增
 *   GPIO     PA4/PA5   电机方向 DIR（左/右）                       ★ 新增
 *   ADC1     PA2       电池电压采样（分压后）                      ★ 改通道
 *   GPIO     PC13      板载 LED 心跳                                [已有]
 *   SysTick  -         1ms 时基（50Hz 控制循环 = 每 20 tick）        [已有]
 *
 * ★ 说明：参考骨架注释里写「TIM3 → 两个轮子编码器」是不严谨的——
 *   一个 TIM 的编码器接口模式只能接【一个】编码器（CH1/CH2 = A/B 两相）。
 *   两个轮子 = 两个编码器 = 两个 TIM。所以这里用 TIM2 + TIM3 各接一个，
 *   TIM4 腾出来做 PWM，时基交给已有的 SysTick。这是对骨架的一处纠正。
 *
 * ============================================================================
 * 验收判据总表（及格线，逐条打勾）
 * ============================================================================
 *  [ ] A1  上行帧率 50Hz ± 2Hz（ros2 topic hz /odom）
 *  [ ] A2  静止时 vel_l/vel_r 在 ±5 mm/s 内（零漂可接受）
 *  [ ] A3  手推轮子慢转，vel 符号方向正确（前进为正）
 *  [ ] A4  seq 单调递增、无回绕跳变（PC 侧统计丢帧数应为 0）
 *  [ ] B1  收到 V 指令后 100ms 内轮子开始动
 *  [ ] B2  ★ 断开 PC（拔 USB）后 500ms 内自动停车
 *  [ ] B3  收到非法下行帧（字段数不对）时不动作、不崩
 *  [ ] C1  连续跑 10 分钟不重启、不丢帧累积
 *  [ ] C2  电池电压读数与万用表实测误差 < 0.2V
 *  [ ] D1  ★ 不看代码能说出：为什么速度用 mm/s 整数
 *  [ ] D2  ★ 不看代码能说出：为什么必须有 seq 和 tick_ms
 *  [ ] D3  ★ 不看代码能说出：失联保护为什么设 500ms
 *  [ ] D4  ★ 能说出：关中断保护共享变量 (a) 方案的缺点是什么
 * ==========================================================================*/

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ============================================================================
 * 外设句柄（MspInit 里按 Instance 判断是哪个定时器）
 * ==========================================================================*/
TIM_HandleTypeDef  htim2;   /* 左轮编码器 */
TIM_HandleTypeDef  htim3;   /* 右轮编码器 */
TIM_HandleTypeDef  htim4;   /* 电机 PWM */
ADC_HandleTypeDef  hadc1;
UART_HandleTypeDef huart1;

#define RX_LINE_BUFFER_SIZE 64U

static uint8_t rx_byte;
static char rx_line[RX_LINE_BUFFER_SIZE];
static volatile uint16_t rx_line_length;
static volatile uint8_t rx_line_ready;
static volatile uint8_t rx_discard_line;

/* 0 = HSE(8MHz 晶振 ×9 = 72MHz)，1 = HSI 兜底(8MHz/2 ×16 = 64MHz) */
uint8_t g_clk_fallback = 0;

/* ============================================================================
 * 一、可调参数（★★ 这些值必须和 PC 侧 / URDF 保持一致）
 * ==========================================================================*/

/* 轮距 [m]：★★ 必须等于 navbot.xacro 里的 wheel_sep，也等于
 *           nav2_params 里 diff_drive 的 wheel_separation。
 *           三处不一致 = 里程计系统性偏差，且极难查。 */
#define WHEEL_SEPARATION_M      0.16f

/* 轮半径 [m]：★ 必须和 URDF 的 wheel_r 一致 */
#define WHEEL_RADIUS_M          0.0325f

/* 编码器每转脉冲数 [脉冲/圈]
 * ★ TODO-B1：查你的电机手册 + 减速比，算出这个数。
 *   公式：编码器线数 × 减速比 × 4（四倍频，TIM 编码器模式用了 TI12）
 *   典型 TT 马达：11 线 × 1:48 × 4 ≈ 2112
 *   ★ 判据：代回公式，手转轮子一圈，enc 增量应约等于该值，偏差 >10% 就是算错。 */
#define ENCODER_PPR             (2112.0f)

/* 控制周期：50Hz 上行 = 20ms */
#define CTRL_PERIOD_MS          20

/* ★ 失联保护超时 [ms] —— 理由见 navbot_protocol.md §4 */
#define CMD_TIMEOUT_MS          500

/* 最大轮速限制 [mm/s]：保护电机和机械
 * ★ 判据：PC 发 2.0 m/s，实际轮速应被截到 800 mm/s = 0.8 m/s。 */
#define MAX_WHEEL_SPEED_MMS     800

/* PWM 上限（对应 TIM4 的 ARR 值）与死区 */
#define PWM_MAX                 1000
#define PWM_DEADZONE            30

/* ============================================================================
 * 二、全局状态
 * ==========================================================================*/
typedef struct {
    int32_t  enc_l;         /* 左编码器累计计数 */
    int32_t  enc_r;         /* 右编码器累计计数 */
    int32_t  vel_l;          /* 左轮速度 mm/s */
    int32_t  vel_r;          /* 右轮速度 mm/s */
    int16_t  pwm_l;         /* 左轮当前 PWM */
    int16_t  pwm_r;         /* 右轮当前 PWM */
    int32_t  target_l_mms;  /* 左轮目标速度 [mm/s] */
    int32_t  target_r_mms;  /* 右轮目标速度 [mm/s] */
    uint32_t seq_out;       /* 上行帧序号 */
    uint32_t seq_in;        /* 下行收到的最后序号 */
    uint32_t last_cmd_tick; /* 最后收到指令时刻（失联保护用）*/
    uint16_t vbat_mv;       /* 电池电压 [mV] */
    int16_t  temp_c100;     /* 板温 ×100 */
} chassis_t;

/* 骨架阶段 g_chassis 还没被 TODO 引用，先不加 static 避免 unused 警告；
 * 实现完控制循环后可改回 static（模块私有更规范）。 */
chassis_t g_chassis;

/* ============================================================================
 * 三、核心逻辑 —— 全部留空（TODO），你来实现
 * ============================================================================
 * 这些函数故意不加 static：骨架阶段它们还没被 main 调用，
 * 非 static 全局函数不会被 -Wall 报 "unused function"，编译干净。
 * 实现完成后，如果确认不需要被别的文件引用，可以改回 static。
 * ==========================================================================*/

/* ★ TODO-A1：实现「读编码器」。
 *   需求：读 TIM2/TIM3 的 CNT 得到「本次增量」（不是累计值），累加到 enc_l/enc_r。
     读 TIM2/TIM3 的 CNT
     算 本次增量（20ms 内的脉冲数）
     累加 到 g_chassis.enc_l / enc_r
 *   关键坑：TIM 的 CNT 是 16 位，会在 0↔65535 回绕，必须正确处理，
 *           否则转向时出现 ±65535 跳变 = 里程计瞬间跳几十米。
 *   判据：手转轮子连续 5 圈以上，enc 平滑单调，无任何 >5000 单步跳变。
 *   提示：判断回绕的思路——|delta| > 32767 说明回绕，实际 delta = delta ± 65536。
 *         但先想清楚：20ms 内轮子可能转超过 32767 个脉冲吗？如果不可能，
 *         这个判断是"防御性编程"还是"掩盖真正的 bug"？这个思考比代码重要。 */
void read_encoders(void)
{
    /* TODO-A1：在这里实现 */
    uint16_t cnt_tim2 = __HAL_TIM_GET_COUNTER(&htim2);   /* 读 CNT 当前值 */
    uint16_t cnt_tim3 =__HAL_TIM_GET_COUNTER(&htim3); /* 读 CNT 当前值 */
    int32_t delta_tim2 = cnt_tim2;
    int32_t delta_tim3 = cnt_tim3;

    if (delta_tim2 > 32767)
    {
        delta_tim2 -= 65536;
    }
    if (delta_tim3 > 32767)
    {
        delta_tim3 -= 65536;
    }

    g_chassis.enc_l += delta_tim2;
    g_chassis.enc_r += delta_tim3;

    __HAL_TIM_SET_COUNTER(&htim2, 0); /* 清零 CNT */
    __HAL_TIM_SET_COUNTER(&htim3, 0); /* 清零 CNT */
   
}

/* ★ TODO-A2：实现「脉冲增量 → 线速度 mm/s」。
 *   公式自己推：弧长/脉冲 = (2π×R)/PPR；位移 = delta × 该值；速度 = 位移 / 周期。
 *   坑1：单位，最后 ×1000 变 mm/s。
 *   坑2：整数截断——delta 小（1~2 脉冲）时低速严重失真，判据 A2 就是抓这个。
 *   坑3：50Hz 调用不能太重，想一下「2πR/PPR/0.02×1000」这串常数能否预计算。
 *     int32_t K = (2*3.14*0.0325/ENCODER_PPR) * 1000000000;
 *     int32_t vel = ((delta_enc*K)/0.02) / 1000000;
 *  */
int32_t enc_delta_to_mms(int32_t delta_enc)
{
    /* TODO-A2：在这里实现 */
    // 弧长/脉冲 = 2πR / PPR            R=0.0325  PPR=2112
    // 位移      = delta × (弧长/脉冲)
    // 速度      = 位移 / 周期           周期=0.02s
    // mm/s      = 速度 × 1000

    return delta_enc * 4834 / 1000;
}

static int parse_u32_decimal(const char *text, uint32_t *value)
{
    uint32_t result = 0;

    if (*text < '0' || *text > '9')
    {
        return 0;
    }

    while (*text >= '0' && *text <= '9')
    {
        uint32_t digit = (uint32_t)(*text - '0');
        if (result > (UINT32_MAX - digit) / 10U)
        {
            return 0;
        }
        result = result * 10U + digit;
        text++;
    }

    if (*text != '\0')
    {
        return 0;
    }

    *value = result;
    return 1;
}

static int parse_i32_decimal(const char *text, int32_t *value)
{
    uint32_t magnitude = 0;
    uint32_t limit;
    int negative = 0;

    if (*text == '-' || *text == '+')
    {
        negative = (*text == '-');
        text++;
    }
    if (*text < '0' || *text > '9')
    {
        return 0;
    }

    limit = (uint32_t)INT32_MAX + (uint32_t)negative;
    while (*text >= '0' && *text <= '9')
    {
        uint32_t digit = (uint32_t)(*text - '0');
        if (magnitude > (limit - digit) / 10U)
        {
            return 0;
        }
        magnitude = magnitude * 10U + digit;
        text++;
    }

    if (*text != '\0')
    {
        return 0;
    }

    if (negative)
    {
        *value = (magnitude == (uint32_t)INT32_MAX + 1U)
               ? INT32_MIN : -(int32_t)magnitude;
    }
    else
    {
        *value = (int32_t)magnitude;
    }
    return 1;
}

/* ★ TODO-B3：实现「逐字节接收 + 整行解析」（下行 V 帧）。
 *   需求：
 *     1. 接收侧把字节塞进缓冲，收到 '\n' 置"有一整行"标志交给主循环解析
 *        （★ 别在中断里做 strtok/atoi 解析，太重，会拖长中断、丢字节）
 *     2. 帧头不是 'V' → 静默丢弃（判据 B3：收到回环的 O 帧/乱码不能动作）
 *     3. 严格校验字段数 = 4（"V,seq,vel_l,vel_r"），多一个少一个都当坏帧
 *     4. vel 限幅到 ±MAX_WHEEL_SPEED_MMS（判据：V,1,5000,5000 → target 截成 800）
 *        想一下：为什么 PC 侧截幅了，MCU 侧还要再截一次？
 *     5. 刷新 last_cmd_tick 和 seq_in
 *   提示：经典坑——用 HAL_UART_Receive_IT 时忘了重启接收，只收到第一个字节；
 *         现象是"PC 发什么都收不到"，容易被误判成接线问题。 */
void parse_line(const char *line)
{
    char frame[64];
    char *fields[4];
    char *comma;
    size_t length = 0;
    uint32_t sequence;
    int32_t velocity_left;
    int32_t velocity_right;

    while (length < sizeof(frame) && line[length] != '\0')
    {
        length++;
    }
    if (length == sizeof(frame))
    {
        return;
    }

    memcpy(frame, line, length + 1U);
    while (length > 0U && (frame[length - 1U] == '\n' || frame[length - 1U] == '\r'))
    {
        frame[--length] = '\0';
    }

    fields[0] = frame;
    for (size_t i = 1U; i < 4U; i++)
    {
        comma = strchr(fields[i - 1U], ',');
        if (comma == NULL)
        {
            return;
        }
        *comma = '\0';
        fields[i] = comma + 1;
    }
    if (strchr(fields[3], ',') != NULL || strcmp(fields[0], "V") != 0)
    {
        return;
    }

    if (!parse_u32_decimal(fields[1], &sequence)
        || !parse_i32_decimal(fields[2], &velocity_left)
        || !parse_i32_decimal(fields[3], &velocity_right))
    {
        return;
    }

    if (velocity_left > MAX_WHEEL_SPEED_MMS)
    {
        velocity_left = MAX_WHEEL_SPEED_MMS;
    }
    else if (velocity_left < -MAX_WHEEL_SPEED_MMS)
    {
        velocity_left = -MAX_WHEEL_SPEED_MMS;
    }

    if (velocity_right > MAX_WHEEL_SPEED_MMS)
    {
        velocity_right = MAX_WHEEL_SPEED_MMS;
    }
    else if (velocity_right < -MAX_WHEEL_SPEED_MMS)
    {
        velocity_right = -MAX_WHEEL_SPEED_MMS;
    }

    g_chassis.seq_in = sequence;
    g_chassis.target_l_mms = velocity_left;
    g_chassis.target_r_mms = velocity_right;
    g_chassis.last_cmd_tick = HAL_GetTick();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_line_ready)
        {
            if (rx_byte == '\n')
            {
                rx_discard_line = 0U;
            }
            else
            {
                rx_discard_line = 1U;
            }
        }
        else if (rx_discard_line)
        {
            if (rx_byte == '\n')
            {
                rx_discard_line = 0U;
                rx_line_length = 0U;
            }
        }
        else if (rx_byte == '\n')
        {
            rx_line[rx_line_length++] = '\n';
            rx_line[rx_line_length] = '\0';
            rx_line_ready = 1U;
        }
        else if (rx_line_length < RX_LINE_BUFFER_SIZE - 2U)
        {
            rx_line[rx_line_length++] = (char)rx_byte;
        }
        else
        {
            rx_line_length = 0U;
            rx_discard_line = 1U;
        }

        (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    }
}

/* ★ TODO-B4：实现「速度 → PWM」映射。
 *   需求：
 *     1. target_mms 映射成 PWM（-PWM_MAX ~ +PWM_MAX）
 *     2. ★ 死区补偿：|PWM| 落在 (0, PWM_DEADZONE) 时，要么补到 PWM_DEADZONE，要么给 0。
 *        不做的后果：小速度指令下电机嗡嗡响但不转，Nav2 以为车在动 → 定位漂移。
 *     3. 正负号决定 DIR 引脚电平
 *   分岔口（选一种并说明理由）：
 *     甲·开环：PWM ∝ 目标速度。简单，但电压下降/负载变化会掉速。
 *     乙·闭环 PID：用编码器反馈做速度环。抗扰动，但要调 PID。
 *     ★ 建议先甲跑通整条链路，再把乙当提分项，对比两者轨迹误差。 */
static int16_t target_speed_to_pwm(int32_t target_mms)
{
    int32_t pwm;

    if (target_mms > MAX_WHEEL_SPEED_MMS)
    {
        target_mms = MAX_WHEEL_SPEED_MMS;
    }
    else if (target_mms < -MAX_WHEEL_SPEED_MMS)
    {
        target_mms = -MAX_WHEEL_SPEED_MMS;
    }

    pwm = target_mms * PWM_MAX / MAX_WHEEL_SPEED_MMS;
    if (pwm > 0 && pwm < PWM_DEADZONE)
    {
        pwm = 0;
    }
    else if (pwm < 0 && pwm > -PWM_DEADZONE)
    {
        pwm = 0;
    }

    return (int16_t)pwm;
}

void set_motor_pwm(int32_t target_l_mms, int32_t target_r_mms)
{
    int16_t pwm_l = target_speed_to_pwm(target_l_mms);
    int16_t pwm_r = target_speed_to_pwm(target_r_mms);
    uint16_t duty_l = (uint16_t)(pwm_l < 0 ? -pwm_l : pwm_l);
    uint16_t duty_r = (uint16_t)(pwm_r < 0 ? -pwm_r : pwm_r);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,
                      pwm_l > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
                      pwm_r > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty_l);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty_r);

    g_chassis.pwm_l = pwm_l;
    g_chassis.pwm_r = pwm_r;
}

void send_odom_frame(void);

/* ★ TODO-B5：实现 50Hz 控制循环（TIM 中断里调用，或主循环轮询标志）。
 *   顺序很重要，按这个顺序：
 *     1. read_encoders()                 读新编码器增量
 *     2. 算 vel_l/vel_r                  用 enc_delta_to_mms
 *     3. set_motor_pwm(...)              把 target 下发给电机
 *     4. 检查失联：now - last_cmd_tick > CMD_TIMEOUT_MS ?
 *          是 → target 强制清零（★ 清 target，不是直接 PWM=0，恢复连接时行为一致）
 *     5. 组装上行帧并发出
 *   ★ 想一下：为什么"检查失联"要放在"下发 PWM"【之后】而不是之前？
 *     这体现的是"状态更新 vs 状态使用"的边界，是实时控制代码的核心能力。 */
void control_loop_50hz(void)
{
    static int32_t previous_enc_l = 0;
    static int32_t previous_enc_r = 0;
    int32_t delta_l;
    int32_t delta_r;
    uint32_t now;

    read_encoders();

    delta_l = g_chassis.enc_l - previous_enc_l;
    delta_r = g_chassis.enc_r - previous_enc_r;
    previous_enc_l = g_chassis.enc_l;
    previous_enc_r = g_chassis.enc_r;

    g_chassis.vel_l = enc_delta_to_mms(delta_l);
    g_chassis.vel_r = enc_delta_to_mms(delta_r);

    set_motor_pwm(g_chassis.target_l_mms, g_chassis.target_r_mms);

    now = HAL_GetTick();
    if (now - g_chassis.last_cmd_tick > CMD_TIMEOUT_MS)
    {
        g_chassis.target_l_mms = 0;
        g_chassis.target_r_mms = 0;
    }

    send_odom_frame();
}

/* ★ TODO-B6：实现上行帧组装与发送。
 *   格式：O,<seq>,<enc_l>,<enc_r>,<vel_l>,<vel_r>,<pwm_l>,<pwm_r>,<vbat_mv>,<tick_ms>,<temp_c100>\n
 *   三个必须注意的点：
 *     【1】★★ main 初始化里必须 setvbuf(stdout, NULL, _IONBF, 0)。
 *          nano.specs 下 stdout 接 UART 不是 tty，走全缓冲（约 1024B），
 *          约 20 帧才 flush，50Hz 下延迟明显（adc_serial_nocube 实测踩过：17s 才吐）。
 *          判据：ros2 topic hz /odom 从启动到出数据延迟 < 1s。
 *     【2】不要用 %f。整数格式说明符（%d/%u/%ld）才可用——和「速度用 mm/s 整数」呼应。
 *     【3】发送别阻塞太久。50 字节 @115200 ≈ 4.3ms，占 20ms 周期的 21%。
 *          选择：(a) DMA/IT 异步（帧缓冲要双缓冲）(b) 接受 4.3ms 阻塞，放周期末尾。
 *          判据：能说出 (a) 的额外复杂度在哪（双缓冲的写入完成回调时序）。 */
void send_odom_frame(void)
{
    uint32_t tick_ms = HAL_GetTick();

    g_chassis.seq_out++;

    printf("O,%lu,%ld,%ld,%ld,%ld,%d,%d,%u,%lu,%d\n",
           (unsigned long)g_chassis.seq_out,
           (long)g_chassis.enc_l,
           (long)g_chassis.enc_r,
           (long)g_chassis.vel_l,
           (long)g_chassis.vel_r,
           (int)g_chassis.pwm_l,
           (int)g_chassis.pwm_r,
           (unsigned int)g_chassis.vbat_mv,
           (unsigned long)tick_ms,
           (int)g_chassis.temp_c100);
}

/* ============================================================================
 * 四、外设初始化 —— 胶水层，已写死，可直接用（CubeMX 也会生成等价代码）
 * ==========================================================================*/

static void SystemClock_Config(void);   /* 被 main 调用，保持 static */
void MX_GPIO_Init(void);                /* 以下 6 个待 TODO-C1 调用，暂不加 static 避免 unused 警告 */
void MX_TIM2_Encoder_Init(void);
void MX_TIM3_Encoder_Init(void);
void MX_TIM4_PWM_Init(void);
void MX_ADC1_Init(void);
void MX_USART1_UART_Init(void);

/* ============================================================================
 * 五、main
 * ==========================================================================*/
int main(void)
{
    uint32_t last_control_tick;

    HAL_Init();
    SystemClock_Config();          /* 时钟必须先：HAL 的时间基准、外设都依赖它 */

    /* ========================================================================
     * ★ TODO-C1：初始化顺序。下面这些初始化函数已写好（胶水），
     *            但【按什么顺序调用】是你的题。
     *
     *   参考理由（每条都要能说清"为什么是这个顺序"）：
     *     1. 串口最先        ← 尽早能收 PC 的停车指令，尽早 setvbuf
     *     2. setvbuf(_IONBF) ← 见 TODO-B6【1】
     *     3. 电机 PWM        ← 初始化后立刻把占空比置 0，上电不狂转（安全优先）
     *     4. 编码器接口      ← 在 PWM 之后，避免编码器初始化的信号被当成目标速度
     *     5. 定时器/时基     ← 控制循环的节拍
     *     6. ADC 电池电压
     *     7. g_chassis 清零，last_cmd_tick 设成当前 tick
     *      last_cmd_tick：最后一次收到 PC 运动指令的时间戳          
     *   ★ 判据 C3：上电瞬间电机不能有任何转动。若上电狂转，检查 3 和 4 的顺序。
     *   ========================================================================*/
    /* TODO-C1：把上面的初始化函数按正确顺序调用 */
    MX_USART1_UART_Init();
    setvbuf(stdout, NULL, _IONBF, 0);
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1U) != HAL_OK)
    {
        Error_Handler();
    }
    MX_GPIO_Init();     
    MX_TIM4_PWM_Init();   
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK
        || HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    MX_TIM2_Encoder_Init();
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    MX_TIM3_Encoder_Init();
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  
    MX_ADC1_Init();
    memset(&g_chassis, 0, sizeof(g_chassis));
    g_chassis.last_cmd_tick = HAL_GetTick() - CMD_TIMEOUT_MS - 1;
    last_control_tick = HAL_GetTick();

    printf("navbot_chassis boot");


    /* ========================================================================
     * ★ TODO-C2：主循环架构。两种选一种并说出理由：
     *   (甲) 纯中断驱动：50Hz 全部工作在 TIM 中断里做完，主循环为空/只做低压采样
     *   (乙) 中断只置标志，主循环做实际工作
     *   判断依据：control_loop_50hz 里有没有"耗时不定"的操作（浮点/printf）？
     *   有则甲方案会拖长中断、影响串口接收，应选乙。
     *   ★ 判据 C4：能说出你选的那种方案在什么情况下会出问题。
     *   ========================================================================*/
    while (1)
    {
        char completed_line[RX_LINE_BUFFER_SIZE];
        uint16_t completed_length;
        uint32_t interrupt_state = __get_PRIMASK();

        __disable_irq();
        if (rx_line_ready)
        {
            completed_length = rx_line_length;
            memcpy(completed_line, rx_line, (size_t)completed_length + 1U);
            rx_line_ready = 0U;
            rx_line_length = 0U;
        }
        else
        {
            completed_length = 0U;
        }
        __set_PRIMASK(interrupt_state);

        if (completed_length > 0U)
        {
            parse_line(completed_line);
        }

        uint32_t now = HAL_GetTick();
        if (now - last_control_tick >= CTRL_PERIOD_MS)
        {
            last_control_tick = now;
            control_loop_50hz();
        }
    }
}

/* ============================================================================
 * 系统时钟：HSE 8MHz × PLL9 = 72MHz，失败自动退回 HSI（沿用 adc_serial_nocube）
 * ==========================================================================*/
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* HSE 起不来 → 退 HSI 8MHz/2 ×16 = 64MHz，串口波特率由 HAL 重算，照样准 */
        g_clk_fallback = 1;
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI;
        RCC_OscInitStruct.HSEState       = RCC_HSE_OFF;
        RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
        RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI_DIV2;
        RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL16;
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        {
            Error_Handler();
        }
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 * GPIO：PC13 LED（低电平点亮）+ PA4/PA5 电机 DIR
 * ==========================================================================*/
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* LED：先写高 = 灭，避免上电闪一下 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* 电机 DIR：上电先给"停"（低电平），配合 PWM=0 保证不转 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ============================================================================
 * TIM2 编码器接口（左轮）：PA0/PA1，TI12 四倍频，CNT 自由回绕
 * ==========================================================================*/
void MX_TIM2_Encoder_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler         = 0;                            /* 编码器不预分频 */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFF;                       /* 16 位满量程 */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;                /* 两相都计数 = 4 倍频 */
    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 0;
    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 0;

    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 * TIM3 编码器接口（右轮）：PA6/PA7，配置同 TIM2
 * ==========================================================================*/
void MX_TIM3_Encoder_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 0xFFFF;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode   = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC1Filter     = 0;
    sConfig.IC2Polarity   = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler  = TIM_ICPSC_DIV1;
    sConfig.IC2Filter     = 0;

    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 * TIM4 PWM（左右轮）：PB6/PB7
 *   时钟 = 72MHz，Prescaler=7 → 9MHz 计数，ARR=999 → PWM 频率 9kHz
 *   初始占空比 0（Pulse=0），配合 DIR=停，上电不转
 * ==========================================================================*/
void MX_TIM4_PWM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler         = 7;                            /* 72MHz / 8 = 9MHz */
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 999;                          /* ARR=999 → 9kHz */
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;                                    /* 初始占空比 0 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 * ADC1 电池电压：PA2 = ADC1_IN2（分压后），轮询单次转换
 * ==========================================================================*/
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel      = ADC_CHANNEL_2;                        /* PA2 = IN2 */
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 * USART1：PA9/PA10 · 115200 8N1（下行接收的中断启动见 TODO-B3）
 * ==========================================================================*/
void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ============================================================================
 * 出错兜底：关中断死循环，方便挂调试器看现场
 * ==========================================================================*/
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    while (1) { }
}
#endif
