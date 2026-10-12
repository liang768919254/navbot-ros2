/* ============================================================================
 * stm32f1xx_hal_msp.c —— 外设的「硬件底层」初始化
 *
 * CubeMX 会把这些代码生成到 stm32f1xx_hal_msp.c 里；这里手写等价内容。
 * 分界线（HAL 的设计约定）：
 *   - 外设「逻辑参数」（波特率、采样时间、对齐方式）→ MX_xxx_Init() 在 main.c
 *   - 外设「硬件依赖」（时钟使能、引脚复用、外设时钟源）→ 这里的 MspInit
 * ==========================================================================*/

#include "main.h"

/**
  * @brief  全局 MSP：所有外设 MspInit 之前先跑一次
  */
void HAL_MspInit(void)
{
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* ⚠️ 对应《Step3 操作手册》3.3 表格第 1 条：SYS → Debug = Serial Wire
   *
   * 下面这句就是「Serial Wire」模式的实际动作：关掉 JTAG-DP，保留 SW-DP。
   * 漏了它，PA13/PA14 可能被当成普通 GPIO 或 JTDI/NJTRST 用掉，
   * 后果是**一次烧录之后 ST-Link 就再也连不上**，只能拨 BOOT0 救砖。
   * CubeMX 里选 Serial Wire，生成的就是这一行。
   */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

/**
  * @brief  ADC1 底层初始化：时钟 + 时钟源 + 引脚
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* ── ADC 时钟 = PCLK2 / 6 ────────────────────────────────────────
     * F103 的硬上限是 14MHz。
     *   HCLK=72MHz → PCLK2=72MHz → /6 = 12MHz ✓
     *   HSI 兜底 HCLK=64MHz → PCLK2=64MHz → /6 = 10.67MHz ✓
     * 两种情况下都安全，所以这里可以写死 DIV6。
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* PA2 = ADC1_IN2（电池电压分压后），必须配成模拟输入（不带上拉/下拉） */
    GPIO_InitStruct.Pin  = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

/**
  * @brief  USART1 底层初始化：时钟 + 引脚
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA9 = USART1_TX，复用推挽输出 */
    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA10 = USART1_RX，输入。不使能上拉，悬空时也别乱收数据 */
    GPIO_InitStruct.Pin  = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
}

/**
  * @brief  定时器编码器模式底层初始化：时钟 + 编码器引脚
  *         TIM2 = 左轮编码器（PA0/PA1），TIM3 = 右轮编码器（PA6/PA7）
  *
  *         ★ F103 的编码器接口（输入捕获）引脚只需配成输入模式，
  *           不需要 AF 复用（AF 复用是给 PWM 输出那类推挽用的）。
  */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim->Instance == TIM2)
  {
    /* 左轮编码器：PA0 = TIM2_CH1(A相), PA1 = TIM2_CH2(B相) */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    /* ★ 编码器若是开漏输出（霍尔/光耦常见）要上拉抗干扰；
     *   若是推挽输出（带整形芯片的模块），上拉也无害。 */
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (htim->Instance == TIM3)
  {
    /* 右轮编码器：PA6 = TIM3_CH1(A相), PA7 = TIM3_CH2(B相) */
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin  = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

/**
  * @brief  定时器 PWM 底层初始化：时钟 + 电机 PWM 引脚
  *         TIM4 = 左右轮 PWM（PB6/PB7）
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim->Instance == TIM4)
  {
    /* 电机 PWM：PB6 = TIM4_CH1(左轮), PB7 = TIM4_CH2(右轮)，复用推挽输出 */
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}
