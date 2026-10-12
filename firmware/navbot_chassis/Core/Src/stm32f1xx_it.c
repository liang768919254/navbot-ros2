/* stm32f1xx_it.c —— Cortex-M3 异常与系统节拍
 *
 * 这个工程没有用中断做业务（ADC 轮询、UART 阻塞发送），
 * 唯一真正被用到的中断是 SysTick —— HAL_Delay / HAL_GetTick 靠它。
 */
#include "main.h"
#include "stm32f1xx_it.h"

extern UART_HandleTypeDef huart1;

/* ── Cortex-M3 内核异常 ───────────────────────────────── */

void NMI_Handler(void)
{
  while (1) { }
}

/**
  * @brief  HardFault：访问非法地址、除零、栈溢出都会落到这里。
  *         调试时在这里下断点，看 LR / PC 就能定位跑飞的指令。
  */
void HardFault_Handler(void)
{
  while (1) { }
}

void MemManage_Handler(void)
{
  while (1) { }
}

void BusFault_Handler(void)
{
  while (1) { }
}

void UsageFault_Handler(void)
{
  while (1) { }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

/* ── 系统节拍：1ms 一次，HAL 的时间基准 ───────────────── */
void SysTick_Handler(void)
{
  HAL_IncTick();
}
void USART1_IRQHandler(void)
{ 
  HAL_UART_IRQHandler(&huart1); 
}
