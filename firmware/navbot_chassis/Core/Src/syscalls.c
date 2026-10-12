/* ============================================================================
 * syscalls.c —— printf 重定向到 UART1（GCC / newlib 版）
 *
 * 对应教程 3.1 的 `_write` 写法。这里就是它的「正式落点」：
 * CubeMX 工程里你把它贴在 main.c 的 USER CODE BEGIN 0；本工程没有 CubeMX，
 * 所以按 GCC 的习惯单独放一个文件。
 *
 * 为什么是 _write 而不是 fputc？
 *   这两条路径的分界点在「你用哪套 C 库」：
 *     Keil MDK / ARMCC  → 重写 fputc，并且必须勾 Use MicroLIB
 *     GCC / newlib      → printf → vfprintf → __sfvwrite → _write
 *   所以 GCC 侧只有重写 _write 才有用；写 fputc 编译器根本不会调用它
 *   （这也是「printf 一点输出都没有、编译还不报错」的经典原因）。
 *
 * Makefile 里有 -lnosys，它提供了一个「返回 -1 的弱符号 _write」。
 * 弱符号会被下面这个强符号覆盖，所以这段写了就生效。
 * ==========================================================================*/

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include "main.h"

extern UART_HandleTypeDef huart1;

/* 链接脚本里 PROVIDE 出来的符号 */
extern char _end;
extern char _estack;

/**
  * @brief  newlib 的底层写：把 printf 的字节流塞给 USART1
  *
  * @note   HAL_UART_Transmit 是**阻塞**发送，会一直等到最后一个字节进移位寄存器。
  *         10Hz × 每行几字节完全无所谓；但**绝对不要**把这套 printf 用到
  *         1kHz 以上的高频路径，也不要放到中断里（会拖垮系统）——
  *         那时候要换成「中断/DMA 发送 + 环形缓冲」。
  */
int _write(int fd, char *ptr, int len)
{
  (void)fd;
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

/* ── 下面几个是 newlib 要求的系统调用桩 ──────────────────────────────────
 * 不提供它们也能链接（-lnosys 兜底），但提供之后：
 *   ① _isatty/_fstat 认得出「stdout 是个字符设备」——虽然 nano 下不改变缓冲行为
 *      （真正管用的是 main.c 里那句 setvbuf），但语义正确、换个 libc 也对
 *   ② _sbrk 给了个正经的堆实现，万一某个库函数内部 malloc 也不会直接失败
 * ------------------------------------------------------------------------*/

int _isatty(int fd)
{
  (void)fd;
  return 1;                     /* 告诉库：这是个终端型输出 */
}

int _fstat(int fd, struct stat *st)
{
  (void)fd;
  st->st_mode = S_IFCHR;        /* 字符设备 */
  return 0;
}

int _close(int fd)
{
  (void)fd;
  return -1;
}

int _lseek(int fd, int ptr, int dir)
{
  (void)fd; (void)ptr; (void)dir;
  return 0;
}

int _read(int fd, char *ptr, int len)
{
  (void)fd; (void)ptr; (void)len;
  return 0;                     /* 本工程不需要从串口读 */
}

/* 最小的 bump 分配器：从 _end 往上长，撞到栈顶前 1KB 就报 ENOMEM */
void *_sbrk(ptrdiff_t incr)
{
  static char *heap_end = 0;
  char *prev;

  if (heap_end == 0)
  {
    heap_end = &_end;
  }
  if (heap_end + incr > (&_estack - 0x400))
  {
    errno = ENOMEM;
    return (void *)-1;
  }
  prev = heap_end;
  heap_end += incr;
  return prev;
}
