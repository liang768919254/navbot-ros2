/*
 * STM32F103C8Tx_FLASH.ld
 * 蓝板 STM32F103C8T6：64KB Flash / 20KB RAM
 *
 * ⚠️ 与官方 STM32F103XB_FLASH.ld 的唯一区别就是容量：
 *    XB(128K) 的脚本用在 C8(64K) 上，链接器会放行超过 64K 的代码，
 *    烧录时才报错、或者更糟——烧进去运行到一半跑飞。所以必须用这一份。
 *
 * 同时去掉了 ST 官方脚本末尾的 /DISCARD/ { libc.a(*) ... } 段：
 * 那段是把标准库的调试信息丢掉来省体积，但和 newlib-nano 的 printf 一起用有风险，
 * 我们去掉它（代价只是 .elf 大一点），换来 printf 稳定可用。
 */

ENTRY(Reset_Handler)

/* 栈顶：20KB RAM 的最高地址 */
_estack = 0x20004FFF;

/* 堆/栈下限，链接时用来报错检查 */
_Min_Heap_Size  = 0x200;   /* 512B  */
_Min_Stack_Size = 0x400;   /* 1KB   */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS
{
  /* 中断向量表必须放在 Flash 最开头 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* 启动文件靠它把 .data 初值从 Flash 拷到 RAM */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
