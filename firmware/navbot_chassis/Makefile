# ============================================================================
# Makefile —— STM32F103C8T6 · ADC + USART1（免 CubeMX / 免 IDE）
#
# 用法：
#   make -j$(nproc)     编译，产物在 build/
#   make flash          用 st-flash 烧录（板子接着 ST-Link）
#   make size           看 Flash / RAM 占用
#   make clean
# ============================================================================

TARGET = navbot_chassis

# 调试版：-Og + 调试信息。-O2 会打乱单步跟踪，初学阶段别开。
DEBUG = 1
OPT   = -Og

BUILD_DIR = build

# ── 源文件 ─────────────────────────────────────────────────────────────────
C_SOURCES =  \
Core/Src/main.c \
Core/Src/stm32f1xx_it.c \
Core/Src/stm32f1xx_hal_msp.c \
Core/Src/syscalls.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c

ASM_SOURCES = \
Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s

# ── 工具链 ─────────────────────────────────────────────────────────────────
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# ── 目标芯片 ───────────────────────────────────────────────────────────────
CPU = -mcpu=cortex-m3
MCU = $(CPU) -mthumb

# STM32F103xB 是器件的「容量代号」：F103 的 x8/xB 共用同一套寄存器定义
C_DEFS = -DUSE_HAL_DRIVER -DSTM32F103xB

C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Core/Include

ASFLAGS = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# ── 链接 ───────────────────────────────────────────────────────────────────
LDSCRIPT = STM32F103C8Tx_FLASH.ld

# -lnosys 提供 syscalls 的弱符号桩，会被 Core/Src/syscalls.c 里的强符号覆盖
LIBS = -lc -lm -lnosys
LIBDIR =

LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# ── 目标 ───────────────────────────────────────────────────────────────────
OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $<

flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash --reset write $< 0x8000000

clean:
	-rm -rf $(BUILD_DIR)

.PHONY: all clean size flash 

-include $(wildcard $(BUILD_DIR)/*.d)
