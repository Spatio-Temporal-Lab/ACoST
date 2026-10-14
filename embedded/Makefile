######################################################################
# Makefile — STM32F103C8T6 Trajectory Compression Benchmark
# Toolchain: arm-none-eabi-gcc (brew install arm-none-eabi-gcc)
# Programmer: ST-Link V2 via OpenOCD (brew install open-ocd)
######################################################################

# Project
TARGET  = benchmark
BUILD   = build

# Toolchain (set ARM_GCC_PATH if not in PATH)
ARM_GCC_PATH ?= $(HOME)/arm-gcc/bin/
PREFIX  = $(ARM_GCC_PATH)arm-none-eabi-
CC      = $(PREFIX)gcc
CXX     = $(PREFIX)g++
AS      = $(PREFIX)gcc -x assembler-with-cpp
LD      = $(PREFIX)g++
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

# MCU flags
CPU     = -mcpu=cortex-m3
FPU     =
FLOAT   = -mfloat-abi=soft
MCU     = $(CPU) -mthumb $(FPU) $(FLOAT)

# Defines
DEFS    = -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000

# Include paths
INCS    = -I. \
          -IUSER \
          -ICORE \
          -ISource \
          -ISource/utils \
          -ISYSTEM/delay \
          -ISYSTEM/sys \
          -ISYSTEM/common \
          -ISTM32F10x_FWLib/inc

# ── Source files ───────────────────────────────────────────────────────

# Startup (GCC format)
ASM_SRCS = startup_stm32f103c8tx.s

# C sources (only what we need — skip unused peripherals to save Flash)
C_SRCS = \
  USER/main.c \
  USER/system_stm32f10x.c \
  USER/stm32f10x_it.c \
  SYSTEM/delay/delay.c \
  SYSTEM/sys/sys.c \
  Source/gps.c \
  Source/oled.c \
  STM32F10x_FWLib/src/misc.c \
  STM32F10x_FWLib/src/stm32f10x_gpio.c \
  STM32F10x_FWLib/src/stm32f10x_rcc.c \
  STM32F10x_FWLib/src/stm32f10x_usart.c \
  STM32F10x_FWLib/src/stm32f10x_flash.c \
  STM32F10x_FWLib/src/stm32f10x_dma.c \
  STM32F10x_FWLib/src/stm32f10x_tim.c

# C++ sources (all algorithms + benchmark)
CXX_SRCS = \
  Source/acost_embedded.cpp \
  Source/cost_embedded.cpp \
  Source/baselines_embedded.cpp \
  Source/trad_algorithms_embedded.cpp \
  Source/acost_benchmark.cpp

# Linker script
LDSCRIPT = STM32F103C8Tx_FLASH.ld

# ── Compiler/Linker flags ──────────────────────────────────────────────

# Optimization: -Os for size (Flash is only 64KB), or -O2 for speed
OPT = -Os

CFLAGS  = $(MCU) $(DEFS) $(INCS) $(OPT) \
          -Wall -fdata-sections -ffunction-sections \
          -std=c99

CXXFLAGS = $(MCU) $(DEFS) $(INCS) $(OPT) \
           -Wall -fdata-sections -ffunction-sections \
           -std=c++17 -fno-exceptions -fno-rtti -fno-threadsafe-statics

ASFLAGS = $(MCU) $(DEFS) -Wall -fdata-sections -ffunction-sections

LDFLAGS = $(MCU) -specs=nano.specs -specs=nosys.specs \
          -T$(LDSCRIPT) \
          -Wl,--gc-sections \
          -Wl,-Map=$(BUILD)/$(TARGET).map,--cref \
          -lm -lstdc++_nano -lc_nano -lnosys

# ── Object files ───────────────────────────────────────────────────────

C_OBJS   = $(addprefix $(BUILD)/, $(C_SRCS:.c=.o))
CXX_OBJS = $(addprefix $(BUILD)/, $(CXX_SRCS:.cpp=.o))
ASM_OBJS = $(addprefix $(BUILD)/, $(ASM_SRCS:.s=.o))
OBJS     = $(ASM_OBJS) $(C_OBJS) $(CXX_OBJS)

# ── Rules ──────────────────────────────────────────────────────────────

.PHONY: all clean flash size

all: $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex size

# Link
$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo "LINK $@"
	@$(LD) $(OBJS) $(LDFLAGS) -o $@

# Binary outputs
$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	@$(OBJCOPY) -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	@$(OBJCOPY) -O ihex $< $@

# Print size
size: $(BUILD)/$(TARGET).elf
	@echo ""
	@$(SIZE) $<

# Compile C
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "CC  $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# Compile C++
$(BUILD)/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo "CXX $<"
	@$(CXX) $(CXXFLAGS) -c $< -o $@

# Assemble
$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "AS  $<"
	@$(AS) $(ASFLAGS) -c $< -o $@

# Clean
clean:
	rm -rf $(BUILD)

# ── Flash via ST-Link + OpenOCD ────────────────────────────────────────
OPENOCD ?= $(HOME)/openocd/bin/openocd
OPENOCD_SCRIPTS ?= $(HOME)/openocd/openocd/scripts

flash: $(BUILD)/$(TARGET).bin
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) -f interface/stlink.cfg \
	        -f target/stm32f1x.cfg \
	        -c "program $(BUILD)/$(TARGET).bin 0x08000000 verify reset exit"

# ── Flash via st-flash (simpler alternative) ───────────────────────────

stflash: $(BUILD)/$(TARGET).bin
	st-flash write $(BUILD)/$(TARGET).bin 0x08000000
