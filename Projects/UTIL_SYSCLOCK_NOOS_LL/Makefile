######################################
# Makefile by CubeMX2Makefile.py
######################################

######################################
# target
######################################
TARGET = STM32F3

######################################
#JTAG and environment configuration
######################################
GCC_BIN           ?= /usr/bin/
GCC_PREFIX        ?= arm-none-eabi

OPENOCD           ?= openocd
OPENOCD_INTERFACE ?= interface/stlink-v2-1.cfg
OPENOCD_CMDS      ?=
OPENOCD_TARGET    ?= target/stm32f3x.cfg

JLINK             ?= JLinkExe
JLINK_DEVICE      ?= STM32F302R8
JLINK_SPEED       ?= 1000
JLINK_INTERFACE   ?= SWD

######################################
#Choose Debugger
#-SEGGER_JLINK enables Systemview
#- Else default = OPENOCD
######################################
DEBUGGER          ?= SEGGER_JLINK


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O0

#######################################
# pathes
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
C_SOURCES = \
  Src/main.c \
  Src/system_stm32f3xx.c \
  Src/stm32f3xx_it.c \
  ../../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_rcc.c \
  ../../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_gpio.c \
  ../../Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_ll_utils.c \
 
ASM_SOURCES = \
  startup_stm32f302x8.s

#######################################
# binaries
#######################################
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
AR = arm-none-eabi-ar
SZ = arm-none-eabi-size
OBJDUMP = $(GCC_BIN)/$(GCC_PREFIX)-objdump
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# macros for gcc
AS_DEFS = 
C_DEFS = -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DSTM32F302x8 -DARM_MATH_CM4 -D__FPU_PRESENT
# includes for gcc
AS_INCLUDES =
C_INCLUDES = -I Inc
C_INCLUDES += -I../../Drivers/CMSIS/Device/ST/STM32F3xx/Include
C_INCLUDES += -I../../Drivers/CMSIS/Include
C_INCLUDES += -I../../Drivers/STM32F3xx_HAL_Driver/Inc
C_INCLUDES += -I../../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy
C_INCLUDES += -I../../../SystemView/Config
C_INCLUDES += -I../../../SystemView/SEGGER
C_INCLUDES += -I../../../SystemView/Sample/Config

# compile gcc flags
ASFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif
# Generate dependency information
CFLAGS += -MD -MP -MF .dep/$(@F).d

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F302R8Tx_FLASH.ld
# libraries
LIBS = -lc -lm -lnosys
LIBDIR = 
LDFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).lss

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $(OBJECTS)
	$(SZ) $@
	
$(BUILD_DIR)/%.lss: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(OBJDUMP) -S $< > $@
	
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir -p $@		

#######################################
# Flash the stm.

ifeq ($(DEBUGGER),SEGGER_JLINK)

flash:
	JLinkExe -if $(JLINK_INTERFACE) -device $(JLINK_DEVICE) -speed $(JLINK_SPEED) -CommanderScript .jlink/flash.jlink

reset:
	JLinkExe -if $(JLINK_INTERFACE) -device $(JLINK_DEVICE) -speed $(JLINK_SPEED) -CommanderScript .jlink/reset.jlink

jlink:
	JLinkExe -if $(JLINK_INTERFACE) -device $(JLINK_DEVICE) -speed $(JLINK_SPEED)

else

flash:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset halt" \
                 -c "flash write_image erase $(BUILD_DIR)/$(TARGET).elf" -c "verify_image $(BUILD_DIR)/$(TARGET).elf" -c "reset run" -c shutdown

halt:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c shutdown

reset:
	$(OPENOCD) -d0 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "reset" -c shutdown

openocd:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) $(OPENOCD_CMDS) -f $(OPENOCD_TARGET) -c init -c targets -c "\$_TARGETNAME configure -rtos auto"


gdb: $(BUILD_DIR)/$(TARGET).elf
	$(GDB) -ex "target remote localhost:3333" -ex "monitor reset halt"

erase:
	$(OPENOCD) -d2 -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) -c init -c targets -c "halt" -c "stm32f1x mass_erase 0" -c shutdown

endif

#######################################

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

.PHONY: clean all

# *** EOF ***
