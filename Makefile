# UIS Subsea sensorkode makefile
# 
# Author: Håvard Syslak
# Date: 15.03.23
#

CC := arm-none-eabi-gcc
LD := arm-none-eabi-gcc

OBJCOPY := arem-none-eabi-objcopy

TARGET := bin/sensor.elf
MCU := STM32G431KB
MCU_FAM := STM32G431xx
SYSCLK_FREQ = 170000000
CMSIS_DIR := ./Drivers/CMSIS
HAL_DIR := ./Drivers/STM32G4xx_HAL_Driver

LD_SCRIPT := ./STM32G431KBTX_FLASH.ld

SRCDIR := ./Core/Src
HDRDIR := ./Core/Inc
OBJDIR := ./obj

# ./Core/Startup/startup_stm32g431kbtx.s \
# $(OBJDIR)/startup_stm32g431kbtx.o \

SRCS := $(shell find $(SRCDIR) -name "*.c") \
		$(HAL_DIR)/Src/stm32g4xx_hal.c \
		$(HAL_DIR)/Src/stm32g4xx_hal_gpio.c \
		$(HAL_DIR)/Src/stm32g4xx_hal_rcc.c \
		$(HAL_DIR)/Src/stm32g4xx_hal_i2c.c \
		$(HAL_DIR)/Src/stm32g4xx_hal_cordic.c \
		$(HAL_DIR)/Src/stm32g4xx_hal_spi.c \
		$(HAL_DIR)/Src/stm32g4xx_hal_tim.c \
		$(HAL_DIR)/Src/stm32g4xx_hal_fdcan.c \
		$(HAL_DIR)/Src/stm32g4xx_hal_uart.c \

HDRS := $(shell find $(SRCDIR) -name "*.h") $(shell find $(HDRDIR) -name "*.h")

OBJS := $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o,$(filter %.c,$(SRCS))) \
       $(OBJDIR)/stm32g4xx_hal.o \
       $(OBJDIR)/stm32g4xx_hal_gpio.o \
       $(OBJDIR)/stm32g4xx_hal_rcc.o \
       $(OBJDIR)/stm32g4xx_hal_i2c.o \
       $(OBJDIR)/stm32g4xx_hal_cordic.o \
       $(OBJDIR)/stm32g4xx_hal_spi.o \
       $(OBJDIR)/stm32g4xx_hal_tim.o \
       $(OBJDIR)/stm32g4xx_hal_fdcan.o \
       $(OBJDIR)/stm32g4xx_hal_uart.o


DEFINES := 	-DUSE_HAL_DRIVER \
		   	-D$(MCU_FAM) \
			-D$(MCU_FAM)

INC_DIRS = -I./Drivers/STM32G4xx_HAL_Driver/Inc \
		   -I./Drivers/CMSIS/Device/ST/STM32G4xx/Include \
		   -I./Drivers/CMSIS/Include \
		   -I./Drivers/STM32G4xx_HAL_Driver/Inc/Legacy \
		   -I./Core/Inc \

CFLAGS := 	-mthumb \
			-mcpu=cortex-m4 \
			-mfpu=fpv4-sp-d16 \
			-mfloat-abi=hard \
			-O0 \
			-g \
			-Wall \
			-c \
			$(DEFINES) \
			$(INC_DIRS)


LDFLAGS := 	-mthumb \
         	-mcpu=cortex-m4 \
			-mfpu=fpv4-sp-d16 \
			-mfloat-abi=hard \
			-specs=nosys.specs \
			-oformat=elf32-littlearm \
			-T$(LD_SCRIPT)


vpath %.c $(sort $(dir $(C_SOURCES)))

# $(TARGET): $(OBJS)
# 	@mkdir -p bin
# 	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^


$(OBJDIR)/stm32g4xx_hal.o: $(HAL_DIR)/Src/stm32g4xx_hal.c $(HDRS)
	@mkdir -p $(shell dirname $@)
	$(CC) $(CFLAGS) -o $@ $<

$(OBJDIR)/stm32g4xx_hal_gpio.o: $(HAL_DIR)/Src/stm32g4xx_hal_gpio.c $(HDRS)
	@mkdir -p $(shell dirname $@)
	$(CC) $(CFLAGS) -o $@ $<

$(OBJDIR)/stm32g4xx_hal_rcc.o: $(HAL_DIR)/Src/stm32g4xx_hal_rcc.c $(HDRS)
	@mkdir -p $(shell dirname $@)
	$(CC) $(CFLAGS) -o $@ $<

$(OBJDIR)/stm32g4xx_hal_i2c.o: $(HAL_DIR)/Src/stm32g4xx_hal_i2c.c $(HDRS)
	@mkdir -p $(shell dirname $@)
	$(CC) $(CFLAGS) -o $@ $<

$(OBJDIR)/stm32g4xx_hal_cordic.o: $(HAL_DIR)/Src/stm32g4xx_hal_cordic.c $(HDRS)
	@mkdir -p $(shell dirname $@)
	$(CC) $(CFLAGS) -o $@ $<

$(OBJDIR)/stm32g4xx_hal_tim.o: $(HAL_DIR)/Src/stm32g4xx_hal_tim.c $(HDRS)
	@mkdir -p $(shell dirname $@)
	$(CC) $(CFLAGS) -o $@ $<

$(OBJDIR)/stm32g4xx_hal_spi.o: $(HAL_DIR)/Src/stm32g4xx_hal_spi.c $(HDRS)
	@mkdir -p $(shell dirname $@)
	$(CC) $(CFLAGS) -o $@ $<

$(OBJDIR)/stm32g4xx_hal_fdcan.o: $(HAL_DIR)/Src/stm32g4xx_hal_fdcan.c $(HDRS)
	@mkdir -p $(shell dirname $@)
	$(CC) $(CFLAGS) -o $@ $<

$(OBJDIR)/stm32g4xx_hal_uart.o: $(HAL_DIR)/Src/stm32g4xx_hal_uart.c $(HDRS)
	@mkdir -p $(shell dirname $@)
	$(CC) $(CFLAGS) -o $@ $<


$(OBJDIR)/%.o: $(SRCDIR)/%.c $(HDRS)
	@mkdir -p $(shell dirname $@)
	$(CC) $(CFLAGS) -o $@ $<



clean: 
	rm -f $(OBJDIR)/*.o

all: $(OBJS)
	$(LD) $(LDFLAGS) -o $(TARGET) $(OBJS)
