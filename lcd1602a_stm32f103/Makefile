# Copyright (c) 2015 Max Asaulov

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to 
# deal in the Software without restriction, including without limitation the 
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
# sell copies of the Software, and to permit persons to whom the Software is 
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in 
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
# IN THE SOFTWARE.

# Project name
NAME := lcd1602a_stm32f103C8T6

# Echo
NOECHO ?= @

# Base pas
SOURCE := .
BUILD := build

CORTEX_M := 3
family:=stm32f1xx
FAMILY:=STM32F1xx
SOC  := STM32F103xB
	
# USER_DEFINES
USER_DEFINES += -include Inc/config.h

# Files
VPATH += ../src
lib_objs := lcd1602a_stm32.o

objs := \
	$(lib_objs) \
	$(patsubst %.c,%.o,$(wildcard Src/*.c)) \
	Drivers/CMSIS/Device/ST/$(FAMILY)/Source/Templates/system_$(family).o \
	Drivers/$(FAMILY)_HAL_Driver/Src/$(family)_hal.o \
	Drivers/$(FAMILY)_HAL_Driver/Src/$(family)_hal_cortex.o \
	Drivers/$(FAMILY)_HAL_Driver/Src/$(family)_hal_rcc.o \
	Drivers/$(FAMILY)_HAL_Driver/Src/$(family)_hal_tim.o \
	Drivers/$(FAMILY)_HAL_Driver/Src/$(family)_hal_tim_ex.o \
	Drivers/$(FAMILY)_HAL_Driver/Src/$(family)_hal_gpio.o \
	Drivers/$(FAMILY)_HAL_Driver/Src/$(family)_hal_uart.o \
	Drivers/$(FAMILY)_HAL_Driver/Src/$(family)_hal_pwr.o \
	Drivers/$(FAMILY)_HAL_Driver/Src/$(family)_hal_dma.o \

IPATH +=-IInc/ -IDrivers/CMSIS/Include -IDrivers/$(FAMILY)_HAL_Driver/Inc -IDrivers/CMSIS/Device/ST/$(FAMILY)/Include -I../inc

# Toolchain config
TOOLCHAIN_PREFIX := arm-none-eabi-
CC := $(TOOLCHAIN_PREFIX)gcc
LD := $(TOOLCHAIN_PREFIX)ld
OBJCOPY := $(TOOLCHAIN_PREFIX)objcopy

# Core config 
ARCH := ARMv7E

CORE := CM$(CORTEX_M)
CPU  := CORTEX_M$(CORTEX_M)

# Start up files
SOC_LOVER = $(shell echo $(SOC) | tr A-Z a-z)
STARTUP := ./Drivers/CMSIS/Device/ST/$(FAMILY)/Source/Templates/gcc/startup_$(SOC_LOVER).o
objs += $(STARTUP)

# Linker script
LD_SCRIPT := ldscripts/gcc.ld

# Options

CONFIG_DEFINES += -D__thumb__ -D${ARCH} -D${CORE} -D${CPU} -D${SOC} \
                 -D__ARCH__=${ARCH} -D__CORE__=${CORE} -D__CPU__=${CPU} \
                 -D__SOC__=${SOC}

WARN_FLAGS = -Wall -Wextra -Wextra \
             -Wlogical-op -Werror=logical-op \
              
CFLAGS := -Os -mthumb --specs=nosys.specs --specs=nano.specs\
          -mcpu=cortex-m$(CORTEX_M) \
          -D__STARTUP_CLEAR_BSS -D__START=main \
          -ffunction-sections -fdata-sections \
          -std=c99 $(WARN_FLAGS) $(CONFIG_DEFINES) $(USER_DEFINES) $(IPATH) -g

ASFLAGS := $(CFLAGS)

LDFLAGS := --gc-sections,-T,$(LD_SCRIPT),--print-memory-usage

##########################################################################
LDFLAGS := -Map=$(BUILD)/$(NAME).map,$(LDFLAGS)
OBJS := $(addprefix $(BUILD)/, $(objs))
DIR_LIST := $(dir $(OBJS))

#Rules
all : dir $(BUILD)/$(NAME).bin Makefile ## Make all (default)
.PHONY : all

.PHONY: help
export
help:
	@cat $(MAKEFILE_LIST) | sed 's/$$(\([A-Za-z0-9_]*\))/$${\1}/g' | grep -e '^[a-zA-Z_\-\$$\(\)\.\%/\{\}\-]*: *.*## *' | envsubst | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

show_vars: ## Show all env variables
	$(NOECHO) $(foreach v, $(.VARIABLES), $(info $(v) = $($(v))))

print-%: ## Print variable
	$(NOECHO) $* = $($*)

.PHONY : dir
dir: ## Generate dir tree
	$(NOECHO) echo "Creating dir tree $< ..."
	$(NOECHO) mkdir -p $(DIR_LIST)

.PRECIOUS: $(BUILD)/%.i
$(BUILD)/%.i: %.c ## Generate preprocessed files from c source files
	$(NOECHO) echo "Compile $< ..."
	$(NOECHO) $(CC) $(CFLAGS) -E $< -o $@

$(BUILD)/%.o: $(BUILD)/%.i ## Generate object files from c source files
	$(NOECHO) echo "Compile $< ..."
	$(NOECHO) $(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.S ## Generate object files from S source files
	$(NOECHO) echo "Compile $< ..."
	$(NOECHO) $(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s ## Generate object files from s source files
	$(NOECHO) echo "Compile $< ..."
	$(NOECHO) $(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(NAME).elf: $(OBJS) ## Generate elf $(Name)
	$(NOECHO) echo "Link $@ ..."
	$(NOECHO) $(CC) $(CFLAGS) -Wl,$(LDFLAGS) -o $@ $^ 

$(BUILD)/$(NAME).bin: $(BUILD)/$(NAME).elf ## Strip down elf to bin
	$(NOECHO) echo "Striping $< ..."
	$(NOECHO) $(OBJCOPY) -O binary $< $@

.PHONY : cppcheck
cppcheck : ## run cppcheck
	find . -name \*.i | xargs cppcheck  --xml-version=2 --xml 2> $(BUILD)/cppcheck.result

.PHONY : clean
clean: ## Remove build artifacts
	$(NOECHO) echo "Clean up ..."
	$(NOECHO) rm -rf $(BUILD)
