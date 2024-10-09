STM32_PROJECT_ROOT_DIR = /home/kash/fw/omni-mapper/fw/omni-mapper
STM32_INCLUDE_DIR = $(STM32_PROJECT_ROOT_DIR)/Core/Inc
STM32_DRIVERS_DIR = $(STM32_PROJECT_ROOT_DIR)/Drivers
STM32_SRC_DIR = $(STM32_PROJECT_ROOT_DIR)/Core/Src

# Toolchain
CC = arm-none-eabi-gcc

# Files
TARGET = omni-mapper

# Flags
F_MCU = cortex-m0
F_WFLAGS = -Wall -Wextra -Wshadow
F_DEBUG = -g3
F_OPTIMIZATION = -O0

$(TARGET): $(STM32_SRC_DIR)/main.c
	$(CC) "$(STM32_SRC_DIR)/main.c" \
	-mcpu=$(F_MCU) -std=gnu11 \
	$(F_DEBUG) \
	-I$(STM32_INCLUDE_DIR) \
	-I$(STM32_DRIVERS_DIR)/STM32F0xx_HAL_Driver/Inc \
	-I$(STM32_DRIVERS_DIR)/STM32F0xx_HAL_Driver/Inc/Legacy  \
	-I$(STM32_DRIVERS_DIR)/CMSIS/Device/ST/STM32F0xx/Include  \
	-I$(STM32_DRIVERS_DIR)/CMSIS/Include  \
	-DDEBUG -DUSE_HAL_DRIVER -DSTM32F030x8 \
	$(F_OPTIMIZATION) \
	-ffunction-sections -fdata-sections \
	$(F_WFLAGS) \
	-fstack-usage  -MMD -MP \
	-MF"Core/Src/main.d" -MT"Core/Src/main.o" \
	--specs=nano.specs -mfloat-abi=soft -mthumb -o \
	"Core/Src/main.o" -c
