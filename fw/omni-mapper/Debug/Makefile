blank:=
define newline

$(blank)
endef

STM32_PROJECT_ROOT_DIR = ..

STM32_SRC_DIR = $(STM32_PROJECT_ROOT_DIR)/Core/Src
STM32_INCLUDE_DIR = $(STM32_PROJECT_ROOT_DIR)/Core/Inc

# STM32 HAL and CMSIS Compilation
STM32_DRIVERS_DIR = $(STM32_PROJECT_ROOT_DIR)/Drivers

STM32_HAL_DRIVERS_DIR = $(STM32_DRIVERS_DIR)/STM32F0xx_HAL_Driver
STM32_HAL_DRIVERS_INC_DIR = $(STM32_HAL_DRIVERS_DIR)/Inc
STM32_HAL_DRIVERS_INC_LEG_DIR = $(STM32_HAL_DRIVERS_DIR)/Legacy

STM32_CMSIS_DRIVERS_DIR = $(STM32_DRIVERS_DIR)/CMSIS
STM32_CMSIS_DRIVERS_INC_DIR = $(STM32_CMSIS_DRIVERS_DIR)/Include
STM32_CMSIS_DRIVERS_ST_INC_DIR = $(STM32_CMSIS_DRIVERS_DIR)/Device/ST/STM32F0xx/Include

STM32_HAL_DRIVERS_SRC_DIR = $(STM32_HAL_DRIVERS_DIR)/Src
STM32_HAL_DRIVERS_OBJ_DIR = Drivers/STM32F0xx_HAL_Driver/Src
STM32_HAL_DRIVERS_SRC = $(wildcard $(STM32_HAL_DRIVERS_SRC_DIR)/*.c)
STM32_HAL_DRIVERS_OBJ = $(patsubst $(STM32_HAL_DRIVERS_SRC_DIR)/%.c, $(STM32_HAL_DRIVERS_OBJ_DIR)/%.o, $(STM32_HAL_DRIVERS_SRC))

# Src file Compilation
STM32_SRC = $(wildcard $(STM32_SRC_DIR)/*.c)
STM32_OBJ_DIR = Core/Src
STM32_OBJ = $(patsubst $(STM32_SRC_DIR)/%.c, $(STM32_OBJ_DIR)/%.o, $(STM32_SRC))

# Startup File Compilation
STM32_STARTUP_DIR = $(STM32_PROJECT_ROOT_DIR)/Core/Startup
STM32_STARTUP_FILE = $(wildcard $(STM32_STARTUP_DIR)/*.s)
STM32_STARTUP_OBJ_DIR = Core/Startup
STM32_STARTUP_OBJ = $(patsubst $(STM32_STARTUP_DIR)/%.s, $(STM32_STARTUP_OBJ_DIR)/%.o, $(STM32_STARTUP_FILE))

# Compiler Flags
STM32_BOARD_BUILD = -mcpu=cortex-m0

STM32_COMP_FLAGS =  -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER \
										-DSTM32F030x8 \
										-O0 -ffunction-sections -fdata-sections -Wall -fstack-usage \
										--specs=nano.specs \
										-mfloat-abi=soft -mthumb 

STM32_INCLUDE_FLAGS = -I$(STM32_INCLUDE_DIR) \
										  -I$(STM32_HAL_DRIVERS_INC_DIR) \
										  -I$(STM32_HAL_DRIVERS_INC_LEG_DIR) \
										  -I$(STM32_CMSIS_DRIVERS_INC_DIR) \
										  -I$(STM32_CMSIS_DRIVERS_ST_INC_DIR)

STM32_STARTUP_FLAGS = -g3 -DDEBUG -c -x assembler-with-cpp --specs=nano.specs \
											-mfloat-abi=soft -mthumb

# Linking
TARGET = omni-mapper
STM32_LINKER_ELF = $(TARGET).elf
STM32_LINKER_OBJECTS_LIST = objects.list
STM32_LINKER_FLAGS = -T"$(STM32_PROJECT_ROOT_DIR)/STM32F030R8TX_FLASH.ld" \
										--specs=nosys.specs \
										-Wl,-Map="$(TARGET).map" \
										-Wl,--gc-sections \
										-static \
										--specs=nano.specs \
										-mfloat-abi=soft \
										-mthumb \
										-Wl,--start-group \
										-lc \
										-lm \
										-Wl,--end-group

# Toolchain
CC = arm-none-eabi-gcc

all: $(STM32_LINKER_ELF)  

$(info Building STM32 Hal Drivers...$(newline))
$(STM32_HAL_DRIVERS_OBJ_DIR)/%.o: $(STM32_HAL_DRIVERS_SRC_DIR)/%.c | $(STM32_HAL_DRIVERS_OBJ_DIR)
	$(CC) $(STM32_BOARD_BUILD) $(STM32_INCLUDE_FLAGS) $(STM32_COMP_FLAGS) -c $< -o $@

$(STM32_HAL_DRIVERS_OBJ_DIR):
	mkdir -p $(STM32_HAL_DRIVERS_OBJ_DIR)

$(info Building STM32 Src Files...$(newline))
$(STM32_OBJ_DIR)/%.o: $(STM32_SRC_DIR)/%.c | $(STM32_OBJ_DIR)
	$(CC) $(STM32_BOARD_BUILD) $(STM32_INCLUDE_FLAGS) $(STM32_COMP_FLAGS) -c $< -o $@

$(STM32_OBJ_DIR):
	mkdir -p $(STM32_OBJ_DIR)

$(info Building STM32 Startup File...$(newline))
$(STM32_STARTUP_OBJ_DIR)/%.o: $(STM32_STARTUP_DIR)/%.s | $(STM32_STARTUP_OBJ_DIR)
	$(CC) $(STM32_BOARD_BUILD) $(STM32_STARTUP_FLAGS) -c $< -o $@

$(STM32_STARTUP_OBJ_DIR):
	mkdir -p $(STM32_STARTUP_OBJ_DIR)


$(STM32_LINKER_ELF): $(STM32_HAL_DRIVERS_OBJ) $(STM32_OBJ) $(STM32_STARTUP_OBJ)
	$(CC) -o $(STM32_LINKER_ELF) @"$(STM32_LINKER_OBJECTS_LIST)" $(STM32_BOARD_BUILD) $(STM32_LINKER_FLAGS)

clean:
	rm -f $(STM32_LINKER_ELF) $(STM32_OBJ_DIR)/*.o $(STM32_HAL_DRIVERS_OBJ_DIR)/*.o $(STM32_STARTUP_OBJ_DIR)/*.o

flash: omni-mapper.bin
	st-flash write omni-mapper.bin 0x08000000 && st-flash reset

omni-mapper.bin: omni-mapper.elf
	arm-none-eabi-objcopy -O binary omni-mapper.elf omni-mapper.bin


