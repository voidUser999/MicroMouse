################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include TOF/Target/subdir.mk
-include TOF/App/subdir.mk
-include Drivers/STM32G0xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/STM32G0xx_Nucleo/subdir.mk
-include Drivers/BSP/Components/vl53l5cx/porting/subdir.mk
-include Drivers/BSP/Components/vl53l5cx/modules/subdir.mk
-include Drivers/BSP/Components/vl53l5cx/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := STSYS-NS101 Jan2024
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STSYS-NS101\ Jan2024.elf \

MAP_FILES += \
STSYS-NS101\ Jan2024.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STSYS-NS101\ Jan2024.list \


# All Target
all: main-build

# Main-build Target
main-build: STSYS-NS101\ Jan2024.elf secondary-outputs

# Tool invocations
STSYS-NS101\ Jan2024.elf STSYS-NS101\ Jan2024.map: $(OBJS) $(USER_OBJS) C:\Users\jagan\OneDrive\Documents\STSYS-NS101-IR-COMPLETE\STSYS-NS101\STM32G070RBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "STSYS-NS101 Jan2024.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"C:\Users\jagan\OneDrive\Documents\STSYS-NS101-IR-COMPLETE\STSYS-NS101\STM32G070RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STSYS-NS101 Jan2024.map" -Wl,--gc-sections -static -u _printf_float --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STSYS-NS101\ Jan2024.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STSYS-NS101 Jan2024.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) STSYS-NS101\ Jan2024.elf STSYS-NS101\ Jan2024.list STSYS-NS101\ Jan2024.map STSYS-NS101\\ Jan2024.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
