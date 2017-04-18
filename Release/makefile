################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -f

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include libraries/Wire/utility/subdir.mk
-include libraries/Wire/subdir.mk
-include libraries/LSM6/subdir.mk
-include libraries/LIS3MDL/subdir.mk
-include libraries/Adafruit_LSM303DLHC/subdir.mk
-include libraries/Adafruit_L3GD20/subdir.mk
-include libraries/Adafruit_9DOF/pitchrollheading/subdir.mk
-include libraries/Adafruit_9DOF/ahrs/processing/cuberotate/subdir.mk
-include libraries/Adafruit_9DOF/ahrs/subdir.mk
-include libraries/Adafruit_9DOF/subdir.mk
-include core/core/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
/Users/farandhigh/Documents/sloeber_workspace/main/Release/arduino.ar \

ELF += \
main.elf \

HEX += \
main.hex \


# All Target
all: main

# Tool invocations
main: $(HEX)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"/Applications/Arduino.app/Contents/Java/hardware/teensy/../tools/arm/bin/arm-none-eabi-size" -A "/Users/farandhigh/Documents/sloeber_workspace/main/Release/main.elf"
	@echo 'Finished building target: $@'
	@echo ' '

/Users/farandhigh/Documents/sloeber_workspace/main/Release/arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"/Applications/Arduino.app/Contents/Java/hardware/teensy/../tools/arm/bin/arm-none-eabi-gcc-ar" rcs    /Users/farandhigh/Documents/sloeber_workspace/main/Release/arduino.ar $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

main.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"/Applications/Arduino.app/Contents/Java/hardware/teensy/../tools/arm/bin/arm-none-eabi-gcc" -O1 -Wl,--gc-sections,--relax,--defsym=__rtc_localtime=1489860128 "-T/Applications/Arduino.app/Contents/Java/hardware/teensy/avr/cores/teensy3/mk20dx256.ld" -mthumb -mcpu=cortex-m4 -fsingle-precision-constant -o "/Users/farandhigh/Documents/sloeber_workspace/main/Release/main.elf"   $(LINK_OBJ) $(AR)   /Users/farandhigh/Documents/sloeber_workspace/main/Release/arduino.ar  "-L/Users/farandhigh/Documents/sloeber_workspace/main/Release" -larm_cortexM4l_math -lm
	@echo 'Finished building: $@'
	@echo ' '

main.hex: $(ELF)
	@echo 'Create eeprom image'
	${A.RECIPE.OBJCOPY.EEP.PATTERN}
	${A.RECIPE.OBJCOPY.HEX.PATTERN}
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(PDE_DEPS)$(C_DEPS)$(AR)$(CC_DEPS)$(AR_OBJ)$(C++_DEPS)$(LINK_OBJ)$(CXX_DEPS)$(ASM_DEPS)$(HEX)$(INO_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(ELF)$(CPP_DEPS)
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
