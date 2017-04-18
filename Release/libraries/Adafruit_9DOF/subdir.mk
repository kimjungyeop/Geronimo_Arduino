################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/Users/farandhigh/Documents/Arduino/libraries/Adafruit_9DOF/Adafruit_9DOF.cpp 

LINK_OBJ += \
./libraries/Adafruit_9DOF/Adafruit_9DOF.cpp.o 

CPP_DEPS += \
./libraries/Adafruit_9DOF/Adafruit_9DOF.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/Adafruit_9DOF/Adafruit_9DOF.cpp.o: /Users/farandhigh/Documents/Arduino/libraries/Adafruit_9DOF/Adafruit_9DOF.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/Applications/Arduino.app/Contents/Java/hardware/teensy/../tools/arm/bin/arm-none-eabi-g++" -c -O1 -g -Wall -ffunction-sections -fdata-sections -nostdlib -fno-exceptions -felide-constructors -std=gnu++0x -fno-rtti -mthumb -mcpu=cortex-m4 -fsingle-precision-constant -D__MK20DX256__ -DTEENSYDUINO=134 -DARDUINO=10609 -DF_CPU=72000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH  -I"/Applications/Arduino.app/Contents/Java/hardware/teensy/avr/cores/teensy3" -I"/Applications/Arduino.app/Contents/Java/hardware/teensy/avr/libraries/Wire" -I"/Applications/Arduino.app/Contents/Java/hardware/teensy/avr/libraries/Wire/utility" -I"/Users/farandhigh/Documents/Arduino/libraries/LIS3MDL" -I"/Users/farandhigh/Documents/Arduino/libraries/LSM6" -I"/Users/farandhigh/Documents/Arduino/libraries/Adafruit_9DOF" -I"/Users/farandhigh/Documents/Arduino/libraries/Adafruit_Sensor" -I"/Users/farandhigh/Documents/Arduino/libraries/Adafruit_L3GD20" -I"/Users/farandhigh/Documents/Arduino/libraries/Adafruit_LSM303DLHC" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


