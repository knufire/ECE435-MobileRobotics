################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
C:\Users\Rahul\Documents\Arduino\libraries\TimerOne\TimerOne.cpp 

LINK_OBJ += \
.\libraries\TimerOne\TimerOne.cpp.o 

CPP_DEPS += \
.\libraries\TimerOne\TimerOne.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
libraries\TimerOne\TimerOne.cpp.o: C:\Users\Rahul\Documents\Arduino\libraries\TimerOne\TimerOne.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:\sloeber\/arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10802 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"C:\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.20\cores\arduino" -I"C:\sloeber\arduinoPlugin\packages\arduino\hardware\avr\1.6.20\variants\mega" -I"C:\Users\Rahul\Documents\Arduino\libraries\AccelStepper" -I"C:\Users\Rahul\Documents\Arduino\libraries\AccelStepper\src" -I"C:\Users\Rahul\Documents\Arduino\libraries\TimerOne" -I"C:\Users\Rahul\Documents\Arduino\libraries\NewPing" -I"C:\Users\Rahul\Documents\Arduino\libraries\NewPing\src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


