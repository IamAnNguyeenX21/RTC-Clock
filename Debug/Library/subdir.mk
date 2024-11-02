################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library/Button.c \
../Library/DFPLAYER.c \
../Library/LiquidCrystal_I2C.c \
../Library/rtc.c 

OBJS += \
./Library/Button.o \
./Library/DFPLAYER.o \
./Library/LiquidCrystal_I2C.o \
./Library/rtc.o 

C_DEPS += \
./Library/Button.d \
./Library/DFPLAYER.d \
./Library/LiquidCrystal_I2C.d \
./Library/rtc.d 


# Each subdirectory must supply rules for building sources it contributes
Library/%.o Library/%.su Library/%.cyclo: ../Library/%.c Library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/Code projects/Stm32/RTC_clock/Library" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Library

clean-Library:
	-$(RM) ./Library/Button.cyclo ./Library/Button.d ./Library/Button.o ./Library/Button.su ./Library/DFPLAYER.cyclo ./Library/DFPLAYER.d ./Library/DFPLAYER.o ./Library/DFPLAYER.su ./Library/LiquidCrystal_I2C.cyclo ./Library/LiquidCrystal_I2C.d ./Library/LiquidCrystal_I2C.o ./Library/LiquidCrystal_I2C.su ./Library/rtc.cyclo ./Library/rtc.d ./Library/rtc.o ./Library/rtc.su

.PHONY: clean-Library

