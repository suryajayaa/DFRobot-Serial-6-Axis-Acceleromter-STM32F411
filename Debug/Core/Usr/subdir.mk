################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Usr/wt61pc.c 

OBJS += \
./Core/Usr/wt61pc.o 

C_DEPS += \
./Core/Usr/wt61pc.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Usr/%.o Core/Usr/%.su: ../Core/Usr/%.c Core/Usr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Surya/STM32CubeIDE/Line Follower/F411 Gyro DFRobot/Core/Usr" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Usr

clean-Core-2f-Usr:
	-$(RM) ./Core/Usr/wt61pc.d ./Core/Usr/wt61pc.o ./Core/Usr/wt61pc.su

.PHONY: clean-Core-2f-Usr

