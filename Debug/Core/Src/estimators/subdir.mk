################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/estimators/lowpassfilters.cpp 

OBJS += \
./Core/Src/estimators/lowpassfilters.o 

CPP_DEPS += \
./Core/Src/estimators/lowpassfilters.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/estimators/%.o Core/Src/estimators/%.su: ../Core/Src/estimators/%.cpp Core/Src/estimators/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I"C:/Users/danie/Documents/My Code/repos/eigen-3.4.0/Eigen" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-estimators

clean-Core-2f-Src-2f-estimators:
	-$(RM) ./Core/Src/estimators/lowpassfilters.d ./Core/Src/estimators/lowpassfilters.o ./Core/Src/estimators/lowpassfilters.su

.PHONY: clean-Core-2f-Src-2f-estimators

