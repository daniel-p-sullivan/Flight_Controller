################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/comms/communications.cpp \
../Core/Src/comms/nrf24.cpp 

OBJS += \
./Core/Src/comms/communications.o \
./Core/Src/comms/nrf24.o 

CPP_DEPS += \
./Core/Src/comms/communications.d \
./Core/Src/comms/nrf24.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/comms/%.o Core/Src/comms/%.su: ../Core/Src/comms/%.cpp Core/Src/comms/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I"C:/Users/danie/Documents/My Code/repos/eigen-3.4.0/Eigen" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-comms

clean-Core-2f-Src-2f-comms:
	-$(RM) ./Core/Src/comms/communications.d ./Core/Src/comms/communications.o ./Core/Src/comms/communications.su ./Core/Src/comms/nrf24.d ./Core/Src/comms/nrf24.o ./Core/Src/comms/nrf24.su

.PHONY: clean-Core-2f-Src-2f-comms

