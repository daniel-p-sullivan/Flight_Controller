################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/threads/actuatorthread.cpp \
../Core/Src/threads/controllerthread.cpp \
../Core/Src/threads/estimatorthread.cpp \
../Core/Src/threads/initializerthread.cpp \
../Core/Src/threads/sensorthread.cpp 

OBJS += \
./Core/Src/threads/actuatorthread.o \
./Core/Src/threads/controllerthread.o \
./Core/Src/threads/estimatorthread.o \
./Core/Src/threads/initializerthread.o \
./Core/Src/threads/sensorthread.o 

CPP_DEPS += \
./Core/Src/threads/actuatorthread.d \
./Core/Src/threads/controllerthread.d \
./Core/Src/threads/estimatorthread.d \
./Core/Src/threads/initializerthread.d \
./Core/Src/threads/sensorthread.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/threads/%.o Core/Src/threads/%.su: ../Core/Src/threads/%.cpp Core/Src/threads/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -c -I../Core/Inc -I"C:/Users/danie/Documents/My Code/repos/eigen-3.4.0/Eigen" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-threads

clean-Core-2f-Src-2f-threads:
	-$(RM) ./Core/Src/threads/actuatorthread.d ./Core/Src/threads/actuatorthread.o ./Core/Src/threads/actuatorthread.su ./Core/Src/threads/controllerthread.d ./Core/Src/threads/controllerthread.o ./Core/Src/threads/controllerthread.su ./Core/Src/threads/estimatorthread.d ./Core/Src/threads/estimatorthread.o ./Core/Src/threads/estimatorthread.su ./Core/Src/threads/initializerthread.d ./Core/Src/threads/initializerthread.o ./Core/Src/threads/initializerthread.su ./Core/Src/threads/sensorthread.d ./Core/Src/threads/sensorthread.o ./Core/Src/threads/sensorthread.su

.PHONY: clean-Core-2f-Src-2f-threads

