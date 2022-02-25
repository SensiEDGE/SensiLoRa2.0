################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/gitProject/SensiLoRa2.0/Projects/B-L072Z-LRWAN1/Applications/LoRa/End_Node/STM32CubeIDE/startup_stm32l072xx.s 

OBJS += \
./Application/SW4STM32/startup_stm32l072xx.o 

S_DEPS += \
./Application/SW4STM32/startup_stm32l072xx.d 


# Each subdirectory must supply rules for building sources it contributes
Application/SW4STM32/startup_stm32l072xx.o: C:/gitProject/SensiLoRa2.0/Projects/B-L072Z-LRWAN1/Applications/LoRa/End_Node/STM32CubeIDE/startup_stm32l072xx.s Application/SW4STM32/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

