################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Lib_LED7/Src/LED7.c 

OBJS += \
./Drivers/Lib_LED7/Src/LED7.o 

C_DEPS += \
./Drivers/Lib_LED7/Src/LED7.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Lib_LED7/Src/%.o Drivers/Lib_LED7/Src/%.su Drivers/Lib_LED7/Src/%.cyclo: ../Drivers/Lib_LED7/Src/%.c Drivers/Lib_LED7/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"C:/Users/DucAnh/Desktop/STM32_F1_HAL/LED7/Drivers/Lib_LED7/Inc" -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Lib_LED7-2f-Src

clean-Drivers-2f-Lib_LED7-2f-Src:
	-$(RM) ./Drivers/Lib_LED7/Src/LED7.cyclo ./Drivers/Lib_LED7/Src/LED7.d ./Drivers/Lib_LED7/Src/LED7.o ./Drivers/Lib_LED7/Src/LED7.su

.PHONY: clean-Drivers-2f-Lib_LED7-2f-Src

