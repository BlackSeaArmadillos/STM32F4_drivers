################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f407xx_gpio.c \
../drivers/Src/stm32f407xx_spi.c 

OBJS += \
./drivers/Src/stm32f407xx_gpio.o \
./drivers/Src/stm32f407xx_spi.o 

C_DEPS += \
./drivers/Src/stm32f407xx_gpio.d \
./drivers/Src/stm32f407xx_spi.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/Projects si ckt-uri/STM32 Projects/Mastering MCUs Course/workspace/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f407xx_gpio.d ./drivers/Src/stm32f407xx_gpio.o ./drivers/Src/stm32f407xx_gpio.su ./drivers/Src/stm32f407xx_spi.d ./drivers/Src/stm32f407xx_spi.o ./drivers/Src/stm32f407xx_spi.su

.PHONY: clean-drivers-2f-Src

