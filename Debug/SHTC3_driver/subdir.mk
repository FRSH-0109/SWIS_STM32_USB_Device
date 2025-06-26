################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SHTC3_driver/shtc3_driver.c 

OBJS += \
./SHTC3_driver/shtc3_driver.o 

C_DEPS += \
./SHTC3_driver/shtc3_driver.d 


# Each subdirectory must supply rules for building sources it contributes
SHTC3_driver/%.o SHTC3_driver/%.su SHTC3_driver/%.cyclo: ../SHTC3_driver/%.c SHTC3_driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc -I"D:/dev/SWIS/SWIS_USB_device/SHTC3_driver" -I"D:/dev/SWIS/SWIS_USB_device/BME280_driver" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-SHTC3_driver

clean-SHTC3_driver:
	-$(RM) ./SHTC3_driver/shtc3_driver.cyclo ./SHTC3_driver/shtc3_driver.d ./SHTC3_driver/shtc3_driver.o ./SHTC3_driver/shtc3_driver.su

.PHONY: clean-SHTC3_driver

