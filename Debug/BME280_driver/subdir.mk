################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BME280_driver/bme280_driver.c 

OBJS += \
./BME280_driver/bme280_driver.o 

C_DEPS += \
./BME280_driver/bme280_driver.d 


# Each subdirectory must supply rules for building sources it contributes
BME280_driver/%.o BME280_driver/%.su BME280_driver/%.cyclo: ../BME280_driver/%.c BME280_driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc -I"D:/dev/SWIS/SWIS_USB_device/SHTC3_driver" -I"D:/dev/SWIS/SWIS_USB_device/BME280_driver" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BME280_driver

clean-BME280_driver:
	-$(RM) ./BME280_driver/bme280_driver.cyclo ./BME280_driver/bme280_driver.d ./BME280_driver/bme280_driver.o ./BME280_driver/bme280_driver.su

.PHONY: clean-BME280_driver

