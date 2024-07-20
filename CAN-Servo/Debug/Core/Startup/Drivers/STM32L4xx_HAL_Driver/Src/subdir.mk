################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c 

OBJS += \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.o \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.o 

C_DEPS += \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.d \
./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/%.o Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/%.su Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/%.cyclo: ../Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/%.c Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L431xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Startup-2f-Drivers-2f-STM32L4xx_HAL_Driver-2f-Src

clean-Core-2f-Startup-2f-Drivers-2f-STM32L4xx_HAL_Driver-2f-Src:
	-$(RM) ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.su ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.cyclo ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.d ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.o ./Core/Startup/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.su

.PHONY: clean-Core-2f-Startup-2f-Drivers-2f-STM32L4xx_HAL_Driver-2f-Src

