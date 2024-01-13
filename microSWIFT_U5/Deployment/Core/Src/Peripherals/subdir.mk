################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Peripherals/battery.c \
../Core/Src/Peripherals/ct_sensor.c \
../Core/Src/Peripherals/gnss.c \
../Core/Src/Peripherals/imu.c \
../Core/Src/Peripherals/iridium.c \
../Core/Src/Peripherals/rf_switch.c \
../Core/Src/Peripherals/u_ubx_protocol.c 

OBJS += \
./Core/Src/Peripherals/battery.o \
./Core/Src/Peripherals/ct_sensor.o \
./Core/Src/Peripherals/gnss.o \
./Core/Src/Peripherals/imu.o \
./Core/Src/Peripherals/iridium.o \
./Core/Src/Peripherals/rf_switch.o \
./Core/Src/Peripherals/u_ubx_protocol.o 

C_DEPS += \
./Core/Src/Peripherals/battery.d \
./Core/Src/Peripherals/ct_sensor.d \
./Core/Src/Peripherals/gnss.d \
./Core/Src/Peripherals/imu.d \
./Core/Src/Peripherals/iridium.d \
./Core/Src/Peripherals/rf_switch.d \
./Core/Src/Peripherals/u_ubx_protocol.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Peripherals/%.o Core/Src/Peripherals/%.su Core/Src/Peripherals/%.cyclo: ../Core/Src/Peripherals/%.c Core/Src/Peripherals/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32U575xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Peripherals

clean-Core-2f-Src-2f-Peripherals:
	-$(RM) ./Core/Src/Peripherals/battery.cyclo ./Core/Src/Peripherals/battery.d ./Core/Src/Peripherals/battery.o ./Core/Src/Peripherals/battery.su ./Core/Src/Peripherals/ct_sensor.cyclo ./Core/Src/Peripherals/ct_sensor.d ./Core/Src/Peripherals/ct_sensor.o ./Core/Src/Peripherals/ct_sensor.su ./Core/Src/Peripherals/gnss.cyclo ./Core/Src/Peripherals/gnss.d ./Core/Src/Peripherals/gnss.o ./Core/Src/Peripherals/gnss.su ./Core/Src/Peripherals/imu.cyclo ./Core/Src/Peripherals/imu.d ./Core/Src/Peripherals/imu.o ./Core/Src/Peripherals/imu.su ./Core/Src/Peripherals/iridium.cyclo ./Core/Src/Peripherals/iridium.d ./Core/Src/Peripherals/iridium.o ./Core/Src/Peripherals/iridium.su ./Core/Src/Peripherals/rf_switch.cyclo ./Core/Src/Peripherals/rf_switch.d ./Core/Src/Peripherals/rf_switch.o ./Core/Src/Peripherals/rf_switch.su ./Core/Src/Peripherals/u_ubx_protocol.cyclo ./Core/Src/Peripherals/u_ubx_protocol.d ./Core/Src/Peripherals/u_ubx_protocol.o ./Core/Src/Peripherals/u_ubx_protocol.su

.PHONY: clean-Core-2f-Src-2f-Peripherals

