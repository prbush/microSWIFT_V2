################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FFTImplementationCallback.c \
../Core/Src/NEDwaves.c \
../Core/Src/NEDwaves_data.c \
../Core/Src/NEDwaves_emxAPI.c \
../Core/Src/NEDwaves_emxutil.c \
../Core/Src/NEDwaves_initialize.c \
../Core/Src/NEDwaves_terminate.c \
../Core/Src/app_threadx.c \
../Core/Src/battery.c \
../Core/Src/blockedSummation.c \
../Core/Src/ct_sensor.c \
../Core/Src/detrend.c \
../Core/Src/div.c \
../Core/Src/fft.c \
../Core/Src/gps.c \
../Core/Src/imu.c \
../Core/Src/iridium.c \
../Core/Src/log.c \
../Core/Src/main.c \
../Core/Src/mean.c \
../Core/Src/mem_replacements.c \
../Core/Src/minOrMax.c \
../Core/Src/nullAssignment.c \
../Core/Src/qrsolve.c \
../Core/Src/rtGetInf.c \
../Core/Src/rtGetNaN.c \
../Core/Src/rt_nonfinite.c \
../Core/Src/rtwhalf.c \
../Core/Src/std.c \
../Core/Src/stm32u5xx_hal_msp.c \
../Core/Src/stm32u5xx_hal_timebase_tim.c \
../Core/Src/stm32u5xx_it.c \
../Core/Src/sum.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32u5xx.c \
../Core/Src/u_ubx_protocol.c \
../Core/Src/var.c \
../Core/Src/xnrm2.c 

S_UPPER_SRCS += \
../Core/Src/tx_initialize_low_level.S 

C_DEPS += \
./Core/Src/FFTImplementationCallback.d \
./Core/Src/NEDwaves.d \
./Core/Src/NEDwaves_data.d \
./Core/Src/NEDwaves_emxAPI.d \
./Core/Src/NEDwaves_emxutil.d \
./Core/Src/NEDwaves_initialize.d \
./Core/Src/NEDwaves_terminate.d \
./Core/Src/app_threadx.d \
./Core/Src/battery.d \
./Core/Src/blockedSummation.d \
./Core/Src/ct_sensor.d \
./Core/Src/detrend.d \
./Core/Src/div.d \
./Core/Src/fft.d \
./Core/Src/gps.d \
./Core/Src/imu.d \
./Core/Src/iridium.d \
./Core/Src/log.d \
./Core/Src/main.d \
./Core/Src/mean.d \
./Core/Src/mem_replacements.d \
./Core/Src/minOrMax.d \
./Core/Src/nullAssignment.d \
./Core/Src/qrsolve.d \
./Core/Src/rtGetInf.d \
./Core/Src/rtGetNaN.d \
./Core/Src/rt_nonfinite.d \
./Core/Src/rtwhalf.d \
./Core/Src/std.d \
./Core/Src/stm32u5xx_hal_msp.d \
./Core/Src/stm32u5xx_hal_timebase_tim.d \
./Core/Src/stm32u5xx_it.d \
./Core/Src/sum.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32u5xx.d \
./Core/Src/u_ubx_protocol.d \
./Core/Src/var.d \
./Core/Src/xnrm2.d 

OBJS += \
./Core/Src/FFTImplementationCallback.o \
./Core/Src/NEDwaves.o \
./Core/Src/NEDwaves_data.o \
./Core/Src/NEDwaves_emxAPI.o \
./Core/Src/NEDwaves_emxutil.o \
./Core/Src/NEDwaves_initialize.o \
./Core/Src/NEDwaves_terminate.o \
./Core/Src/app_threadx.o \
./Core/Src/battery.o \
./Core/Src/blockedSummation.o \
./Core/Src/ct_sensor.o \
./Core/Src/detrend.o \
./Core/Src/div.o \
./Core/Src/fft.o \
./Core/Src/gps.o \
./Core/Src/imu.o \
./Core/Src/iridium.o \
./Core/Src/log.o \
./Core/Src/main.o \
./Core/Src/mean.o \
./Core/Src/mem_replacements.o \
./Core/Src/minOrMax.o \
./Core/Src/nullAssignment.o \
./Core/Src/qrsolve.o \
./Core/Src/rtGetInf.o \
./Core/Src/rtGetNaN.o \
./Core/Src/rt_nonfinite.o \
./Core/Src/rtwhalf.o \
./Core/Src/std.o \
./Core/Src/stm32u5xx_hal_msp.o \
./Core/Src/stm32u5xx_hal_timebase_tim.o \
./Core/Src/stm32u5xx_it.o \
./Core/Src/sum.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32u5xx.o \
./Core/Src/tx_initialize_low_level.o \
./Core/Src/u_ubx_protocol.o \
./Core/Src/var.o \
./Core/Src/xnrm2.o 

S_UPPER_DEPS += \
./Core/Src/tx_initialize_low_level.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I../Middlewares/ST/threadx/utility/low_power -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/app_threadx.o: ../Core/Src/app_threadx.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I../Middlewares/ST/threadx/utility/low_power -I"C:/Users/veteran/STM32CubeIDE/microSWIFT_V2/microSWIFT_U5/Core/Inc/Waves" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/blockedSummation.o: ../Core/Src/blockedSummation.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I../Middlewares/ST/threadx/utility/low_power -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o: ../Core/Src/%.S Core/Src/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m33 -g3 -DDEBUG -DTX_SINGLE_MODE_NON_SECURE=1 -DTX_LOW_POWER -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/FFTImplementationCallback.d ./Core/Src/FFTImplementationCallback.o ./Core/Src/FFTImplementationCallback.su ./Core/Src/NEDwaves.d ./Core/Src/NEDwaves.o ./Core/Src/NEDwaves.su ./Core/Src/NEDwaves_data.d ./Core/Src/NEDwaves_data.o ./Core/Src/NEDwaves_data.su ./Core/Src/NEDwaves_emxAPI.d ./Core/Src/NEDwaves_emxAPI.o ./Core/Src/NEDwaves_emxAPI.su ./Core/Src/NEDwaves_emxutil.d ./Core/Src/NEDwaves_emxutil.o ./Core/Src/NEDwaves_emxutil.su ./Core/Src/NEDwaves_initialize.d ./Core/Src/NEDwaves_initialize.o ./Core/Src/NEDwaves_initialize.su ./Core/Src/NEDwaves_terminate.d ./Core/Src/NEDwaves_terminate.o ./Core/Src/NEDwaves_terminate.su ./Core/Src/app_threadx.d ./Core/Src/app_threadx.o ./Core/Src/app_threadx.su ./Core/Src/battery.d ./Core/Src/battery.o ./Core/Src/battery.su ./Core/Src/blockedSummation.d ./Core/Src/blockedSummation.o ./Core/Src/blockedSummation.su ./Core/Src/ct_sensor.d ./Core/Src/ct_sensor.o ./Core/Src/ct_sensor.su ./Core/Src/detrend.d ./Core/Src/detrend.o ./Core/Src/detrend.su ./Core/Src/div.d ./Core/Src/div.o ./Core/Src/div.su ./Core/Src/fft.d ./Core/Src/fft.o ./Core/Src/fft.su ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/gps.su ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/iridium.d ./Core/Src/iridium.o ./Core/Src/iridium.su ./Core/Src/log.d ./Core/Src/log.o ./Core/Src/log.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mean.d ./Core/Src/mean.o ./Core/Src/mean.su ./Core/Src/mem_replacements.d ./Core/Src/mem_replacements.o ./Core/Src/mem_replacements.su ./Core/Src/minOrMax.d ./Core/Src/minOrMax.o ./Core/Src/minOrMax.su ./Core/Src/nullAssignment.d ./Core/Src/nullAssignment.o ./Core/Src/nullAssignment.su ./Core/Src/qrsolve.d ./Core/Src/qrsolve.o ./Core/Src/qrsolve.su ./Core/Src/rtGetInf.d ./Core/Src/rtGetInf.o ./Core/Src/rtGetInf.su ./Core/Src/rtGetNaN.d ./Core/Src/rtGetNaN.o ./Core/Src/rtGetNaN.su ./Core/Src/rt_nonfinite.d ./Core/Src/rt_nonfinite.o ./Core/Src/rt_nonfinite.su ./Core/Src/rtwhalf.d ./Core/Src/rtwhalf.o ./Core/Src/rtwhalf.su ./Core/Src/std.d ./Core/Src/std.o ./Core/Src/std.su ./Core/Src/stm32u5xx_hal_msp.d ./Core/Src/stm32u5xx_hal_msp.o ./Core/Src/stm32u5xx_hal_msp.su ./Core/Src/stm32u5xx_hal_timebase_tim.d ./Core/Src/stm32u5xx_hal_timebase_tim.o ./Core/Src/stm32u5xx_hal_timebase_tim.su ./Core/Src/stm32u5xx_it.d ./Core/Src/stm32u5xx_it.o ./Core/Src/stm32u5xx_it.su ./Core/Src/sum.d ./Core/Src/sum.o ./Core/Src/sum.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32u5xx.d ./Core/Src/system_stm32u5xx.o ./Core/Src/system_stm32u5xx.su ./Core/Src/tx_initialize_low_level.d ./Core/Src/tx_initialize_low_level.o ./Core/Src/u_ubx_protocol.d ./Core/Src/u_ubx_protocol.o ./Core/Src/u_ubx_protocol.su ./Core/Src/var.d ./Core/Src/var.o ./Core/Src/var.su ./Core/Src/xnrm2.d ./Core/Src/xnrm2.o ./Core/Src/xnrm2.su

.PHONY: clean-Core-2f-Src

