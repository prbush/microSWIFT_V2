################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FFTImplementationCallback.c \
../Core/Src/NEDwaves_memlight.c \
../Core/Src/NEDwaves_memlight_data.c \
../Core/Src/NEDwaves_memlight_emxAPI.c \
../Core/Src/NEDwaves_memlight_emxutil.c \
../Core/Src/NEDwaves_memlight_initialize.c \
../Core/Src/NEDwaves_memlight_terminate.c \
../Core/Src/app_threadx.c \
../Core/Src/battery.c \
../Core/Src/bsearch.c \
../Core/Src/byte_array.c \
../Core/Src/ct_sensor.c \
../Core/Src/fft.c \
../Core/Src/gnss.c \
../Core/Src/imu.c \
../Core/Src/interp1.c \
../Core/Src/iridium.c \
../Core/Src/log.c \
../Core/Src/main.c \
../Core/Src/mean.c \
../Core/Src/mem_replacements.c \
../Core/Src/nullAssignment.c \
../Core/Src/rtGetInf.c \
../Core/Src/rtGetNaN.c \
../Core/Src/rt_nonfinite.c \
../Core/Src/rtwhalf.c \
../Core/Src/stm32u5xx_hal_msp.c \
../Core/Src/stm32u5xx_hal_timebase_tim.c \
../Core/Src/stm32u5xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32u5xx.c \
../Core/Src/u_ubx_protocol.c \
../Core/Src/var.c 

S_UPPER_SRCS += \
../Core/Src/tx_initialize_low_level.S 

C_DEPS += \
./Core/Src/FFTImplementationCallback.d \
./Core/Src/NEDwaves_memlight.d \
./Core/Src/NEDwaves_memlight_data.d \
./Core/Src/NEDwaves_memlight_emxAPI.d \
./Core/Src/NEDwaves_memlight_emxutil.d \
./Core/Src/NEDwaves_memlight_initialize.d \
./Core/Src/NEDwaves_memlight_terminate.d \
./Core/Src/app_threadx.d \
./Core/Src/battery.d \
./Core/Src/bsearch.d \
./Core/Src/byte_array.d \
./Core/Src/ct_sensor.d \
./Core/Src/fft.d \
./Core/Src/gnss.d \
./Core/Src/imu.d \
./Core/Src/interp1.d \
./Core/Src/iridium.d \
./Core/Src/log.d \
./Core/Src/main.d \
./Core/Src/mean.d \
./Core/Src/mem_replacements.d \
./Core/Src/nullAssignment.d \
./Core/Src/rtGetInf.d \
./Core/Src/rtGetNaN.d \
./Core/Src/rt_nonfinite.d \
./Core/Src/rtwhalf.d \
./Core/Src/stm32u5xx_hal_msp.d \
./Core/Src/stm32u5xx_hal_timebase_tim.d \
./Core/Src/stm32u5xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32u5xx.d \
./Core/Src/u_ubx_protocol.d \
./Core/Src/var.d 

OBJS += \
./Core/Src/FFTImplementationCallback.o \
./Core/Src/NEDwaves_memlight.o \
./Core/Src/NEDwaves_memlight_data.o \
./Core/Src/NEDwaves_memlight_emxAPI.o \
./Core/Src/NEDwaves_memlight_emxutil.o \
./Core/Src/NEDwaves_memlight_initialize.o \
./Core/Src/NEDwaves_memlight_terminate.o \
./Core/Src/app_threadx.o \
./Core/Src/battery.o \
./Core/Src/bsearch.o \
./Core/Src/byte_array.o \
./Core/Src/ct_sensor.o \
./Core/Src/fft.o \
./Core/Src/gnss.o \
./Core/Src/imu.o \
./Core/Src/interp1.o \
./Core/Src/iridium.o \
./Core/Src/log.o \
./Core/Src/main.o \
./Core/Src/mean.o \
./Core/Src/mem_replacements.o \
./Core/Src/nullAssignment.o \
./Core/Src/rtGetInf.o \
./Core/Src/rtGetNaN.o \
./Core/Src/rt_nonfinite.o \
./Core/Src/rtwhalf.o \
./Core/Src/stm32u5xx_hal_msp.o \
./Core/Src/stm32u5xx_hal_timebase_tim.o \
./Core/Src/stm32u5xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32u5xx.o \
./Core/Src/tx_initialize_low_level.o \
./Core/Src/u_ubx_protocol.o \
./Core/Src/var.o 

S_UPPER_DEPS += \
./Core/Src/tx_initialize_low_level.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/app_threadx.o: ../Core/Src/app_threadx.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I../Middlewares/ST/threadx/utility/low_power -I"C:/Users/veteran/STM32CubeIDE/microSWIFT_V2/microSWIFT_U5/Core/Inc/Waves" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o: ../Core/Src/%.S Core/Src/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m33 -g3 -DDEBUG -DTX_SINGLE_MODE_NON_SECURE=1 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/FFTImplementationCallback.d ./Core/Src/FFTImplementationCallback.o ./Core/Src/FFTImplementationCallback.su ./Core/Src/NEDwaves_memlight.d ./Core/Src/NEDwaves_memlight.o ./Core/Src/NEDwaves_memlight.su ./Core/Src/NEDwaves_memlight_data.d ./Core/Src/NEDwaves_memlight_data.o ./Core/Src/NEDwaves_memlight_data.su ./Core/Src/NEDwaves_memlight_emxAPI.d ./Core/Src/NEDwaves_memlight_emxAPI.o ./Core/Src/NEDwaves_memlight_emxAPI.su ./Core/Src/NEDwaves_memlight_emxutil.d ./Core/Src/NEDwaves_memlight_emxutil.o ./Core/Src/NEDwaves_memlight_emxutil.su ./Core/Src/NEDwaves_memlight_initialize.d ./Core/Src/NEDwaves_memlight_initialize.o ./Core/Src/NEDwaves_memlight_initialize.su ./Core/Src/NEDwaves_memlight_terminate.d ./Core/Src/NEDwaves_memlight_terminate.o ./Core/Src/NEDwaves_memlight_terminate.su ./Core/Src/app_threadx.d ./Core/Src/app_threadx.o ./Core/Src/app_threadx.su ./Core/Src/battery.d ./Core/Src/battery.o ./Core/Src/battery.su ./Core/Src/bsearch.d ./Core/Src/bsearch.o ./Core/Src/bsearch.su ./Core/Src/byte_array.d ./Core/Src/byte_array.o ./Core/Src/byte_array.su ./Core/Src/ct_sensor.d ./Core/Src/ct_sensor.o ./Core/Src/ct_sensor.su ./Core/Src/fft.d ./Core/Src/fft.o ./Core/Src/fft.su ./Core/Src/gnss.d ./Core/Src/gnss.o ./Core/Src/gnss.su ./Core/Src/imu.d ./Core/Src/imu.o ./Core/Src/imu.su ./Core/Src/interp1.d ./Core/Src/interp1.o ./Core/Src/interp1.su ./Core/Src/iridium.d ./Core/Src/iridium.o ./Core/Src/iridium.su ./Core/Src/log.d ./Core/Src/log.o ./Core/Src/log.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mean.d ./Core/Src/mean.o ./Core/Src/mean.su ./Core/Src/mem_replacements.d ./Core/Src/mem_replacements.o ./Core/Src/mem_replacements.su ./Core/Src/nullAssignment.d ./Core/Src/nullAssignment.o ./Core/Src/nullAssignment.su ./Core/Src/rtGetInf.d ./Core/Src/rtGetInf.o ./Core/Src/rtGetInf.su ./Core/Src/rtGetNaN.d ./Core/Src/rtGetNaN.o ./Core/Src/rtGetNaN.su ./Core/Src/rt_nonfinite.d ./Core/Src/rt_nonfinite.o ./Core/Src/rt_nonfinite.su ./Core/Src/rtwhalf.d ./Core/Src/rtwhalf.o ./Core/Src/rtwhalf.su ./Core/Src/stm32u5xx_hal_msp.d ./Core/Src/stm32u5xx_hal_msp.o ./Core/Src/stm32u5xx_hal_msp.su ./Core/Src/stm32u5xx_hal_timebase_tim.d ./Core/Src/stm32u5xx_hal_timebase_tim.o ./Core/Src/stm32u5xx_hal_timebase_tim.su ./Core/Src/stm32u5xx_it.d ./Core/Src/stm32u5xx_it.o ./Core/Src/stm32u5xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32u5xx.d ./Core/Src/system_stm32u5xx.o ./Core/Src/system_stm32u5xx.su ./Core/Src/tx_initialize_low_level.d ./Core/Src/tx_initialize_low_level.o ./Core/Src/u_ubx_protocol.d ./Core/Src/u_ubx_protocol.o ./Core/Src/u_ubx_protocol.su ./Core/Src/var.d ./Core/Src/var.o ./Core/Src/var.su

.PHONY: clean-Core-2f-Src

