################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/NEDWaves/FFTImplementationCallback.c \
../Core/Src/NEDWaves/NEDwaves_memlight.c \
../Core/Src/NEDWaves/NEDwaves_memlight_data.c \
../Core/Src/NEDWaves/NEDwaves_memlight_emxAPI.c \
../Core/Src/NEDWaves/NEDwaves_memlight_emxutil.c \
../Core/Src/NEDWaves/NEDwaves_memlight_initialize.c \
../Core/Src/NEDWaves/NEDwaves_memlight_terminate.c \
../Core/Src/NEDWaves/bsearch.c \
../Core/Src/NEDWaves/colon.c \
../Core/Src/NEDWaves/combineVectorElements.c \
../Core/Src/NEDWaves/div.c \
../Core/Src/NEDWaves/fft.c \
../Core/Src/NEDWaves/mean.c \
../Core/Src/NEDWaves/mem_replacements.c \
../Core/Src/NEDWaves/minOrMax.c \
../Core/Src/NEDWaves/nullAssignment.c \
../Core/Src/NEDWaves/rtGetInf.c \
../Core/Src/NEDWaves/rtGetNaN.c \
../Core/Src/NEDWaves/rt_nonfinite.c \
../Core/Src/NEDWaves/rtwhalf.c \
../Core/Src/NEDWaves/std.c \
../Core/Src/NEDWaves/var.c 

C_DEPS += \
./Core/Src/NEDWaves/FFTImplementationCallback.d \
./Core/Src/NEDWaves/NEDwaves_memlight.d \
./Core/Src/NEDWaves/NEDwaves_memlight_data.d \
./Core/Src/NEDWaves/NEDwaves_memlight_emxAPI.d \
./Core/Src/NEDWaves/NEDwaves_memlight_emxutil.d \
./Core/Src/NEDWaves/NEDwaves_memlight_initialize.d \
./Core/Src/NEDWaves/NEDwaves_memlight_terminate.d \
./Core/Src/NEDWaves/bsearch.d \
./Core/Src/NEDWaves/colon.d \
./Core/Src/NEDWaves/combineVectorElements.d \
./Core/Src/NEDWaves/div.d \
./Core/Src/NEDWaves/fft.d \
./Core/Src/NEDWaves/mean.d \
./Core/Src/NEDWaves/mem_replacements.d \
./Core/Src/NEDWaves/minOrMax.d \
./Core/Src/NEDWaves/nullAssignment.d \
./Core/Src/NEDWaves/rtGetInf.d \
./Core/Src/NEDWaves/rtGetNaN.d \
./Core/Src/NEDWaves/rt_nonfinite.d \
./Core/Src/NEDWaves/rtwhalf.d \
./Core/Src/NEDWaves/std.d \
./Core/Src/NEDWaves/var.d 

OBJS += \
./Core/Src/NEDWaves/FFTImplementationCallback.o \
./Core/Src/NEDWaves/NEDwaves_memlight.o \
./Core/Src/NEDWaves/NEDwaves_memlight_data.o \
./Core/Src/NEDWaves/NEDwaves_memlight_emxAPI.o \
./Core/Src/NEDWaves/NEDwaves_memlight_emxutil.o \
./Core/Src/NEDWaves/NEDwaves_memlight_initialize.o \
./Core/Src/NEDWaves/NEDwaves_memlight_terminate.o \
./Core/Src/NEDWaves/bsearch.o \
./Core/Src/NEDWaves/colon.o \
./Core/Src/NEDWaves/combineVectorElements.o \
./Core/Src/NEDWaves/div.o \
./Core/Src/NEDWaves/fft.o \
./Core/Src/NEDWaves/mean.o \
./Core/Src/NEDWaves/mem_replacements.o \
./Core/Src/NEDWaves/minOrMax.o \
./Core/Src/NEDWaves/nullAssignment.o \
./Core/Src/NEDWaves/rtGetInf.o \
./Core/Src/NEDWaves/rtGetNaN.o \
./Core/Src/NEDWaves/rt_nonfinite.o \
./Core/Src/NEDWaves/rtwhalf.o \
./Core/Src/NEDWaves/std.o \
./Core/Src/NEDWaves/var.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/NEDWaves/%.o Core/Src/NEDWaves/%.su: ../Core/Src/NEDWaves/%.c Core/Src/NEDWaves/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-NEDWaves

clean-Core-2f-Src-2f-NEDWaves:
	-$(RM) ./Core/Src/NEDWaves/FFTImplementationCallback.d ./Core/Src/NEDWaves/FFTImplementationCallback.o ./Core/Src/NEDWaves/FFTImplementationCallback.su ./Core/Src/NEDWaves/NEDwaves_memlight.d ./Core/Src/NEDWaves/NEDwaves_memlight.o ./Core/Src/NEDWaves/NEDwaves_memlight.su ./Core/Src/NEDWaves/NEDwaves_memlight_data.d ./Core/Src/NEDWaves/NEDwaves_memlight_data.o ./Core/Src/NEDWaves/NEDwaves_memlight_data.su ./Core/Src/NEDWaves/NEDwaves_memlight_emxAPI.d ./Core/Src/NEDWaves/NEDwaves_memlight_emxAPI.o ./Core/Src/NEDWaves/NEDwaves_memlight_emxAPI.su ./Core/Src/NEDWaves/NEDwaves_memlight_emxutil.d ./Core/Src/NEDWaves/NEDwaves_memlight_emxutil.o ./Core/Src/NEDWaves/NEDwaves_memlight_emxutil.su ./Core/Src/NEDWaves/NEDwaves_memlight_initialize.d ./Core/Src/NEDWaves/NEDwaves_memlight_initialize.o ./Core/Src/NEDWaves/NEDwaves_memlight_initialize.su ./Core/Src/NEDWaves/NEDwaves_memlight_terminate.d ./Core/Src/NEDWaves/NEDwaves_memlight_terminate.o ./Core/Src/NEDWaves/NEDwaves_memlight_terminate.su ./Core/Src/NEDWaves/bsearch.d ./Core/Src/NEDWaves/bsearch.o ./Core/Src/NEDWaves/bsearch.su ./Core/Src/NEDWaves/colon.d ./Core/Src/NEDWaves/colon.o ./Core/Src/NEDWaves/colon.su ./Core/Src/NEDWaves/combineVectorElements.d ./Core/Src/NEDWaves/combineVectorElements.o ./Core/Src/NEDWaves/combineVectorElements.su ./Core/Src/NEDWaves/div.d ./Core/Src/NEDWaves/div.o ./Core/Src/NEDWaves/div.su ./Core/Src/NEDWaves/fft.d ./Core/Src/NEDWaves/fft.o ./Core/Src/NEDWaves/fft.su ./Core/Src/NEDWaves/mean.d ./Core/Src/NEDWaves/mean.o ./Core/Src/NEDWaves/mean.su ./Core/Src/NEDWaves/mem_replacements.d ./Core/Src/NEDWaves/mem_replacements.o ./Core/Src/NEDWaves/mem_replacements.su ./Core/Src/NEDWaves/minOrMax.d ./Core/Src/NEDWaves/minOrMax.o ./Core/Src/NEDWaves/minOrMax.su ./Core/Src/NEDWaves/nullAssignment.d ./Core/Src/NEDWaves/nullAssignment.o ./Core/Src/NEDWaves/nullAssignment.su ./Core/Src/NEDWaves/rtGetInf.d ./Core/Src/NEDWaves/rtGetInf.o ./Core/Src/NEDWaves/rtGetInf.su ./Core/Src/NEDWaves/rtGetNaN.d ./Core/Src/NEDWaves/rtGetNaN.o ./Core/Src/NEDWaves/rtGetNaN.su ./Core/Src/NEDWaves/rt_nonfinite.d ./Core/Src/NEDWaves/rt_nonfinite.o ./Core/Src/NEDWaves/rt_nonfinite.su ./Core/Src/NEDWaves/rtwhalf.d ./Core/Src/NEDWaves/rtwhalf.o ./Core/Src/NEDWaves/rtwhalf.su ./Core/Src/NEDWaves/std.d ./Core/Src/NEDWaves/std.o ./Core/Src/NEDWaves/std.su ./Core/Src/NEDWaves/var.d ./Core/Src/NEDWaves/var.o ./Core/Src/NEDWaves/var.su

.PHONY: clean-Core-2f-Src-2f-NEDWaves

