################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/005_3_maindriver_spi_rev_it.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/005_3_maindriver_spi_rev_it.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/005_3_maindriver_spi_rev_it.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F429I_DISC1 -DSTM32 -DSTM32F429ZITx -DSTM32F4 -c -I../Inc -I"D:/STM32IDE/MCU1/005DriverSTM32F429xx/drivers/Inc" -I"D:/STM32IDE/MCU1/005DriverSTM32F429xx/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/005_3_maindriver_spi_rev_it.cyclo ./Src/005_3_maindriver_spi_rev_it.d ./Src/005_3_maindriver_spi_rev_it.o ./Src/005_3_maindriver_spi_rev_it.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

