################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../0_Src/BaseSw/iLLD/TC26B/Tricore/Gtm/Tom/Pwm/IfxGtm_Tom_Pwm.c 

OBJS += \
./0_Src/BaseSw/iLLD/TC26B/Tricore/Gtm/Tom/Pwm/IfxGtm_Tom_Pwm.o 

C_DEPS += \
./0_Src/BaseSw/iLLD/TC26B/Tricore/Gtm/Tom/Pwm/IfxGtm_Tom_Pwm.d 


# Each subdirectory must supply rules for building sources it contributes
0_Src/BaseSw/iLLD/TC26B/Tricore/Gtm/Tom/Pwm/%.o: ../0_Src/BaseSw/iLLD/TC26B/Tricore/Gtm/Tom/Pwm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TriCore C Compiler'
	"$(TRICORE_TOOLS)/bin/tricore-gcc" -c -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\AppSw" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\AppSw\Tricore" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\AppSw\Tricore\APP" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\AppSw\Tricore\Driver" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\AppSw\Tricore\Main" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\AppSw\Tricore\User" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\_Build" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\_Impl" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\_Lib" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\_Lib\DataHandling" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\_Lib\InternalMux" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\_PinMap" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Asclin" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Asclin\Asc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Asclin\Lin" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Asclin\Spi" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Asclin\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Ccu6" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Ccu6\Icu" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmBc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Ccu6\PwmHl" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Ccu6\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Ccu6\Timer" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Ccu6\TimerWithTrigger" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Ccu6\TPwm" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Cif" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Cif\Cam" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Cif\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Cpu" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Cpu\CStart" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Cpu\Irq" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Cpu\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Cpu\Trap" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Dma" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Dma\Dma" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Dma\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Dsadc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Dsadc\Dsadc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Dsadc\Rdc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Dsadc\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Dts" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Dts\Dts" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Dts\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Emem" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Emem\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Eray" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Eray\Eray" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Eray\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Eth" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Eth\Phy_Pef7071" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Eth\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Fce" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Fce\Crc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Fce\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Fft" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Fft\Fft" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Fft\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Flash" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Flash\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gpt12" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gpt12\IncrEnc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gpt12\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\Pwm" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\PwmHl" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Atom\Timer" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tim" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tim\In" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\Pwm" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\PwmHl" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Tom\Timer" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Gtm\Trig" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Hssl" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Hssl\Hssl" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Hssl\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\I2c" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\I2c\I2c" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\I2c\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Iom" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Iom\Driver" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Iom\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Msc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Msc\Msc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Msc\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Mtu" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Mtu\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Multican" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Multican\Can" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Multican\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Port" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Port\Io" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Port\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Psi5" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Psi5\Psi5" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Psi5\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Psi5s" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Psi5s\Psi5s" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Psi5s\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Qspi" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Qspi\SpiMaster" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Qspi\SpiSlave" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Qspi\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Scu" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Scu\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Sent" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Sent\Sent" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Sent\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Smu" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Smu\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Src" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Src\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Stm" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Stm\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Stm\Timer" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Vadc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Vadc\Adc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\iLLD\TC26B\Tricore\Vadc\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Infra" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Infra\Platform" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Infra\Platform\Tricore" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Infra\Platform\Tricore\Compilers" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Infra\Sfr" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Infra\Sfr\TC26B" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Infra\Sfr\TC26B\_Reg" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service\CpuGeneric" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service\CpuGeneric\_Utilities" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service\CpuGeneric\If" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service\CpuGeneric\If\Ccu6If" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service\CpuGeneric\StdIf" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service\CpuGeneric\SysSe" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service\CpuGeneric\SysSe\Bsp" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service\CpuGeneric\SysSe\Comm" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service\CpuGeneric\SysSe\General" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service\CpuGeneric\SysSe\Math" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\0_Src\BaseSw\Service\CpuGeneric\SysSe\Time" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\0_Utilities" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\1_Config" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\0_AppSw" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\0_AppSw\Tricore" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\0_AppSw\Tricore\Main" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\1_SrvSw" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\1_SrvSw\StdIf" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\1_SrvSw\SysSe" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\1_SrvSw\SysSe\Bsp" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\1_SrvSw\SysSe\Comm" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\1_SrvSw\SysSe\General" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\1_SrvSw\SysSe\Math" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\1_SrvSw\SysSe\Time" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\1_SrvSw\Tricore" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\1_SrvSw\Tricore\Compilers" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\_Impl" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\_Lib" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\_Lib\DataHandling" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\_Lib\InternalMux" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\_PinMap" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Asclin" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Asclin\Asc" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Asclin\Lin" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Asclin\Spi" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Asclin\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Cpu" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Cpu\CStart" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Cpu\Irq" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Cpu\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Cpu\Trap" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Dma" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Dma\Dma" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Dma\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Emem" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Emem\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Fft" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Fft\Fft" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Fft\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Port" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Port\Io" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Port\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Scu" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Scu\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Src" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Src\Std" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Stm" -I"D:\BUPT-SmartCar\7.5\WorkSpace\Algorithm\NotSoSmartCar\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Stm\Std" -fno-common -Os -g3 -W -Wall -Wextra -Wdiv-by-zero -Warray-bounds -Wcast-align -Wignored-qualifiers -Wformat -Wformat-security -pipe -DAPPKIT_TC265B -fshort-double -mcpu=tc26xx -mversion-info -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


