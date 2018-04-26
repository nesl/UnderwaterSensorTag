################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
core/driver/msp430/USB_API/USB_CDC_API/UsbCdc.obj: ../core/driver/msp430/USB_API/USB_CDC_API/UsbCdc.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccsv7/tools/compiler/msp430_4.1.7/bin/cl430" -vmspx --abi=coffabi --code_model=large --data_model=restricted --near_data=globals -O2 --opt_for_speed=4 -g --include_path="C:/ti/ccsv7/ccs_base/msp430/include" --include_path="C:/ti/ccsv7/tools/compiler/msp430_4.1.7/include" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/driver/eMPL" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/driver/include" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/driver/battery_management" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/driver/env_sensors" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/driver/MX25L25635E" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/driver/msp430" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/driver/msp430/F5xx_F6xx_Core_Lib" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/driver/msp430/USB_API" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/driver/msp430/USB_eMPL" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/eMPL-hal" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/mllite" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/core/mpl" --include_path="C:/Users/EUN SUN LEE/workspace_v7/CA-SDK-firmware test/simple_apps/msp430" --gcc --define=__MSP430F5528__ --define=USE_DMP --define=MPL_LOG_NDEBUG=1 --define=MPU9250 --define=AK8693_SECONDARY --define=CONFIG_INTERFACE_USB --define=I2C_B0 --define=EMPL --define=EMPL_TARGET_MSP430 --diag_warning=225 --display_error_number --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --printf_support=full --preproc_with_compile --preproc_dependency="core/driver/msp430/USB_API/USB_CDC_API/UsbCdc.d" --obj_directory="core/driver/msp430/USB_API/USB_CDC_API" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


