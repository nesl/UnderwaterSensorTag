################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
core/eMPL-hal/eMPL_outputs.obj: ../core/eMPL-hal/eMPL_outputs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"/Applications/ti/ccsv7/tools/compiler/ti-cgt-msp430_16.9.0.LTS/bin/cl430" -vmspx --abi=coffabi --code_model=large --data_model=restricted --near_data=globals -O2 --opt_for_speed=4 -g --include_path="/Applications/ti/ccsv7/ccs_base/msp430/include" --include_path="/Applications/ti/ccsv7/tools/compiler/ti-cgt-msp430_16.9.0.LTS/include" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/driver/eMPL" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/driver/include" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/driver/battery_management" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/driver/env_sensors" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/driver/MX25L25635E" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/driver/msp430" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/driver/msp430/F5xx_F6xx_Core_Lib" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/driver/msp430/USB_API" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/driver/msp430/USB_eMPL" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/eMPL-hal" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/mllite" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/core/mpl" --include_path="/Users/eunsunlee/Documents/NESL/CA-SDK/Release_CA_SDK_Firmware_08212013/simple_apps/msp430" --gcc --define=__MSP430F5528__ --define=USE_DMP --define=MPL_LOG_NDEBUG=1 --define=MPU9250 --define=AK8693_SECONDARY --define=CONFIG_INTERFACE_USB --define=I2C_B0 --define=EMPL --define=EMPL_TARGET_MSP430 --diag_warning=225 --display_error_number --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --printf_support=full --preproc_with_compile --preproc_dependency="core/eMPL-hal/eMPL_outputs.d" --obj_directory="core/eMPL-hal" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


