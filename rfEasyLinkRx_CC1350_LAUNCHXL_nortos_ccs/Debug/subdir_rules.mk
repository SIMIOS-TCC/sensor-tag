################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
CC1350_LAUNCHXL.obj: ../CC1350_LAUNCHXL.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/laris/Documents/SIMIOS-TCC/SensorTag/rfEasyLinkRx_CC1350_LAUNCHXL_nortos_ccs" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/source" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/kernel/nortos" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/kernel/nortos/posix" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/include" --define=DeviceFamily_CC13X0 --define=CCFG_FORCE_VDDR_HH=0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="CC1350_LAUNCHXL.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

ccfg.obj: ../ccfg.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/laris/Documents/SIMIOS-TCC/SensorTag/rfEasyLinkRx_CC1350_LAUNCHXL_nortos_ccs" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/source" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/kernel/nortos" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/kernel/nortos/posix" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/include" --define=DeviceFamily_CC13X0 --define=CCFG_FORCE_VDDR_HH=0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="ccfg.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

main_nortos.obj: ../main_nortos.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/laris/Documents/SIMIOS-TCC/SensorTag/rfEasyLinkRx_CC1350_LAUNCHXL_nortos_ccs" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/source" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/kernel/nortos" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/kernel/nortos/posix" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/include" --define=DeviceFamily_CC13X0 --define=CCFG_FORCE_VDDR_HH=0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="main_nortos.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

rfEasyLinkRx_nortos.obj: ../rfEasyLinkRx_nortos.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/bin/armcl" -mv7M3 --code_state=16 --float_support=vfplib -me --include_path="C:/Users/laris/Documents/SIMIOS-TCC/SensorTag/rfEasyLinkRx_CC1350_LAUNCHXL_nortos_ccs" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/source" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/kernel/nortos" --include_path="C:/ti/simplelink_cc13x0_sdk_2_10_00_36/kernel/nortos/posix" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/include" --define=DeviceFamily_CC13X0 --define=CCFG_FORCE_VDDR_HH=0 -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="rfEasyLinkRx_nortos.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


