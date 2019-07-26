################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.5.LTS/bin/armcl" -mv7M3 --code_state=16 -me --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.5.LTS/include" --include_path="C:/ti/controlSUITE/device_support/f28m35x/v100/MWare" --define=ccs -g --gcc --diag_warning=225 --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


