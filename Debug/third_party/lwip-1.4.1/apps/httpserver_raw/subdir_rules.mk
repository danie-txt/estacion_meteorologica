################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
third_party/lwip-1.4.1/apps/httpserver_raw/httpd.obj: C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps/httpserver_raw/httpd.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-arm_18.2.7.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --include_path="C:/Users/danic/workspace_Tiva/proy_https3" --include_path="C:/Users/danic/workspace_Tiva/proy_https3" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/examples/boards/ek-tm4c1294xl" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/src/include/ipv4" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/ports/tiva-tm4c129/include" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party/lwip-1.4.1/apps" --include_path="C:/ti/TivaWare_C_Series-2.2.0.295/third_party" --include_path="C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-arm_18.2.7.LTS/include" --define=ccs="ccs" --define=PART_TM4C1294NCPDT --define=TARGET_IS_TM4C129_RA2 --define=EK_TM4C129_BP1 -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --ual --preproc_with_compile --preproc_dependency="third_party/lwip-1.4.1/apps/httpserver_raw/$(basename $(<F)).d_raw" --obj_directory="third_party/lwip-1.4.1/apps/httpserver_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


