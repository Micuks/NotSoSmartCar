###############################################################################
#                                                                             #
#       Copyright (C) 2015 Infineon Technologies AG. All rights reserved.     #
#                                                                             #
#                                                                             #
#                              IMPORTANT NOTICE                               #
#                                                                             #
#                                                                             #
# Infineon Technologies AG (Infineon) is supplying this file for use          #
# exclusively with Infineonís microcontroller products. This file can be      #
# freely distributed within development tools that are supporting such        #
# microcontroller products.                                                   #
#                                                                             #
# THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED #
# OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF          #
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.#
# INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,#
# OR CONSEQUENTIAL DAMAGES, FOR	ANY REASON WHATSOEVER.                        #
#                                                                             #
###############################################################################
# Subdirectory make file for 0_Src/1_SrvSw/SysSe/Comm
# This is for core type "MAIN" and Gnuc toolchain !
###############################################################################


OBJ_FILES_MAIN+= 2_Out/Gnuc/0_Src/1_SrvSw/SysSe/Comm/Ifx_Console.o
DEP_FILES_MAIN+= 2_Out/Gnuc/0_Src/1_SrvSw/SysSe/Comm/Ifx_Console.d

2_Out/Gnuc/0_Src/1_SrvSw/SysSe/Comm/Ifx_Console.o: 0_Src/1_SrvSw/SysSe/Comm/Ifx_Console.c 1_ToolEnv/0_Build/1_Config/CfgCompiler_Gnuc.mk
	@mkdir -p $(@D)
	@rm -f $(ELF_BIN_MAIN) $(ADDITIONAL_EXE_MAIN) $(MAP_FILE_MAIN) $(LIB_OBJ_MAIN)
	@echo 'Compiling 0_Src/1_SrvSw/SysSe/Comm/Ifx_Console.c'
	$(CC) $(CC_OPTS) @1_ToolEnv/0_Build/9_Make/cIncludePathList.opt -c $< -o $@ -save-temps=obj -MMD
	@echo ' '

OBJ_FILES_MAIN+= 2_Out/Gnuc/0_Src/1_SrvSw/SysSe/Comm/Ifx_Message.o
DEP_FILES_MAIN+= 2_Out/Gnuc/0_Src/1_SrvSw/SysSe/Comm/Ifx_Message.d

2_Out/Gnuc/0_Src/1_SrvSw/SysSe/Comm/Ifx_Message.o: 0_Src/1_SrvSw/SysSe/Comm/Ifx_Message.c 1_ToolEnv/0_Build/1_Config/CfgCompiler_Gnuc.mk
	@mkdir -p $(@D)
	@rm -f $(ELF_BIN_MAIN) $(ADDITIONAL_EXE_MAIN) $(MAP_FILE_MAIN) $(LIB_OBJ_MAIN)
	@echo 'Compiling 0_Src/1_SrvSw/SysSe/Comm/Ifx_Message.c'
	$(CC) $(CC_OPTS) @1_ToolEnv/0_Build/9_Make/cIncludePathList.opt -c $< -o $@ -save-temps=obj -MMD
	@echo ' '

OBJ_FILES_MAIN+= 2_Out/Gnuc/0_Src/1_SrvSw/SysSe/Comm/Ifx_Shell.o
DEP_FILES_MAIN+= 2_Out/Gnuc/0_Src/1_SrvSw/SysSe/Comm/Ifx_Shell.d

2_Out/Gnuc/0_Src/1_SrvSw/SysSe/Comm/Ifx_Shell.o: 0_Src/1_SrvSw/SysSe/Comm/Ifx_Shell.c 1_ToolEnv/0_Build/1_Config/CfgCompiler_Gnuc.mk
	@mkdir -p $(@D)
	@rm -f $(ELF_BIN_MAIN) $(ADDITIONAL_EXE_MAIN) $(MAP_FILE_MAIN) $(LIB_OBJ_MAIN)
	@echo 'Compiling 0_Src/1_SrvSw/SysSe/Comm/Ifx_Shell.c'
	$(CC) $(CC_OPTS) @1_ToolEnv/0_Build/9_Make/cIncludePathList.opt -c $< -o $@ -save-temps=obj -MMD
	@echo ' '

