@rem *********************
@rem Makefile for c6657
@rem *********************


@rem *** Path ***
@set PEOS_PATH="C:\Program Files (x86)\Myway Plus\PE-ViewX\pe-viewx\PEOS\c6657\3_11"


@rem *** Target File ***
@set TARGET=srmcode
@set GOAL=target.btbl


@rem *** Tools Name ***
@set COMPILER_DIR="C:\ti\c6000_7.4.24\bin"
@set CC=%COMPILER_DIR%\cl6x
@set LD=%COMPILER_DIR%\cl6x
@set HEX=%COMPILER_DIR%\hex6x
@set NM=%COMPILER_DIR%\nm6x
@set MAKE_DEF=make_def
@set MAKE_FUNCDEF=make_funcdef
@set MAKE_SBL=c6657_make_sbl
@set MAKE_FUNCSBL=c6657_make_funcsbl
@set MAKE_TYP=make_typ


@rem *** Objects & Libraries ***
@set OBJS=^
	srmcode.obj ^
	KlabIMP.obj ^
	KlabVector.obj

@set LIBS="C:\Program Files (x86)\Myway Plus\PE-ViewX\pe-viewx\PEOS\c6657\3_11\lib\Main.obj" ^
 "C:\Program Files (x86)\Myway Plus\PE-ViewX\pe-viewx\PEOS\c6657\3_11\lib\mwio4.lib" ^
 "C:\ti\mathlib_c66x_3_0_1_1\lib\mathlib.ae66" ^
 "C:\ti\c6000_7.4.24\bin\..\lib\rts6600_elf.lib" ^
 "C:\ti\pdk_C6657_1_1_2_6\packages\ti\csl\lib\ti.csl.ae66" ^
 "C:\ti\pdk_C6657_1_1_2_6\packages\ti\csl\lib\ti.csl.intc.ae66"


@rem *** Others ***
@set SYMBOL=^
	srmcode.@sbl ^
	 + KlabIMP.@sbl ^
	 + KlabVector.@sbl ^
 

@set SYMBOL_CLEAR=^
	srmcode.@sbl ^
	KlabIMP.@sbl ^
	KlabVector.@sbl ^
 

@set FUNCSYMBOL=^
	srmcode.@funcsbl ^
	 + KlabIMP.@funcsbl ^
	 + KlabVector.@funcsbl ^
 

@set FUNCSYMBOL_CLEAR=^
	srmcode.@funcsbl ^
	KlabIMP.@funcsbl ^
	KlabVector.@funcsbl ^
 


@rem *** Flags ***
@set BASE_CFLAGS=-mv6600 --display_error_number --preproc_with_compile --diag_warning=225 --abi=eabi -O2 -i"C:/Users/SMCProject-Chiba Lab/Documents/Xiang/1f_srm_code_20230418 copy used by xiang" -i"C:/Program Files (x86)/Myway Plus/PE-ViewX/pe-viewx/PEOS/c6657/3_11/inc" -i"C:/ti/c6000_7.4.24/bin/../include" -i"C:/ti/pdk_C6657_1_1_2_6/packages/ti/csl" -i"C:/ti/mathlib_c66x_3_0_1_1/inc" -i"C:/ti/mathlib_c66x_3_0_1_1/packages" -i"C:/ti/pdk_C6657_1_1_2_6/packages/ti/csl/../.."
@set CFLAGS=%BASE_CFLAGS% -k
@set ASMFLAGS=%BASE_CFLAGS%
@set LDFLAGS=--run_linker --rom_model --map_file=%TARGET%.map -l=%LIBS% -l=%TARGET%.cmd --zero_init=off 
@set HEXFLAGS=-m3 --memwidth=8


@rem *** Delete ***
@echo off
del %OBJS% %TARGET%.out target.btbl %TARGET%.typ %TARGET%.map %TARGET%.def %TARGET%.funcdef %TARGET%.tmp@@ %TARGET%.functmp@@ %TARGET%.all_sym %SYMBOL_CLEAR% %FUNCSYMBOL_CLEAR% 2> nul
@echo on


@rem *** Compile ***
%CC% %CFLAGS% srmcode.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% srmcode.asm srmcode.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% srmcode.asm srmcode.@funcsbl
@echo off & if errorlevel 1 goto ERR
del srmcode.asm 2> nul
@echo on


%CC% %CFLAGS% KlabIMP.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% KlabIMP.asm KlabIMP.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% KlabIMP.asm KlabIMP.@funcsbl
@echo off & if errorlevel 1 goto ERR
del KlabIMP.asm 2> nul
@echo on


%CC% %CFLAGS% KlabVector.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% KlabVector.asm KlabVector.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% KlabVector.asm KlabVector.@funcsbl
@echo off & if errorlevel 1 goto ERR
del KlabVector.asm 2> nul
@echo on


@rem *** Link ***
@echo off
copy %SYMBOL%  %TARGET%.tmp@@ > nul
@echo on
@echo off
copy %FUNCSYMBOL%  %TARGET%.functmp@@ > nul
@echo on
%MAKE_TYP% %TARGET%.tmp@@ srmcode.typ
@echo off & if errorlevel 1 goto ERR
@echo on
%LD% -o %OBJS% %LDFLAGS% --output_file=%TARGET%.out
@echo off & if errorlevel 1 goto ERR
@echo on


@rem *** Generates *.def ***
%HEX% -order L linker_image.rmd %TARGET%.out
@echo off & if errorlevel 1 goto ERR
@echo on
%NM% -a %TARGET%.out > %TARGET%.all_sym
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_DEF% %TARGET%.typ %PEOS_PATH%\config\Type.cfg %TARGET%.all_sym
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCDEF% %TARGET%.functmp@@ %TARGET%.all_sym
@echo off & if errorlevel 1 goto ERR
del %OBJS% %TARGET%.typ %TARGET%.tmp@@ %TARGET%.functmp@@ %TARGET%.all_sym %SYMBOL_CLEAR% %FUNCSYMBOL_CLEAR% 2> nul


@echo off
exit 0


@rem ** Error Process ***
:ERR
@echo off
del %OBJS% %TARGET%.typ %TARGET%.tmp@@ %TARGET%.functmp@@ %TARGET%.all_sym %SYMBOL_CLEAR% %FUNCSYMBOL_CLEAR% 2> nul
exit 1
