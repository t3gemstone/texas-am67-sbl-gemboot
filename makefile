#
# Auto generated makefile
#

export MCU_PLUS_SDK_PATH?=$(abspath ../../../../../../..)
include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak

CG_TOOL_ROOT=$(CGT_TI_ARM_CLANG_PATH)

CC=$(CG_TOOL_ROOT)/bin/tiarmclang
LNK=$(CG_TOOL_ROOT)/bin/tiarmclang
STRIP=$(CG_TOOL_ROOT)/bin/tiarmstrip
OBJCOPY=$(CG_TOOL_ROOT)/bin/tiarmobjcopy
ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

PROFILE?=release
ConfigName:=$(PROFILE)

OUTNAME:=build/texas_am67_sbl_gemboot.$(PROFILE).out

BOOTIMAGE_PATH=$(abspath .)
BOOTIMAGE_NAME_GP:=build/texas_am67_sbl_gemboot.$(PROFILE).tiimage
BOOTIMAGE_NAME_HS:=build/texas_am67_sbl_gemboot.$(PROFILE).hs.tiimage
BOOTIMAGE_NAME_HS_FS:=build/texas_am67_sbl_gemboot.$(PROFILE).hs_fs.tiimage
BOOTIMAGE_BIN_NAME:=build/texas_am67_sbl_gemboot.$(PROFILE).bin

ifeq ($(DEVICE_TYPE),HS)
	BOOTIMAGE_NAME=$(BOOTIMAGE_NAME_HS)
else ifeq ($(DEVICE_TYPE), HS_FS)
	BOOTIMAGE_NAME=$(BOOTIMAGE_NAME_HS_FS)
else
	BOOTIMAGE_NAME=$(BOOTIMAGE_NAME_GP)
endif

FILES_common := \
	main.c \
	ti_drivers_config.c \
	ti_drivers_open_close.c \
	ti_board_config.c \
	ti_board_open_close.c \
	ti_dpl_config.c \
	ti_pinmux_config.c \
	ti_power_clock_config.c \

FILES_PATH_common = \
	.. \
	../../.. \
	build/generated \

INCLUDES_common := \
	-I${CG_TOOL_ROOT}/include/c \
	-I${MCU_PLUS_SDK_PATH}/source \
	-Ibuild/generated \
	-I./ \

DEFINES_common := \
	-DSOC_J722S \
	-DENABLE_SCICLIENT_DIRECT \

CFLAGS_common := \
	-DBUILD_WKUP_R5 \
	-mcpu=cortex-r5 \
	-mfloat-abi=hard \
	-mfpu=vfpv3-d16 \
	-mthumb \
	-Wall \
	-Werror \
	-g \
	-Wno-gnu-variable-sized-type-not-at-end \
	-Wno-unused-function \

CFLAGS_cpp_common := \
	-Wno-c99-designator \
	-Wno-extern-c-compat \
	-Wno-c++11-narrowing \
	-Wno-reorder-init-list \
	-Wno-deprecated-register \
	-Wno-writable-strings \
	-Wno-enum-compare \
	-Wno-reserved-user-defined-literal \
	-Wno-unused-const-variable \
	-x c++ \

CFLAGS_debug := \
	-D_DEBUG_=1 \

CFLAGS_release := \
	-D_DEBUG_=1 \

LNK_FILES_common = \
	linker.cmd \

LIBS_PATH_common = \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/board/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/sciclient_direct/sbl/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/rm_pm_hal/sbl/lib \
	-Wl,-i${CG_TOOL_ROOT}/lib \

LIBS_common = \
	-lnortos.j722s.r5f.ti-arm-clang.${ConfigName}.lib \
	-ldrivers.j722s.wkup-r5f.ti-arm-clang.${ConfigName}.lib \
	-lboard.j722s.r5f.ti-arm-clang.${ConfigName}.lib \
	-lsciclient_direct_sbl.j722s.wkup-r5f.ti-arm-clang.${ConfigName}.lib \
	-lrm_pm_hal_sbl.j722s.wkup-r5f.ti-arm-clang.${ConfigName}.lib \
	-llibc.a \
	-llibsysbm.a \

LFLAGS_common = \
	-Wl,--diag_suppress=10063 \
	-Wl,--priority \
	-Wl,--ram_model \
	-Wl,--reread_libs \


LIBS_NAME = \
	nortos.j722s.r5f.ti-arm-clang.${ConfigName}.lib \
	drivers.j722s.wkup-r5f.ti-arm-clang.${ConfigName}.lib \
	board.j722s.r5f.ti-arm-clang.${ConfigName}.lib \
	sciclient_direct_sbl.j722s.wkup-r5f.ti-arm-clang.${ConfigName}.lib \
	rm_pm_hal_sbl.j722s.wkup-r5f.ti-arm-clang.${ConfigName}.lib \
	libc.a \
	libsysbm.a \

LIBS_PATH_NAME = \
	${MCU_PLUS_SDK_PATH}/source/kernel/nortos/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	${MCU_PLUS_SDK_PATH}/source/board/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/sciclient_direct/sbl/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/rm_pm_hal/sbl/lib \
	${CG_TOOL_ROOT}/lib \

FILES := $(FILES_common) $(FILES_$(PROFILE))
ASMFILES := $(ASMFILES_common) $(ASMFILES_$(PROFILE))
FILES_PATH := $(FILES_PATH_common) $(FILES_PATH_$(PROFILE))
CFLAGS := $(CFLAGS_common) $(CFLAGS_$(PROFILE))
DEFINES := $(DEFINES_common) $(DEFINES_$(PROFILE))
INCLUDES := $(INCLUDES_common) $(INCLUDE_$(PROFILE))
LIBS := $(LIBS_common) $(LIBS_$(PROFILE))
LIBS_PATH := $(LIBS_PATH_common) $(LIBS_PATH_$(PROFILE))
LFLAGS := $(LFLAGS_common) $(LFLAGS_$(PROFILE))
LNKOPTFLAGS := $(LNKOPTFLAGS_common) $(LNKOPTFLAGS_$(PROFILE))
LNK_FILES := $(LNK_FILES_common) $(LNK_FILES_$(PROFILE))

OBJDIR := build/obj/$(PROFILE)/
OBJS := $(FILES:%.c=%.obj)
OBJS += $(ASMFILES:%.S=%.obj)
DEPS := $(FILES:%.c=%.d)

vpath %.obj $(OBJDIR)
vpath %.c $(FILES_PATH)
vpath %.S $(FILES_PATH)
vpath %.lib $(LIBS_PATH_NAME)
vpath %.a $(LIBS_PATH_NAME)

ifneq ($(PROFILE), debug)
$(OBJDIR)/%.obj %.obj: %.c
	@echo  Compiling: j722s:wkup-r5fss0-0:nortos:ti-arm-clang $(OUTNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -MMD -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: j722s:wkup-r5fss0-0:nortos:ti-arm-clang $(LIBNAME): $<
	$(CC) -c $(CFLAGS) -o $(OBJDIR)/$@ $<

all: $(BOOTIMAGE_NAME)

SYSCFG_GEN_FILES=build/generated/ti_drivers_config.c build/generated/ti_drivers_config.h
SYSCFG_GEN_FILES+=build/generated/ti_drivers_open_close.c build/generated/ti_drivers_open_close.h
SYSCFG_GEN_FILES+=build/generated/ti_dpl_config.c build/generated/ti_dpl_config.h
SYSCFG_GEN_FILES+=build/generated/ti_pinmux_config.c build/generated/ti_power_clock_config.c
SYSCFG_GEN_FILES+=build/generated/ti_board_config.c build/generated/ti_board_config.h
SYSCFG_GEN_FILES+=build/generated/ti_board_open_close.c build/generated/ti_board_open_close.h

$(OUTNAME): syscfg $(SYSCFG_GEN_FILES) $(OBJS) $(LNK_FILES) $(LIBS_NAME)
	@echo  .
	@echo  Linking: j722s:wkup-r5fss0-0:nortos:ti-arm-clang $@ ...
	$(LNK) $(LNKOPTFLAGS) $(LFLAGS) $(LIBS_PATH) -Wl,-m=$(basename $@).map -o $@ $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) $(LNK_FILES)
	@echo  Linking: j722s:wkup-r5fss0-0:nortos:ti-arm-clang $@ Done !!!
	@echo  .

clean:
	@echo  Cleaning: j722s:wkup-r5fss0-0:nortos:ti-arm-clang $(OUTNAME) ...
	$(RMDIR) $(OBJDIR)
	$(RM) $(OUTNAME)
	$(RM) $(BOOTIMAGE_NAME)
	$(RM) $(BOOTIMAGE_BIN_NAME)
	$(RMDIR) build/generated/

scrub:
	@echo  Scrubing: j722s:wkup-r5fss0-0:nortos:ti-arm-clang sbl_uart ...
	$(RMDIR) obj
ifeq ($(OS),Windows_NT)
	$(RM) \*.out
	$(RM) \*.map
	$(RM) \*.appimage*
	$(RM) \*.rprc*
	$(RM) \*.tiimage*
	$(RM) \*.bin
else
	$(RM) *.out
	$(RM) *.map
	$(RM) *.appimage*
	$(RM) *.rprc*
	$(RM) *.tiimage*
	$(RM) *.bin
endif
	$(RMDIR) build/generated

$(OBJS): | $(OBJDIR)

$(OBJDIR):
	$(MKDIR) $@

.NOTPARALLEL:

.INTERMEDIATE: syscfg
$(SYSCFG_GEN_FILES): syscfg

syscfg: ./example.syscfg
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context wkup-r5fss0-0 --part Default --package AMW --output build/generated/ ./example.syscfg

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device J722S_TDA4VEN_TDA4AEN_AM67 --context wkup-r5fss0-0 --part Default --package AMW --output build/generated/  ./example.syscfg

#
# Generation of boot image which can be loaded by ROM Boot Loader (RBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif
ifeq ($(OS),Windows_NT)
  BOOTIMAGE_CERT_GEN_CMD=powershell -executionpolicy unrestricted -command $(MCU_PLUS_SDK_PATH)/tools/boot/signing/x509CertificateGen.ps1
else
  BOOTIMAGE_CERT_GEN_CMD=$(MCU_PLUS_SDK_PATH)/tools/boot/signing/x509CertificateGen.sh
endif
BOOTIMAGE_TEMP_OUT_FILE=temp_stdout_$(PROFILE).txt

BOOTIMAGE_CERT_KEY=$(APP_SIGNING_KEY)


BOOTIMAGE_CERT_KEY=$(APP_SIGNING_KEY)
BOOTIMAGE_CERT_GEN_CMD=$(PYTHON) $(MCU_PLUS_SDK_PATH)/tools/boot/signing/rom_image_gen.py
SYSFW_PATH=$(MCU_PLUS_SDK_PATH)/source/drivers/sciclient/soc/j722s
TIFS_LOAD_ADDR=0x40000
BOARDCFG_LOAD_ADDR=0x67000
BOARDCFG_SBLDATA_LOAD_ADDR=0x43c7c800
BOARDCFG_BLOB=$(MCU_PLUS_SDK_PATH)/source/drivers/sciclient/sciclient_default_boardcfg/j722s/boardcfg_blob.bin
BOARDCFG_SBLDATA_BLOB=$(MCU_PLUS_SDK_PATH)/source/drivers/sciclient/sciclient_default_boardcfg/j722s/boardcfg_sbldata_blob.bin
ENABLE_SBLDATA=yes

SBL_RUN_ADDRESS=0x43C40000

SBL_PREBUILT_PATH=$(MCU_PLUS_SDK_PATH)/tools/boot/sbl_prebuilt/j722s-evm

$(BOOTIMAGE_BIN_NAME): $(OUTNAME)
	$(OBJCOPY) --strip-sections -O binary $(OUTNAME) $(BOOTIMAGE_BIN_NAME)

$(BOOTIMAGE_NAME): $(BOOTIMAGE_BIN_NAME)
	@echo  Boot image: j722s:wkup-r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ ...
ifeq ($(DEVICE_TYPE),HS)
ifeq ($(ENC_SBL_ENABLED),yes)
	$(BOOTIMAGE_CERT_GEN_CMD) --swrv 1 --sbl-enc --enc-key $(APP_ENCRYPTION_KEY) --sbl-bin $(BOOTIMAGE_PATH)/$(BOOTIMAGE_BIN_NAME) --tifs-bin $(SYSFW_PATH)/tifs-hs-enc.bin --tifs-inner-cert $(SYSFW_PATH)/tifs-hs-enc-cert.bin --boardcfg-blob $(BOARDCFG_BLOB) --boardcfg-sbldata-blob $(BOARDCFG_SBLDATA_BLOB) --sbl-loadaddr $(SBL_RUN_ADDRESS) --tifs-loadaddr $(TIFS_LOAD_ADDR) --bcfg-loadaddr $(BOARDCFG_LOAD_ADDR) --bcfg-sbldata-loadaddr $(BOARDCFG_SBLDATA_LOAD_ADDR) --key $(BOOTIMAGE_CERT_KEY) --rom-image $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) --enable-sbldata $(ENABLE_SBLDATA)
else
	$(BOOTIMAGE_CERT_GEN_CMD) --swrv 1 --sbl-bin $(BOOTIMAGE_PATH)/$(BOOTIMAGE_BIN_NAME) --tifs-bin $(SYSFW_PATH)/tifs-hs-enc.bin --tifs-inner-cert $(SYSFW_PATH)/tifs-hs-enc-cert.bin --boardcfg-blob $(BOARDCFG_BLOB) --boardcfg-sbldata-blob $(BOARDCFG_SBLDATA_BLOB) --sbl-loadaddr $(SBL_RUN_ADDRESS) --tifs-loadaddr $(TIFS_LOAD_ADDR) --bcfg-loadaddr $(BOARDCFG_LOAD_ADDR) --bcfg-sbldata-loadaddr $(BOARDCFG_SBLDATA_LOAD_ADDR) --key $(BOOTIMAGE_CERT_KEY) --rom-image $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) --enable-sbldata $(ENABLE_SBLDATA)
endif
else ifeq ($(DEVICE_TYPE), HS_FS)
	$(BOOTIMAGE_CERT_GEN_CMD) --swrv 1 --sbl-bin $(BOOTIMAGE_PATH)/$(BOOTIMAGE_BIN_NAME) --tifs-bin $(SYSFW_PATH)/tifs-hs-fs-enc.bin --tifs-inner-cert $(SYSFW_PATH)/tifs-hs-fs-enc-cert.bin --boardcfg-blob $(BOARDCFG_BLOB) --boardcfg-sbldata-blob $(BOARDCFG_SBLDATA_BLOB) --sbl-loadaddr $(SBL_RUN_ADDRESS) --tifs-loadaddr $(TIFS_LOAD_ADDR) --bcfg-loadaddr $(BOARDCFG_LOAD_ADDR) --bcfg-sbldata-loadaddr $(BOARDCFG_SBLDATA_LOAD_ADDR) --key $(BOOTIMAGE_CERT_KEY) --rom-image $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_HS_FS) --enable-sbldata $(ENABLE_SBLDATA)
endif
	$(COPY) $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) build/tiboot3.bin
	$(COPY) $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME) $(SBL_PREBUILT_PATH)/

	$(RM) $(BOOTIMAGE_TEMP_OUT_FILE)
	@echo  Boot image: j722s:wkup-r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ Done !!!
ifeq ($(DEVICE_TYPE),GP)
	$(COPY) $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_HS_FS) $(SBL_PREBUILT_PATH)/
	@echo  Boot image: j722s:wkup-r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_HS_FS) Done !!!
endif

	@echo  .

else
all clean scrub syscfg syscfg-gui:
    $(info SBL does not support compiling in debug profile, Please try building in release profile)
endif
-include $(addprefix $(OBJDIR)/, $(DEPS))
