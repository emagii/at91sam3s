# ----------------------------------------------------------------------------
#         ATMEL Microcontroller Software Support
# ----------------------------------------------------------------------------
# Copyright (c) 2010, Atmel Corporation
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the disclaimer below.
#
# Atmel's name may not be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

# Makefile for compiling libusb
.SUFFIXES: .o .a .c .s
SUB_MAKEFILES=debug.mk gcc.mk iar.mk mdk.mk release.mk win.mk linux.mk

#-------------------------------------------------------------------------------
# User-modifiable options
#-------------------------------------------------------------------------------
SERIE=sam3s
CHIP=$(SERIE)4
BOARD=$(SERIE)_ek
LIBNAME=libusb
TOOLCHAIN=gcc

#-------------------------------------------------------------------------------
# we detect OS (Linux/Windows/Cygwin)
# not defined for Cygwin
#ifdef $(OS)
ifeq ($(OS), Windows_NT)
include win.mk
else
include linux.mk
endif
#else
# Cygwin case
#include linux.mk
#endif

#-------------------------------------------------------------------------------
# Path
#-------------------------------------------------------------------------------

# Output directories
OUTPUT_BIN = ../../lib

# Libraries
PROJECT_BASE_PATH = ../..

#-------------------------------------------------------------------------------
# Files
#-------------------------------------------------------------------------------

vpath %.h $(PROJECT_BASE_PATH)/include
vpath %.c $(PROJECT_BASE_PATH)/../usb/include
vpath %.c $(PROJECT_BASE_PATH)/../usb/include/ccid
vpath %.c $(PROJECT_BASE_PATH)/common/audio
vpath %.c $(PROJECT_BASE_PATH)/common/cdc
vpath %.c $(PROJECT_BASE_PATH)/common/core
vpath %.c $(PROJECT_BASE_PATH)/common/hid
vpath %.c $(PROJECT_BASE_PATH)/common/massstorage
vpath %.c $(PROJECT_BASE_PATH)/device/audio-speaker
vpath %.c $(PROJECT_BASE_PATH)/device/audio-speakerphone
vpath %.c $(PROJECT_BASE_PATH)/device/ccid
vpath %.c $(PROJECT_BASE_PATH)/device/cdc-serial
vpath %.c $(PROJECT_BASE_PATH)/device/composite
vpath %.c $(PROJECT_BASE_PATH)/device/core
vpath %.c $(PROJECT_BASE_PATH)/device/hid-keyboard
vpath %.c $(PROJECT_BASE_PATH)/device/hid-mouse
vpath %.c $(PROJECT_BASE_PATH)/device/hid-transfer
vpath %.c $(PROJECT_BASE_PATH)/device/massstorage
vpath %.s $(PROJECT_BASE_PATH)/source

VPATH += $(PROJECT_BASE_PATH)/common/audio
VPATH += $(PROJECT_BASE_PATH)/common/cdc
VPATH += $(PROJECT_BASE_PATH)/common/core
VPATH += $(PROJECT_BASE_PATH)/common/hid
VPATH += $(PROJECT_BASE_PATH)/common/massstorage
VPATH += $(PROJECT_BASE_PATH)/device/audio-speaker
VPATH += $(PROJECT_BASE_PATH)/device/audio-speakerphone
VPATH += $(PROJECT_BASE_PATH)/device/ccid
VPATH += $(PROJECT_BASE_PATH)/device/cdc-serial
VPATH += $(PROJECT_BASE_PATH)/device/composite
VPATH += $(PROJECT_BASE_PATH)/device/core
VPATH += $(PROJECT_BASE_PATH)/device/hid-keyboard
VPATH += $(PROJECT_BASE_PATH)/device/hid-mouse
VPATH += $(PROJECT_BASE_PATH)/device/hid-transfer
VPATH += $(PROJECT_BASE_PATH)/device/massstorage

INCLUDES = -I$(PROJECT_BASE_PATH)
INCLUDES += -I$(PROJECT_BASE_PATH)/include
INCLUDES += -I$(PROJECT_BASE_PATH)/../libboard_$(SERIE)-ek
INCLUDES += -I$(PROJECT_BASE_PATH)/../libchip_$(SERIE)
INCLUDES += -I$(PROJECT_BASE_PATH)/../memories
INCLUDES += -I$(PROJECT_BASE_PATH)/../usb/include
INCLUDES += -I$(PROJECT_BASE_PATH)/../usb/include/ccid

#-------------------------------------------------------------------------------
ifdef DEBUG
include debug.mk
else
include release.mk
endif

#-------------------------------------------------------------------------------
# Tools
#-------------------------------------------------------------------------------

include $(TOOLCHAIN).mk

#-------------------------------------------------------------------------------
ifdef DEBUG
OUTPUT_OBJ=debug
OUTPUT_LIB=$(LIBNAME)_$(SERIE)_$(TOOLCHAIN)_dbg.a
else
OUTPUT_OBJ=release
OUTPUT_LIB=$(LIBNAME)_$(SERIE)_$(TOOLCHAIN)_rel.a
endif

OUTPUT_PATH=$(OUTPUT_OBJ)_$(BOARD)

#-------------------------------------------------------------------------------
# C source files and objects
#-------------------------------------------------------------------------------
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/common/audio/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/common/cdc/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/common/core/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/common/hid/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/common/massstorage/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/device/audio-speaker/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/device/audio-speakerphone/*.c)
#C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/device/ccid/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/device/cdc-serial/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/device/composite/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/device/core/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/device/hid-keyboard/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/device/hid-mouse/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/device/hid-transfer/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/device/massstorage/*.c)

C_OBJ_TEMP=$(patsubst %.c, %.o, $(notdir $(C_SRC)))

# during development, remove some files
C_OBJ_FILTER=

ifneq '$(TOOLCHAIN)' 'gcc'
#C_OBJ_FILTER+=
endif

ifneq '$(TOOLCHAIN)' 'ewarm'
#C_OBJ_FILTER+=
endif

ifneq '$(TOOLCHAIN)' 'mdk'
#C_OBJ_FILTER+=
endif

C_OBJ=$(filter-out $(C_OBJ_FILTER), $(C_OBJ_TEMP))

#-------------------------------------------------------------------------------
# Assembler source files and objects
#-------------------------------------------------------------------------------
A_SRC=$(wildcard $(PROJECT_BASE_PATH)/source/*.s)

A_OBJ_TEMP=$(patsubst %.s, %.o, $(notdir $(A_SRC)))

# during development, remove some files
A_OBJ_FILTER=

A_OBJ=$(filter-out $(A_OBJ_FILTER), $(A_OBJ_TEMP))

#-------------------------------------------------------------------------------
# Rules
#-------------------------------------------------------------------------------
all: $(BOARD)

$(BOARD): create_output $(OUTPUT_LIB)

debug: create_output $(OUTPUT_LIB)

release: create_output $(OUTPUT_LIB)

.PHONY: create_output
create_output:
#	@echo --- Preparing $(BOARD) files $(OUTPUT_PATH)  $(OUTPUT_BIN) $(OS) $(TOOLCHAIN)
#	@echo -------------------------
#	@echo *$(C_SRC)
#	@echo -------------------------
#	@echo *$(C_OBJ)
#	@echo -------------------------
#	@echo *$(addprefix $(OUTPUT_PATH)/, $(C_OBJ))
#	@echo -------------------------
#	@echo *$(A_SRC)
#	@echo -------------------------

	-@mkdir $(subst /,$(SEP),$(OUTPUT_BIN)) 1>NUL 2>&1
	-@mkdir $(OUTPUT_PATH) 1>NUL 2>&1

$(addprefix $(OUTPUT_PATH)/,$(C_OBJ)): $(OUTPUT_PATH)/%.o: %.c
	@$(CC) -c $(CFLAGS) $< -o $@

$(addprefix $(OUTPUT_PATH)/,$(A_OBJ)): $(OUTPUT_PATH)/%.o: %.s
	@$(AS) -c $(ASFLAGS) $< -o $@

$(OUTPUT_LIB): $(addprefix $(OUTPUT_PATH)/, $(C_OBJ)) $(addprefix $(OUTPUT_PATH)/, $(A_OBJ))
	@$(AR) -r $(OUTPUT_BIN)/$@ $^

.PHONY: clean
clean:
	@echo --- Cleaning $(LIBNAME) files [$(OUTPUT_PATH)$(SEP)*.o]
#	-cs-rm -fR $(OUTPUT_PATH)
#	-cs-rm $(subst /,$(SEP),$(OUTPUT_BIN)/$(OUTPUT_LIB))
	
	-@cs-rm -fR $(OUTPUT_PATH) 1>NUL 2>&1
	-@cs-rm -fR $(OUTPUT_BIN)/$(OUTPUT_LIB) 1>NUL 2>&1

$(addprefix $(OUTPUT_PATH)/,$(C_OBJ)):
