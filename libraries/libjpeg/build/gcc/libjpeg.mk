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

# Makefile for compiling libjpeg
.SUFFIXES: .o .a .c .s
SUB_MAKEFILES=debug.mk gcc.mk iar.mk mdk.mk release.mk win.mk linux.mk

#-------------------------------------------------------------------------------
# User-modifiable options
#-------------------------------------------------------------------------------
CORE=CM3
SERIE=sam3s
CHIP=$(SERIE)4
BOARD=$(SERIE)_ek
LIBNAME=libjpeg
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
vpath %.c $(PROJECT_BASE_PATH)/source

VPATH+=$(PROJECT_BASE_PATH)/source


INCLUDES = -I$(PROJECT_BASE_PATH)
INCLUDES += -I$(PROJECT_BASE_PATH)/include


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
OUTPUT_LIB=$(LIBNAME)_$(CORE)_$(TOOLCHAIN)_dbg.a
else
OUTPUT_OBJ=release
OUTPUT_LIB=$(LIBNAME)_$(CORE)_$(TOOLCHAIN)_rel.a
endif

OUTPUT_PATH=$(OUTPUT_OBJ)_$(CORE)

#-------------------------------------------------------------------------------
# C source files and objects
#-------------------------------------------------------------------------------
C_SRC=$(wildcard $(PROJECT_BASE_PATH)/source/*.c)

C_OBJ_TEMP=$(patsubst %.c, %.o, $(notdir $(C_SRC)))

# during development, remove some files
C_OBJ_FILTER  = ansi2knr.o
C_OBJ_FILTER += cdjpeg.o
C_OBJ_FILTER += cjpeg.o
C_OBJ_FILTER += ckconfig.o
C_OBJ_FILTER += djpeg.o
C_OBJ_FILTER += example.o
C_OBJ_FILTER += jdatadst.o
C_OBJ_FILTER += jdatasrc.o
C_OBJ_FILTER += jdatasrc_stdio.o
C_OBJ_FILTER += jdatadst_stdio.o
C_OBJ_FILTER += jmemansi.o
C_OBJ_FILTER += jmemdos.o
C_OBJ_FILTER += jmemmac.o
C_OBJ_FILTER += jmemname.o
C_OBJ_FILTER += jpegtran.o
C_OBJ_FILTER += memsrc.o
C_OBJ_FILTER += rdbmp.o
C_OBJ_FILTER += rdcolmap.o
C_OBJ_FILTER += rdgif.o
C_OBJ_FILTER += rdjpgcom.o
C_OBJ_FILTER += rdppm.o
C_OBJ_FILTER += rdrle.o
C_OBJ_FILTER += rdswitch.o
C_OBJ_FILTER += rdtarga.o
C_OBJ_FILTER += transupp.o
C_OBJ_FILTER += wrbmp.o
C_OBJ_FILTER += wrgif.o
C_OBJ_FILTER += wrjpgcom.o
C_OBJ_FILTER += wrppm.o
C_OBJ_FILTER += wrrle.o
C_OBJ_FILTER += wrtarga.o


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
	@echo --- Cleaning $(BOARD) files [$(OUTPUT_PATH)$(SEP)*.o]
#	-cs-rm -fR $(OUTPUT_PATH)
#	-cs-rm $(subst /,$(SEP),$(OUTPUT_BIN)/$(OUTPUT_LIB))
	
	-@cs-rm -fR $(OUTPUT_PATH) 1>NUL 2>&1
	-@cs-rm -fR $(OUTPUT_BIN)/$(OUTPUT_LIB) 1>NUL 2>&1

$(addprefix $(OUTPUT_PATH)/,$(C_OBJ)):