
MCU_FLAG_NAME	= mstm8 -D


all:

########################################################################
# Makefile distribution path
#
# Prefer absolute paths to avoid problems with possible symlinks.
# But Windows requires a relative path for ARDMK_DIR and ARDUINO_DIR.

# Detect OS
ifeq ($(OS),Windows_NT)
    ifndef ARDMK_DIR
        # presume it's the same path to our own file
        # Windows requires relative paths here
        ARDMK_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
    else
        # show_config_variable macro is defined in Common.mk file and is not available yet.
        # Let's define a variable to know that user specified ARDMK_DIR
        ARDMK_DIR_MSG = USER
    endif
else
    # Linux (and Mac)
    ifndef ARDMK_DIR
        # presume it's the same path to our own file
        # Windows requires relative paths here
        ARDMK_DIR := $(realpath $(dir $(realpath $(lastword $(MAKEFILE_LIST)))))
    else
        # show_config_variable macro is defined in Common.mk file and is not available yet.
        # Let's define a variable to know that user specified ARDMK_DIR
        ARDMK_DIR_MSG = USER
    endif
endif

ifndef ARDUINO_DIR
    # presume it's the same path to our own file
    ARDUINO_DIR = $(ARDMK_DIR)
else
	#FIXME: this message is not yet implemented
    # show_config_variable macro is defined in Common.mk file and is not available yet.
    # Let's define a variable to know that user specified ARDUINO_DIR
    ARDUINO_DIR_MSG = USER
endif



########################################################################
# SDCC path
#
# It really should search in a set of well known directories like
# /usr/bin, /usr/local/bin, ~/.local/bin
# But the needed macros are defined in Common.mk and not known here yet.
# This part should really be integrated into Arduino.mk, than we could do it
# right.

# Detect OS
ifeq ($(OS),Windows_NT)
    ifndef SDCC_PATH
        # for Windows always assume the tools to be in this dir tree
        SDCC_PATH := $(ARDUINO_DIR)/../tools/sdcc/bin
	AVR_TOOLS_DIR = $(ARDUINO_DIR)/../tools/sdcc
    endif
    SHELL := $(ARDUINO_DIR)/../tools/win/busybox.exe
    .SHELLFLAGS=ash -c
    SIZE := $(ARDUINO_DIR)/../tools/wrapper/sdsize.sh
#    PATH  := $(realpath $(ARDUINO_DIR)/../tools/win):$(PATH)
else
    # Linux (and Mac): expect SDCC to be in /opt/sdcc
    ifndef SDCC_PATH
        # for Windows always assume the tools to be in this dir tree
        SDCC_PATH := /opt/sdcc/bin
	AVR_TOOLS_DIR = /opt/sdcc
    endif
endif


OVERRIDE_EXECUTABLES=yes
    CC      = $(SDCC_PATH)/sdcc
    AS      = $(SDCC_PATH)/sdasstm8
    AR      = $(SDCC_PATH)/sdar
    SIZE    ?= /usr/bin/size


# avoid using the regular paths starting at $(ARDUINO_DIR)/hardware/$(VENDOR)
# as our makefile is now part of the core archive
ARDUINO_VERSION = 186
ALTERNATE_CORE    = sduino
ALTERNATE_CORE_PATH = $(ARDUINO_DIR)

ARDUINO_SKETCHBOOK	= /tmp	# temporarly, to prevent usage of the real libs
ARDMK_VENDOR	= sduino
ARCHITECTURE	= stm8
CPPFLAGS	+= -Ddouble=float -DUSE_STDINT \
	-I. -I$(ARDUINO_DIR)/STM8S_StdPeriph_Driver/inc \
	-I/opt/sdcc/share/sdcc/include/
#CFLAGS		= -I ../STM8S_StdPeriph_Driver/inc
#LDFLAGS		= --out-fmt-elf 
LDFLAGS		+= -L $(ARDUINO_DIR)/STM8S_StdPeriph_Driver/lib -L/opt/sdcc/share/sdcc/lib/stm8



# include Common.mk now we know where it is
include $(ARDMK_DIR)/Arduino.mk

# link the cpu-specific version of the ST spl
OTHER_LIBS += -l$(MCU)

STM8FLASH = stm8flash
STM8FLASH_COM_OPTS =
STM8FLASH_ARD_OPTS = -c$(AVRDUDE_ARD_PROGRAMMER) -p$(AVRDUDE_MCU)
STM8FLASH_UPLOAD_HEX = -s flash -w $(TARGET_HEX)
STM8FLASH_UPLOAD_EEP = -s eeprom -w $(TARGET_EEP)


upload: $(TARGET_HEX) verify_size
	$(STM8FLASH) $(STM8FLASH_COM_OPTS) $(STM8FLASH_ARD_OPTS) \
		$(STM8FLASH_UPLOAD_HEX)
