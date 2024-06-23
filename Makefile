# Project Name
TARGET = Blink

# Build Project for Daisy Bootloader
APP_TYPE = BOOT_SRAM

# Sources
#CPP_SOURCES = Blink.cpp
#CPP_SOURCES = Blink.cpp kissfft/kiss_fft.c kissfft/kiss_fftr.c
CPP_SOURCES = spectral_band.cpp
C_SOURCES = kissfft/kiss_fft.c kissfft/kiss_fftr.c

# Path to the root of the Aurora-SDK
# When building custom applications outside of this repo
# update this to point to the Aurora-SDK/ folder
AURORA_SDK_PATH = ../Aurora-SDK/

CMSIS_PATH = ./CMSIS-DSP/
CMSIS_CORE_PATH = ./CMSIS_5/CMSIS/Core

CMSIS_SOURCES = $(CMSIS_PATH)/Source/TransformFunctions/arm_cfft_radix4_f32.c \
                $(CMSIS_PATH)/Source/CommonTables/arm_common_tables.c \
                $(CMSIS_PATH)/Source/BasicMathFunctions/arm_mult_f32.c
C_SOURCES += $(CMSIS_SOURCES)

# Location of Hardware Support File within the SDK
#C_INCLUDES += -I$(AURORA_SDK_PATH)/include/
C_INCLUDES += -I$(AURORA_SDK_PATH)/include/ \
              -Ikissfft/ \
              -I$(CMSIS_PATH)/Include \
              -I$(CMSIS_PATH)/PrivateInclude \
              -I$(CMSIS_CORE_PATH)/Include

# Library Locations
LIBDAISY_DIR = $(AURORA_SDK_PATH)/libs/libDaisy/
DAISYSP_DIR = $(AURORA_SDK_PATH)/libs/DaisySP/

# To DEBUG the project with an ST-Link Probe:
# 1. Compile the program with the below lines uncommented
# 2. Load the firmware via the USB drive
# 3. Make sure your .vscode/launch.json points to the 
#	 build/*.elf for the desired program
# 4. Navigate and run the "Cortex Debug" Run and Debug configuration
#    or simply press F5 in VS Code.

# DEBUG = 1
# OPT = -O0

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
