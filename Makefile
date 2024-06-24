# Project Name
TARGET = FrequencyBands

# Build Project for Daisy Bootloader
APP_TYPE = BOOT_SRAM

# Sources
CPP_SOURCES = spectral_band.cpp

# Path to the root of the Aurora-SDK
# When building custom applications outside of this repo
# update this to point to the Aurora-SDK/ folder
AURORA_SDK_PATH = ../Aurora-SDK/

# Path to CMSIS-DSP and CMSIS-Core
CMSIS_PATH = ./CMSIS-DSP/
CMSIS_CORE_PATH = ./CMSIS_5/CMSIS/Core

# Include the main source files for each function category
CMSIS_SOURCES = $(CMSIS_PATH)/Source/TransformFunctions/TransformFunctions.c \
                $(CMSIS_PATH)/Source/CommonTables/CommonTables.c \
                $(CMSIS_PATH)/Source/BasicMathFunctions/BasicMathFunctions.c \
                $(CMSIS_PATH)/Source/SupportFunctions/SupportFunctions.c \
                $(CMSIS_PATH)/Source/FastMathFunctions/FastMathFunctions.c \
                $(CMSIS_PATH)/Source/FilteringFunctions/FilteringFunctions.c

C_SOURCES = $(CMSIS_SOURCES)

# Location of Hardware Support File within the SDK
C_INCLUDES += -I$(AURORA_SDK_PATH)/include/ \
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
