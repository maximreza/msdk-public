# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration

# **********************************************************

# Enable CORDIO library
LIB_CORDIO = 1

# Optimize for size
MXC_OPTIMIZE_CFLAGS = -Os

USE_DUAL_CORE = 0
ifeq "$(USE_DUAL_CORE)" "1"
BLE_HOST = 1
BLE_CONTROLLER = 0
endif