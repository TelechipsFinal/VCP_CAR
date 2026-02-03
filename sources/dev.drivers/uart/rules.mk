# SPDX-License-Identifier: Apache-2.0

###################################################################################################
#
#   FileName : ruls.mk
#
#   Copyright (c) Telechips Inc.
#
#   Description :
#
#
###################################################################################################

MCU_BSP_DEV_DRIVERS_UART_PATH := $(MCU_BSP_BUILD_CURDIR)

# Paths
VPATH += $(MCU_BSP_DEV_DRIVERS_UART_PATH)
VPATH += $(MCU_BSP_DEV_DRIVERS_UART_PATH)/$(MCU_BSP_CHIPSET_FAMILY_NAME)

# Includes
INCLUDES += -I$(MCU_BSP_DEV_DRIVERS_UART_PATH)
INCLUDES += -I$(MCU_BSP_DEV_DRIVERS_UART_PATH)/$(MCU_BSP_CHIPSET_FAMILY_NAME)

# Sources
SRCS += uart.c
<<<<<<< HEAD
#SRCS += uart_example.c
=======
SRCS += uart_example.c
>>>>>>> 42751a94e87fd7818ed9565993e97421c1eabd81
