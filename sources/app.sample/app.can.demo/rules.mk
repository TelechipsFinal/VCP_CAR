<<<<<<< HEAD
# app/sample/can_demo/rules.mk
=======
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
>>>>>>> 42751a94e87fd7818ed9565993e97421c1eabd81

MCU_BSP_APP_SAMPLE_CAN_DEMO_PATH := $(MCU_BSP_BUILD_CURDIR)

# Flags
COMMON_FLAGS += -DMCU_BSP_SUPPORT_CAN_DEMO=1
<<<<<<< HEAD
COMMON_FLAGS += -DMCU_BSP_SUPPORT_CAN_MSG_HANDLER=1  # 추가
=======
>>>>>>> 42751a94e87fd7818ed9565993e97421c1eabd81

# Paths
VPATH += $(MCU_BSP_APP_SAMPLE_CAN_DEMO_PATH)

# Include
INCLUDES += -I$(MCU_BSP_APP_SAMPLE_CAN_DEMO_PATH)

# Sources
SRCS += can_demo.c
<<<<<<< HEAD
SRCS += can_msg_handler.c  # 추가
=======

>>>>>>> 42751a94e87fd7818ed9565993e97421c1eabd81
