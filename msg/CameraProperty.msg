# Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# Types
uint8 V4L2_CTRL_TYPE_INTEGER = 1
uint8 V4L2_CTRL_TYPE_BOOLEAN = 2
uint8 V4L2_CTRL_TYPE_MENU = 3
uint8 V4L2_CTRL_TYPE_BUTTON = 4
uint8 V4L2_CTRL_TYPE_INTEGER64 = 5
uint8 V4L2_CTRL_TYPE_CTRL_CLASS = 6
uint8 V4L2_CTRL_TYPE_STRING = 7
uint8 V4L2_CTRL_TYPE_BITMASK = 8
uint8 V4L2_CTRL_TYPE_INTEGER_MENU = 9
# End of types
int32 index
string name
uint8 type
int64 default_value
