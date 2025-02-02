################################################################################
# \file GCC_ARM.mk
#
# \brief
# GCC ARM toolchain configuration.
#
################################################################################
# \copyright
# Copyright 2018-2020 Cypress Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif

#
# The base path to the GCC cross compilation executables
#
ifeq ($(CY_COMPILER_PATH),)
CY_CROSSPATH=$(CY_COMPILER_GCC_ARM_DIR)/bin
else
CY_CROSSPATH=$(CY_COMPILER_PATH)/bin
endif

#
# Build tools
#
CC=$(CY_CROSSPATH)/arm-none-eabi-gcc
CXX=$(CY_CROSSPATH)/arm-none-eabi-g++
AS=$(CC)
AR=$(CY_CROSSPATH)/arm-none-eabi-ar
LD=$(CXX)

#
# DEBUG/NDEBUG selection
#
ifeq ($(CONFIG),Debug)
CY_TOOLCHAIN_DEBUG_FLAG=-DDEBUG
CY_TOOLCHAIN_OPTIMIZATION=-Og
else ifeq ($(CONFIG),Release)
CY_TOOLCHAIN_DEBUG_FLAG=-DNDEBUG
CY_TOOLCHAIN_OPTIMIZATION=-Os
else
CY_TOOLCHAIN_DEBUG_FLAG=
CY_TOOLCHAIN_OPTIMIZATION=
endif

#
# Flags common to compile and link
#
CY_TOOLCHAIN_COMMON_FLAGS=\
	-mthumb\
	-ffunction-sections\
	-fdata-sections\
	-ffat-lto-objects\
	-g\
	-Wall
    
#
# CPU core specifics
#
ifeq ($(CORE),CM0P)
CY_TOOLCHAIN_FLAGS_CORE=-mcpu=cortex-m0plus
CY_TOOLCHAIN_VFP_FLAGS=
else
CY_TOOLCHAIN_FLAGS_CORE=-mcpu=cortex-m4
ifeq ($(VFP_SELECT),hardfp)
CY_TOOLCHAIN_VFP_FLAGS=-mfloat-abi=hard -mfpu=fpv4-sp-d16
else
CY_TOOLCHAIN_VFP_FLAGS=-mfloat-abi=softfp -mfpu=fpv4-sp-d16
endif
endif

#
# Command line flags for c-files
#
CY_TOOLCHAIN_CFLAGS=\
	-c\
	$(CY_TOOLCHAIN_FLAGS_CORE)\
	$(CY_TOOLCHAIN_OPTIMIZATION)\
	$(CY_TOOLCHAIN_VFP_FLAGS)\
	$(CY_TOOLCHAIN_COMMON_FLAGS)

#
# Command line flags for cpp-files
#
CY_TOOLCHAIN_CXXFLAGS=\
	$(CY_TOOLCHAIN_CFLAGS)\
	-fno-rtti\
	-fno-exceptions

#
# Command line flags for s-files
#
CY_TOOLCHAIN_ASFLAGS=\
	-c\
	$(CY_TOOLCHAIN_FLAGS_CORE)\
	$(CY_TOOLCHAIN_COMMON_FLAGS)

#
# Command line flags for linking
#
CY_TOOLCHAIN_LDFLAGS=\
	$(CY_TOOLCHAIN_FLAGS_CORE)\
	$(CY_TOOLCHAIN_VFP_FLAGS)\
	$(CY_TOOLCHAIN_COMMON_FLAGS)\
	--enable-objc-gc\
	--specs=nano.specs\
	-Wl,--gc-sections

#
# Command line flags for archiving
#
CY_TOOLCHAIN_ARFLAGS=rvs

#
# Toolchain-specific suffixes
#
CY_TOOLCHAIN_SUFFIX_S=S
CY_TOOLCHAIN_SUFFIX_s=s
CY_TOOLCHAIN_SUFFIX_C=c
CY_TOOLCHAIN_SUFFIX_H=h
CY_TOOLCHAIN_SUFFIX_CPP=cpp
CY_TOOLCHAIN_SUFFIX_HPP=hpp
CY_TOOLCHAIN_SUFFIX_O=o
CY_TOOLCHAIN_SUFFIX_A=a
CY_TOOLCHAIN_SUFFIX_D=d
CY_TOOLCHAIN_SUFFIX_LS=ld
CY_TOOLCHAIN_SUFFIX_MAP=map
CY_TOOLCHAIN_SUFFIX_TARGET=elf
CY_TOOLCHAIN_SUFFIX_ARCHIVE=a

#
# Toolchain specific flags
#
CY_TOOLCHAIN_OUTPUT_OPTION=-o
CY_TOOLCHAIN_MAPFILE=-Wl,-Map,
CY_TOOLCHAIN_STARTGROUP=-Wl,--start-group
CY_TOOLCHAIN_ENDGROUP=-Wl,--end-group
CY_TOOLCHAIN_LSFLAGS=-T
CY_TOOLCHAIN_INCRSPFILE=@
CY_TOOLCHAIN_INCRSPFILE_ASM=@
CY_TOOLCHAIN_OBJRSPFILE=@

#
# Produce a makefile dependency rule for each input file
#
CY_TOOLCHAIN_DEPENDENCIES=-MMD -MP -MF "$(subst .$(CY_TOOLCHAIN_SUFFIX_O),.$(CY_TOOLCHAIN_SUFFIX_D),$@)" -MT "$@"
CY_TOOLCHAIN_EXPLICIT_DEPENDENCIES=-MMD -MP -MF "$$(subst .$(CY_TOOLCHAIN_SUFFIX_O),.$(CY_TOOLCHAIN_SUFFIX_D),$$@)" -MT "$$@"

#
# Additional includes in the compilation process based on this
# toolchain
#
CY_TOOLCHAIN_INCLUDES=

#
# Additional libraries in the link process based on this toolchain
#
CY_TOOLCHAIN_DEFINES=

