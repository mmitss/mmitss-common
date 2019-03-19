# common definitions for VMware builds with Savari SDK

# add the following line into individual user's ~/.bashrc
#  export SAVARI_COMMON_MK_DEFS=/home/mmitss-common/savari/build/libs_savari.mk
# then source ~/.bashrc
# can use 'echo $SAVARI_COMMON_MK_DEFS' from the command line to check whether SAVARI_COMMON_MK_DEFS has been set.

BASE_DIR      := /home/mmitss-common
MRP_DIR       := $(BASE_DIR)/mrp
ASN1_DIR      := $(MRP_DIR)/asn1
J2735_DIR	    := $(MRP_DIR)/asn1j2735
TOOLS_DIR     := $(MRP_DIR)/tools
SAVARI_DIR    := $(BASE_DIR)/savari
SAVARI_SO_DIR := $(SAVARI_DIR)/lib
SAVARI_BIN_DIR:= $(SAVARI_DIR)/bin

# Savari SDK toolchain
SDK_BASE_DIR  := /home/Savari_SDK_5.10.1.7/savari_sdk
TOOLCHAIN_DIR := $(SDK_BASE_DIR)/toolchain
V2X_SDK_DIR   := $(SDK_BASE_DIR)/v2x_sdk

# common path
SOURCE_DIR    := src
HEADER_DIR    := include
OBJ_DIR       := obj
LIB_DIR       := lib
V2X_OBJ_DIR   := obj_v2x
V2X_LIB_DIR   := lib_v2x

# compiler flags to use with Savari toolchain
V2X_CC        := $(TOOLCHAIN_DIR)/bin/arm-openwrt-linux-gcc
V2X_C++       := $(TOOLCHAIN_DIR)/bin/arm-openwrt-linux-g++
V2X_INCL      := -I$(TOOLCHAIN_DIR)/$(HEADER_DIR) -I$(V2X_SDK_DIR)/$(HEADER_DIR) -I$(HEADER_DIR)
V2X_CFLAGS    := -Wall -W -Wshadow -Wcast-qual -Wwrite-strings -O2 -fno-strict-aliasing -fPIC -I$(V2X_INCL)
V2X_C++FLAGS  := -std=c++11 -Wconversion $(V2X_CFLAGS)
