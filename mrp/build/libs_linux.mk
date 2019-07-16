# common definitions for MRP builds on Linux-like systems

# add the following line into individual user's ~/.bashrc
#  export MRP_COMMON_MK_DEFS=/home/mmitss-common/mrp/build/libs_linux.mk
# then source ~/.bashrc
# use 'echo $MRP_COMMON_MK_DEFS' from the command line to check whether MRP_COMMON_MK_DEFS has been set

MRP_DIR       := /home/mmitss-common/mrp
ASN1_DIR      := $(MRP_DIR)/asn1
J2735_DIR     := $(MRP_DIR)/asn1j2735
MAPENGINE_DIR := $(MRP_DIR)/mapEngine
TOOLS_DIR     := $(MRP_DIR)/tools
MRP_SO_DIR    := $(MRP_DIR)/lib
MRP_EXEC_DIR  := $(MRP_DIR)/bin

SOURCE_DIR    := src
HEADER_DIR    := include
OBJ_DIR       := obj
LIB_DIR       := lib

# compiler flags
MRP_CC        := gcc
MRP_C++       := g++
LOCAL_INCL    := -I/usr/$(HEADER_DIR) -I$(HEADER_DIR)
MRP_CFLAGS    := -Wall -W -Wshadow -Wcast-qual -Wwrite-strings -O3 -fno-strict-aliasing -fPIC $(LOCAL_INCL)
MRP_C++FLAGS  := -std=c++11 -Wconversion $(MRP_CFLAGS)
