#
# Makefile for a Loadable User Module (LEGO Mindstorms EV3)
#

# Specify the directory of HRP2 source
KERNELDIR = ../..
TARGETDIR = $(KERNELDIR)/target/ev3_gcc
LOADERDIR = $(KERNELDIR)/target/ev3_gcc/dmloader
SRCDIRS = ../ETR2015_L_normal_ref $(KERNELDIR)/target/ev3_gcc/api/src $(KERNELDIR)/target/ev3_gcc/TLSF-2.4.6/src

# Compiler options
COPTS += -DBUILD_MODULE
GCC_TARGET = arm-none-eabi
INCLUDES += -I../ETR2015_L_normal_ref \
			-I../ETR2015_L_normal_ref/../common \
			-I$(LOADERDIR)/app \
			-I$(KERNELDIR)/target/ev3_gcc \
		   	-I$(KERNELDIR)/target/ev3_gcc/api/include \
			-I$(KERNELDIR)/target/ev3_gcc/drivers/common/include \
			-I$(KERNELDIR)/arch/arm_gcc/am1808 \
			-I$(KERNELDIR)/arch/arm_gcc/common \
		   	-I$(KERNELDIR)/target/ev3_gcc/TLSF-2.4.6/include \
		   	-I$(KERNELDIR)/target/ev3_gcc/platform/include \
		   	-I$(KERNELDIR)/target/ev3_gcc/pil/include


OBJNAME = app

OBJFILE = $(OBJNAME)

all: $(OBJFILE)

MODOBJS += t_perror.o strerror.o vasyslog.o tlsf.o

MODCFG = ../ETR2015_L_normal_ref/app.cfg

MODDIR = $(PWD)

OMIT_DEBUG_INFO = 1

#
# Include common part
#
BUILD_LOADABLE_MODULE = 1
CONFIG_EV3RT_APPLICATION = 1
WORKSPACE_LIB_DIR := ../common/lib
include ../ETR2015_L_normal_ref/Makefile.inc
ifeq ($(SRCLANG),c++)
  USE_CXX = true
  APPL_CXXOBJS += app.o 
  CXXLIBS = -lstdc++ -lc -lm -lgcc  
  #CXXRTS = crtbegin.o crtend.o
  #CXXRTS = cxxrt.o newlibrt.o
else
  MODOBJS += app.o 
endif
include $(KERNELDIR)/target/ev3_gcc/api/Makefile
MODOBJS += $(APPL_COBJS)

include $(LOADERDIR)/app/Makefile.lum
