TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += tlccs
PROD_NAME = tlccs

CXXFLAGS += -fno-operator-names

# install tlccs.dbd into <top>/dbd
DBD += $(PROD_NAME)Support.dbd

#$(warning T_A = $(T_A))
ifeq (debug,$(findstring debug, $(T_A)))
#	USR_CPPFLAGS += -D_DBG=2
	USR_CPPFLAGS += -D_DBG=1
else
	USR_CPPFLAGS += -D_DBG=0
endif
#$(warning USR_CPPFLAGS = $(USR_CPPFLAGS))

# specify all source files to be compiled and added to the library
$(PROD_NAME)_SRCS += tlccs.cpp
$(PROD_NAME)_SRCS += tlccsdrv.c
$(PROD_NAME)_SRCS += fakevisa.c
$(PROD_NAME)_SRCS += common.c

include $(ADCORE)/ADApp/commonLibraryMakefile

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

