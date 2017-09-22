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
DBD += tlccsSupport.dbd

# specify all source files to be compiled and added to the library
#tlccs_SRCS += xxx
$(PROD_NAME)_SRCS += tlccsApp_registerRecordDeviceDriver.cpp tlccsAppMain.cpp
$(PROD_NAME)_SRCS += tlccs.cpp
$(PROD_NAME)_SRCS += tlccsdrv.c
$(PROD_NAME)_SRCS += fakevisa.c
$(PROD_NAME)_SRCS += common.c

# CCS library and includes are here
#USR_INCLUDES += -I$(TLCCSDRV)/src
#USR_LDFLAGS += -L$(TLCCSDRV)/src

#$(PROD_NAME)_LIBS += $(EPICS_BASE_IOC_LIBS)
include $(ADCORE)/ADApp/commonLibraryMakefile

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

