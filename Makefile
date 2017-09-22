#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure
DIRS += tlccsApp

ifeq ($(BUILD_IOCS), YES)
DIRS += tlccsDemoApp
tlccsDemoApp_DEPEND_DIRS += tlccsApp
iocBoot_DEPEND_DIRS += tlccsDemoApp
DIRS += iocBoot
endif

include $(TOP)/configure/RULES_TOP
