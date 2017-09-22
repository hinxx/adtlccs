< envPaths

errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/tlccsDemoApp.dbd")
tlccsDemoApp_registerRecordDeviceDriver(pdbbase) 

# Resource string: USB::VID::PID::SERIAL::RAW
epicsEnvSet("RSCSTR", "USB::0x1313::0x8089::M00414547::RAW")

epicsEnvSet("PREFIX", "CCS1:")
epicsEnvSet("PORT",   "CCS1")
epicsEnvSet("CAM",    "cam1:")
epicsEnvSet("QSIZE",  "20")
epicsEnvSet("XSIZE",  "3648")
epicsEnvSet("YSIZE",  "1")
epicsEnvSet("NCHANS", "2048")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

# Not for Linux?
#asynSetMinTimerPeriod(0.001)

# The EPICS environment variable EPICS_CA_MAX_ARRAY_BYTES needs to be set to a value at least as large
# as the largest image that the standard arrays plugin will send.
# That vlaue is $(XSIZE) * $(YSIZE) * sizeof(FTVL data type) for the FTVL used when loading the NDStdArrays.template file.
# The variable can be set in the environment before running the IOC or it can be set here.
# It is often convenient to set it in the environment outside the IOC to the largest array any client 
# or server will need.  For example 10000000 (ten million) bytes may be enough.
# If it is set here then remember to also set it outside the IOC for any CA clients that need to access the waveform record.  
# Do not set EPICS_CA_MAX_ARRAY_BYTES to a value much larger than that required, because EPICS Channel Access actually
# allocates arrays of this size every time it needs a buffer larger than 16K.
# Uncomment the following line to set it in the IOC.
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "10000000")

# Create a Thorlabs CCSxxx driver
# tlCCSConfig(const char *portName, int maxBuffers, size_t maxMemory, 
#             const char *resourceName, int priority, int stackSize)
tlCCSConfig("$(PORT)", 0, 0, "$(RSCSTR)", 0, 100000)
dbLoadRecords("$(ADTLCCS)/tlccsApp/Db/tlccs.template",  "P=$(PREFIX),R=$(CAM),PORT=$(PORT),ADDR=0,TIMEOUT=1,NELEMENTS=$(XSIZE)")

# Create ASCII file saving plugin
NDFileAsciiConfigure("Ascii1", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("$(ADMISC)/db/NDFileAscii.template", "P=$(PREFIX),R=Ascii1:,PORT=Ascii1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")

# Create standard arrays plugin for a trace
NDStdArraysConfigure("TRACE1", 5, 0, "$(PORT)", 0, 0)
dbLoadRecords("$(ADCORE)/ADApp/Db/NDStdArrays.template", "P=$(PREFIX),R=Trace1:,PORT=TRACE1,ADDR=0,TIMEOUT=1,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=4000,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")


## Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd

set_requestfile_path("$(TLCCS)/tlccsApp/Db")
set_requestfile_path("$(ADFILEASCII)/adfileasciiApp/Db")

#asynSetTraceIOMask("$(PORT)",0,2)
#asynSetTraceMask("$(PORT)",0,255)

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30, "P=$(PREFIX)")

