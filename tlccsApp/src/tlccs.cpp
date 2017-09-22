/**
 * Area Detector driver for the Thorlabs CCS.
 *
 * @author Hinko Kocevar
 * @date January 2015
 *
 * Based on andorCCD driver.
 *
 */

#include <stdio.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsExit.h>

#define NIVISA_USB   // this is necessary to access NI-VISA-USB
#include <visa.h>
#include "tlccs.h"

static const char *driverName = "tlccs";

//Definitions of static class data members

const epicsUInt32 TlCCS::TLWavelengthTargetFactory = 0;
const epicsUInt32 TlCCS::TLWavelengthTargetUser = 1;

const epicsUInt32 TlCCS::TLAmplitudeCorrectionModeCurrent = 1;
const epicsUInt32 TlCCS::TLAmplitudeCorrectionModeNVMem = 2;

const epicsUInt32 TlCCS::TLAmplitudeCorrectionTargetFactory = 0;
const epicsUInt32 TlCCS::TLAmplitudeCorrectionTargetUser = 1;
const epicsUInt32 TlCCS::TLAmplitudeCorrectionTargetThorlabs = 2;

const epicsUInt32 TlCCS::TLASingle = 0;
const epicsUInt32 TlCCS::TLACont = 1;
const epicsUInt32 TlCCS::TLAExtTrig = 2;
const epicsUInt32 TlCCS::TLAContExtTrig = 3;

const epicsUInt32 TlCCS::TLARaw = 0;
const epicsUInt32 TlCCS::TLAProcessed = 1;

const epicsUInt32 TlCCS::TLSScanIdle = TLCCS_STATUS_SCAN_IDLE;
const epicsUInt32 TlCCS::TLSScanTriggered = TLCCS_STATUS_SCAN_TRIGGERED;
const epicsUInt32 TlCCS::TLSScanStartTrans = TLCCS_STATUS_SCAN_START_TRANS;
const epicsUInt32 TlCCS::TLSScanTransfer = TLCCS_STATUS_SCAN_TRANSFER;
const epicsUInt32 TlCCS::TLSWaitForExtTrigger = TLCCS_STATUS_WAIT_FOR_EXT_TRIG;

const epicsInt32 TlCCS::AFFRAW  = 0;
const epicsInt32 TlCCS::AFFBMP  = 1;

//C Function prototypes to tie in with EPICS
static void tlDeviceTaskC(void *drvPvt);
static void tlDataTaskC(void *drvPvt);
static void exitHandler(void *drvPvt);

/** Constructor for Thorlabs driver; most parameters are simply passed to ADDriver::ADDriver.
 * After calling the base class constructor this method creates a thread to collect the detector data,
 * and sets reasonable default values the parameters defined in this class, asynNDArrayDriver, and ADDriver.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
 * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
 * \param[in] firmwarePath The path to the Thorlabs directory containing the spectrometer firmware files.
 * \param[in] resourceName The resource string that describes Thorlabs instrument (eg. USB::0x1313::0x8081::M00310589::RAW).
 * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
 * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
 */
TlCCS::TlCCS(const char *portName, int maxBuffers, size_t maxMemory,
		const char *resourceName, int priority, int stackSize)

:
		ADDriver(portName, 1, NUM_TL_DET_PARAMS, maxBuffers, maxMemory,
		asynEnumMask | asynFloat64ArrayMask | asynInt32ArrayMask,
		asynEnumMask | asynFloat64ArrayMask | asynInt32ArrayMask,
		ASYN_CANBLOCK, 1, priority, stackSize) {

	int status = asynSuccess;
	int sizeX, sizeY;
	static const char *functionName = "TlCCS";

	sizeX = TLCCS_NUM_PIXELS;
	sizeY = 3;
	strncpy(mResourceName, resourceName, TLCCS_BUFFER_SIZE);

	/* Create an EPICS exit handler */
	epicsAtExit(exitHandler, this);

	createParam(TlMessageString, asynParamOctet, &TlMessage);
	createParam(TlWavelengthDataString, asynParamFloat64Array, &TlWavelengthData);
	createParam(TlWavelengthDataTargetString, asynParamInt32, &TlWavelengthDataTarget);
	createParam(TlWavelengthDataSetString, asynParamInt32, &TlWavelengthDataSet);
	createParam(TlWavelengthDataGetString, asynParamInt32, &TlWavelengthDataGet);
	createParam(TlWavelengthArrayString, asynParamFloat64Array, &TlWavelengthArray);
	createParam(TlWavelengthPixelArrayString, asynParamInt32Array, &TlWavelengthPixelArray);
	createParam(TlWavelengthMinimumString, asynParamFloat64, &TlWavelengthMinimum);
	createParam(TlWavelengthMaximumString, asynParamFloat64, &TlWavelengthMaximum);
	createParam(TlUserCalibrationPointsString, asynParamOctet, &TlUserCalibrationPoints);
	createParam(TlAmplitudeDataString, asynParamFloat64Array, &TlAmplitudeData);
	createParam(TlAmplitudeDataModeString, asynParamInt32, &TlAmplitudeDataMode);
	createParam(TlAmplitudeDataTargetString, asynParamInt32,&TlAmplitudeDataTarget);
	createParam(TlAmplitudeDataSetString, asynParamInt32, &TlAmplitudeDataSet);
	createParam(TlAmplitudeDataGetString, asynParamInt32, &TlAmplitudeDataGet);
	createParam(TlAcquisitionTypeString, asynParamInt32, &TlAcquisitionType);

	// Use this to signal the data acquisition task that acquisition has started.
	this->mDataEvent = epicsEventMustCreate(epicsEventEmpty);
	if (!this->mDataEvent) {
		printf("%s:%s epicsEventCreate failure for data event\n", driverName, functionName);
		return;
	}
	// Use this to signal the device task to unblock.
	this->mDeviceEvent = epicsEventMustCreate(epicsEventEmpty);
	if (!this->mDeviceEvent) {
		printf("%s:%s epicsEventCreate failure for device event\n", driverName, functionName);
		return;
	}

	// addVisaDevice() will try to find and initialize spectrometer
	mInstr = VI_NULL;

	/* Set some default values for parameters */

	status = setStringParam(ADManufacturer, mManufacturerName);
	status |= setStringParam(ADModel, mDeviceName);
	status |= setIntegerParam(ADSizeX, sizeX);
	status |= setIntegerParam(ADSizeY, sizeY);
	status |= setIntegerParam(ADBinX, 1);
	status |= setIntegerParam(ADBinY, 1);
	status |= setIntegerParam(ADMinX, 0);
	status |= setIntegerParam(ADMinY, 0);
	status |= setIntegerParam(ADMaxSizeX, sizeX);
	status |= setIntegerParam(ADMaxSizeY, sizeY);
	status |= setIntegerParam(ADImageMode, ADImageSingle);
	mIntegrationTime = TLCCS_DEF_INT_TIME;
	status |= setDoubleParam(ADAcquireTime, mIntegrationTime);
	status |= setIntegerParam(ADNumImages, 1);
	status |= setIntegerParam(ADNumExposures, 1);
	status |= setIntegerParam(NDArraySizeX, sizeX);
	status |= setIntegerParam(NDArraySizeY, sizeY);
	status |= setIntegerParam(NDDataType, NDFloat64);
	status |= setIntegerParam(NDArraySize, sizeX * sizeY * sizeof(epicsFloat64));
	status |= setDoubleParam(ADShutterOpenDelay, 0.);
	status |= setDoubleParam(ADShutterCloseDelay, 0.);

	status |= setIntegerParam(TlAmplitudeDataMode, TLAmplitudeCorrectionModeCurrent);
	status |= setIntegerParam(TlAmplitudeDataTarget, TLAmplitudeCorrectionTargetUser);
	status |= setIntegerParam(TlWavelengthDataTarget, TLWavelengthTargetUser);
	status |= setIntegerParam(TlWavelengthDataSet, 0);
	status |= setIntegerParam(TlWavelengthDataGet, 0);
	status |= setDoubleParam(TlWavelengthMinimum, 0);
	status |= setDoubleParam(TlWavelengthMaximum, 0);
	status |= setIntegerParam(TlAmplitudeDataSet, 0);
	status |= setIntegerParam(TlAmplitudeDataGet, 0);
	status |= setIntegerParam(TlAcquisitionType, TLAProcessed);

	memset(mWavelengthData, 0, TLCCS_NUM_PIXELS);
	memset(mWavelengthArray, 0, TLCCS_MAX_NUM_USR_ADJ);
	memset(mWavelengthPixelArray, 0, TLCCS_MAX_NUM_USR_ADJ);
	memset(mAmplitudeData, 0, TLCCS_NUM_PIXELS);

	callParamCallbacks();

	if (status) {
		printf("%s:%s: unable to set spectrometer parameters\n", driverName, functionName);
		return;
	}

	mAcquiringData = 0;

	if (stackSize == 0) {
		stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);
	}

	/* for stopping threads */
	mFinish = 0;

	/* Create the thread that does USB device detection */
	status = (epicsThreadCreate("TlDeviceTask",
			epicsThreadPriorityMedium, stackSize, (EPICSTHREADFUNC) tlDeviceTaskC, this) == NULL);
	if (status) {
		printf("%s:%s: epicsThreadCreate failure for device task\n", driverName, functionName);
		return;
	}

	/* Create the thread that does data readout */
	status = (epicsThreadCreate("TlDataTask",
			epicsThreadPriorityMedium, stackSize, (EPICSTHREADFUNC) tlDataTaskC, this) == NULL);
	if (status) {
		printf("%s:%s: epicsThreadCreate failure for data task\n", driverName, functionName);
		return;
	}

	printf("CCS initialized OK!\n");
}

/**
 * Destructor.  Free resources and closes the Thorlabs library
 */
TlCCS::~TlCCS() {
	printf("Shutdown and freeing up memory...\n");
	this->lock();
	/* make sure threads are cleanly stopped */
	printf("Waiting for threads...\n");
	this->mFinish = 1;
	epicsEventSignal(mDataEvent);
	epicsEventSignal(mDeviceEvent);
	sleep(1);
	printf("Threads are down!\n");

	removeVisaDevice();

	this->unlock();
}

/**
 * Add USB device.
 */
void TlCCS::addVisaDevice(void) {

	ViChar *rscPtr = NULL;
	ViSession instr = VI_NULL;
	static const char *functionName = "addVisaDevice";

	try {
		rscPtr = mResourceName;
		printf("%s:%s: trying to open session to '%s' ...\n\n", driverName, functionName, rscPtr);
		checkStatus(tlccs_init(rscPtr, VI_OFF, VI_OFF, &instr));

		mInstr = instr;

		// set integration time
		checkStatus(tlccs_setIntegrationTime(mInstr, TLCCS_DEF_INT_TIME));

		checkStatus(tlccs_identificationQuery(mInstr,
				mManufacturerName,
				mDeviceName,
				mSerialNumber,
				mFirmwareRevision,
				mInstrumentDriverRevision));

	} catch (const std::string &e) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n", driverName,
				functionName, e.c_str());
		return;
	}

	setStringParam(ADManufacturer, mManufacturerName);
	setStringParam(ADModel, mDeviceName);
	// XXX: Uncomment once the serial number support is in areaDetector
	//setStringParam(ADSerial, mSerialNumber);
	setStringParam(TlMessage, "USB device is attached!");
	callParamCallbacks();

	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
			"%s:%s: Spectrometer '%s' successfully initialized\n", driverName, functionName, mResourceName);
}

/**
 * Remove USB device.
 */
void TlCCS::removeVisaDevice(void) {
	static const char *functionName = "removeVisaDevice";

	try {
		/* Close instrument handle if opened */
		printf("closing device..\n");
		if (mInstr != VI_NULL) {
			/* do not use checkStatus() as we can enter infinite loop! */
			tlccs_close(mInstr);
			mInstr = VI_NULL;
		}
	} catch (const std::string &e) {
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n", driverName,
				functionName, e.c_str());
		return;
	}

	setStringParam(TlMessage, "USB device is detached!");
	callParamCallbacks();

	asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
			"%s:%s: Spectrometer successfully shutdown\n", driverName, functionName);
}

/**
 * Exit handler, delete the TlCCS object.
 */
static void exitHandler(void *drvPvt) {
	TlCCS *pTlCCS = (TlCCS *) drvPvt;
	delete pTlCCS;
}

/** Report status of the driver.
 * Prints details about the detector in us if details>0.
 * It then calls the ADDriver::report() method.
 * \param[in] fp File pointed passed by caller where the output is written to.
 * \param[in] details Controls the level of detail in the report. */
void TlCCS::report(FILE *fp, int details) {
	static const char *functionName = "report";

	fprintf(fp, "Thorlabs CCS port=%s\n", this->portName);
	if (details > 0) {
		try {
			checkStatus(tlccs_identificationQuery(mInstr, mManufacturerName,
							mDeviceName, mSerialNumber, mFirmwareRevision,
							mInstrumentDriverRevision));
			fprintf(fp, "  Manufacturer: %s\n", mManufacturerName);
			fprintf(fp, "  Model: %s\n", mDeviceName);
			fprintf(fp, "  Serial number: %s\n", mSerialNumber);
			fprintf(fp, "  Driver version: %s\n", mInstrumentDriverRevision);
			fprintf(fp, "  Firmware version: %s\n", mFirmwareRevision);

		} catch (const std::string &e) {
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n",
					driverName, functionName, e.c_str());
		}
	}
	// Call the base class method
	ADDriver::report(fp, details);
}

/** Called when asyn clients call pasynInt32->write().
 * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus TlCCS::writeInt32(asynUser *pasynUser, epicsInt32 value) {
	int function = pasynUser->reason;
	int adstatus = 0;
	double dummy;
	int wavelengthTarget;
	double minWavelength, maxWavelength;
	int ampCorrectionMode, ampCorrectionTarget;
	int bufferLength = -1;
	int bufferStart = -1;

	asynStatus status = asynSuccess;
	static const char *functionName = "writeInt32";

	//Set in param lib so the user sees a readback straight away. Save a backup in case of errors.
	status = setIntegerParam(function, value);

	if (function == ADAcquire) {
		getIntegerParam(ADStatus, &adstatus);
		if (value && (adstatus == ADStatusIdle)) {
			try {
				mAcquiringData = 1;
				//We send an event at the bottom of this function.
			} catch (const std::string &e) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n",
						driverName, functionName, e.c_str());
				status = asynError;
			}
		}
		if (!value && (adstatus != ADStatusIdle)) {
			try {
				/* According to the comment in tlccs.h this should cancel the scan
				 * in progress. */
				asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
						"%s:%s:, Aborting acquisition by calling tlccs_setIntegrationTime()..\n",
						driverName, functionName);
				checkStatus(tlccs_getIntegrationTime(mInstr, &dummy));
				mAcquiringData = 0;
			} catch (const std::string &e) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n",
						driverName, functionName, e.c_str());
				status = asynError;
			}
		}
	} else if (function == TlWavelengthDataGet) {
		if (value) {
			try {
				getIntegerParam(TlWavelengthDataTarget, &wavelengthTarget);
				checkStatus(tlccs_getWavelengthData(mInstr, wavelengthTarget,
						mWavelengthData, &minWavelength, &maxWavelength));
				setDoubleParam(TlWavelengthMinimum, minWavelength);
				setDoubleParam(TlWavelengthMaximum, maxWavelength);
			} catch (const std::string &e) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n",
						driverName, functionName, e.c_str());
				status = asynError;
			}
		}
	} else if (function == TlWavelengthDataSet) {
		if (value) {
			try {
				getIntegerParam(TlWavelengthDataTarget, &wavelengthTarget);
				bufferLength = 10;
				/* See tlccs_setWavelengthData() for more information about the bufferLength
				 * and how target selection affects the value. */
				switch (wavelengthTarget) {
				case TLWavelengthTargetFactory:
					bufferLength += 62749006;
					break;
				case TLAmplitudeCorrectionTargetUser:
					// bufferLength is 10
					break;
				}

				checkStatus(tlccs_setWavelengthData(mInstr, mWavelengthPixelArray,
								mWavelengthArray, bufferLength));

			} catch (const std::string &e) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n",
						driverName, functionName, e.c_str());
				status = asynError;
			}
		}
	} else if (function == TlAmplitudeDataGet) {
		getIntegerParam(TlAmplitudeDataMode, &ampCorrectionMode);
		getIntegerParam(TlAmplitudeDataTarget, &ampCorrectionTarget);
		bufferStart = 0;
		/* See tlccs_setAmplitudeData() for more information about the bufferStart
		 * and how target selection affect the value. */
		switch (ampCorrectionTarget) {
		case TLAmplitudeCorrectionTargetFactory:
			bufferStart += 19901201;
			break;
		case TLAmplitudeCorrectionTargetUser:
			// bufferStart is at 0
			break;
		case TLAmplitudeCorrectionTargetThorlabs:
			bufferStart += 91901201;
			break;
		}
		checkStatus(tlccs_getAmplitudeData(mInstr, mAmplitudeData, bufferStart, TLCCS_NUM_PIXELS, ampCorrectionMode));
	} else if (function == TlAmplitudeDataSet) {
		if (value) {
			try {
				getIntegerParam(TlAmplitudeDataMode, &ampCorrectionMode);
				getIntegerParam(TlAmplitudeDataTarget, &ampCorrectionTarget);
				bufferStart = 0;
				/* See tlccs_setAmplitudeData() for more information about the bufferStart
				 * and how target selection affects the value. */
				switch (ampCorrectionTarget) {
				case TLAmplitudeCorrectionTargetFactory:
					bufferStart += 19901201;
					break;
				case TLAmplitudeCorrectionTargetUser:
					// bufferStart is at 0
					break;
				case TLAmplitudeCorrectionTargetThorlabs:
					bufferStart += 91901201;
					break;
				}
				checkStatus(tlccs_setAmplitudeData(mInstr, mAmplitudeData, TLCCS_NUM_PIXELS, bufferStart, ampCorrectionMode));
			} catch (const std::string &e) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n",
						driverName, functionName, e.c_str());
				status = asynError;
			}
		}
	}

	else {
		status = ADDriver::writeInt32(pasynUser, value);
	}

	if (status == asynSuccess) {
		//For a successful write, clear the error message.
		setStringParam(TlMessage, " ");
	}

	/* Do callbacks so higher layers see any changes */
	callParamCallbacks();

	if (mAcquiringData) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
				"%s:%s:, Sending mDataEvent to dataTask ...\n", driverName,
				functionName);
		//Also signal the data readout thread
		epicsEventSignal(mDataEvent);
	}

	if (status)
		asynPrint(pasynUser, ASYN_TRACE_ERROR,
				"%s:%s: error, status=%d function=%d, value=%d\n", driverName,
				functionName, status, function, value);
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
				"%s:%s: function=%d, value=%d\n", driverName, functionName,
				function, value);
	return status;
}

/** Called when asyn clients call pasynFloat64->write().
 * This function performs actions for some parameters.
 * For all parameters it sets the value in the parameter library and calls any registered callbacks.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus TlCCS::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	static const char *functionName = "writeFloat64";

	/* Set the parameter and readback in the parameter library.  */
	status = setDoubleParam(function, value);

	if (function == ADAcquireTime) {
		if ((value <= TLCCS_MAX_INT_TIME) && (value >= TLCCS_MIN_INT_TIME)) {
			mIntegrationTime = value;
			try {
				checkStatus(tlccs_setIntegrationTime(mInstr, mIntegrationTime));
				asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
						"%s:%s:, tlccs_setIntegrationTime(%f)\n", driverName,
						functionName, value);
			} catch (const std::string &e) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n",
						driverName, functionName, e.c_str());
				status = asynError;
			}
		} else {
			asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s: Invalid integration time supplied (%f)\n",
					driverName, functionName, value);
			status = asynError;
		}
	}

	else {
		status = ADDriver::writeFloat64(pasynUser, value);
	}

	if (status == asynSuccess) {
		//For a successful write, clear the error message.
		setStringParam(TlMessage, " ");
	}

	/* Do callbacks so higher layers see any changes */
	callParamCallbacks();
	if (status)
		asynPrint(pasynUser, ASYN_TRACE_ERROR,
				"%s:%s: error, status=%d function=%d, value=%f\n", driverName,
				functionName, status, function, value);
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
				"%s:%s: function=%d, value=%f\n", driverName, functionName,
				function, value);
	return status;
}

/** Called when asyn clients call pasynFloat64Array->write().
 * This function performs actions for some parameters.
 * For all parameters it sets the value in the parameter library and calls any registered callbacks.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write.
 * \param[in] nElements Number of values to write. */
asynStatus TlCCS::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
		size_t nElements) {

	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	static const char *functionName = "writeFloat64Array";

	if (function == TlAmplitudeData) {
		status = asynError;
		if (nElements == TLCCS_NUM_PIXELS) {
			memcpy(mAmplitudeData, value, nElements * sizeof(epicsFloat64));
			status = asynSuccess;
		}
	} else if (function == TlWavelengthArray) {
		status = asynError;
		if (nElements == TLCCS_MAX_NUM_USR_ADJ) {
			memcpy(mWavelengthArray, value, nElements * sizeof(epicsFloat64));
			status = asynSuccess;
		}
	}

	else {
		status = ADDriver::writeFloat64Array(pasynUser, value, nElements);
	}

	/* Do callbacks so higher layers see any changes */
	callParamCallbacks();

	if (status)
		asynPrint(pasynUser, ASYN_TRACE_ERROR,
				"%s:%s: error, status=%d function=%d, nElements=%d\n",
				driverName, functionName, status, function, (int )nElements);
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
				"%s:%s: error, status=%d function=%d, nElements=%d\n",
				driverName, functionName, status, function, (int )nElements);
	return status;
}

/** Called when asyn clients call pasynFloat64Array->read().
 * This function performs actions for some parameters.
 * For all parameters it gets the value in the parameter library.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to read.
 * \param[in] nElements Number of values to read.
 * \param[in] nIn Number of values read. */
asynStatus TlCCS::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
		size_t nElements, size_t *nIn) {

	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	static const char *functionName = "readFloat64Array";

	if (function == TlAmplitudeData) {
		status = asynError;
		if (nElements == TLCCS_NUM_PIXELS) {
			memcpy(value, mAmplitudeData, nElements * sizeof(epicsFloat64));
			*nIn = TLCCS_NUM_PIXELS;
			status = asynSuccess;
		}
	} else if (function == TlWavelengthData) {
		status = asynError;
		if (nElements == TLCCS_NUM_PIXELS) {
			memcpy(value, mWavelengthData, nElements * sizeof(epicsFloat64));
			*nIn = TLCCS_NUM_PIXELS;
			status = asynSuccess;
		}
	} else if (function == TlWavelengthArray) {
		status = asynError;
		if (nElements == TLCCS_MAX_NUM_USR_ADJ) {
			memcpy(value, mWavelengthArray, nElements * sizeof(epicsFloat64));
			*nIn = TLCCS_MAX_NUM_USR_ADJ;
			status = asynSuccess;
		}
	}

	else {
		status = ADDriver::readFloat64Array(pasynUser, value, nElements, nIn);
	}

	if (status)
		asynPrint(pasynUser, ASYN_TRACE_ERROR,
				"%s:%s: error, status=%d function=%d, nElements=%d\n",
				driverName, functionName, status, function, (int )nElements);
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
				"%s:%s: error, status=%d function=%d, nElements=%d\n",
				driverName, functionName, status, function, (int )nElements);
	return status;
}

/** Called when asyn clients call pasynInt32Array->write().
 * This function performs actions for some parameters.
 * For all parameters it sets the value in the parameter library and calls any registered callbacks.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write.
 * \param[in] nElements Number of values to write. */
asynStatus TlCCS::writeInt32Array(asynUser *pasynUser, epicsInt32 *value,
		size_t nElements) {

	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	static const char *functionName = "writeInt32Array";

	if (function == TlWavelengthPixelArray) {
		//status = asynError;
		if (nElements == TLCCS_MAX_NUM_USR_ADJ) {
			memcpy(mWavelengthPixelArray, value, nElements * sizeof(epicsInt32));
			status = asynSuccess;
		}
	}

	else {
		status = ADDriver::writeInt32Array(pasynUser, value, nElements);
	}

	/* Do callbacks so higher layers see any changes */
	callParamCallbacks();

	if (status)
		asynPrint(pasynUser, ASYN_TRACE_ERROR,
				"%s:%s: error, status=%d function=%d, nElements=%d\n",
				driverName, functionName, status, function, (int )nElements);
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
				"%s:%s: error, status=%d function=%d, nElements=%d\n",
				driverName, functionName, status, function, (int )nElements);
	return status;
}

/**
 * Function to check the return status of Thorlabs SDK library functions.
 * @param returnStatus The return status of the SDK function
 * @return 0=success. Does not return in case of failure.
 * @throw std::string An exception is thrown in case of failure.
 */
unsigned int TlCCS::checkStatus(ViStatus returnStatus) {
	ViChar ebuf[TLCCS_ERR_DESCR_BUFFER_SIZE];

	if (returnStatus == VI_SUCCESS) {
		return 0;
	} else {
		// Print error
		memset(ebuf, 0, sizeof(ebuf));
		tlccs_error_message(mInstr, returnStatus, ebuf);
		// Set the error for user to see
		setStringParam(TlMessage, ebuf);

		// Close the device if it was removed (USB cable unplugged)
		if ((returnStatus == VI_USB_PIPE_STATE_UNKNOWN) ||
			(returnStatus == VI_ERROR_INV_OBJECT) ||
			(returnStatus == VI_ERROR_RSRC_NFOUND)) {
			this->lock();

			removeVisaDevice();

			this->unlock();
		}

		throw std::string(ebuf);
	}

	return 0;
}

void TlCCS::deviceTask(void) {
	static const char *functionName = "deviceTask";

	epicsEventWaitWithTimeout(mDeviceEvent, 2.0);

	while (1) {

		if (this->mFinish) {
			asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
					"%s:%s: Stopping thread!\n", driverName, functionName);
			break;
		}

		this->lock();

		// Try to add USB device if not yet present
		if (mInstr == VI_NULL) {
			addVisaDevice();
		}

		this->unlock();

		epicsEventWaitWithTimeout(mDeviceEvent, 2.0);
	}

	printf("Device thread is down!\n");
}

/**
 * Do data readout from the detector. Meant to be run in own thread.
 */
void TlCCS::dataTask(void) {
	int acquireStatus;
	int acquireStatus2;
	int acquiring = 0;
	epicsInt32 numImagesCounter;
	epicsInt32 numExposuresCounter;
	epicsInt32 imageCounter;
	epicsInt32 arrayCallbacks;
	epicsInt32 sizeX, sizeY;
	NDDataType_t dataType;
	int i;
	size_t dims[2];
	int nDims = 2;
	epicsTimeStamp startTime;
	NDArray *pArray;
	int imageMode;
	int triggerMode;
	int acqType;
	struct timeval tv;
	epicsInt32 rawBuf[TLCCS_NUM_RAW_PIXELS];
	static const char *functionName = "dataTask";

	sleep(3);

	printf("%s:%s: Data thread started...\n", driverName, functionName);

	this->lock();

	while (1) {

		//Wait for event from main thread to signal that data acquisition has started.
		this->unlock();
		epicsEventWait(mDataEvent);
		asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
				"%s:%s:, got data event\n", driverName, functionName);

		if (this->mFinish) {
			asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
					"%s:%s: Stopping thread!\n", driverName, functionName);
			break;
		}
		this->lock();

		//Sanity check that main thread thinks we are acquiring data
		if (mAcquiringData) {
			try {
				getDoubleParam(ADAcquireTime, &mIntegrationTime);
				getIntegerParam(ADImageMode, &imageMode);
				getIntegerParam(ADTriggerMode, &triggerMode);

				this->unlock();
				if (triggerMode == ADTriggerInternal) {
					if (imageMode == ADImageSingle) {
						checkStatus(tlccs_startScan(mInstr));
					} else if (imageMode == ADImageContinuous) {
						checkStatus(tlccs_startScanCont(mInstr));
					}
				} else if (triggerMode == ADTriggerExternal) {
					if (imageMode == ADImageSingle) {
						checkStatus(tlccs_startScanExtTrg(mInstr));
					} else if (imageMode == ADImageContinuous) {
						checkStatus(tlccs_startScanContExtTrg(mInstr));
					}
				}
				acquiring = 1;
				this->lock();
			} catch (const std::string &e) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n",
						driverName, functionName, e.c_str());
				this->lock();
				setIntegerParam(ADAcquire, 0);
				mAcquiringData = 0;
				acquiring = 0;
				callParamCallbacks();
				continue;
			}

			//Read some parameters
			getIntegerParam(NDDataType, &i);
			dataType = (NDDataType_t) i;
			getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
			getIntegerParam(NDArraySizeX, &sizeX);
			getIntegerParam(NDArraySizeY, &sizeY);
			getIntegerParam(TlAcquisitionType, &acqType);
			// Reset the counters
			setIntegerParam(ADNumImagesCounter, 0);
			setIntegerParam(ADNumExposuresCounter, 0);
			callParamCallbacks();
		} else {
			asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s:%s:, Data thread is running but main thread thinks we are not acquiring.\n",
					driverName, functionName);
			acquiring = 0;
		}

		while (acquiring) {
			this->unlock();
			if (mAcquiringData == 0) {
				this->lock();
				break;
			}

			try {
				gettimeofday(&tv, NULL);
				checkStatus(tlccs_getDeviceStatus(mInstr, &acquireStatus));
				asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
						"%s:%s:, tlccs_getDeviceStatus() returned %X\n",
						driverName, functionName, acquireStatus);
				// some debugging
				if (acquireStatus != acquireStatus2) {
					acquireStatus2 = acquireStatus;
					//printf("%s: Status: 0x%08X (%ld %06ld)\n", __func__, acquireStatus, tv.tv_sec, tv.tv_usec);
				}
				this->lock();

				if (acquireStatus & TLSScanIdle) {
					setIntegerParam(ADStatus, ADStatusIdle);
					setStringParam(ADStatusMessage,
							"IDLE. Waiting on instructions.");
					break;
				} else if (acquireStatus & TLSScanTransfer) {
					setIntegerParam(ADStatus, ADStatusReadout);
					setStringParam(ADStatusMessage, "Ready to send scan data.");
					/* Continue to scan data readout below.. */
				} else if (acquireStatus & TLSScanTriggered) {
					setIntegerParam(ADStatus, ADStatusWaiting);
					setStringParam(ADStatusMessage,
							"Data acquisition in progress.");
					continue;
				} else if (acquireStatus & TLSScanStartTrans) {
					setIntegerParam(ADStatus, ADStatusAcquire);
					setStringParam(ADStatusMessage,
							"Data acquisition starting.");
					continue;
				} else if (acquireStatus & TLSWaitForExtTrigger) {
					setIntegerParam(ADStatus, ADStatusWaiting);
					setStringParam(ADStatusMessage,
							"IDLE. External trigger is armed.");
					continue;
				} else {
					setIntegerParam(ADStatus, ADStatusError);
					setStringParam(ADStatusMessage, "Unknown scan state.");
					continue;
				}

				// If we are here then the acquisition has stopped and data is ready
				// to be transferred
				getIntegerParam(ADNumExposuresCounter, &numExposuresCounter);
				numExposuresCounter++;
				setIntegerParam(ADNumExposuresCounter, numExposuresCounter);
				// Update counters
				getIntegerParam(NDArrayCounter, &imageCounter);
				imageCounter++;
				setIntegerParam(NDArrayCounter, imageCounter);
				getIntegerParam(ADNumImagesCounter, &numImagesCounter);
				numImagesCounter++;
				setIntegerParam(ADNumImagesCounter, numImagesCounter);
				callParamCallbacks();

				// If array callbacks are enabled then read data into NDArray, do callbacks
				if (arrayCallbacks) {
					epicsTimeGetCurrent(&startTime);

					// Allocate an NDArray to hold:
					// y=0 processed or raw data (only data pixels),
					// y=2 wavelength data
					// y=3 amplitude data
					dims[0] = TLCCS_NUM_PIXELS;
					dims[1] = 3;
					pArray = this->pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);

					this->unlock();
					if (acqType == TLAProcessed) {
						checkStatus(tlccs_getScanData(mInstr, (epicsFloat64 *)pArray->pData));
					} else {
						checkStatus(tlccs_getRawScanData(mInstr, rawBuf));
						// convert to float64, raw data starts at offset 32
						for (i = 0; i < TLCCS_NUM_PIXELS; i++) {
							*((epicsFloat64 *)pArray->pData + i) = rawBuf[i + 32];
						}
					}
					this->lock();

					/* Add wavelength and amplitude data */
					memcpy((epicsFloat64 *)pArray->pData + 1 * TLCCS_NUM_PIXELS,
							mWavelengthData, TLCCS_NUM_PIXELS * sizeof(epicsFloat64));
					memcpy((epicsFloat64 *)pArray->pData + 2 * TLCCS_NUM_PIXELS,
							mAmplitudeData, TLCCS_NUM_PIXELS * sizeof(epicsFloat64));

					/* Get any attributes that have been defined for this driver */
					this->getAttributes(pArray->pAttributeList);
					/* Add attributes used in NDFileAscii plugin */
					i = 0;
					pArray->pAttributeList->add("Row", "", NDAttrInt32, &i);
					i = 1;
					pArray->pAttributeList->add("RowSkip", "", NDAttrInt32, &i);
					i = 1;
					pArray->pAttributeList->add("Col", "", NDAttrInt32, &i);
					i = TLCCS_NUM_PIXELS;
					pArray->pAttributeList->add("ColSkip", "", NDAttrInt32, &i);

					/* Put the frame number and time stamp into the buffer */
					pArray->uniqueId = imageCounter;
					pArray->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;
					/* Call the NDArray callback */
					/* Must release the lock here, or we can get into a deadlock, because we can
					 * block on the plugin lock, and the plugin can be calling us */
					this->unlock();
					asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
							"%s:%s:, calling array callbacks\n", driverName,
							functionName);
					doCallbacksGenericPointer(pArray, NDArrayData, 0);
					this->lock();

					pArray->release();
				} else {
					acquiring = 0;
				}

				callParamCallbacks();
			} catch (const std::string &e) {
				asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: %s\n",
						driverName, functionName, e.c_str());
				this->lock();
				acquiring = 0;
			}
		}	// End of while(acquiring)

		//Now clear main thread flag
		mAcquiringData = 0;
		setIntegerParam(ADAcquire, 0);
		setIntegerParam(ADStatus, ADStatusIdle);
		setStringParam(ADStatusMessage,
				"IDLE. Waiting on instructions.");

		/* Call the callbacks to update any changes */
		callParamCallbacks();
	} // End of while(1) loop

	printf("Data thread is down!\n");
}


static void tlDeviceTaskC(void *drvPvt) {
	TlCCS *pPvt = (TlCCS *) drvPvt;

	pPvt->deviceTask();
}

static void tlDataTaskC(void *drvPvt) {
	TlCCS *pPvt = (TlCCS *) drvPvt;

	pPvt->dataTask();
}

/** IOC shell configuration command for Thorlabs driver
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
 * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
 * \param[in] resourceName The resource string that describes Thorlabs instrument (eg. USB::0x1313::0x8081::M00310589::RAW).
 * \param[in] priority The thread priority for the asyn port driver thread
 * \param[in] stackSize The stack size for the asyn port driver thread
 */
extern "C" {
int tlCCSConfig(const char *portName, int maxBuffers, size_t maxMemory,
		const char *resourceName, int priority, int stackSize) {
	/*Instantiate class.*/
	new TlCCS(portName, maxBuffers, maxMemory, resourceName, priority, stackSize);
	return (asynSuccess);
}

/* Code for iocsh registration */

/* tlCCSConfig */
static const iocshArg arg0 = { "Port name", iocshArgString };
static const iocshArg arg1 = { "maxBuffers", iocshArgInt };
static const iocshArg arg2 = { "maxMemory", iocshArgInt };
static const iocshArg arg3 = { "resourceName", iocshArgString };
static const iocshArg arg4 = { "priority", iocshArgInt };
static const iocshArg arg5 = { "stackSize", iocshArgInt };
static const iocshArg * const args[] = {
		&arg0,
		&arg1,
		&arg2,
		&arg3,
		&arg4,
		&arg5
};

static const iocshFuncDef configtlCCS = { "tlCCSConfig", 6, args };
static void configtlCCSCallFunc(const iocshArgBuf *args) {
	tlCCSConfig(args[0].sval,
			args[1].ival,
			args[2].ival,
			args[3].sval,
			args[4].ival,
			args[5].ival);
}

static void tlCCSRegister(void) {

	iocshRegister(&configtlCCS, configtlCCSCallFunc);
}

epicsExportRegistrar(tlCCSRegister);

#include <aSubRecord.h>
#include <registryFunction.h>
#include <epicsExport.h>
static long tlCCSAmplitudeDataCopy(aSubRecord *prec) {
	epicsUInt32 i;
	epicsFloat64 *a, *vala;
	a = (epicsFloat64 *) prec->a;
	vala = (epicsFloat64 *) prec->vala;
	for (i = 0; i < prec->noa; i++) {
		vala[i] = a[i];
	}
	return 0; /* process output links */
}
epicsRegisterFunction(tlCCSAmplitudeDataCopy);

static long tlCCSWavelengthArrayInit(aSubRecord *prec) {
	epicsUInt32 i;
	epicsFloat64 *vala;
	epicsInt32 *valb;
	vala = (epicsFloat64 *) prec->vala;
	for (i = 0; i < prec->nova; i++) {
		vala[i] = 0.0;
	}
	valb = (epicsInt32 *) prec->valb;
	for (i = 0; i < prec->novb; i++) {
		valb[i] = 0;
	}
	return 0; /* process output links */
}
epicsRegisterFunction(tlCCSWavelengthArrayInit);

} /* extern "C" */
