/**
 * Area Detector driver for the Thorlabs CCS.
 *
 * @author Hinko Kocevar
 * @date January 2015
 *
 * Based on andorCCD driver.
 *
 */

#ifndef TLCCS_H
#define TLCCS_H

#include <ADDriver.h>

#define NIVISA_USB   // this is necessary to access NI-VISA-USB
#include <visa.h>
#include <tlccsdrv.h>

#define TlMessageString                   "TL_MESSAGE"
#define TlWavelengthDataString            "TL_WAVELENGTH_DATA"
#define TlWavelengthDataTargetString      "TL_WAVELENGTH_DATA_TARGET"
#define TlWavelengthDataSetString         "TL_WAVELENGTH_DATA_SET"
#define TlWavelengthDataGetString         "TL_WAVELENGTH_DATA_GET"
#define TlWavelengthArrayString           "TL_WAVELENGTH_ARRAY"
#define TlWavelengthPixelArrayString      "TL_WAVELENGTH_PIXEL_ARRAY"
#define TlWavelengthMinimumString         "TL_WAVELENGTH_MINIMUM"
#define TlWavelengthMaximumString         "TL_WAVELENGTH_MAXIMUM"
#define TlUserCalibrationPointsString     "TL_USER_CALIB_POINTS"
#define TlAmplitudeDataString             "TL_AMPLITUDE_DATA"
#define TlAmplitudeDataModeString         "TL_AMPLITUDE_DATA_MODE"
#define TlAmplitudeDataTargetString       "TL_AMPLITUDE_DATA_TARGET"
#define TlAmplitudeDataSetString          "TL_AMPLITUDE_DATA_SET"
#define TlAmplitudeDataGetString          "TL_AMPLITUDE_DATA_GET"
#define TlAcquisitionTypeString           "TL_ACQUISITION_TYPE"

/**
 * Driver class for Thorlabs CCS. This inherits from ADDriver class in areaDetector.
 *
 */
class TlCCS : public ADDriver {
 public:
  TlCCS(const char *portName, int maxBuffers, size_t maxMemory,
		  const char *resourceName, int priority, int stackSize);
  virtual ~TlCCS();

  /* These are the methods that we override from ADDriver */
  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  virtual asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                      size_t nElements);
  virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                      size_t nElements, size_t *nIn);
  virtual asynStatus writeInt32Array(asynUser *pasynUser, epicsInt32 *value,
                                      size_t nElements);
  virtual void report(FILE *fp, int details);

  // Should be private, but are called from C so must be public
  void dataTask(void);
  void deviceTask(void);

 protected:
  int TlMessage;
  #define FIRST_TL_PARAM TlMessage
  int TlWavelengthData;
  int TlWavelengthDataTarget;
  int TlWavelengthDataSet;
  int TlWavelengthDataGet;
  int TlWavelengthArray;
  int TlWavelengthPixelArray;
  int TlWavelengthMinimum;
  int TlWavelengthMaximum;
  int TlUserCalibrationPoints;
  int TlAmplitudeData;
  int TlAmplitudeDataMode;
  int TlAmplitudeDataTarget;
  int TlAmplitudeDataSet;
  int TlAmplitudeDataGet;
  int TlAcquisitionType;
  #define LAST_TL_PARAM TlAcquisitionType

 private:

  void addVisaDevice(void);
  void removeVisaDevice(void);
  unsigned int checkStatus(ViStatus returnStatus);

  /**
   * Calibration data set
   */
  static const epicsUInt32 TLWavelengthTargetFactory;
  static const epicsUInt32 TLWavelengthTargetUser;

  /**
   * Amplitude correction modes
   */
  static const epicsUInt32 TLAmplitudeCorrectionModeCurrent;
  static const epicsUInt32 TLAmplitudeCorrectionModeNVMem;

  /**
   * Amplitude correction targets
   */
  static const epicsUInt32 TLAmplitudeCorrectionTargetFactory;
  static const epicsUInt32 TLAmplitudeCorrectionTargetUser;
  static const epicsUInt32 TLAmplitudeCorrectionTargetThorlabs;

  /**
   * List of acquisition modes
   */
  static const epicsUInt32 TLASingle;
  static const epicsUInt32 TLACont;
  static const epicsUInt32 TLAExtTrig;
  static const epicsUInt32 TLAContExtTrig;

  /**
   * List of acquisition types
   */
  static const epicsUInt32 TLARaw;
  static const epicsUInt32 TLAProcessed;

  /**
   * List of detector scan status states
   */
  static const epicsUInt32 TLSScanIdle;
  static const epicsUInt32 TLSScanTriggered;
  static const epicsUInt32 TLSScanStartTrans;
  static const epicsUInt32 TLSScanTransfer;
  static const epicsUInt32 TLSWaitForExtTrigger;

  /**
   * List of file formats
   */
  static const epicsInt32 AFFRAW;
  static const epicsInt32 AFFBMP;

  epicsEventId mDataEvent;
  epicsEventId mDeviceEvent;
  unsigned int mAcquiringData;
  double mIntegrationTime;

  unsigned int mFinish;

  // VISA instrument handle
  ViSession mInstr;
  char mResourceName[TLCCS_BUFFER_SIZE];
  char mManufacturerName[TLCCS_BUFFER_SIZE];
  char mDeviceName[TLCCS_BUFFER_SIZE];
  char mSerialNumber[TLCCS_BUFFER_SIZE];
  char mFirmwareRevision[TLCCS_BUFFER_SIZE];
  char mInstrumentDriverRevision[TLCCS_BUFFER_SIZE];
  epicsUInt32 mWavelengthPixelArray[TLCCS_MAX_NUM_USR_ADJ];
  epicsFloat64 mWavelengthArray[TLCCS_MAX_NUM_USR_ADJ];
  epicsFloat64 mWavelengthData[TLCCS_NUM_PIXELS];
  epicsFloat64 mAmplitudeData[TLCCS_NUM_PIXELS];
};

#define NUM_TL_DET_PARAMS ((int)(&LAST_TL_PARAM - &FIRST_TL_PARAM + 1))

#endif //TLCCS_H

