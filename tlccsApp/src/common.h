/*
 * common.h
 *
 *  Created on: Jan 23, 2015
 *      Author: essdev
 */

#ifndef COMMON_H_
#define COMMON_H_


#include <stdarg.h>
#include <stdio.h>
#include <assert.h>
#include <pthread.h>
#include <libusb-1.0/libusb.h>

#include "list.h"


/* See Makefile */
#ifndef _TLCCSDEBUG
#define _TLCCSDEBUG		2
#endif
#define TLCCSDEBUG		_TLCCSDEBUG

#define INF(format, ...) \
	do { if (1) \
		fprintf(stderr, \
				"INF %s:%d:%s(): " \
				format, \
				__FILE__, \
				__LINE__, \
				__func__, \
				## __VA_ARGS__); \
	} while (0)

#define DBG(format, ...) \
	do { if (TLCCSDEBUG) \
		fprintf(stderr, \
				"DBG %s:%d:%s(): " \
				format, \
				__FILE__, \
				__LINE__, \
				__func__, \
				## __VA_ARGS__); \
	} while (0)

#define DBG2(format, ...) \
	do { if (TLCCSDEBUG > 1) \
		fprintf(stderr, \
				"DBG %s:%d:%s(): " \
				format, \
				__FILE__, \
				__LINE__, \
				__func__, \
				## __VA_ARGS__); \
	} while (0)

#define ERR(format, ...) \
	fprintf(stderr, \
			"ERR %s:%d:%s(): " \
			format, \
			__FILE__, \
			__LINE__, \
			__func__, \
			## __VA_ARGS__)

#define ASSERT(x) \
	do { if (TLCCSDEBUG) \
		assert(x); \
	} while(0)

#define ENTER	DBG("Enter >>>\n");
#define LEAVE	DBG("Leave <<<\n");
#define ENTER2	DBG2("Enter >>>\n");
#define LEAVE2	DBG2("Leave <<<\n");

/*---------------------------------------------------------------------------
 USB VIDs and PIDs
---------------------------------------------------------------------------*/
/* Thorlabs VID */
#define THORLABS_VID			0x1313
/* CCS100 Compact Spectrometer */
#define CCS100_VANILLA_PID		0x8080
#define CCS100_READY_PID		(CCS100_VANILLA_PID + 1)
/* CCS125 Special Spectrometer */
#define CCS125_VANILLA_PID		0x8082
#define CCS125_READY_PID		(CCS125_VANILLA_PID + 1)
/* CCS150 UV Spectrometer */
#define CCS150_VANILLA_PID		0x8084
#define CCS150_READY_PID		(CCS150_VANILLA_PID + 1)
/* CCS175 NIR Spectrometer */
#define CCS175_VANILLA_PID		0x8086
#define CCS175_READY_PID		(CCS175_VANILLA_PID + 1)
/* CCS200 UV-NIR Spectrometer */
#define CCS200_VANILLA_PID		0x8088
#define CCS200_READY_PID		(CCS200_VANILLA_PID + 1)

#define MAX_STR_SZ				256

typedef enum {
	e_usb_dev_state_unknown = 0,
	e_usb_dev_state_arrived,
	e_usb_dev_state_added,
	e_usb_dev_state_left,
	e_usb_dev_state_removed
} usb_dev_state_t;

typedef struct {
	libusb_device *dev;
	libusb_device_handle *handle;
	int claimed;
	uint16_t vid;
	uint16_t pid;
	uint8_t bulk_endpoint;
	char manf[MAX_STR_SZ];
	char model[MAX_STR_SZ];
	char serial[MAX_STR_SZ];
	usb_dev_state_t state;

	struct list_head list;
} usb_dev_t;

typedef int ((*usb_cb_fn)(usb_dev_t *dev));

typedef struct {
	usb_cb_fn fn;
	int handle;

	struct list_head list;
} usb_cb_t;

typedef struct {
	int vid;
	int pid;
	libusb_hotplug_callback_handle cb_handle;

	struct list_head list;
} usb_hp_cb_t;

typedef struct {
	pthread_mutex_t mutex;
	int run;
	usb_hp_cb_t hp_cbs;
	pthread_t thr_id;
	pthread_cond_t cond;
	int nr_devs;
	usb_dev_t devs;
	int nr_cbs;
	int idx_cbs;
	usb_cb_t cbs;
} usb_store_t;

struct fwheader {
	char tag[4];					/* tag string: CSPT */
	uint32_t msgLen;				/* length of this message w/ header */
	uint32_t hdrLen;				/* length of header 32 (0x20) bytes */
	unsigned char res00000050[4];	/* ?? always 00 00 00 50 */
	unsigned char bRequest;			/* bRequest in USB msg, always A0 */
	unsigned char res70;			/* ?? always 70 */
	uint16_t wValue;				/* wValue in USB msg */
	uint16_t wIndex;				/* wIndex in USB msg, always 00 00 */
	unsigned char res6C65[2];		/* ?? always 6C 65 */
	unsigned char res0F000000[4];	/* ?? always 0F 00 00 00 */
	uint32_t wLength;				/* wLength in USB msg */
};
#define SIZEOF_FWHEADER		(sizeof(struct fwheader))

int usbInit(void);
void usbExit(void);
int usbListenForDevices(int vid, int pid);
int usbOpen(usb_dev_t *dev);
int usbClose(usb_dev_t *dev);
int usbListenForDevices(int vid, int pid);
int usbRegisterCallback(usb_cb_fn fn, int *handle);
int usbDeregisterCallback(int handle);
int usbTriggerCallback(int handle);
int usbControl(usb_dev_t *dev,
		uint8_t bmRequestType,
		uint8_t bRequest,
		uint16_t wValue,
		uint16_t wIndex,
		unsigned char *data,
		uint16_t wLength,
		unsigned int timeout);
int usbBulk(usb_dev_t *dev,
		unsigned char *data,
		int wLength,
		int *wActualLength,
		unsigned int timeout);

#endif /* COMMON_H_ */
