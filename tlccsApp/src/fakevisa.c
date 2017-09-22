/*
 * fakevisa.c
 *
 *  Created on: Jan 23, 2015
 *      Author: essdev
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <libusb-1.0/libusb.h>
#include <pthread.h>

#define NIVISA_USB
#include "visa.h"

#include "common.h"
#include "list.h"

#define RESOURCE_TAG			0x10000000
#define SESSION_TAG				0x20000000
#define MAN_RES_TAG				0x40000000
#define MAN_SES_TAG				0x80000000

typedef struct {
	ViAttrState manf_id;
	char manf_name[VI_FIND_BUFLEN];
	ViAttrState model_code;
	char model_name[VI_FIND_BUFLEN];
	ViAttrState tmo_value;
	char usb_serial_num[VI_FIND_BUFLEN];
	char rsrc[VI_FIND_BUFLEN];
	usb_dev_t *usb_dev;

	struct list_head list;
} vi_resource_t;

typedef struct {
	ViSession vi;
	ViAttrState user_data;
	vi_resource_t *res;
	ViSession man_ses;

	struct list_head list;
} vi_session_t;

typedef struct {
	ViSession vi;
	ViSession man_res;
	vi_resource_t resources;
	vi_resource_t *next_resource;
	int nr_resources;
	vi_session_t sessions;
	int nr_sessions;

	struct list_head list;
} vi_man_ses_t;

typedef struct {
	ViSession vi;
	vi_man_ses_t man_ses;
	int nr_man_sessions;
	int cb_handle;
} vi_man_res_t;

static vi_man_res_t g_man_res = {0};
static int g_index = 0;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
	ViStatus err;
	ViString descr;
} vi_err_t;

static const vi_err_t vi_err[] =
{
		{VI_SUCCESS, 					"Operation completed successfully."},
		{VI_SUCCESS_EVENT_EN, 			"Specified event is already enabled for at least one of the specified mechanisms."},
		{VI_SUCCESS_EVENT_DIS, 			"Specified event is already disabled for at least one of the specified mechanisms."},
		{VI_SUCCESS_QUEUE_EMPTY, 		"Operation completed successfully, but queue was already empty."},
		{VI_SUCCESS_TERM_CHAR, 			"The specified termination character was read."},
		{VI_SUCCESS_MAX_CNT, 			"The number of bytes transferred is equal to the requested input count. More data may be available."},
		{VI_SUCCESS_DEV_NPRESENT, 		"Session opened successfully, but the device at the specified address is not responding."},
		{VI_SUCCESS_TRIG_MAPPED, 		"The path from trigSrc to trigDest is already mapped."},
		{VI_SUCCESS_QUEUE_NEMPTY, 		"Wait terminated successfully on receipt of an event notification. There is at least one more event object of the requested type(s) available for this session."},
		{VI_SUCCESS_NCHAIN, 			"Event handled successfully. Do not invoke any other handlers on this session for this event."},
		{VI_SUCCESS_NESTED_SHARED, 		"Operation completed successfully, and this session has nested shared locks."},
		{VI_SUCCESS_NESTED_EXCLUSIVE, 	"Operation completed successfully, and this session has nested exclusive locks."},
		{VI_SUCCESS_SYNC, 				"Operation completed successfully, but the operation was actually synchronous rather than asynchronous."},
		{VI_WARN_QUEUE_OVERFLOW, 		"VISA received more event information of the specified type than the configured queue size could hold."},
		{VI_WARN_CONFIG_NLOADED, 		"The specified configuration either does not exist or could not be loaded. VISA-specified defaults will be used."},
		{VI_WARN_NULL_OBJECT, 			"The specified object reference is uninitialized."},
		{VI_WARN_NSUP_ATTR_STATE, 		"Although the specified state of the attribute is valid, it is not supported by this implementation."},
		{VI_WARN_UNKNOWN_STATUS, 		"The status code passed to the operation could not be interpreted."},
		{VI_WARN_NSUP_BUF, 				"The specified I/O buffer type is not supported."},
		{VI_WARN_EXT_FUNC_NIMPL, 		"The operation succeeded, but a lower level driver did not implement the extended functionality."},
		{VI_ERROR_SYSTEM_ERROR, 		"Unknown system error (miscellaneous error)."},
		{VI_ERROR_INV_OBJECT, 			"The given session or object reference is invalid."},
		{VI_ERROR_RSRC_LOCKED, 			"Specified type of lock cannot be obtained, or specified operation cannot be performed, because the resource is locked."},
		{VI_ERROR_INV_EXPR, 			"Invalid expression specified for search."},
		{VI_ERROR_RSRC_NFOUND, 			"Insufficient location information or the requested device or resource is not present in the system."},
		{VI_ERROR_INV_RSRC_NAME, 		"Invalid resource reference specified. Parsing error."},
		{VI_ERROR_INV_ACC_MODE, 		"Invalid access mode."},
		{VI_ERROR_TMO, 					"Timeout expired before operation completed."},
		{VI_ERROR_CLOSING_FAILED, 		"The VISA driver failed to properly close the session or object reference. This might be due to an error freeing internal or OS resources, a failed network connection, or a lower-level driver or OS error."},
		{VI_ERROR_INV_DEGREE, 			"Specified degree is invalid."},
		{VI_ERROR_INV_JOB_ID, 			"Specified job identifier is invalid."},
		{VI_ERROR_NSUP_ATTR, 			"The specified attribute is not defined or supported by the referenced object."},
		{VI_ERROR_NSUP_ATTR_STATE, 		"The specified state of the attribute is not valid, or is not supported as defined by the object."},
		{VI_ERROR_ATTR_READONLY, 		"The specified attribute is read-only."},
		{VI_ERROR_INV_LOCK_TYPE, 		"The specified type of lock is not supported by this resource."},
		{VI_ERROR_INV_ACCESS_KEY, 		"The access key to the resource associated with the specified session is invalid."},
		{VI_ERROR_INV_EVENT, 			"Specified event type is not supported by the resource."},
		{VI_ERROR_INV_MECH, 			"Invalid mechanism specified."},
		{VI_ERROR_HNDLR_NINSTALLED, 	"A handler was not installed."},
		{VI_ERROR_INV_HNDLR_REF, 		"The given handler reference is either invalid or was not installed."},
		{VI_ERROR_INV_CONTEXT, 			"Specified event context is invalid."},
		{VI_ERROR_QUEUE_OVERFLOW, 		"The event queue for the specified type has overflowed (usually due to previous events not having been closed)."},
		{VI_ERROR_NENABLED, 			"You must be enabled for events of the specified type in order to receive them."},
		{VI_ERROR_ABORT, 				"User abort occurred during transfer."},
		{VI_ERROR_RAW_WR_PROT_VIOL, 	"Violation of raw write protocol occurred during transfer."},
		{VI_ERROR_RAW_RD_PROT_VIOL, 	"Violation of raw read protocol occurred during transfer."},
		{VI_ERROR_OUTP_PROT_VIOL, 		"Device reported an output protocol error during transfer."},
		{VI_ERROR_INP_PROT_VIOL, 		"Device reported an input protocol error during transfer."},
		{VI_ERROR_BERR, 				"Bus error occurred during transfer."},
		{VI_ERROR_IN_PROGRESS, 			"Unable to queue the asynchronous operation because there is already an operation in progress."},
		{VI_ERROR_INV_SETUP, 			"Unable to start operation because setup is invalid (usually due to attributes being set to an inconsistent state)."},
		{VI_ERROR_QUEUE_ERROR, 			"Unable to queue the asynchronous operation (usually due to the I/O completion event not being enabled or insufficient space in the session's queue)."},
		{VI_ERROR_ALLOC, 				"Insufficient system resources to perform necessary memory allocation."},
		{VI_ERROR_INV_MASK, 			"Invalid buffer mask specified."},
		{VI_ERROR_IO, 					"Could not perform operation because of I/O error."},
		{VI_ERROR_INV_FMT, 				"A format specifier in the format string is invalid."},
		{VI_ERROR_NSUP_FMT, 			"A format specifier in the format string is not supported."},
		{VI_ERROR_LINE_IN_USE, 			"The specified trigger line is currently in use."},
		{VI_ERROR_NSUP_MODE, 			"The specified mode is not supported by this VISA implementation."},
		{VI_ERROR_SRQ_NOCCURRED, 		"Service request has not been received for the session."},
		{VI_ERROR_INV_SPACE, 			"Invalid address space specified."},
		{VI_ERROR_INV_OFFSET, 			"Invalid offset specified."},
		{VI_ERROR_INV_WIDTH, 			"Invalid access width specified."},
		{VI_ERROR_NSUP_OFFSET, 			"Specified offset is not accessible from this hardware."},
		{VI_ERROR_NSUP_VAR_WIDTH, 		"Cannot support source and destination widths that are different."},
		{VI_ERROR_WINDOW_NMAPPED, 		"The specified session is not currently mapped."},
		{VI_ERROR_RESP_PENDING, 		"A previous response is still pending, causing a multiple query error."},
		{VI_ERROR_NLISTENERS, 			"No listeners condition is detected (both NRFD and NDAC are deasserted)."},
		{VI_ERROR_NCIC, 				"The interface associated with this session is not currently the controller in charge."},
		{VI_ERROR_NSYS_CNTLR, 			"The interface associated with this session is not the system controller."},
		{VI_ERROR_NSUP_OPER, 			"The given session or object reference does not support this operation."},
		{VI_ERROR_INTR_PENDING, 		"An interrupt is still pending from a previous call."},
		{VI_ERROR_ASRL_PARITY, 			"A parity error occurred during transfer."},
		{VI_ERROR_ASRL_FRAMING, 		"A framing error occurred during transfer."},
		{VI_ERROR_ASRL_OVERRUN, 		"An overrun error occurred during transfer. A character was not read from the hardware before the next character arrived."},
		{VI_ERROR_TRIG_NMAPPED, 		"The path from trigSrc to trigDest is not currently mapped."},
		{VI_ERROR_NSUP_ALIGN_OFFSET, 	"The specified offset is not properly aligned for the access width of the operation."},
		{VI_ERROR_USER_BUF, 			"A specified user buffer is not valid or cannot be accessed for the required size."},
		{VI_ERROR_RSRC_BUSY, 			"The resource is valid, but VISA cannot currently access it."},
		{VI_ERROR_NSUP_WIDTH, 			"Specified width is not supported by this hardware."},
		{VI_ERROR_INV_PARAMETER, 		"The value of some parameter (which parameter is not known) is invalid."},
		{VI_ERROR_INV_PROT, 			"The protocol specified is invalid."},
		{VI_ERROR_INV_SIZE, 			"Invalid size of window specified."},
		{VI_ERROR_WINDOW_MAPPED, 		"The specified session currently contains a mapped window."},
		{VI_ERROR_NIMPL_OPER, 			"The given operation is not implemented."},
		{VI_ERROR_INV_LENGTH, 			"Invalid length specified."},
		{VI_ERROR_INV_MODE, 			"Invalid mode specified."},
		{VI_ERROR_SESN_NLOCKED, 		"The current session did not have a lock on the resource."},
		{VI_ERROR_MEM_NSHARED, 			"The device does not export any memory."},
		{VI_ERROR_LIBRARY_NFOUND, 		"A code library required by VISA could not be located or loaded."},
		{VI_ERROR_NSUP_INTR, 			"The interface cannot generate an interrupt on the requested level or with the requested statusID value."},
		{VI_ERROR_INV_LINE, 			"The value specified by the line parameter is invalid."},
		{VI_ERROR_FILE_ACCESS, 			"An error occurred while trying to open the specified file. Possible reasons include an invalid path or lack of access rights."},
		{VI_ERROR_FILE_IO, 				"An error occurred while performing I/O on the specified file."},
		{VI_ERROR_NSUP_LINE, 			"One of the specified lines (trigSrc or trigDest) is not supported by this VISA implementation, or the combination of lines is not a valid mapping."},
		{VI_ERROR_NSUP_MECH, 			"The specified mechanism is not supported for the given event type."},
		{VI_ERROR_INTF_NUM_NCONFIG, 	"The interface type is valid but the specified interface number is not configured."},
		{VI_ERROR_CONN_LOST, 			"The connection for the given session has been lost."},
		{VI_ERROR_MACHINE_NAVAIL, 		"The remote machine does not exist or is not accepting any connections. If the NI-VISA server is installed and running on the remote machine, it may have an incompatible version or may be listening on a different port."},
		{VI_ERROR_NPERMISSION, 			"Access to the resource or remote machine is denied. This is due to lack of sufficient privileges for the current user or machine"},
		{VI_USB_PIPE_STATE_UNKNOWN,		"USB pipe status is unknown."}
};

/*
 * add resource to the RM session
 */
static ViStatus __add_resource(vi_man_ses_t *man_ses, usb_dev_t *usb_dev) {
	vi_resource_t *res;

	ASSERT(man_ses != NULL);

	if (usb_dev->vid != THORLABS_VID) {
		return VI_SUCCESS_DEV_NPRESENT;
	}

	if (usb_dev->pid != CCS100_READY_PID &&
		usb_dev->pid != CCS125_READY_PID &&
		usb_dev->pid != CCS150_READY_PID &&
		usb_dev->pid != CCS175_READY_PID &&
		usb_dev->pid != CCS200_READY_PID) {
		return VI_SUCCESS_DEV_NPRESENT;
	}

	/* setup session for found device */
	res = (vi_resource_t *)calloc(1, sizeof(vi_resource_t));
	strcpy(res->manf_name, usb_dev->manf);
	strcpy(res->model_name, usb_dev->model);
	strcpy(res->usb_serial_num, usb_dev->serial);
	res->manf_id = usb_dev->vid;
	res->model_code = usb_dev->pid;
	res->tmo_value = 2000;
	/* USB[board]::manufacturer ID::modelcode::serial number[::USBinterfacenumber]::RAW */
	sprintf(res->rsrc, "USB::0x%04X::0x%04X::%s::RAW",
			(int)usb_dev->vid, (int)usb_dev->pid, usb_dev->serial);
	res->usb_dev = usb_dev;

	/* add to this resource manager session resource list */
	DBG("adding resource %s to man ses 0x%X\n", res->rsrc, man_ses->vi);
	list_add_tail(&(res)->list, &(man_ses->resources).list);

	man_ses->nr_resources++;
	man_ses->next_resource = NULL;

	return VI_SUCCESS;
}

/**
 * remove the resource from RM session
 * underlying USB helpers shall close the USB device and clean up
 */
static ViStatus __remove_resource(vi_man_ses_t *man_ses, usb_dev_t *usb_dev) {
	vi_resource_t *del_res, *tmp_res;
	vi_session_t *del_ses, *tmp_ses;

	ASSERT(man_ses != NULL);
	ASSERT(usb_dev != NULL);


	list_for_each_entry_safe(del_res, tmp_res, &man_ses->resources.list, list) {
		if (del_res->usb_dev == usb_dev) {
			list_for_each_entry_safe(del_ses, tmp_ses, &man_ses->sessions.list, list) {
				if (del_ses->res == del_res) {
					list_del(&del_ses->list);
					man_ses->nr_sessions--;
					DBG("removed session 0x%X from man ses 0x%X\n", del_ses->vi, man_ses->vi);
					free(del_ses);
				}
			}
			list_del(&del_res->list);
			man_ses->nr_resources--;
			man_ses->next_resource = NULL;
			DBG("removed resource %s from man ses 0x%X\n", del_res->rsrc, man_ses->vi);
			free(del_res);
		}
	}

	/* do not complain if device is not in the list */
	return VI_SUCCESS;
}

/**
 * called with lock held
 */
static vi_resource_t *__find_resource(vi_man_ses_t *man_ses, usb_dev_t *usb_dev) {
	vi_resource_t *res;

	/* go through all resource manager session resources */
	list_for_each_entry(res, &man_ses->resources.list, list) {
		if (res->usb_dev == usb_dev) {
			DBG("found resource %s for manager session 0x%X\n",
					usb_dev->serial, man_ses->vi);
			return res;
		}
	}

	DBG("not found resource %s for manager session 0x%X\n",
			usb_dev->serial, man_ses->vi);
	return NULL;
}

int __usb_callback_fn(usb_dev_t *usb_dev) {
	vi_man_ses_t *man_ses;
	char *state;

	ASSERT(usb_dev != NULL);

	switch(usb_dev->state) {
	case e_usb_dev_state_unknown:
		state = "UNKNOWN";
		break;
	case e_usb_dev_state_arrived:
		state = "ARRIVED";
		break;
	case e_usb_dev_state_added:
		state = "ADDED";
		break;
	case e_usb_dev_state_left:
		state = "LEFT";
		break;
	case e_usb_dev_state_removed:
		state = "REMOVED";
		break;
	}

	DBG("USB device %x:%x serial %s, state %d (%s)\n",
			usb_dev->vid, usb_dev->pid, usb_dev->serial,
			usb_dev->state, state);

	/* find resource manager session */
	list_for_each_entry(man_ses, &g_man_res.man_ses.list, list) {
		if (usb_dev->state == e_usb_dev_state_added) {
			if (__find_resource(man_ses, usb_dev) == NULL) {
				__add_resource(man_ses, usb_dev);
			}
		} else if (usb_dev->state == e_usb_dev_state_removed) {
			__remove_resource(man_ses, usb_dev);
		} else {
			ERR("unknown state %d\n", usb_dev->state);
			ASSERT(1 == 0);
		}
	}

	return VI_SUCCESS;
}

/**
 * called with lock held
 */
static vi_session_t *__find_session(ViObject vi) {
	vi_man_ses_t *man_ses;
	vi_session_t *ses;

	if (vi == VI_NULL) {
		ERR("Invalid ViObject\n");
		return NULL;
	}

	if ((vi & SESSION_TAG) == 0) {
		ERR("Not a session ViObject\n");
		return NULL;
	}

	ses = NULL;
	/* go through all resource manager sessions */
	list_for_each_entry(man_ses, &g_man_res.man_ses.list, list) {
		/* find client session */
		list_for_each_entry(ses, &man_ses->sessions.list, list) {
			if (ses->vi == vi) {
				/* found session */
				DBG("found session 0x%X\n", vi);
				return ses;
			}
		}
	}

	ERR("not found session 0x%X\n", vi);

	return NULL;
}

static int __create_manager_session(vi_man_ses_t **man_ses) {
	vi_man_ses_t *_man_ses;

	/* create resource manager session - one per call to viOpenDefaultRM() */
	_man_ses = (vi_man_ses_t *)calloc(1, sizeof(vi_man_ses_t));
	_man_ses->vi = MAN_SES_TAG + g_index;
	g_index++;
	_man_ses->man_res = g_man_res.vi;
	INIT_LIST_HEAD(&_man_ses->resources.list);
	INIT_LIST_HEAD(&_man_ses->sessions.list);
	list_add_tail(&_man_ses->list, &g_man_res.man_ses.list);
	g_man_res.nr_man_sessions++;

	*man_ses = _man_ses;
	DBG("created manager session 0x%X\n", _man_ses->vi);

	return 0;
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/visa_resource_manager/
 *
 * VISA Resource Manager
 *
 * This section lists the attributes, events, and operations for the VISA
 * Resource Manager. The attributes, events, and operations in the VISA
 * Resource Template are available to this resource in addition to the
 * operations listed below.
 *
 * Attributes
 *
 * The attributes for the VISA Resource Template are available to this
 * resource. This resource has no defined attributes of its own.
 *
 * Events
 *
 * None
 *
 * Operations
 *
 * viFindNext (findList, instrDesc)
 * viFindRsrc (sesn, expr, findList, retcnt, instrDesc)
 * viOpen (sesn, rsrcName, accessMode, timeout, vi)
 * viOpenDefaultRM (sesn)
 * viParseRsrc(sesn, rsrcName, intfType, intfNum)
 * viParseRsrcEx (sesn, rsrcName, intfType, intfNum, rsrcClass, unaliasdExpandedRsrcName, aliasIfExists)
 *
 * XXX: We will implement limited and simplified set of above operations to
 *      mimic VISA Resource Manager.
 */

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/viopendefaultrm/
 *
 * The viOpenDefaultRM() function must be called before any VISA operations
 * can be invoked. The first call to this function initializes the VISA
 * system, including the Default Resource Manager resource, and also returns
 * a session to that resource. Subsequent calls to this function return unique
 * sessions to the same Default Resource Manager resource.
 *
 * When a Resource Manager session is passed to viClose(), not only is that
 * session closed, but also all find lists and device sessions (which that
 * Resource Manager session was used to create) are closed.
 */
ViStatus _VI_FUNC  viOpenDefaultRM (ViPSession vi) {
	int r;
	vi_man_ses_t *man_ses;

	if (vi == NULL) {
		ERR("Invalid vi ViObject\n");
		return VI_ERROR_INV_OBJECT;
	}

	pthread_mutex_lock(&g_mutex);

	/* initialize resource manager resource once */
	if (g_man_res.vi == 0) {
		INIT_LIST_HEAD(&g_man_res.man_ses.list);
		g_man_res.vi = MAN_RES_TAG + g_index;
		g_index++;

		/* create resource manager session - one per call to viOpenDefaultRM() */
		__create_manager_session(&man_ses);

		r = usbInit();
		if (r) {
			ERR("usbInit() failed\n");
			pthread_mutex_unlock(&g_mutex);
			return VI_ERROR_SYSTEM_ERROR;
		}

		r = usbRegisterCallback(__usb_callback_fn, &g_man_res.cb_handle);
		if (r) {
			usbExit();
			ERR("usbRegisterCallback() failed\n");
			return -1;
		}

		/* We can not use -1 for PID because other Thorlabs USB devices
		 * (i.e PM100USB), also with VID 0x1313, would be treated as CCSxxx! */
		r = usbListenForDevices(THORLABS_VID, CCS100_VANILLA_PID);
		r |= usbListenForDevices(THORLABS_VID, CCS100_READY_PID);
		r |= usbListenForDevices(THORLABS_VID, CCS125_VANILLA_PID);
		r |= usbListenForDevices(THORLABS_VID, CCS125_READY_PID);
		r |= usbListenForDevices(THORLABS_VID, CCS150_VANILLA_PID);
		r |= usbListenForDevices(THORLABS_VID, CCS150_READY_PID);
		r |= usbListenForDevices(THORLABS_VID, CCS175_VANILLA_PID);
		r |= usbListenForDevices(THORLABS_VID, CCS175_READY_PID);
		r |= usbListenForDevices(THORLABS_VID, CCS200_VANILLA_PID);
		r |= usbListenForDevices(THORLABS_VID, CCS200_READY_PID);
		if (r) {
			usbExit();
			ERR("usbAddCallback() failed\n");
			pthread_mutex_unlock(&g_mutex);
			return VI_ERROR_SYSTEM_ERROR;
		}
	} else {
		/* create resource manager session - one per call to viOpenDefaultRM() */
		__create_manager_session(&man_ses);
	}

	ASSERT(man_ses != NULL);

	usbTriggerCallback(0);

	/* return new session to resource manager */
	*vi = man_ses->vi;

	pthread_mutex_unlock(&g_mutex);
	return VI_SUCCESS;
}

/**
 * This is not part of VISA.
 * Used to call viClose() on resource manager resource ViSession.
 * It will deallocate all resources and sessions and call usbExit().
 */
ViStatus _VI_FUNC  viCLoseDefaultRM () {
	return viClose(g_man_res.vi);
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/vifindrsrc/
 *
 * The viFindRsrc() operation matches the value specified in the expr parameter
 * with the resources available for a particular interface. A regular
 * expression is a string consisting of ordinary characters as well as
 * special characters. You use a regular expression to specify patterns to
 * match in a given string; in other words, it is a search criterion.
 *
 * On successful completion, this function returns the first resource found
 * (instrDesc) and returns a count (retCnt) to indicate if there were more
 * resources found for the designated interface. This function also returns,
 * in the findList parameter, a handle to a find list. This handle points to
 * the list of resources and it must be used as an input to viFindNext().
 * When this handle is no longer needed, it should be passed to viClose().
 * Notice that retCnt and findList are optional parameters. This is useful
 * if only the first match is important, and the number of matches is not
 * needed. If you specify VI_NULL in the findList parameter and the
 * operation completes successfully, VISA automatically invokes viClose()
 * on the find list handle rather than returning it to the application.
 */
ViStatus _VI_FUNC  viFindRsrc      (ViSession sesn, ViString expr, ViPFindList findList,
                                    ViPUInt32 retCnt, ViChar _VI_FAR instrDesc[]) {
	vi_man_ses_t *man_ses, *tmp_man_ses;
	vi_resource_t *res;

	if (sesn == VI_NULL) {
		ERR("Invalid sesn ViObject\n");
		return VI_ERROR_INV_OBJECT;
	}

	pthread_mutex_lock(&g_mutex);

	/* find resource manager session */
	man_ses = NULL;
	list_for_each_entry(tmp_man_ses, &g_man_res.man_ses.list, list) {
		if (sesn == tmp_man_ses->vi) {
			man_ses = tmp_man_ses;
			break;
		}
	}

	/* if not found signal error */
	if (man_ses == NULL) {
		ERR("manager session 0x%X not found\n", sesn);
		memset(instrDesc, 0, VI_FIND_BUFLEN);
		if (findList) {
			*findList = 0;
		}
		if (retCnt) {
			*retCnt = 0;
		}
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}

	/* if there are no resources signal error */
	if (man_ses->nr_resources == 0) {
		ERR("manager session 0x%X has no resources\n", sesn);
		memset(instrDesc, 0, VI_FIND_BUFLEN);
		if (findList) {
			*findList = 0;
		}
		if (retCnt) {
			*retCnt = 0;
		}
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}

	/* get first resource descriptor */
	res = list_first_entry(&man_ses->resources.list, vi_resource_t, list);
	strcpy(instrDesc, res->rsrc);
	/* set the find list for viFindNext() if needed */
	if (findList) {
		/* XXX: set resource manager session ViSession instead of creating
		 * another list */
		*findList = man_ses->vi;
		/* set the next resource to be returned when calling viFindNext() */
		man_ses->next_resource = list_next_entry(res, list);
	}
	/* set the number of resources if needed */
	if (retCnt) {
		*retCnt = man_ses->nr_resources;
	}

	pthread_mutex_unlock(&g_mutex);
	return VI_SUCCESS;
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/vifindnext/
 *
 * The viFindNext() operation returns the next device found in the list
 * created by viFindRsrc(). The list is referenced by the handle that was
 * returned by viFindRsrc().
 */
ViStatus _VI_FUNC  viFindNext      (ViFindList findList, ViChar _VI_FAR instrDesc[]) {
	vi_man_ses_t *man_ses, *tmp_man_ses;

	if (findList == VI_NULL) {
		ERR("Invalid ViObject\n");
		return VI_ERROR_INV_OBJECT;
	}

	pthread_mutex_lock(&g_mutex);

	/* XXX: we are returning resource manager session ViSession in
	 * viFindRsrc() to avoid using additional list */
	man_ses = NULL;
	list_for_each_entry(tmp_man_ses, &g_man_res.man_ses.list, list) {
		if (findList == tmp_man_ses->vi) {
			man_ses = tmp_man_ses;
			break;
		}
	}

	/* if not found signal error */
	if (man_ses == NULL) {
		ERR("manager session 0x%X not found\n", findList);
		memset(instrDesc, 0, VI_FIND_BUFLEN);
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}

	if (man_ses->next_resource == NULL) {
		/* we have NOT returned first resource through viFindRsrc() */
		man_ses->next_resource = list_first_entry(&man_ses->resources.list, vi_resource_t, list);
	}

	/* if next element does not point to head, it is valid next */
	if (man_ses->next_resource != &man_ses->resources) {
		strcpy(instrDesc, man_ses->next_resource->rsrc);
		DBG("next resource found\n");
	} else {
		DBG("next resource NOT found\n");
		memset(instrDesc, 0, VI_FIND_BUFLEN);
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}

	/* set the next resource to be returned when calling viFindNext() */
	man_ses->next_resource = list_next_entry(man_ses->next_resource, list);

	pthread_mutex_unlock(&g_mutex);
	return VI_SUCCESS;
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/viopen/
 *
 * The viOpen() operation opens a session to the specified resource. It
 * returns a session identifier that can be used to call any other operations
 * of that resource. The address string passed to viOpen() must uniquely
 * identify a resource. Refer to VISA Resource Syntax and Examples for the
 * syntax of resource strings and examples.
 *
 * For the parameter accessMode, the value VI_EXCLUSIVE_LOCK (1) is used to
 * acquire an exclusive lock immediately upon opening a session; if a lock
 * cannot be acquired, the session is closed and an error is returned.
 * The value VI_LOAD_CONFIG (4) is used to configure attributes to values
 * specified by some external configuration utility. Multiple access modes
 * can be used simultaneously by specifying a bit-wise OR of the values other
 * than VI_NULL. NI-VISA currently supports VI_LOAD_CONFIG only on Serial
 * INSTR sessions.
 *
 * All resource strings returned by viFindRsrc() will always be recognized by
 * viOpen(). However, viFindRsrc() will not necessarily return all strings
 * that you can pass to viParseRsrc() or viOpen(). This is especially true
 * for network and TCPIP resources.
 */
ViStatus _VI_FUNC  viOpen          (ViSession sesn, ViRsrc name, ViAccessMode mode,
                                    ViUInt32 timeout, ViPSession vi) {
	int r;
	vi_session_t *ses;
	vi_resource_t *res, *tmp_res;
	vi_man_ses_t *man_ses, *tmp_man_ses;

	if (sesn == VI_NULL) {
		ERR("Invalid sesn ViObject\n");
		return VI_ERROR_INV_OBJECT;
	}

	pthread_mutex_lock(&g_mutex);

	man_ses = NULL;
	list_for_each_entry(tmp_man_ses, &g_man_res.man_ses.list, list) {
		if (sesn == tmp_man_ses->vi) {
			man_ses = tmp_man_ses;
			break;
		}
	}

	/* if not found signal error */
	if (man_ses == NULL) {
		ERR("manager session 0x%X not found\n", sesn);
		*vi = VI_NULL;
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}

	res = NULL;
	list_for_each_entry(tmp_res, &man_ses->resources.list, list) {
		if (strncmp(name, tmp_res->rsrc, strlen(name)) == 0) {
			res = tmp_res;
			break;
		}
	}

	/* if not found signal error */
	if (res == NULL) {
		ERR("resource with name %s not found\n", name);
		*vi = VI_NULL;
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}

	/* if the access mode is exclusive and USB device is already opened,
	 * signal error */
	if ((mode == VI_EXCLUSIVE_LOCK) && res->usb_dev->claimed) {
		ERR("resource with name %s already opened and locked\n", name);
		*vi = VI_NULL;
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_LOCKED;
	}

	/* open the USB device */
	r = usbOpen(res->usb_dev);
	if (r) {
		ERR("usbOpen() failed\n");
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}

	/* create new session for resource */
	ses = (vi_session_t *)calloc(1, sizeof(vi_session_t));
	ses->vi = SESSION_TAG + g_index;
	g_index++;
	ses->man_ses = man_ses->vi;
	ses->res = res;

	/* add to this resource manager session client sessions list */
	DBG("adding session 0x%x for %s to man ses 0x%X\n", ses->vi, name, man_ses->vi);
	list_add_tail(&ses->list, &man_ses->sessions.list);
	man_ses->nr_sessions++;

	/* used with other functions to access linked resource */
	*vi = ses->vi;

	pthread_mutex_unlock(&g_mutex);
	return VI_SUCCESS;
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/viclose/
 *
 * The viClose() operation closes a session, event, or a find list. In this
 * process all the data structures that had been allocated for the specified
 * vi are freed. Calling viClose() on a VISA Resource Manager session will
 * also close all I/O sessions associated with that resource manager session.
 */
ViStatus _VI_FUNC  viClose         (ViObject vi) {
	vi_man_ses_t *man_ses, *tmp_man_ses;
	vi_session_t *ses, *tmp_ses;
	vi_resource_t *res, *tmp_res;

	if (vi == VI_NULL) {
		ERR("Invalid ViObject\n");
		return VI_ERROR_INV_OBJECT;
	}

	pthread_mutex_lock(&g_mutex);

	if (vi & SESSION_TAG) {
		/* go through all resource manager sessions */
		list_for_each_entry(man_ses, &g_man_res.man_ses.list, list) {
			/* find client session */
			list_for_each_entry_safe(ses, tmp_ses, &man_ses->sessions.list, list) {
				if (ses->vi == vi) {
					/* do NOT close USB connection here! */
					ses->res = NULL;
					/* delete the session, decrement count and free the memory */
					DBG("session 0x%X closed!\n", ses->vi);
					list_del(&ses->list);
					man_ses->nr_sessions--;
					free(ses);

					pthread_mutex_unlock(&g_mutex);
					return VI_SUCCESS;
				}
			}
		}

	} else if (vi & MAN_SES_TAG) {
		/* go through all resource manager sessions */
		list_for_each_entry_safe(man_ses, tmp_man_ses, &g_man_res.man_ses.list, list) {
			if (man_ses->vi == vi) {
				/* go through all client sessions and close them */
				list_for_each_entry_safe(ses, tmp_ses, &man_ses->sessions.list, list) {
					/* do NOT close USB connection here! */
					ses->res = NULL;
					/* delete the session, decrement count and free the memory */
					DBG("session 0x%X closed!\n", ses->vi);
					list_del(&ses->list);
					man_ses->nr_sessions--;
					free(ses);
				}
				ASSERT(man_ses->nr_sessions == 0);

				/* go through all resources and delete them */
				list_for_each_entry_safe(res, tmp_res, &man_ses->resources.list, list) {
					/* do NOT close USB connection here! */
					/* delete the session, decrement count and free the memory */
					DBG("resource %s removed!\n", res->rsrc);
					list_del(&res->list);
					man_ses->nr_resources--;
					free(res);
				}
				man_ses->next_resource = NULL;
				ASSERT(man_ses->nr_resources == 0);

				/* delete the resource manager session, decrement count and free the memory */
				DBG("resource manager session 0x%X closed!\n", man_ses->vi);
				list_del(&man_ses->list);
				g_man_res.nr_man_sessions--;
				free(man_ses);

				pthread_mutex_unlock(&g_mutex);
				return VI_SUCCESS;
			}
		}

	} else if (vi & MAN_RES_TAG) {
		/* this is a hack to destroy the complete fake VI stack and libusb */

		/* go through all resource manager sessions */
		list_for_each_entry_safe(man_ses, tmp_man_ses, &g_man_res.man_ses.list, list) {
			/* go through all client sessions and close them */
			list_for_each_entry_safe(ses, tmp_ses, &man_ses->sessions.list, list) {
				/* do NOT close USB connection here! */
				ses->res = NULL;
				/* delete the session, decrement count and free the memory */
				DBG("session 0x%X closed!\n", ses->vi);
				list_del(&ses->list);
				man_ses->nr_sessions--;
				free(ses);
			}
			ASSERT(man_ses->nr_sessions == 0);

			/* go through all resources and delete them */
			list_for_each_entry_safe(res, tmp_res, &man_ses->resources.list, list) {
				/* close USB connection only here if needed */
				if (res) {
					usbClose(res->usb_dev);
				}
				/* delete the session, decrement count and free the memory */
				DBG("resource %s removed!\n", res->rsrc);
				list_del(&res->list);
				man_ses->nr_resources--;
				free(res);
			}
			man_ses->next_resource = NULL;
			ASSERT(man_ses->nr_resources == 0);

			/* delete the resource manager session, decrement count and free the memory */
			DBG("resource manager session 0x%X closed!\n", man_ses->vi);
			list_del(&man_ses->list);
			g_man_res.nr_man_sessions--;
			free(man_ses);
		}
		g_man_res.vi = 0;
		ASSERT(g_man_res.nr_man_sessions == 0);

		usbExit();

		pthread_mutex_unlock(&g_mutex);
		return VI_SUCCESS;
	}

	/* not allowed */
	ERR("Invalid object!\n");
	pthread_mutex_unlock(&g_mutex);
	return VI_ERROR_INV_OBJECT;
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/visetattribute/
 *
 * The viSetAttribute() operation is used to modify the state of an attribute
 * for the specified object.
 *
 * Both VI_WARN_NSUP_ATTR_STATE and VI_ERROR_NSUP_ATTR_STATE indicate that
 * the specified attribute state is not supported. A resource normally
 * returns the error code VI_ERROR_NSUP_ATTR_STATE when it cannot set a
 * specified attribute state. The completion code VI_WARN_NSUP_ATTR_STATE
 * is intended to alert the application that although the specified optional
 * attribute state is not supported, the application should not fail.
 * One example is attempting to set an attribute value that would increase
 * performance speeds. This is different than attempting to set an attribute
 * value that specifies required but nonexistent hardware (such as specifying
 * a VXI ECL trigger line when no hardware support exists) or a value
 * that would change assumptions a resource might make about the way
 * data is stored or formatted (such as byte order).
 */
ViStatus _VI_FUNC  viSetAttribute  (ViObject vi, ViAttr attrName, ViAttrState attrValue) {
	vi_session_t *ses;
	vi_resource_t *res;


	pthread_mutex_lock(&g_mutex);

	if ((vi & SESSION_TAG) == 0) {
		ERR("Not a session ViObject\n");
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_INV_OBJECT;
	}

	ses = __find_session(vi);
	if (ses == VI_NULL) {
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}
	res = ses->res;

	ASSERT(res != NULL);

	/* XXX: Are there any other attributes to take care of? */
	switch (attrName) {
	case VI_ATTR_USER_DATA:
		ses->user_data = attrValue;
		break;
	case VI_ATTR_RM_SESSION:
		ses->man_ses = attrValue;
		break;
	case VI_ATTR_MANF_ID:
		res->manf_id = attrValue;
		break;
	case VI_ATTR_MANF_NAME:
		strncpy(res->manf_name, (char *)attrValue, VI_FIND_BUFLEN);
		break;
	case VI_ATTR_MODEL_CODE:
		res->model_code = attrValue;
		break;
	case VI_ATTR_MODEL_NAME:
		strncpy(res->model_name, (char *)attrValue, VI_FIND_BUFLEN);
		break;
	case VI_ATTR_USB_SERIAL_NUM:
		strncpy(res->usb_serial_num, (char *)attrValue, VI_FIND_BUFLEN);
		break;
	case VI_ATTR_TMO_VALUE:
		res->tmo_value = attrValue;
		break;
	default:
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_NSUP_ATTR;
	}

	pthread_mutex_unlock(&g_mutex);
	return VI_SUCCESS;
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/vigetattribute/
 *
 * The viGetAttribute() operation is used to retrieve the state of an
 * attribute for the specified session, event, or find list.
 *
 * The output parameter attrState is of the type of the attribute actually
 * being retrieved. For example, when retrieving an attribute that is defined
 * as a ViBoolean, your application should pass a reference to a variable of
 * type ViBoolean. Similarly, if the attribute is defined as being ViUInt32,
 * your application should pass a reference to a variable of type ViUInt32.
 */
ViStatus _VI_FUNC  viGetAttribute  (ViObject vi, ViAttr attrName, void _VI_PTR attrValue) {
	vi_session_t *ses;
	vi_resource_t *res;

	pthread_mutex_lock(&g_mutex);

	if ((vi & SESSION_TAG) == 0) {
		ERR("Not a session ViObject\n");
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_INV_OBJECT;
	}

	ses = __find_session(vi);
	if (ses == VI_NULL) {
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}
	res = ses->res;

	ASSERT(res != NULL);

	/* XXX: Are there any other attributes to take care of? */
	switch (attrName) {
	case VI_ATTR_USER_DATA:
		*(ViAttrState *)attrValue = ses->user_data;
		break;
	case VI_ATTR_RM_SESSION:
		*(ViAttrState *)attrValue = ses->man_ses;
		break;
	case VI_ATTR_MANF_ID:
		*(ViAttrState *)attrValue = res->manf_id;
		break;
	case VI_ATTR_MANF_NAME:
		strncpy((char *)attrValue, res->manf_name, VI_FIND_BUFLEN);
		break;
	case VI_ATTR_MODEL_CODE:
		*(ViAttrState *)attrValue = res->model_code;
		break;
	case VI_ATTR_MODEL_NAME:
		strncpy((char *)attrValue, res->model_name, VI_FIND_BUFLEN);
		break;
	case VI_ATTR_USB_SERIAL_NUM:
		strncpy((char *)attrValue, res->usb_serial_num, VI_FIND_BUFLEN);
		break;
	case VI_ATTR_TMO_VALUE:
		*(ViAttrState *)attrValue = res->tmo_value;
		break;
	default:
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_NSUP_ATTR;
	}

	pthread_mutex_unlock(&g_mutex);
	return VI_SUCCESS;
}

/**
 * From
 *
 * The viStatusDesc() operation is used to retrieve a user-readable string
 * that describes the status code presented. If the string cannot be
 * interpreted, the operation returns the warning code VI_WARN_UNKNOWN_STATUS.
 * However, the output string desc is valid regardless of the status
 * return value.
 *
 * Note: The size of the desc parameter should be at least 256 bytes.
 */
ViStatus _VI_FUNC  viStatusDesc    (ViObject vi, ViStatus status, ViChar _VI_FAR desc[]) {
	const vi_err_t *ptr;

	if (!desc) {
		return VI_ERROR_INV_PARAMETER;
	}

	/* VISA errors */
	ptr = vi_err;
	while (ptr->descr != VI_NULL) {
		if (ptr->err == status) {
			strcpy(desc, ptr->descr);
			return (VI_SUCCESS);
		}
		ptr++;
	}

	return VI_WARN_UNKNOWN_STATUS;
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/viread/
 *
 * The viRead() operation synchronously transfers data. The data read is
 * to be stored in the buffer represented by buf. This operation
 * returns only when the transfer terminates. Only one synchronous read
 * operation can occur at any one time.
 *
 * Note: The retCount and buf parameters always are valid on both success
 * and error.
 */
ViStatus _VI_FUNC  viRead          (ViSession vi, ViPBuf buf, ViUInt32 cnt, ViPUInt32 retCnt) {
	int r;
	vi_session_t *ses;
	vi_resource_t *res;

	pthread_mutex_lock(&g_mutex);

	if ((vi & SESSION_TAG) == 0) {
		ERR("Not a session ViObject\n");
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_INV_OBJECT;
	}

	ses = __find_session(vi);
	if (ses == VI_NULL) {
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}
	res = ses->res;

	ASSERT(res != NULL);

	r = usbBulk(res->usb_dev, buf, cnt, (int *)retCnt, res->tmo_value);
	if (r < 0) {
		pthread_mutex_unlock(&g_mutex);
		return VI_USB_PIPE_STATE_UNKNOWN;
	}

	pthread_mutex_unlock(&g_mutex);
	return VI_SUCCESS;
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/viclear/
 *
 * Clear for Non-488.2 Instruments (Serial INSTR, TCPIP SOCKET, and USB RAW)
 *
 * For Serial INSTR sessions, VISA flushes (discards) the I/O output buffer,
 *  sends a break, and then flushes (discards) the I/O input buffer.
 * For TCPIP SOCKET sessions, VISA flushes (discards) the I/O buffers.
 * For USB RAW sessions, VISA resets the endpoints referred to by the
 *  attributes VI_ATTR_USB_BULK_IN_PIPE and VI_ATTR_USB_BULK_OUT_PIPE.
 *
 *  Invoking viClear() also discards the read and write buffers used
 *  by the formatted I/O services for that session.
 */
ViStatus _VI_FUNC  viClear         (ViSession vi) {
	/* XXX: Do something here? */
	return VI_SUCCESS;
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/viflush/
 *
 * It is possible to combine any of these read flags and write flags for
 * different buffers by ORing the flags. However, combining two flags for
 * the same buffer in the same call to viFlush() is illegal.
 *
 * Notice that when using formatted I/O operations with a session to a
 * Serial device or Ethernet socket, a flush of the formatted I/O buffers
 * also causes the corresponding I/O communication buffers to be flushed.
 * For example, calling viFlush() with VI_WRITE_BUF also flushes the
 * VI_IO_OUT_BUF.
 */
ViStatus _VI_FUNC  viFlush         (ViSession vi, ViUInt16 mask) {
	/* XXX: Do something here? */
	return VI_SUCCESS;
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/viusbcontrolout/
 *
 * The viUsbControlOut() operation synchronously performs a USB control
 * pipe transfer to the device. The values of the data payload in the setup
 * stage of the control transfer are taken as parameters and include
 * bmRequestType, bRequest, wValue, wIndex, and wLength. An optional data
 * buffer buf contains the data to send if a data stage is required for
 * this transfer. Only one USB control pipe transfer operation can occur
 * at any one time.
 */
ViStatus _VI_FUNC  viUsbControlOut (ViSession vi, ViInt16 bmRequestType, ViInt16 bRequest,
                                    ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength,
                                    ViBuf buf) {
	int r;
	vi_session_t *ses;
	vi_resource_t *res;

	pthread_mutex_lock(&g_mutex);

	if ((vi & SESSION_TAG) == 0) {
		ERR("Not a session ViObject\n");
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_INV_OBJECT;
	}

	ses = __find_session(vi);
	if (ses == VI_NULL) {
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}
	res = ses->res;

	ASSERT(res != NULL);

	r = usbControl(res->usb_dev, bmRequestType, bRequest,
			wValue, wIndex, buf, wLength, res->tmo_value);
	if (r < 0) {
		pthread_mutex_unlock(&g_mutex);
		return VI_USB_PIPE_STATE_UNKNOWN;
	}

	pthread_mutex_unlock(&g_mutex);
	return VI_SUCCESS;
}

/**
 * From http://zone.ni.com/reference/en-XX/help/370131S-01/ni-visa/viusbcontrolin/
 *
 * The viUsbControlIn() operation synchronously performs a USB control pipe
 * transfer from the device. The values of the data payload in the setup
 * stage of the control transfer are taken as parameters and include
 * bmRequestType, bRequest, wValue, wIndex, and wLength. An optional data
 * buffer buf receives data if a data stage is required for this transfer.
 * Only one USB control pipe transfer operation can occur at any one time.
 */
ViStatus _VI_FUNC  viUsbControlIn  (ViSession vi, ViInt16 bmRequestType, ViInt16 bRequest,
                                    ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength,
                                    ViPBuf buf, ViPUInt16 retCnt) {
	int r;
	vi_session_t *ses;
	vi_resource_t *res;

	pthread_mutex_lock(&g_mutex);

	if ((vi & SESSION_TAG) == 0) {
		ERR("Not a session ViObject\n");
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_INV_OBJECT;
	}

	ses = __find_session(vi);
	if (ses == VI_NULL) {
		pthread_mutex_unlock(&g_mutex);
		return VI_ERROR_RSRC_NFOUND;
	}
	res = ses->res;

	ASSERT(res != NULL);

	r = usbControl(res->usb_dev, bmRequestType, bRequest,
			wValue, wIndex, buf, wLength, res->tmo_value);
	if (r < 0) {
		pthread_mutex_unlock(&g_mutex);
		return VI_USB_PIPE_STATE_UNKNOWN;
	}

	*retCnt = r;

	pthread_mutex_unlock(&g_mutex);
	return VI_SUCCESS;
}
