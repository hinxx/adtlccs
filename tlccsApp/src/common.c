/*
 * common.c
 *
 *  Created on: Jan 23, 2015
 *      Author: essdev
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/time.h>

/* Firmware is bundled in header files generated with:
 *  $ xxd -i CCSxxx.spt CCSxxx_spt.h
 */
#include "CCS100_spt.h"
#include "CCS125_spt.h"
#include "CCS150_spt.h"
#include "CCS175_spt.h"
#include "CCS200_spt.h"

#include "common.h"


static usb_store_t *g_store = NULL;

#if _DBG > 1
static void __hexdump(FILE * stream, void const * data, unsigned int len) {
	unsigned int i;
	unsigned int r, c;

	if (!stream)
		return;
	if (!data)
		return;

	for (r = 0, i = 0; r < (len / 16 + (len % 16 != 0)); r++, i += 16) {
		fprintf(stream, "%04X:   ", i); /* location of first byte in line */

		for (c = i; c < i + 8; c++) /* left half of hex dump */
			if (c < len)
				fprintf(stream, "%02X ", ((unsigned char const *) data)[c]);
			else
				fprintf(stream, "   "); /* pad if short line */

		fprintf(stream, "  ");

		for (c = i + 8; c < i + 16; c++) /* right half of hex dump */
			if (c < len)
				fprintf(stream, "%02X ", ((unsigned char const *) data)[c]);
			else
				fprintf(stream, "   "); /* pad if short line */

		fprintf(stream, "   ");

		for (c = i; c < i + 16; c++) /* ASCII dump */
			if (c < len)
				if (((unsigned char const *) data)[c] >= 32
						&& ((unsigned char const *) data)[c] < 127)
					fprintf(stream, "%c", ((char const *) data)[c]);
				else
					fprintf(stream, "."); /* put this for non-printables */
			else
				fprintf(stream, " "); /* pad if short line */

		fprintf(stream, "\n");
	}

	fflush(stream);
}
#else
static void __hexdump(FILE * stream, void const * data, unsigned int len) {}
#endif

static int __load_firmware(usb_dev_t *dev) {
	int r;
	int pos;
	unsigned char *buf;
	struct fwheader *hdr;
	char model[MAX_STR_SZ] = {0};
	unsigned char *data;
	unsigned int len;
	int addr, bus;

	ENTER

	switch (dev->pid) {
	case CCS100_VANILLA_PID:
		/* CCS100 */
		snprintf(model, MAX_STR_SZ, "CCS100");
		data = CCS100_spt;
		len = CCS100_spt_len;
		break;
	case CCS125_VANILLA_PID:
		/* CCS125 */
		snprintf(model, 16, "CCS125");
		data = CCS125_spt;
		len  = CCS125_spt_len;
		break;
	case CCS150_VANILLA_PID:
		/* CCS150 */
		snprintf(model, 16, "CCS150");
		data = CCS150_spt;
		len  = CCS150_spt_len;
		break;
	case CCS175_VANILLA_PID:
		/* CCS175 */
		snprintf(model, 16, "CCS175");
		data = CCS175_spt;
		len  = CCS175_spt_len;
		break;
	case CCS200_VANILLA_PID:
		/* CCS200 */
		snprintf(model, 16, "CCS200");
		data = CCS200_spt;
		len  = CCS200_spt_len;
		break;
	default:
		return -1;
		break;
	}

	addr = libusb_get_device_address(dev->dev);
	bus = libusb_get_bus_number(dev->dev);

	DBG("uploading firmware for bus %d, address %d, model %s..\n",
			bus, addr, model);

	pos = 0;
	do {
		/* get header */
		if ((pos + SIZEOF_FWHEADER) <= len) {
			hdr = (struct fwheader *)(data + pos);
		} else {
			ERR("premature end of file\n");
			return -1;
		}
		__hexdump(stdout, hdr, SIZEOF_FWHEADER);
		DBG2("TAG:         %.*s\n", 4, hdr->tag);
		DBG2("wValue:      0x%X\n", hdr->wValue);
		DBG2("wLength:     0x%X\n", hdr->wLength);

		pos += SIZEOF_FWHEADER;
		if ((pos + hdr->wLength) <= len) {
			buf = (data + pos);
		} else {
			ERR("premature end of file\n");
			return -1;
		}
		DBG2("data length: 0x%X (%d)\n", r, r);
		__hexdump(stdout, buf, hdr->wLength);

		r = libusb_control_transfer(
				dev->handle,				/* handle */
				0x40,		 				/* bmRequestType */
				hdr->bRequest,				/* bRequest */
				hdr->wValue,				/* wValue */
				hdr->wIndex,				/* wIndex */
				buf,						/* data buffer */
				hdr->wLength,				/* data size */
				1000						/* timeout */
				);
		if (r != hdr->wLength) {
			ERR("failed to send all bytes; %d / %d\n", r, hdr->wLength);
	    	return -1;
		} else if (r < 0) {
	    	ERR("libusb_control_transfer failed (%d): %s\n", r, libusb_strerror(r));
	    	return -1;
		}

		pos += hdr->wLength;
	} while (pos < len);

	/* Last message sent to the USB device needs to be with proper wValue! */
	if (hdr->wValue != 0xE600) {
		ERR("invalid last command 0x%X; should be 0xE600\n", hdr->wValue);
		return -1;
	}

	DBG("firmware upload done!\n");
	LEAVE
	return 0;
}

static int __get_strings(usb_dev_t *dev) {
	int r;
	struct libusb_device_descriptor desc;
	char manf[MAX_STR_SZ] = {0};
	char model[MAX_STR_SZ] = {0};
	char serial[MAX_STR_SZ] = {0};

	ENTER

	r = libusb_get_device_descriptor(dev->dev, &desc);
	if (r) {
		ERR("libusb_get_device_descriptor failed: %s\n", libusb_strerror(r));
		return -1;
	}

	if (desc.iManufacturer) {
		r = libusb_get_string_descriptor_ascii(dev->handle, desc.iManufacturer,
				(unsigned char *)manf, MAX_STR_SZ);
		if (r < 0) {
			ERR("libusb_get_string_descriptor_ascii failed: %s\n", libusb_strerror(r));
			LEAVE
			return -1;
		}
		strncpy(dev->manf, manf, strlen(manf));
	}

	if (desc.iProduct) {
		r = libusb_get_string_descriptor_ascii(dev->handle, desc.iProduct,
				(unsigned char *)model, MAX_STR_SZ);
		if (r < 0) {
			ERR("libusb_get_string_descriptor_ascii failed: %s\n", libusb_strerror(r));
			LEAVE
			return -1;
		}
		strncpy(dev->model, model, strlen(model));
	}
	if (desc.iSerialNumber) {
		r = libusb_get_string_descriptor_ascii(dev->handle, desc.iSerialNumber,
				(unsigned char *)serial, MAX_STR_SZ);
		if (r < 0) {
			ERR("libusb_get_string_descriptor_ascii failed: %s\n", libusb_strerror(r));
			LEAVE
			return -1;
		}
		strncpy(dev->serial, serial, strlen(serial));
	}

	LEAVE
	return 0;
}

static int __create_dev(libusb_device *dev, usb_dev_t **new_dev) {
	usb_dev_t *_dev;
	struct libusb_device_descriptor desc;
	int r;

	ENTER

	ASSERT(dev != NULL);

	r = libusb_get_device_descriptor(dev, &desc);
	if (r) {
		ERR("libusb_get_device_descriptor failed: %s\n", libusb_strerror(r));
		return -1;
	}

	_dev = calloc(1, sizeof(usb_dev_t));
	_dev->dev = dev;
	_dev->vid = desc.idVendor;
	_dev->pid = desc.idProduct;
	_dev->bulk_endpoint = 0x86;

	DBG("device create %04X:%04X, %s %s %s\n",
			_dev->vid, _dev->pid, _dev->manf, _dev->model, _dev->serial);

	*new_dev = _dev;

	LEAVE
	return 0;
}

static int __deregister_user_callback(int handle) {
	usb_cb_t *del_cb, *tmp_cb;

	ENTER

	list_for_each_entry_safe(del_cb, tmp_cb, &g_store->cbs.list, list) {
		if ((handle == 0) || (del_cb->handle == handle)) {
			/* de-register specific callback or all callback (handle == 0) */
			list_del(&del_cb->list);
			g_store->nr_cbs--;

			DBG("de-registered callback %d from list, have %d callbacks\n",
					del_cb->handle, g_store->nr_cbs);

			free(del_cb);

			if (handle != 0) {
				/* de-registered one specific callback */
				LEAVE
				return 0;
			}
		}
	}

	if (handle != 0) {
		/* we have not found our specific callback */
		DBG("not found user callback with handle %d\n", handle);
		LEAVE
		return -1;
	}

	/* all callbacks de-registered */
	ASSERT(g_store->nr_cbs== 0);
	ASSERT(list_empty(&g_store->cbs.list) == 1);

	LEAVE
	return 0;
}

static int __do_user_callbacks(usb_dev_t *dev) {
	int r;
	usb_cb_t *cb;

	ENTER

	list_for_each_entry(cb, &g_store->cbs.list, list) {
		r = cb->fn(dev);
		if (r) {
			ERR("callback %d failed!\n", cb->handle);
		}
	}

	LEAVE
	return 0;
}

static int __open_dev(usb_dev_t *dev) {
	int r;
	int config;
	libusb_device_handle *handle;

	ENTER

	r = libusb_open(dev->dev, &handle);
	if (r) {
		ERR("libusb_open failed: %s!\n", libusb_strerror(r));
		LEAVE
		return -1;
	}
	DBG("opened USB device..\n");

	r = libusb_kernel_driver_active(handle, 0);
	if (r == 1) {
		DBG("kernel driver is active for interface 0\n");
		r = libusb_detach_kernel_driver(handle, 0);
		if (r) {
			ERR("libusb_detach_kernel_driver failed: %s\n", libusb_strerror(r));
			LEAVE
			return -1;
		}
	} else if (r == 0) {
		DBG("kernel driver is NOT active for interface 0\n");
	} else {
		ERR("libusb_kernel_driver_active failed: %s\n", libusb_strerror(r));
		LEAVE
		return -1;
	}

	r = libusb_get_configuration(handle, &config);
	if (r) {
		ERR("libusb_get_configuration failed: %s\n", libusb_strerror(r));
		LEAVE
		return -1;
	}
	DBG("current configuration is %d\n", config);

	r = libusb_claim_interface(handle, 0);
	if (r) {
		ERR("libusb_claim_interface failed: %s\n", libusb_strerror(r));
		LEAVE
		return -1;
	}
	DBG("claimed interface 0\n");
	DBG("handle %p\n", handle);

	dev->handle = handle;
	dev->claimed = 1;

	LEAVE
	return 0;
}

static int __close_dev(usb_dev_t *dev) {
	int r;

	ENTER

	if (! dev->handle) {
		DBG("USB device not opened\n");
		LEAVE
		return 0;
	}

	if (dev->claimed) {
		r = libusb_release_interface(dev->handle, 0);
		if (r) {
			ERR("libusb_release_interface() failed: %s\n", libusb_strerror(r));
		}
		DBG("released interface 0\n");
	}
	DBG("handle %p\n", dev->handle);

	libusb_close(dev->handle);
	dev->handle = NULL;
	dev->claimed = 0;

	DBG("closed USB device\n");

	LEAVE
	return 0;
}

static int __delete_dev(usb_dev_t *dev) {
	usb_dev_t *del_dev, *tmp_dev;

	ENTER

	list_for_each_entry_safe(del_dev, tmp_dev, &g_store->devs.list, list) {
		if ((dev == NULL) || (dev == del_dev)) {
			/* remove a specific device or all devices (dev == NULL) */
			del_dev->state = e_usb_dev_state_removed;
			__do_user_callbacks(del_dev);
			__close_dev(del_dev);

			list_del(&del_dev->list);
			g_store->nr_devs--;

			DBG("device remove %04X:%04X, %s %s %s, have %d devices\n",
					del_dev->vid, del_dev->pid, del_dev->manf, del_dev->model,
					del_dev->serial, g_store->nr_devs);

			free(del_dev);

			if (dev != NULL) {
				/* removed one specific device */
				LEAVE
				return 0;
			}
		}
	}

	if (dev != NULL) {
		/* we have not found our specific device */
		ERR("device not found in the store!\n");
		LEAVE
		return -1;
	}

	/* all devices de-registered */
	ASSERT(g_store->nr_devs == 0);
	ASSERT(list_empty(&g_store->devs.list) == 1);

	LEAVE
	return 0;
}

static int __dev_arrived(libusb_device *dev) {
	int r;
	usb_dev_t *new_dev;

	ENTER

	ASSERT(dev != NULL);

	r = __create_dev(dev, &new_dev);
	if (r) {
		return -1;
	}

	/* add the new device to the list */
	new_dev->state = e_usb_dev_state_arrived;
	list_add_tail(&(new_dev->list), &g_store->devs.list);
	g_store->nr_devs++;

	DBG("device arrive %04X:%04X, have %d devices\n",
			new_dev->vid, new_dev->pid, g_store->nr_devs);

	LEAVE
	return 0;
}

static int __dev_handle_event() {
	int r;
	int add;
	usb_dev_t *dev, *tmp_dev;

	ENTER

	list_for_each_entry_safe(dev, tmp_dev, &g_store->devs.list, list) {
		if (dev->state == e_usb_dev_state_left) {
			r = __delete_dev(dev);
		} else if (dev->state == e_usb_dev_state_arrived) {
			r = __open_dev(dev);
			if (r) {
				continue;
			}

			add = 0;
			if (dev->pid == CCS100_VANILLA_PID ||
				dev->pid == CCS125_VANILLA_PID ||
				dev->pid == CCS150_VANILLA_PID ||
				dev->pid == CCS175_VANILLA_PID ||
				dev->pid == CCS200_VANILLA_PID) {
				r = __load_firmware(dev);
				dev->state = e_usb_dev_state_unknown;
			} else if (dev->pid == CCS100_READY_PID ||
				dev->pid == CCS125_READY_PID ||
				dev->pid == CCS150_READY_PID ||
				dev->pid == CCS175_READY_PID ||
				dev->pid == CCS200_READY_PID) {
				r = __get_strings(dev);
				add = 1;
			}
			__close_dev(dev);

			if (r) {
				free(dev);
				continue;
			}

			if (add) {
				dev->state = e_usb_dev_state_added;
				DBG("device add %04X:%04X, %s %s %s, have %d devices\n",
						dev->vid, dev->pid, dev->manf, dev->model,
						dev->serial, g_store->nr_devs);
				__do_user_callbacks(dev);
			}
		}
	}

	LEAVE
	return 0;
}

static int __dev_left(libusb_device *dev) {
	usb_dev_t *usb_dev;

	ENTER

	list_for_each_entry(usb_dev, &g_store->devs.list, list) {
		if (dev == usb_dev->dev) {
			usb_dev->state = e_usb_dev_state_left;
			DBG("device left %04X:%04X, %s %s %s, have %d devices\n",
					usb_dev->vid, usb_dev->pid, usb_dev->manf, usb_dev->model,
					usb_dev->serial, g_store->nr_devs);
			break;
		}
	}

	LEAVE
	return 0;
}

static void *__event_thread(void *arg) {
	struct timeval now, wait;
	ENTER

    pthread_mutex_lock(&g_store->mutex);
	while (g_store->run) {
		wait.tv_sec = 1;
		wait.tv_usec = 0;

	    pthread_mutex_unlock(&g_store->mutex);
		libusb_handle_events_timeout_completed(NULL, &wait, NULL);
	    pthread_mutex_lock(&g_store->mutex);

	    if (g_store->run == 0) {
			INF("leaving..\n");
			break;
	    }
		gettimeofday(&now, NULL);
		DBG2("now %ld, have %d devices..\n", now.tv_sec, g_store->nr_devs);

		/* device I/O needs to be done outside the libusb event handling */
		__dev_handle_event();
	}

	pthread_cond_broadcast(&g_store->cond);
	pthread_mutex_unlock(&g_store->mutex);

	LEAVE
	return NULL;
}

static int __init_store() {
	int r;
	pthread_mutexattr_t attr;

	ENTER

	ASSERT(g_store == NULL);

	g_store = calloc(1, sizeof(usb_store_t));
	/* must be recursive mutex! */
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&g_store->mutex, &attr);
	pthread_cond_init(&g_store->cond, NULL);
	INIT_LIST_HEAD(&g_store->devs.list);
	INIT_LIST_HEAD(&g_store->cbs.list);
	INIT_LIST_HEAD(&g_store->hp_cbs.list);
	g_store->run = 1;

    r = pthread_create(&g_store->thr_id, NULL, &__event_thread, NULL);
    if (r) {
    	ERR("pthread_create failed: %s\n", strerror(r));
    	LEAVE
    	return -1;
    }

    LEAVE
	return 0;
}

static void __destroy_store() {
	struct timespec ts;

	ENTER

	clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 2;
    pthread_mutex_lock(&g_store->mutex);
	g_store->run = 0;
    pthread_cond_timedwait(&g_store->cond, &g_store->mutex, &ts);
    pthread_mutex_unlock(&g_store->mutex);
    pthread_join(g_store->thr_id, NULL);
    pthread_mutex_destroy(&g_store->mutex);
	pthread_cond_destroy(&g_store->cond);

	/* delete all devices */
	__delete_dev(NULL);

	/* de-register all callbacks */
	__deregister_user_callback(0);

	free(g_store);
	g_store = NULL;

	LEAVE
}

static int __hotplug_callback(struct libusb_context *ctx,
	struct libusb_device *dev, libusb_hotplug_event event, void *user_data) {

	ENTER

	DBG("Device %p\n", dev);

    pthread_mutex_lock(&g_store->mutex);

	if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
		DBG("USB device %p arrived!\n", dev);
		__dev_arrived(dev);
	} else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
		DBG("USB device %p left!\n", dev);
		__dev_left(dev);
	} else {
		ERR("unhandled event %d\n", event);

	}

    pthread_mutex_unlock(&g_store->mutex);

    LEAVE
	return 0;
}

int usbListenForDevices(int vid, int pid) {
	int r;
	usb_hp_cb_t *usb_hp_cb;
	libusb_hotplug_callback_handle cb_handle;

	ENTER

	r = libusb_hotplug_register_callback(NULL,
			LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED |
			LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
			LIBUSB_HOTPLUG_ENUMERATE,
			vid, pid,
			LIBUSB_HOTPLUG_MATCH_ANY,
			__hotplug_callback, NULL, &cb_handle);
	if (r) {
		ERR("libusb_hotplug_register_callback failed: %s\n", libusb_strerror(r));
		LEAVE
		return -1;
	}
	usb_hp_cb = (usb_hp_cb_t *)calloc(1, sizeof(usb_hp_cb_t));
	usb_hp_cb->cb_handle = cb_handle;
	usb_hp_cb->vid = vid;
	usb_hp_cb->pid = pid;
	list_add_tail(&usb_hp_cb->list, &g_store->hp_cbs.list);
	DBG("libusb_hotplug_register_callback for VID/PID: 0x%04X:0x%04X\n", vid, pid);

	LEAVE
	return 0;
}

int usbRegisterCallback(usb_cb_fn fn, int *handle) {
	usb_cb_t *usb_cb;

	ENTER

	usb_cb = (usb_cb_t *)calloc(1, sizeof(usb_cb_t));
	usb_cb->fn = fn;
	g_store->idx_cbs++;
	usb_cb->handle = g_store->idx_cbs;
	list_add_tail(&usb_cb->list, &g_store->cbs.list);
	g_store->nr_cbs++;
	*handle = usb_cb->handle;

	DBG("registered user callback with handle %d, have %d callbacks\n",
			usb_cb->handle, g_store->nr_cbs);
	LEAVE
	return 0;
}

int usbDeregisterCallback(int handle) {
	int r;

	ENTER

	r = __deregister_user_callback(handle);

	LEAVE
	return r;
}

int usbTriggerCallback(int handle) {
	usb_dev_t *dev;

	ENTER

	if (list_empty(&g_store->devs.list)) {
		LEAVE
		return 0;
	}

	list_for_each_entry(dev, &g_store->devs.list, list) {
		if ((dev->state == e_usb_dev_state_added) ||
			(dev->state == e_usb_dev_state_removed)) {
			__do_user_callbacks(dev);
		}
	}

	LEAVE
	return 0;
}

int usbGetDevCountFromStore() {

	ENTER
	LEAVE
	return g_store->nr_devs;
}

int usbGetDevFromStore(int dev_idx, usb_dev_t **dev) {
	usb_dev_t *tmp;
	int i;

	ENTER

	if (list_empty(&g_store->devs.list)) {
		DBG("device list is empty\n");
		*dev = NULL;
		LEAVE
		return -1;
	}

	i = 0;
	list_for_each_entry(tmp, &g_store->devs.list, list) {
		if (dev_idx == i) {
			*dev = tmp;
			DBG("giving device at %d\n", i);
			LEAVE;
			return 0;
		}
		i++;
	}

	ERR("device at %d not in the store!\n", dev_idx);
	LEAVE
	return -1;
}

int usbInit(void) {
	int r;

	ENTER

	ASSERT(g_store == NULL);

	r = libusb_init(NULL);
	if (r) {
		ERR("libusb_init failed: %s!\n", libusb_strerror(r));
		LEAVE
		return -1;
	}
	libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_ERROR);

	r = __init_store();
	if (r) {
		libusb_exit(NULL);
		LEAVE
		return -1;
	}

	ASSERT(g_store != NULL);

	LEAVE
	return 0;
}

void usbExit(void) {
	usb_hp_cb_t *hp_cb;

	ENTER

	list_for_each_entry(hp_cb, &g_store->hp_cbs.list, list) {
		libusb_hotplug_deregister_callback(NULL, hp_cb->cb_handle);
	}

	__destroy_store();

	libusb_exit(NULL);

	LEAVE
}

int usbOpen(usb_dev_t *dev) {
	int r;

	ENTER

	r = __open_dev(dev);

	LEAVE
	return r;
}

int usbClose(usb_dev_t *dev) {

	ENTER

	__close_dev(dev);

	LEAVE
	return 0;
}

int usbControl(usb_dev_t *dev,
		uint8_t bmRequestType,
		uint8_t bRequest,
		uint16_t wValue,
		uint16_t wIndex,
		unsigned char *data,
		uint16_t wLength,
		unsigned int timeout) {

	int r;

	ENTER2

	ASSERT(dev != NULL);
	if (dev->state != e_usb_dev_state_added) {
		ERR("device not in ADDED state, state = %d!\n", dev->state);
		LEAVE2
		return -EIO;
	}

	r = libusb_control_transfer(
			dev->handle,				/* handle */
			bmRequestType, 				/* bmRequestType */
			bRequest,					/* bRequest */
			wValue,						/* wValue */
			wIndex,						/* wIndex */
			data,						/* data buffer */
			wLength,					/* data size */
			timeout						/* timeout */
			);
	if (r < 0) {
    	ERR("libusb_control_transfer failed: %s\n", libusb_strerror(r));
	}

	__hexdump(stdout, data, wLength);

	LEAVE2

	return r;
}

int usbBulk(usb_dev_t *dev,
		unsigned char *data,
		int wLength,
		int *wActualLength,
		unsigned int timeout) {

	int r;

	ENTER2

	ASSERT(dev != NULL);
	if (dev->state != e_usb_dev_state_added) {
		ERR("device not in ADDED state, state = %d!\n", dev->state);
		LEAVE2
		return -EIO;
	}

	r = libusb_bulk_transfer(
			dev->handle,				/* handle */
			dev->bulk_endpoint,			/* endpoint */
			data,						/* data buffer */
			wLength,					/* wLength */
			wActualLength,				/* wActualLength */
			timeout						/* timeout */
			);
	if (r < 0) {
    	ERR("libusb_bulk_transfer failed: %s\n", libusb_strerror(r));
	}

	__hexdump(stdout, data, wLength);

	LEAVE

	return r;
}
