/*
 *  Motorola Solutions Bluetooth HID driver
 *
 *  Copyright 2014 Motorola Solutions, Inc. All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/cdev.h>

#include <linux/hid.h>
#include <linux/version.h>
#include <linux/kernel.h>

#include "btm_pkt_queue.h"
#include "user.h"

#ifndef NO_BUILD_CFG_H
#include "build_cfg.h"
#endif

/*==================================================================================================
                                            MACROS
===================================================================================================*/
#define DRIVER_VERSION              "1.0.0"
#define DRIVER_AUTHOR               "Motorola, Inc. http://www.motorolasolutions.com"
#define DRIVER_DESC                 "Motorola Bluetooth HID driver"
#define DRIVER_LICENSE              "GPL v2"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);
MODULE_VERSION(DRIVER_VERSION);

#define CHRDEV_NAME                 "btmhid_chrdev"
#define BTMHID_CLASS_NAME           "btmhid_class"
#define FILE_NAME                   "bthid0"

#if (defined CFG_BTMHID_KERNEL_TRACE_LEVEL) && (CFG_BTMHID_KERNEL_TRACE_LEVEL > 0)
    #define BT_DBG(fmt, args...)  printk(KERN_DEBUG "btmhid driver %s[%u]: " fmt "\n", __FUNCTION__, __LINE__,  ##args )
    #define BT_ERR(fmt, args...)  printk(KERN_ERR "btmhid driver %s[%u]: " fmt "\n", __FUNCTION__, __LINE__,  ##args )
#else
    #define BT_DBG(fmt, args...)
    #define BT_ERR(fmt, args...)
#endif

/*==================================================================================================
                                            TYPEDEFS AND ENUMS
===================================================================================================*/
typedef struct btmhid_dev
{
    struct cdev         cdev;

    unsigned            firstminor;

    dev_t               dev;

    struct class        *btmhid_class;

    struct device       *device;

    /* Read Data */
    struct pkt_queue     ev_pkt_queue;

}   btmhid_dev;

/*==================================================================================================
                                            GLOBAL VARIABLES
===================================================================================================*/

static unsigned have_hid = 0;

static btmhid_dev *_btmhid_dev = NULL;

static const struct hid_device_id hidp_table[] = { {HID_BLUETOOTH_DEVICE(HID_ANY_ID, HID_ANY_ID)}, {} };

static struct hid_driver hidp_driver =
{
       .name = "motorola-bluetooth",
       .id_table = hidp_table,
};

static struct hid_descriptor DefaultHidDescriptor =
{
    .bLength = 0x09, // length of HID descriptor
    .bDescriptorType = 0x21, // descriptor type == HID descriptor (0x21, HID_DT_HID)
    .bcdHID = 0x0100, // hid spec release
    .bCountryCode = 0x00, // country code == Not Specified
    .bNumDescriptors = 0x01, // number of HID class descriptors
};

static char *hid_types[] = {"Device", "Pointer", "Mouse", "Device", "Joystick",
                "Gamepad", "Keyboard", "Keypad", "Multi-Axis Controller"};

/*==================================================================================================
                                            FUNCTION PROTOTYPES
===================================================================================================*/

static int register_input_device(device_parameters_t *parameters, struct hid_device **hid_device);
static void deregister_input_device(struct hid_device *hid_device);

static int hid_open(struct hid_device *hid_device)
{
    BT_DBG("<<<<<<");
    hid_device->open ++;
    BT_DBG("hid_device->open = %d", hid_device->open);
    return 0;
}

static int btmhid_open(struct inode *inode, struct file *filep)
{
    BT_DBG("<<<<<<");
    hid_clear_pkt_queue(&_btmhid_dev->ev_pkt_queue);
    return 0;
}

static void hid_close(struct hid_device *hid_device)
{
    BT_DBG("<<<<<<");
    hid_device->open --;
    BT_DBG("hid_device->open = %d", hid_device->open);
}

static int btmhid_start(struct hid_device * dev) {
    BT_DBG("Start catched!");
    return 0;
}

static int btmhid_parse(struct hid_device * dev) {
    device_parameters_t *params = hid_get_drvdata(dev);
    BT_DBG("Parse catched!");
    return hid_parse_report(dev, params->rep_desc, params->rep_desc_len);
}

static void btmhid_stop(struct hid_device * dev) {
    BT_DBG("Stop catched!");
    dev->claimed = 0; //TODO: do we need this one?
}

static int hidinput_input_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
    int err;
    struct hid_device *hid_device = input_get_drvdata(dev);
    struct hid_field *field;
    struct sk_buff *skb = NULL;
    struct get_report_params_t *grp = NULL;
    int offset;
    unsigned char *reportBuffer = NULL;
    unsigned short reportBufferLenInBytes;
    EnumReportType reportType;
    int retval = 0;
    BT_DBG("<<<<<<");

    if (type != EV_LED)
    {
        BT_DBG("type != EV_LED");
        return -1;
    }

    offset = hidinput_find_field(hid_device, type, code, &field);
    if (offset == -1)
    {
        BT_DBG("event field not found");
        return -1;
    }

    hid_set_field(field, offset, value);

    if (field->report->type == HID_INPUT_REPORT)
    {
        BT_DBG("HID_INPUT_REPORT");
    }
    else if (field->report->type == HID_OUTPUT_REPORT)
    {
        BT_DBG("HID_OUTPUT_REPORT");
    }
    else if (field->report->type == HID_FEATURE_REPORT)
    {
        BT_DBG("HID_FEATURE_REPORT");
    }
    else
    {
        BT_DBG("unknown report type");
    }

    if ((field->report->type != HID_OUTPUT_REPORT) && (field->report->type != HID_FEATURE_REPORT))
    {
        BT_DBG("neither HID_OUTPUT_REPORT nor HID_FEATURE_REPORT, exit");
        return -1;
    }
    else if (field->report->type == HID_OUTPUT_REPORT)
    {
        reportType = OUTPUT_REPORT;
    }
    else
    {
        reportType = FEATURE_REPORT;
    }

    BT_DBG("report type = 0x%X", field->report->type);
    BT_DBG("report id = 0x%X", field->report->id);
    BT_DBG("report size(bits) = 0x%X", field->report->size);

    reportBufferLenInBytes = 1 + (field->report->size) / 8;
    reportBuffer = kmalloc(reportBufferLenInBytes, GFP_ATOMIC);
    if (reportBuffer == NULL)
    {
        BT_DBG("kmalloc failed");
        return -ENOMEM;
    }

    grp = kmalloc(sizeof(get_report_params_t), GFP_KERNEL);

    if (grp == NULL)
    {
        BT_DBG("kmalloc failed");
        kfree(reportBuffer);
        return -ENOMEM;
    }

    hid_output_report(field->report, reportBuffer);

    grp->ptr_to_dev = (void *)hid_device;
    grp->report_type = reportType;
    grp->length = reportBufferLenInBytes;
    grp->data = reportBuffer;

    BT_DBG("Report to device with pointer = %p", grp->ptr_to_dev);
    BT_DBG("Report Type = %d", grp->report_type);
    BT_DBG("Report Len = %d", grp->length);

    skb = alloc_skb(sizeof(get_report_params_t), GFP_KERNEL);
    if (skb == NULL)
    {
        BT_DBG("alloc_skb failed");
        kfree(grp->data);
        kfree(grp);
        return -ENOMEM;
    }

    BT_DBG("Put report into queue");

    skb_put(skb, sizeof(get_report_params_t));//add check return value??

    skb->data = (void *)grp;

    err = hid_enqueue_pkt(&_btmhid_dev->ev_pkt_queue, skb);
    BT_DBG("Packet was moved to ev_pkt_queue");
    if (err < 0)
    {
        BT_DBG("Unable to push skb to queue(%d)", err);
        kfree(grp->data);
        kfree(grp);
        return err;
    }

    BT_DBG(">>>>>>");
    return retval;
}

static struct hid_ll_driver btm_hid_driver = {
    .start = btmhid_start,
    .stop = btmhid_stop,
    .parse = btmhid_parse,
    .open  = hid_open,
    .close = hid_close,
    .hidinput_input_event = hidinput_input_event
};

static struct hid_device * alloc_hid_dev(void)
{
    struct hid_device *hid;
    int err;
    BT_DBG("<<<<<<");

    hid = hid_allocate_device();
    if (IS_ERR(hid))
    {
        err = PTR_ERR(hid);
        BT_DBG("error allocating device: %d!", err);
        return NULL;
    }

    BT_DBG(">>>>>>");
    return hid;
}

static int hid_output_raw_report(struct hid_device *hid_device, __u8 *buf, size_t count, unsigned char p)
{
    BT_DBG("Started");
    return 0;
}

static struct hid_device *GetHidDevice(device_parameters_t * parameters)
{
    struct hid_descriptor *hdesc; // pointer to HID descriptor
    struct hid_device *hid_device;
    BT_DBG("<<<<<<");
    if (parameters == NULL)
    {
        BT_ERR("parameters is NULL");
        return NULL;
    }

    if (!parameters->rep_desc_len || parameters->rep_desc_len > HID_MAX_DESCRIPTOR_SIZE)
    {
        BT_ERR("invalid report descriptor size (%u)", parameters->rep_desc_len);
        return NULL;
    }

    if (parameters->name_len == 0)
    {
        BT_ERR("parameters name len is 0");
        return NULL;
    }
    BT_DBG("name len = %u", parameters->name_len);

    if (parameters->name_len > sizeof(hid_device->name) - 1)
    {
        BT_ERR("Too big parameters name len");
        return NULL;
    }
    if (parameters->name == NULL)
    {
        BT_ERR("parameters name is null");
        return NULL;
    }
    BT_DBG("Parameters name = %s", parameters->name);

    // final initialization of DefaultHidDescriptor
    DefaultHidDescriptor.desc[0].bDescriptorType = 0x22; // REPORT descriptor
    DefaultHidDescriptor.desc[0].wDescriptorLength = parameters->rep_desc_len;
    hdesc = &DefaultHidDescriptor;

    if (! (hid_device = alloc_hid_dev()))
    {
        BT_DBG("alloc_hid_Dev returned NULL!");
        return NULL;
    }

    hid_device->bus             = BUS_BLUETOOTH;
    hid_device->dev.parent      = NULL;
    hid_device->ll_driver       = &btm_hid_driver;
    hid_device->vendor          = parameters->vid;
    hid_device->product         = parameters->pid;
    hid_device->version         = parameters->version;
    hid_device->country         = hdesc->bCountryCode;

    strncpy(hid_device->name, parameters->name, parameters->name_len);
    if (!strlen(hid_device->name))
    {
        snprintf(hid_device->name, sizeof(hid_device->name), "HID %04x:%04x",
            (unsigned short)parameters->vid, (unsigned short)parameters->pid);
        hid_device->name[sizeof(hid_device->name)-1] = 0;
    }

    hid_device->phys[0] = 0;
    hid_device->uniq[0] = 0;
    hid_device->hid_output_raw_report = &hid_output_raw_report;

    hid_set_drvdata(hid_device, parameters);

    BT_DBG(">>>>>>");
    return hid_device;
}

int register_input_device(device_parameters_t *parameters, struct hid_device **hid_device)
{
    unsigned i;
    char *c;
    int err;
    BT_DBG("<<<<<<");

    if (have_hid == 0)
    {
        BT_DBG("Register hid driver");
        if (hid_register_driver(&hidp_driver) != 0)
        {
            BT_ERR("Cannot register hid driver");
            return -1;
        }
    }

    *hid_device = GetHidDevice(parameters);
    if (NULL == *hid_device)
    {
        BT_DBG("GetHidDevice failed");
        hid_unregister_driver(&hidp_driver);
        return -1;
    }

    have_hid++;

    BT_DBG("adding device...");
    err = hid_add_device(*hid_device);
    if (err < 0)
    {
        BT_ERR("hid_add_device failed (%d)", err);
        hid_destroy_device(*hid_device);
        hid_unregister_driver(&hidp_driver);
        return -1;
    }
    else
    {
        BT_DBG("hid_add_device fine!");
    }
    c = "Device";
    for (i = 0; i < (*hid_device)->maxcollection; i++) {
        if ((*hid_device)->collection[i].type == HID_COLLECTION_APPLICATION &&
            ((*hid_device)->collection[i].usage & HID_USAGE_PAGE) == HID_UP_GENDESK &&
            ((*hid_device)->collection[i].usage & 0xffff) < ARRAY_SIZE(hid_types)) {
            c = hid_types[(*hid_device)->collection[i].usage & 0xffff];
            break;
        }
    }

    BT_DBG(": Bluetooth HID v%x.%02x %s [%s]", (*hid_device)->version >> 8, (*hid_device)->version & 0xff, c, (*hid_device)->name);

    BT_DBG(">>>>>>");
    return 0;
}

static int btmhid_flush(struct file *filep, fl_owner_t id)
{
    BT_DBG("<<<<<<");
    hid_kill_waiters(&_btmhid_dev->ev_pkt_queue);
    return 0;
}

void deregister_input_device(struct hid_device *hid_device)
{
    device_parameters_t *params = NULL;
    BT_DBG("<<<<<<");

    params = hid_get_drvdata(hid_device);
    kfree(params->name);
    kfree(params->rep_desc);
    kfree(params);

    hid_destroy_device(hid_device);
    BT_DBG("hid_free_device returned");

    if (have_hid == 1)
    {
        hid_unregister_driver(&hidp_driver);
        BT_DBG("hid_unregister_driver returned");
    }

    have_hid--;
    BT_DBG(">>>>>>");
}

static long btmhid_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    int error = 0;
    int retval = 0;

    BT_DBG("<<<<<<");

    if (_IOC_TYPE(cmd) != BTMHID_IOC_GROUP)
    {
        BT_DBG("_IOC_TYPE(cmd) = %d != BTMHID_IOC_GROUP = %d", _IOC_TYPE(cmd), BTMHID_IOC_GROUP);
        return -ENOTTY;
    }

    if (_IOC_NR(cmd) > BTMHID_IOC_MAXNR)
    {
        BT_DBG("_IOC_NR(cmd) > BTMHID_IOC_MAXNR");
        return -ENOTTY;
    }

    switch (cmd)
    {
        case BTMHID_IOCTL_ADD_DEVICE:
        {
            struct hid_device *hid_device = NULL;
            device_t *dev_params = NULL;
            unsigned char *rep_desc = NULL;
            unsigned char *device_name = NULL;

            BT_DBG("BTMHID_IOCTL_ADD_DEVICE");

            dev_params = kmalloc(sizeof(device_t), GFP_KERNEL);
            if (dev_params == NULL)
            {
                BT_ERR("Cannot allocate dev_params");
                return -ENOMEM;
            }

            error = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
            if (error)
            {
                BT_ERR("access_ok(arg) detected error = 0x%X", error);
                kfree(dev_params);
                return -EFAULT;
            }

            if (0 != copy_from_user(dev_params, (void __user*)arg, sizeof(device_t)))
            {
                BT_ERR("copy_from_user(arg) failed");
                kfree(dev_params);
                return -EFAULT;
            }

            BT_DBG("rep_desc_len = %u", dev_params->info.rep_desc_len);
            BT_DBG("device name len = %u", dev_params->info.name_len);

            error = !access_ok(VERIFY_READ, (void __user*)dev_params->info.rep_desc, dev_params->info.rep_desc_len);
            if (error)
            {
                BT_ERR("access_ok(parameters) detected error = 0x%X", error);
                kfree(dev_params);
                return -EFAULT;
            }

            rep_desc = kmalloc(dev_params->info.rep_desc_len, GFP_KERNEL);
            if (rep_desc == NULL)
            {
                BT_ERR("Cannot allocate rep_desc");
                kfree(dev_params);
                return -ENOMEM;
            }

            if (0 != copy_from_user(rep_desc, (void __user*)dev_params->info.rep_desc, dev_params->info.rep_desc_len))
            {
                BT_ERR("Cannot copy from user rep_desc");
                kfree(rep_desc);
                kfree(dev_params);
                return -EFAULT;
            }

            dev_params->info.rep_desc = rep_desc;

            error = !access_ok(VERIFY_READ, (void __user*)dev_params->info.name, dev_params->info.name_len);
            if (error)
            {
                BT_ERR("access_ok(parameters) detected error = 0x%X", error);
                kfree(dev_params->info.rep_desc);
                kfree(dev_params);
                return -EFAULT;
            }

            device_name = kmalloc(dev_params->info.name_len, GFP_KERNEL);
            if (device_name == NULL)
            {
                BT_ERR("Cannot allocate device_name");
                kfree(dev_params->info.rep_desc);
                kfree(dev_params);
                return -ENOMEM;
            }

            if (0 != copy_from_user(device_name, (void __user*)dev_params->info.name, dev_params->info.name_len))
            {
                BT_ERR("Cannot copy from user device_name");
                kfree(device_name);
                kfree(dev_params->info.rep_desc);
                kfree(dev_params);
                return -EFAULT;
            }

            dev_params->info.name = device_name;

            BT_DBG("device_name = %s",dev_params->info.name);

            if (register_input_device(&(dev_params->info), &hid_device) < 0)
            {
                BT_DBG("Register input device failes....");
                kfree(dev_params->info.name);
                kfree(dev_params->info.rep_desc);
                kfree(dev_params);
                return -EFAULT;
            }
            else
            {
                BT_DBG("ptr to hid_device = %p", hid_device);
                dev_params->ptr_to_dev = (void *)hid_device;
            }

            error = !access_ok(VERIFY_WRITE, ((device_t *)arg)->ptr_to_dev, sizeof(dev_params->ptr_to_dev));
            if (error)
            {
                BT_DBG("access_ok(device_parameters_t) failed for writting");
                kfree(dev_params->info.name);
                kfree(dev_params->info.rep_desc);
                kfree(dev_params);
                return -EFAULT;
            }

            if (0 != copy_to_user(&(((device_t *)arg)->ptr_to_dev), &(dev_params->ptr_to_dev), sizeof(((device_t *)arg)->ptr_to_dev)))
            {
                BT_DBG("Cannot copy repDesc to user!");
                kfree(dev_params->info.name);
                kfree(dev_params->info.rep_desc);
                kfree(dev_params);
                return -EFAULT;
            }

            break;
        }

        case BTMHID_IOCTL_DELETE_DEVICE:
        {
            struct hid_device *ptr_to_dev = NULL;
            BT_DBG("BTMHID_IOCTL_DELETE_DEVICE");
            error = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
            if (error)
            {
                BT_DBG("access_ok(arg) detected error = 0x%X", error);
                return -EFAULT;
            }

            if (0 != copy_from_user(&ptr_to_dev, (void __user*)arg, sizeof(void *)))
            {
                BT_DBG("copy_from_user(arg) failed");
                return -EFAULT;
            }

            BT_DBG("Deleting device with pointer = %p", ptr_to_dev);
            deregister_input_device(ptr_to_dev);
            break;
        }

        case BTMHID_IOCTL_SET_REPORT:
        {
            set_report_params_t params;
            unsigned i = 0;
            unsigned char* data = NULL;
            struct hid_device *hid_dev = NULL;
            BT_DBG("BTMHID_IOCTL_SET_REPORT caught");

            error = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
            if (error)
            {
                BT_DBG("access_ok(arg) detected error = 0x%X", error);
                return -EFAULT;
            }

            if (0 != copy_from_user(&params, (void __user*)arg, sizeof(set_report_params_t)))
            {
                BT_DBG("copy_from_user(arg) failed");
                return -EFAULT;
            }
            error = !access_ok(VERIFY_READ, (void __user*)params.data, params.length);
            if (error)
            {
                BT_DBG("access_ok(params.data) detected error = 0x%X", error);
                return -EFAULT;
            }

            data = kmalloc(params.length, GFP_KERNEL);
            if (data == NULL)
            {
                BT_ERR("Cannot allocate data");
                return -ENOMEM;
            }

            if (0 != copy_from_user(data, (void __user*)params.data, params.length))
            {
                BT_DBG("copy_from_user(params.data) failed");
                kfree(data);
                return -EFAULT;
            }

            hid_dev = params.ptr_to_dev;
            BT_DBG("interrupt = %d", params.interrupt);
            BT_DBG("Set report hid_device = %p", hid_dev);

            if (params.report_type == INPUT_REPORT)
            {
                for (i = 0; i < params.length; i++)
                {
                    BT_DBG("params.data[%u] = 0x%02X", i, data[i]);
                }
                BT_DBG("INPUT_REPORT");
                retval = hid_input_report(hid_dev, HID_INPUT_REPORT, data, params.length, params.interrupt);
            }
            else if (params.report_type == FEATURE_REPORT)
            {
                BT_DBG("FEATURE_REPORT");
                retval = hid_input_report(hid_dev, HID_FEATURE_REPORT, data, params.length, params.interrupt);
            }
            else
            {
                BT_DBG("Invalid report type");
                retval = -EINVAL;
            }

            kfree(data);
            break;
        }

        case BTMHID_IOCTL_GET_REPORT:
        {
            struct sk_buff* skb = NULL;
            get_report_params_t *grp = NULL;
            int err = 0;
            BT_DBG("BMHID_IOCTL_GET_REPORT");

            err = hid_dequeue_pkt(&_btmhid_dev->ev_pkt_queue, &skb);
            if (err < 0)
            {
                BT_ERR("Cannot get skb");
                return -EFAULT;
            }

            grp = (get_report_params_t *)skb->data;

            BT_DBG("Device ptr_to_dev = %p", grp->ptr_to_dev);
            BT_DBG("Report Type = %d", grp->report_type);
            BT_DBG("Report Len = %d", grp->length);

            error = !access_ok(VERIFY_WRITE, arg, sizeof(get_report_params_t));
            if (error)
            {
                BT_ERR("access_ok(device_parameters_t) failed for writting");
                kfree(grp->data);
                kfree(grp);
                kfree_skb(skb);
                return -EFAULT;
            }

            if (0 != copy_to_user((void __user*)arg, grp, sizeof(get_report_params_t)))
            {
                BT_ERR("Cannot copy get_report_params_t to user!");
                kfree(grp->data);
                kfree(grp);
                kfree_skb(skb);
                return -EFAULT;
            }

            err = hid_put_to_head_pkt(&_btmhid_dev->ev_pkt_queue, skb);
            BT_DBG("Packet put into head ev_pkt_queue");
            if (err < 0)
            {
                BT_DBG("Unable to put skb to head of queue(%d)", err);
                kfree(grp->data);
                kfree(grp);
                return (long)err;
            }
            break;
        }
        case BTMHID_IOCTL_GET_REPORTBUF:
        {
            struct sk_buff* skb = NULL;
            get_report_params_t *grp = NULL;
            int err = 0;
            BT_DBG("BMHID_IOCTL_GET_REPORTBUF");

            err = hid_dequeue_pkt(&_btmhid_dev->ev_pkt_queue, &skb);
            if (err < 0)
            {
                BT_ERR("Cannot get skb");
                return -EFAULT;
            }

            grp = (get_report_params_t *)skb->data;

            BT_DBG("Device ptr_to_dev = %p", grp->ptr_to_dev);
            BT_DBG("report_type = %d", grp->report_type);
            BT_DBG("report_len = %d", grp->length);

            err = !access_ok(VERIFY_WRITE, arg, grp->length);
            if (err)
            {
                BT_ERR("access_ok(length) failed for writting");
                kfree(grp->data);
                kfree(grp);
                kfree_skb(skb);
                return -EFAULT;
            }
            if (0 != copy_to_user((void __user*)arg, grp->data, grp->length))
            {
                BT_DBG("Cannot copy reportBuffer to user!");
                kfree(grp->data);
                kfree(grp);
                kfree_skb(skb);
                return -EFAULT;
            }

            kfree(grp->data);
            kfree(grp);
            kfree_skb(skb);
            break;
        }

        case BTMHID_IOCTL_STOP:
        {
            BT_DBG("BTMHID_IOCTL_STOP");
            hid_kill_waiters(&_btmhid_dev->ev_pkt_queue);
            break;
        }

        default:
        {
            BT_DBG("default");
            retval = -ENOTTY;
            break;
        }
    }

    BT_DBG("<<<<<<");
    return (long)retval;
}

static struct file_operations btmhid_fops = {
    .owner = THIS_MODULE,
    .open = btmhid_open,
    .flush = btmhid_flush,
    .unlocked_ioctl = btmhid_ioctl,
};

static int __init btmhid_init(void)
{
    int result;

    BT_DBG("<<<<<<");
    BT_DBG("%s:%s", DRIVER_VERSION, DRIVER_DESC);

    // allocate _btmhid_dev structure
    _btmhid_dev = kmalloc(sizeof(btmhid_dev), GFP_KERNEL);
    if (_btmhid_dev == NULL)
    {
        BT_DBG("kmalloc failed...");
        return -ENOMEM;
    }

    if (hid_init_pkt_queue(&_btmhid_dev->ev_pkt_queue))
    {
        BT_DBG("Cannot init event queue!");
        kfree(_btmhid_dev);
        return -1;
    }

    // init btmhid_dev structure
    _btmhid_dev->firstminor = 0;
    _btmhid_dev->dev = 0;
    _btmhid_dev->btmhid_class = NULL;
    _btmhid_dev->device = NULL;

    // allocate chrdev region
    result = alloc_chrdev_region(&(_btmhid_dev->dev), _btmhid_dev->firstminor, 1, CHRDEV_NAME);
    if (result != 0)
    {
        BT_DBG("alloc_chrdev_region failed, result = %d", result);
        kfree(_btmhid_dev);
        _btmhid_dev = NULL;
        return result;
    }

    // add char device
    cdev_init(&(_btmhid_dev->cdev), &btmhid_fops);
    _btmhid_dev->cdev.owner = THIS_MODULE;
    _btmhid_dev->cdev.ops = &btmhid_fops;
    result = cdev_add(&(_btmhid_dev->cdev), _btmhid_dev->dev, 1);
    if (result != 0)
    {
        BT_DBG("cdev_add failed, result = %d", result);
        unregister_chrdev_region(_btmhid_dev->dev, 1);
        kfree(_btmhid_dev);
        _btmhid_dev = NULL;
        return result;
    }

    // create file for char device object
    _btmhid_dev->btmhid_class = class_create(THIS_MODULE, BTMHID_CLASS_NAME);
    if (IS_ERR(_btmhid_dev->btmhid_class))
    {
        BT_DBG("class_create failed");
        cdev_del(&(_btmhid_dev->cdev));
        unregister_chrdev_region(_btmhid_dev->dev, 1);
        kfree(_btmhid_dev);
        _btmhid_dev = NULL;
        return result;
    }
    else
    {
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
        _btmhid_dev->device = device_create(_btmhid_dev->btmhid_class, NULL, _btmhid_dev->dev, NULL, FILE_NAME);
    #else
        _btmhid_dev->device = device_create(_btmhid_dev->btmhid_class, NULL, _btmhid_dev->dev, FILE_NAME);
    #endif

        if (IS_ERR(_btmhid_dev->device))
        {
            BT_DBG("device_create failed");
            class_destroy(_btmhid_dev->btmhid_class);
            _btmhid_dev->btmhid_class = NULL;
            cdev_del(&(_btmhid_dev->cdev));
            unregister_chrdev_region(_btmhid_dev->dev, 1);
            kfree(_btmhid_dev);
            _btmhid_dev = NULL;
            return result;
        }
    }

    BT_DBG(">>>>>>");
    return 0;
}

static void btmhid_exit(void)
{
    BT_DBG("<<<<<<");
    if (!_btmhid_dev)
    {
        return;
    }

    // delete file for char device object
    if (!IS_ERR(_btmhid_dev->device))
    {
        device_destroy(_btmhid_dev->btmhid_class, _btmhid_dev->dev);
    }

    if (!IS_ERR(_btmhid_dev->btmhid_class))
    {
        class_destroy(_btmhid_dev->btmhid_class);
    }

    // // deinitialize bluetooth
    // DeinitializeBluetooth();

    // delete char device
    cdev_del(&(_btmhid_dev->cdev));

    // unregister chrdev region
    unregister_chrdev_region(_btmhid_dev->dev, 1);

    // deinit event queue
    hid_deinit_pkt_queue(&_btmhid_dev->ev_pkt_queue);

    // release _btmhid_dev
    kfree(_btmhid_dev);
    _btmhid_dev = NULL;

    BT_DBG("finished");
}

module_init(btmhid_init);
module_exit(btmhid_exit);
