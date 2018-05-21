/*
 *  Motorola Solutions UNI interface
 *
 *  Copyright 2013 Motorola Solutions, Inc. All rights reserved.
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

#ifndef BTM_UNI_H_
#define BTM_UNI_H_

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/skbuff.h>
#include <linux/rfkill.h>
#include <linux/ioctl.h>

#include "btioctl.h"
#include "btm_pkt_queue.h"

#ifndef NO_BUILD_CFG_H
    #include "build_cfg.h"
#else
    #ifndef CFG_BTUART_KERNEL_TRACE_LEVEL
        #define CFG_BTUART_KERNEL_TRACE_LEVEL 0
    #endif
#endif

#define DEVICE_NAME "btuart0"

/* Previous definition in bluetooth.h */
#undef BT_ERR
#undef BT_DBG
#undef BT_INFO

#define BT_INFO(fmt, arg...) printk(KERN_INFO "Bluetooth: " fmt "\n" , ## arg)
#define BT_ERR(fmt, arg...)  printk(KERN_ERR  "btuart: %d: %s: " fmt "\n" , __LINE__, __FUNCTION__ , ## arg)

#define BTUSB_MAGIC 237
#define BTUSB_BLUETOOTH_IOCTL _IOWR(BTUSB_MAGIC, 10, BluetoothIoctlParameters)

#if (CFG_BTUART_KERNEL_TRACE_LEVEL > 0)
    #define BT_DBG(fmt, arg...)  printk(KERN_INFO "btuart: %d: %s: " fmt "\n" , __LINE__, __FUNCTION__ , ## arg)
#else
    #define BT_DBG(fmt, arg...)
#endif

/* HCI related */

#define HCI_COMMAND_PKT             0x01
#define HCI_ACLDATA_PKT             0x02
#define HCI_SCODATA_PKT             0x03
#define HCI_EVENT_PKT               0x04

/* SCO header constants */
/* SCO header format: handle (2 bytes) + length (1 byte) */
#define SCO_HANDLE_SIZE                0x02 /* Size of Handle in SCO packet header */
#define SCO_LENGTH_SIZE                0x01 /* Size of Payload Length in SCO packet header */
#define SCO_HEADER_SIZE                (SCO_HANDLE_SIZE + SCO_LENGTH_SIZE)

#pragma pack (push, 1)
typedef struct
{
    unsigned int    cmd;
    unsigned int    inputBufferLen;
    void*           inputBuffer;
    unsigned int    outputBufferLen;
    void*           outputBuffer;
    unsigned int    realRead;
}
BluetoothIoctlParameters;
#pragma pack (pop)


struct btm_stats {
        __u32 err_rx;
        __u32 err_tx;
        __u32 cmd_tx;
        __u32 evt_rx;
        __u32 acl_tx;
        __u32 acl_rx;
        __u32 sco_tx;
        __u32 sco_rx;
        __u32 byte_rx;
        __u32 byte_tx;
};

struct btm_device {

    /* Device data */
    void                   *driver_data;
    unsigned long       flags;
    char                name[8];
    
    struct rfkill        *rfkill;
    
    /* Read Data */
    struct pkt_queue     ev_pkt_queue;
    struct pkt_queue     acl_pkt_queue;
    struct pkt_queue     sco_pkt_queue;
    
    /* Stats */
    struct btm_stats     stat;

    /* Universal unterface */
    int (*open)(struct btm_device *hdev);
    int (*close)(struct btm_device *hdev);
    int (*flush)(struct btm_device *hdev);
    int (*send)(struct sk_buff *skb);

    /* SCO Connection handle */
    unsigned short sco_handle_in;
    unsigned short sco_handle_out;
};


/*******************************************************************************
*                          BTM UNI Interface functions                         *
*******************************************************************************/

/**
  * BTM Interface function that inits btm_device
  */
int btm_register_dev(struct btm_device *bdev);


/**
  * BTM Interface function that deinits btm_device
  */
int btm_unregister_dev(struct btm_device *bdev);

/**
  * Function that closes btm device.
  * Called by ioctl() with command field of BluetoothIoctlParameters
  * structure set to BTIOCTL_CLOSE
  *
  * @bdev Pointer to btm device.
  */
int btm_uni_close(struct btm_device *bdev);

/**
  * Function that creates btm device and returns pointer to it.
  * If btm device have been already created it does nothing but
  * pointer to existing device anyway returns.
  */
struct btm_device *btm_alloc_dev(void);

/**
  * Function that releases btm device.
  *
  * @hdev Pointer to btm device.
  */
void btm_free_dev(struct btm_device * bdev);

/**
  * Function that forwards incoming socket buffers to correct queue.
  * Queue name depends on data type.
  *
  * @skb pointer to incoming socket buffer.
  */
int btm_push_to_queue(struct sk_buff *skb);

/**
  * This function sets current driver state. This state accessable through
  * the module parameter and informs upper layers about driver's state
  *
  * @on      - current driver state.
  *            Possible values: 1 - driver is initialized
  *                             0 - driver is deinitialized
  */
int set_driver_state(int on);

#endif /*BTM_UNI_H_*/
