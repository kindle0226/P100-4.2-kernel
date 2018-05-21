/*
 *  Motorola Solutions UNI interface
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

#include <linux/miscdevice.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>
#include "btm_uni.h"
#include "btm_log.h"

#include <linux/spinlock.h> // rwlock_t

#include <linux/kthread.h>

/*******************************************************************************
*                      Device file operations functions                        *
*******************************************************************************/

/**
  * Function that processes device opening request.
  * Only thing we doing here is initing one field in filp.
  *
  * @param inode Is not using
  * @param filp Includes everything related to the device driver. Initing it here.
  */
static int device_open(struct inode *inode, struct file *filp);


/**
  * Function that processes device releasing request.
  * Do nothing here, just a dummy.
  */
static int device_release(struct inode *inode, struct file *filp);


/**
  * Function that processes ioctl() calls for uart device.
  *
  * @param filp Includes everything related to the device driver
  * @param cmd Command code
  * @param arg Pointer to the BluetoothIoctlParameters structure
  */
static long device_ioctl (struct file *filp, unsigned int cmd, unsigned long arg);


/********************************************************************************
*                          Opeartions with sk_buff                              *
********************************************************************************/

/**
  * Function created sk_buff for some ioctl command and returned pointer to the data, which can be writen.
  *
  * @param bdev        - pointer to btm_device structure
  * @param cmd        - ioctl command
  * @param skb        - null pointer to sk_buff
  * @param dlen        - data length, which can be sended
  */
static void* create_skb_and_get_data_pointer(struct btm_device *bdev, unsigned int cmd, struct sk_buff **skb, unsigned int dlen);

/**
  * Function read sk_buff for some ioctl command and returned pointer to the data, which can be readed.
  *
  * @param bdev        - pointer to btm_device structure
  * @param cmd        - ioctl command
  * @param skb        - pointer to sk_buff
  * @param dlen        - data length, which can be readed
  */
static void* read_skb_and_get_data_pointer(struct btm_device *bdev, unsigned int cmd, struct sk_buff *skb, unsigned int *dlen);

/*******************************************************************************
*                                   Actuators                                  *
*******************************************************************************/


/**
  * Function that opens btm device.
  * Called by ioctl() with command field of BluetoothIoctlParameters
  * structure set to BTIOCTL_OPEN
  *
  * @bdev Pointer to btm device.
  */
static int btm_uni_open(struct btm_device *bdev);

/**
  * Function that flushes btm device.
  * Called by ioctl() with command field of BluetoothIoctlParameters
  * structure set to BTIOCTL_FLUSH_ACLDATA, BTIOCTL_FLUSH_EVENT or
  * BTIOCTL_FLUSH_SCODATA.
  *
  * @bdev Pointer to btm device.
  */
static int btm_uni_flush_device(struct btm_device *bdev, unsigned long code);


/**
  * Function that reads incoming command from appropriates queue.
  * Called by ioctl() with command field of BluetoothIoctlParameters
  *
  * @bdev    - Pointer to btm device.
  * @skd    - pointer to pointer to socket buffer where event would be saved
  * @cmd    - ioctl command
  */
static int btm_uni_read(struct btm_device *bdev, struct sk_buff **skb, unsigned int cmd);

/**
  * Function that sends data to btm device.
  * Called by ioctl() with command field of BluetoothIoctlParameters
  *
  * @bdev    - Pointer to btm device.
  * @skb    - pointer to socket buffer that contains data packet.
  * @cmd    - ioctl command
  */
static int btm_uni_write(struct btm_device *bdev, struct sk_buff *skb, unsigned int cmd);

/**
  * Function save SCO handle for incoming data
  * Called by ioctl() with command field of BluetoothIoctlParameters
  *
  * @bdev    - pointer to btm device
  * @skb    - pointer to socket buffer that contains handle (len field)
  */
static int btm_uni_start_sco_in(struct btm_device *bdev, struct sk_buff *skb);

/**
  * Function that called when SCO connection was stopped
  * Called by ioctl() with command field of BluetoothIoctlParameters
  *
  * @bdev    - pointer to btm device
  */
static int btm_uni_stop_sco_in (struct btm_device *bdev);

/**
  * Function save SCO handle for outgoing data
  * Called by ioctl() with command field of BluetoothIoctlParameters
  *
  * @bdev    - pointer to btm device
  * @skb    - pointer to socket buffer that contains handle (len field)
  */
static int btm_uni_start_sco_out(struct btm_device *bdev, struct sk_buff *skb);

/**
  * Function that called when SCO connection was stopped
  * Called by ioctl() with command field of BluetoothIoctlParameters
  *
  * @bdev    - pointer to btm device
  */
static int btm_uni_stop_sco_out (struct btm_device *bdev);

/*******************************************************************************
*                             Global Variables                                 *
*******************************************************************************/

static struct btm_device *btm_dev;

#ifdef SCO_RECORD_DUMP
    static struct btm_dump sco_in_dump;
#endif
#ifdef SCO_PLAYBACK_DUMP
    static struct btm_dump sco_out_dump;
#endif

/// To prevent erasing btm_device while trying write to it
static DEFINE_RWLOCK(bdev_rwlock);

static int device_open(struct inode *inode, struct file *filp)
{
    printk("\n*************\n");
    if (btm_dev == NULL)
    {
        BT_ERR("Global device is not allocated!");
        return -ENXIO;
    }
    
    filp->private_data = btm_dev;
    BT_DBG("Done");

    return 0;
}

static int device_release(struct inode *inode, struct file *filp)
{
    BT_DBG("Done");

    return 0;
}

#define IS_READ_CMD(p) ((p.cmd == BTIOCTL_READ_EVENT \
                        || p.cmd == BTIOCTL_READ_ACLDATA \
                        || p.cmd == BTIOCTL_READ_SCODATA) ? 1 : 0)

#define IS_WRITE_CMD(p) ((p.cmd == BTIOCTL_SEND_COMMAND \
                        || p.cmd == BTIOCTL_SEND_ACLDATA \
                        || p.cmd == BTIOCTL_SEND_SCODATA) ? 1 : 0)

#define IS_START_SCO_CMD(p) ((p.cmd == BTIOCTL_START_SCOIN \
                        || p.cmd == BTIOCTL_START_SCOOUT) ? 1 : 0)

static int device_ioctl_parser(struct btm_device *bdev, unsigned long code, struct sk_buff **p_skb)
                                   
{
    int ret = 0;

    read_lock_bh(&bdev_rwlock);
    
    if (bdev == NULL)
    {
        BT_ERR("bdev is NULL");
        read_unlock_bh(&bdev_rwlock);
        return -ENODEV;
    }

    switch (code)
    {
        case BTIOCTL_OPEN:
            BT_DBG("BTIOCTL_OPEN");
            ret = btm_uni_open(bdev);
            break;

        case BTIOCTL_CLOSE:
            BT_DBG("BTIOCTL_CLOSE");
            ret = btm_uni_close(bdev);
            break;

        case BTIOCTL_FLUSH_ACLDATA:
        case BTIOCTL_FLUSH_EVENT:
        case BTIOCTL_FLUSH_SCODATA:
            BT_DBG("FLUSH DEVICE");
            ret = btm_uni_flush_device(bdev, code);
            break;

        case BTIOCTL_READ_EVENT:
        case BTIOCTL_READ_ACLDATA:
        case BTIOCTL_READ_SCODATA:
            ret = btm_uni_read(bdev, p_skb, code);
           break;
       
        case BTIOCTL_SEND_COMMAND:
        case BTIOCTL_SEND_ACLDATA:
        case BTIOCTL_SEND_SCODATA:
            ret = btm_uni_write(bdev, *p_skb, code);
           break;

        case BTIOCTL_START_SCOIN:
            BT_DBG("BTIOCTL_START_SCOIN");
            ret = btm_uni_start_sco_in(bdev, *p_skb);
            break;
       
        case BTIOCTL_STOP_SCOIN:
            BT_DBG("BTIOCTL_STOP_SCOIN");
            ret = btm_uni_stop_sco_in(bdev);
            break;

        case BTIOCTL_START_SCOOUT:
            BT_DBG("BTIOCTL_START_SCOOUT");
            ret = btm_uni_start_sco_out(bdev, *p_skb);
            break;

        case BTIOCTL_STOP_SCOOUT:
            BT_DBG("BTIOCTL_STOP_SCOOUT");
            ret = btm_uni_stop_sco_out(bdev);
            break;

       // case BTIOCTL_CLEAR_SCOIN:
       //     BT_DBG("BTIOCTL_CLEAR_SCOIN");
       //     ret = btm_uni_clear_read_data_in_ring();
       //     break;

       default:
           BT_DBG("Unknown code: %lu", code);
           ret = -ENOTTY;
    }

    BT_DBG("Ioctl return %d", ret);
    read_unlock_bh(&bdev_rwlock);
    return ret;
}


static long device_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct btm_device *bdev = filp->private_data;
    int retval = 0;
    if (btm_dev == NULL) //if global device was deallocated - it's an unplug event
    {
       BT_ERR("device_ioctl error - no device");
       return -ENODEV;  // no such device
    }

    switch (cmd)
    {

        case BTUSB_BLUETOOTH_IOCTL:
        {
            BluetoothIoctlParameters parameters;
            int err;
            int reallen = 0;
            struct sk_buff* skb = NULL;

            err = !access_ok(VERIFY_WRITE, arg, _IOC_SIZE(cmd));
            if (err)
            {
                BT_ERR("access_ok(arg) failed\n");
                return -EFAULT; // bad address
            }

            if (0 != copy_from_user(&parameters, (void __user*)arg, sizeof(BluetoothIoctlParameters)))
            {
                BT_ERR("copy_from_user(arg) failed\n");
                return -EFAULT;
            }
            
            if (IS_WRITE_CMD(parameters) || IS_START_SCO_CMD(parameters))
            {
                void *pdata;
                err = !access_ok(VERIFY_READ, parameters.inputBuffer, parameters.inputBufferLen);
                if (err)
                {
                    BT_ERR("access_ok(inputBuffer) failed\n");
                    return -EFAULT;
                }

                if (parameters.inputBufferLen > 0)
                {
                    if ((pdata = create_skb_and_get_data_pointer(bdev, parameters.cmd, &skb, parameters.inputBufferLen)) == NULL)
                    {
                        BT_DBG("create_skb_and_get_data_pointer failed");
                        return -ENOMEM;
                    }
                    if (parameters.inputBuffer != NULL)
                    {
                        if (0 != copy_from_user(pdata, parameters.inputBuffer, parameters.inputBufferLen))
                        {
                            BT_ERR("copy_from_user(inputBuffer) failed\n");
                            if (skb)
                                kfree_skb(skb);
                            return -EFAULT;
                        }
                    }
                }
                else
                {
                    BT_ERR("Nothing to read.");
                    return -EFAULT;
                }
            }

            reallen = device_ioctl_parser(bdev, parameters.cmd, &skb);
            
            if (reallen < 0)
            {
                BT_DBG("ioctl parser error: [%d]", reallen);
                if (skb)
                    kfree_skb(skb);
                return reallen;
            }

            if (IS_READ_CMD(parameters) && reallen >= 0)
            {
                void *pdata;
                if ((pdata = read_skb_and_get_data_pointer(bdev, parameters.cmd, skb, &reallen)) == NULL)
                {
                    BT_ERR("read_skb_and_get_data_pointer failed");
                    if (skb)
                        kfree_skb(skb);
                    return -EFAULT;
                }
                if (0 != copy_to_user(&(((BluetoothIoctlParameters*)arg)->realRead), &reallen, sizeof(int)))
                {
                   BT_ERR("copy_to_user(realRead) failed\n");
                   if (skb)
                        kfree_skb(skb);
                   return -EFAULT;
                }
                if (reallen > 0 && parameters.inputBuffer != NULL)
                {
                    err = !access_ok(VERIFY_WRITE, ((BluetoothIoctlParameters*)arg)->inputBuffer, reallen);
                    if (err)
                    {
                        BT_ERR("access_ok(inputBuffer) failed for writting\n");
                        if (skb)
                            kfree_skb(skb);
                        return -EFAULT;
                    }
                    if (0 != copy_to_user(((BluetoothIoctlParameters*)arg)->inputBuffer, pdata, reallen))
                    {
                        BT_ERR("copy_to_user(outputBuffer) failed\n");
                        if (skb)
                            kfree_skb(skb);
                        return -EFAULT;
                    }
                }
                else
                {
                    if (parameters.inputBuffer == NULL)
                        BT_DBG("parameters.inputBuffer is NULL. Is this the intent?\n");
                    else if (reallen <= 0)
                        BT_DBG("reallen <= 0. Nothing to copy to userspace\n");
                }
                if (skb)
                    kfree_skb(skb);
            }
            else
            {
                // Writing real readed bytes by driver in write case
                if (reallen > 0)
                {
                    if (0 != copy_to_user(&(((BluetoothIoctlParameters*)arg)->realRead), &reallen, sizeof(int)))
                    {
                       BT_ERR("copy_to_user(realRead) failed\n");
                       return -EFAULT;
                    }
                }
                else
                        BT_DBG("reallen <= 0. Nothing to copy to userspace\n");
            }
        }
        break;
        default:
            BT_ERR("Wrong IOCTL code");
            return -ENOTTY;
    };

    BT_DBG("<<<");
    return retval;
}

static struct file_operations fops =
{
    .owner   = THIS_MODULE,
    .open    = device_open,
    .release = device_release,
    .write   = NULL,
    .read    = NULL,
    .unlocked_ioctl = device_ioctl,
};

static struct miscdevice btm_miscdev =
{
    .name    = DEVICE_NAME,
    .fops    = &fops,
    .minor    = MISC_DYNAMIC_MINOR,
};

/* -------------------- BTM UNI Interface functions -------------------- */
#ifdef TI_PLATFORM
static int btm_rfkill_set_block(void *data, bool blocked)
{
    struct btm_device *bdev = (struct btm_device *)data;

    BT_DBG("%p name %s blocked %d", bdev, bdev->name, blocked);

    if (!blocked)
        return 0;
/*    Additional clearing and power-off command shall be here!!!
*/
    if (test_bit(HCI_RUNNING, &bdev->flags))
        btm_uni_close(bdev);

    return 0;
}

static const struct rfkill_ops hci_rfkill_ops = {
    .set_block = btm_rfkill_set_block,
};
#endif

int btm_register_dev(struct btm_device *bdev)
{
    int ret = 0;
    
#ifdef TI_PLATFORM
    bdev->rfkill = rfkill_alloc(bdev->name, NULL, RFKILL_TYPE_BLUETOOTH, &hci_rfkill_ops, bdev);
    if (bdev->rfkill)
    {
        if (rfkill_register(bdev->rfkill) < 0)
        {
            BT_ERR("Unable to register rfkill");
            rfkill_destroy(bdev->rfkill);
            bdev->rfkill = NULL;
        }
    }
    else
    {
        BT_ERR("Unable to allocate rfkill");
    }
    BT_DBG("rfkill created!!!");
#endif

    ret = misc_register(&btm_miscdev);

#ifndef TI_PLATFORM
    set_driver_state(1);
#endif

    return ret;

}

int btm_unregister_dev(struct btm_device *bdev)
{
#ifdef TI_PLATFORM
    if (bdev->rfkill)
    {
        rfkill_unregister (bdev->rfkill);
        rfkill_destroy (bdev->rfkill);
        bdev->rfkill = NULL;
    }
#endif

#ifndef TI_PLATFORM
    set_driver_state(0);
#endif
    return misc_deregister(&btm_miscdev);
}

struct btm_device *btm_alloc_dev()
{
    if (btm_dev != NULL)
    {
        BT_ERR("Device object is already present!");
        return btm_dev;
    }
    
    btm_dev = (struct btm_device *)kzalloc(sizeof(struct btm_device), GFP_KERNEL);
    if (btm_dev == NULL)
        return NULL;
    
    if (init_pkt_queue(&btm_dev->ev_pkt_queue))
    {
        BT_ERR("Cannot init event queue!");
        return NULL;
    }

    if (init_pkt_queue(&btm_dev->acl_pkt_queue))
    {
        BT_ERR("Cannot init ACL queue!");
        return NULL;
    }

    if (init_pkt_queue(&btm_dev->sco_pkt_queue))
    {
        BT_ERR("Cannot init SCO queue!");
        return NULL;
    }

    strcpy(btm_dev->name, DEVICE_NAME);

    return btm_dev;
}

void btm_free_dev(struct btm_device * bdev)
{
    if (bdev == NULL)
    {
        BT_DBG("Requested to free NULL pointer!");
        return;
    }
    
    if (btm_dev == NULL)
    {
        BT_DBG("Global device object is already freed!");
        return;
    }
  
    write_lock_bh(&bdev_rwlock);

    deinit_pkt_queue(&btm_dev->ev_pkt_queue);
    deinit_pkt_queue(&btm_dev->acl_pkt_queue);
    deinit_pkt_queue(&btm_dev->sco_pkt_queue);
    
    kfree(btm_dev);
    btm_dev = NULL;

    write_unlock_bh(&bdev_rwlock);
}

int btm_push_to_queue(struct sk_buff *skb)
{
    int err = 0;

    switch(bt_cb(skb)->pkt_type)
    {
        case HCI_EVENT_PKT:
            
            err = enqueue_pkt(&btm_dev->ev_pkt_queue, skb);
            BT_DBG("Packet was moved to ev_pkt_queue");
            break;
            
        case HCI_ACLDATA_PKT:
            
            err = enqueue_pkt(&btm_dev->acl_pkt_queue, skb);

            BT_DBG("Packet was moved to acl_pkt_queue");
            break;
            
        case HCI_SCODATA_PKT:
        
            err = enqueue_pkt(&btm_dev->sco_pkt_queue, skb);
#ifdef SCO_RECORD_DUMP
            btm_add_data_to_dump(&sco_in_dump, skb->data + 3, skb->len - 3);
#endif
            BT_DBG("Packet was moved to sco_pkt_queue");
            break;
            
        default:
            return -EBADF;
    }

    return err;
}

/* -------------------- Actuators -------------------- */

/* Open device */
static int btm_uni_open (struct btm_device *bdev)
{
    int err;

    if (!bdev)
    {
        BT_DBG("!bdev\n");
        return -EINVAL;
    }

    BT_DBG("btm_uni_open:\n");
    if (bdev->rfkill && rfkill_blocked(bdev->rfkill))
    {
        BT_ERR("unable to open device: rfkill blocked");
        return -ERFKILL;
    }

    memset(&btm_dev->stat, 0, sizeof(struct btm_stats));

    err = clear_pkt_queue(&btm_dev->ev_pkt_queue);
    if(err < 0)
    {
        BT_ERR("Unable to clear queue (EVENT)");
        return err;
    }
    err = clear_pkt_queue(&btm_dev->acl_pkt_queue);
    if(err < 0)
    {
        BT_ERR("Unable to clear queue (ACL)");
        return err;
    }
    err = clear_pkt_queue(&btm_dev->sco_pkt_queue);
    if(err < 0)
    {
        BT_ERR("Unable to clear queue (SCO)");
        return err;
    }

/**
 * jfb346: TI ST expected invoke of "st_register" call in non-interruptible context.
 * Therefore, we should unlock mutex on bdev here.
 */
#ifdef TI_PLATFORM
    read_unlock_bh(&bdev_rwlock);
    err = bdev->open(bdev);
    read_lock_bh(&bdev_rwlock);
#else
    err = bdev->open(bdev);
#endif
    if (err < 0)
    {
        BT_ERR("Device low-open failed (%d)!", err);
        return err;
    }

    BT_DBG("Done");
    return 0;
}

/* Close device */
int btm_uni_close(struct btm_device *bdev)
{
    int err;
    
    BT_DBG("btusb_close:\n");

    if (bdev)
    {
        kill_waiters(&bdev->ev_pkt_queue);
        kill_waiters(&bdev->acl_pkt_queue);
        kill_waiters(&bdev->sco_pkt_queue);
        
        err = bdev->close(bdev);
        if (err < 0)
        {
            BT_ERR("Device low-close failed (%d)", err);
            return err;
        }

        BT_DBG("Done");

        return 0;
    }
    else
    {
        BT_DBG("!bdev\n");
        return -EINVAL;
    }
}

/* Flush Device */
static int btm_uni_flush_device(struct btm_device *bdev, unsigned long code)
{
    int res = -EFAULT;
    
    switch (code)
    {
        case BTIOCTL_FLUSH_ACLDATA:
            BT_DBG("Releasing ACL data");
            res = release_waiters(&bdev->acl_pkt_queue);
        break;

        case BTIOCTL_FLUSH_EVENT:
            BT_DBG("Releasing HCI events");
            res = release_waiters(&bdev->ev_pkt_queue);
        break;

        case BTIOCTL_FLUSH_SCODATA:
            BT_DBG("Releasing SCO data");
            res = release_waiters(&bdev->sco_pkt_queue);
        break;

        default:
            BT_ERR("Unknown event code: [%lu]", code);
            res = -EFAULT;
        break;
    }

    BT_DBG("Flush result: [%d]", res);
    return res;
}

/* Read */
static int btm_uni_read(struct btm_device *bdev, struct sk_buff **skb, unsigned int cmd)
{
    int res = -EFAULT;

    if (!test_bit(HCI_RUNNING, &bdev->flags))
    {
        return -EBUSY;
    }

    /// Can't schedule in non interruptible context
    read_unlock_bh(&bdev_rwlock);

    switch (cmd)
    {
        case BTIOCTL_READ_EVENT:
            BT_DBG("BTIOCTL_READ_EVENT");
            res = dequeue_pkt(&bdev->ev_pkt_queue, skb);
            break;
        case BTIOCTL_READ_ACLDATA:
            BT_DBG("BTIOCTL_READ_ACLDATA");
            res = dequeue_pkt(&bdev->acl_pkt_queue, skb);
            break;
        case BTIOCTL_READ_SCODATA:
            BT_DBG("BTIOCTL_READ_SCODATA");
            res = dequeue_pkt(&bdev->sco_pkt_queue, skb);
            break;
        default:
            BT_ERR("Unknown ioctl: %d", cmd);
            res = -EFAULT;
    }

    read_lock_bh(&bdev_rwlock);
    return res;
}

/* Write */
static int btm_uni_write(struct btm_device *bdev, struct sk_buff *skb, unsigned int cmd)
{
    int len = 0;
    if (skb == NULL)
    {
        BT_ERR("skb is not allocated");
        return -ENOMSG;
    }
    
    if (!test_bit(HCI_RUNNING, &bdev->flags))
        return -EBUSY;
    
    switch (cmd)
    {
        case BTIOCTL_SEND_COMMAND:
            BT_DBG("BTIOCTL_SEND_COMMAND");
            bt_cb(skb)->pkt_type = HCI_COMMAND_PKT;
            break;
        case BTIOCTL_SEND_ACLDATA:
            BT_DBG("BTIOCTL_SEND_ACLDATA");
            bt_cb(skb)->pkt_type = HCI_ACLDATA_PKT;
            break;
        case BTIOCTL_SEND_SCODATA:
            BT_DBG("BTIOCTL_SEND_SCODATA");
            bt_cb(skb)->pkt_type = HCI_SCODATA_PKT;
            break;
        default:
            BT_ERR("Unknown ioctl: %d", cmd);
            return -EFAULT;
    }

    len = bdev->send(skb);
    if (len < 0)
    {
        BT_ERR("Can't send frame to device (%d)!", len);
        return len;
    }
        
    return len;

}

static int btm_uni_start_sco_in(struct btm_device *bdev, struct sk_buff *skb)
{
    int err;

    if (skb == NULL)
    {
        BT_ERR("skb is not allocated");
        return -ENOMSG;
    }
    
    if (!test_bit(HCI_RUNNING, &bdev->flags))
        return -EBUSY;
    
    bdev->sco_handle_in = skb->len;
    BT_DBG("SCO IN Handle: %x", bdev->sco_handle_in);
    
    // FIXME: Here may be need to add connection flags

    err = clear_pkt_queue(&btm_dev->sco_pkt_queue);
    if(err < 0)
    {
        BT_ERR("Unable to clear queue (SCO)");
        return err;
    }

#ifdef SCO_RECORD_DUMP
    btm_init_dump(&sco_in_dump, "sco_in_dump");
#endif
    
    return 0;
}

static int btm_uni_stop_sco_in (struct btm_device *bdev)
{
    //NOTE: nothing to do here
#ifdef SCO_RECORD_DUMP
    btm_deinit_dump(&sco_in_dump);
#endif
    BT_DBG("Nothing to do here...");
    return 0;
}

static int btm_uni_start_sco_out(struct btm_device *bdev, struct sk_buff *skb)
{
    if (skb == NULL)
    {
        BT_ERR("skb is not allocated");
        return -ENOMSG;
    }
    
    if (!test_bit(HCI_RUNNING, &bdev->flags))
        return -EBUSY;
    
    bdev->sco_handle_out = skb->len;
    BT_DBG("SCO OUT Handle: %x", bdev->sco_handle_out);
//    btm_init_dump(&sco_out_dump, "sco_out_dump");
    // FIXME: Here may be need to add connection flags

    return 0;
}

static int btm_uni_stop_sco_out (struct btm_device *bdev)
{
    //NOTE: nothing to do here
//    btm_deinit_dump(&sco_out_dump);
    BT_DBG("Nothing to do here...");
    return 0;
}


static void* create_skb_and_get_data_pointer(struct btm_device *bdev, unsigned int cmd, struct sk_buff **skb, unsigned int dlen)
{
    void *pdata = NULL;

    BT_DBG("dlen: %d, cmd: %d", dlen, cmd);
    switch (cmd)
    {
        /* For SCO into first 3 bytes putted SCO header,
         * which contains 2 bytes of handle and 1 bytes of payload length.
         * Returned pointer will point to the data after header.
         */
        case BTIOCTL_SEND_SCODATA:
            BT_DBG("BTIOCTL_SEND_SCODATA");
            (*skb) = bt_skb_alloc(dlen + SCO_HEADER_SIZE, GFP_ATOMIC);
            if (!(*skb))
            {
                BT_ERR("error allocate sk_buff, %s no memory for command", bdev->name);
                return NULL;
            }

            (*skb)->len = dlen + SCO_HEADER_SIZE;
            (*skb)->dev = (void *) bdev;

            // NOTE: Do NOT use hci_sco_hdr structure for SCO header, because was problems with her align and size
            memcpy((*skb)->data, &(bdev->sco_handle_out), SCO_HANDLE_SIZE);

            // FIXME: Change 0xFF on good formula
            if (dlen > 0xFF) // Sended data may be greater than SCO can sends
            {
                BT_ERR("dlen (%x) more then 1 byte, sending 1 byte", dlen);
                dlen = 0xFF;
            }
            memcpy((*skb)->data + SCO_HANDLE_SIZE, &dlen, SCO_LENGTH_SIZE);
            
            pdata = (*skb)->data + SCO_HEADER_SIZE;
            break;

        case BTIOCTL_SEND_COMMAND:
        case BTIOCTL_SEND_ACLDATA:
        case BTIOCTL_START_SCOIN:
        case BTIOCTL_START_SCOOUT:
            if (cmd == BTIOCTL_SEND_COMMAND)
                BT_DBG("BTIOCTL_SEND_COMMAND");
            else if (cmd == BTIOCTL_SEND_ACLDATA)
                BT_DBG("BTIOCTL_SEND_ACLDATA");
            else if (cmd == BTIOCTL_START_SCOIN)
                BT_DBG("BTIOCTL_START_SCOIN");
            else if (cmd == BTIOCTL_START_SCOOUT)
                BT_DBG("BTIOCTL_START_SCOOUT");

            (*skb) = bt_skb_alloc(dlen, GFP_ATOMIC);
            if (!(*skb))
            {
                BT_ERR("error allocate sk_buff, %s no memory for command", bdev->name);
                return NULL;
            }

            (*skb)->len = dlen;
            (*skb)->dev = (void *) bdev;

            pdata = (*skb)->data;
            break;

        default:
            BT_ERR("Unknown ioctl: %d", cmd);
            return NULL;
    }

    BT_DBG("pdata: %p, skb->len: %d", pdata, (*skb)->len);
    return pdata;
}


static void* read_skb_and_get_data_pointer(struct btm_device *bdev, unsigned int cmd, struct sk_buff *skb, unsigned int *dlen)
{
    void *pdata = NULL;

    if (!skb)
    {
        BT_ERR("sk_buff is zero, %s no memory for command", bdev->name);
        return NULL;
    }
    if (dlen == NULL)
    {
        BT_ERR("dlen pointer is NULL");
        return NULL;
    }

    BT_DBG("dlen: %d, skb->len: %d, cmd: %d", *dlen, skb->len, cmd);
    
    switch (cmd)
    {
        /* For SCO into first 3 bytes putted SCO header,
         * which contains 2 bytes of handle and 1 bytes of payload length.
         * Returned pointer will point to the data after header.
         */
        case BTIOCTL_READ_SCODATA:
            BT_DBG("BTIOCTL_READ_SCODATA");
            if (skb->len < 3)
            {
                BT_ERR("Nothing to copy");
                return NULL;
            }

            if (*(uint16_t*)(skb->data) != bdev->sco_handle_in)
            {
                BT_ERR("received SCO handle not equal our SCO handle");
                return NULL;
            }

            *dlen = 0;
            *dlen = *(uint8_t*)(skb->data + SCO_HANDLE_SIZE);
            if (*dlen > skb->len - SCO_HEADER_SIZE)
            {
                BT_ERR("sk_buff length less than data length in SCO header");
                *dlen = skb->len - SCO_HEADER_SIZE;
            }
            pdata = skb->data + SCO_HEADER_SIZE;
            break;
        case BTIOCTL_READ_EVENT:
        case BTIOCTL_READ_ACLDATA:
            if (cmd == BTIOCTL_READ_EVENT)
                BT_DBG("BTIOCTL_READ_EVENT");
            else if (cmd == BTIOCTL_READ_ACLDATA)
                BT_DBG("BTIOCTL_READ_ACLDATA");

            *dlen = skb->len;
            pdata = skb->data;
            break;
        default:
            BT_ERR("Unknown ioctl: %d", cmd);
            return NULL;
    }

    BT_DBG("pdata: %p, skb->len: %d", pdata, *dlen);
    return pdata;
}
