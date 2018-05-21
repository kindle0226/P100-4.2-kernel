/*
 *  HCI_SMD (HCI Shared Memory Driver) is Qualcomm's Shared memory driver
 *  for the BT HCI protocol.
 *
 *  Copyright (c) 2000-2001, 2011 Code Aurora Forum. All rights reserved.
 *  Copyright (C) 2002-2003  Maxim Krasnyansky <maxk@qualcomm.com>
 *  Copyright (C) 2004-2006  Marcel Holtmann <marcel@holtmann.org>
 *  Copyright 2014 Motorola Solutions, Inc. All rights reserved.
 *
 *  This file is based on drivers/bluetooth/hci_vhci.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/wakelock.h>
#include <linux/uaccess.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>
#include <mach/msm_smd.h>
#include "btm_uni.h"

#define EVENT_CHANNEL        "APPS_RIVA_BT_CMD"
#define DATA_CHANNEL        "APPS_RIVA_BT_ACL"
/* release wakelock in 500ms, not immediately, because higher layers
 * don't always take wakelocks when they should
 * This is derived from the implementation for UART transport
 */

#define RX_Q_MONITOR        (500)    /* 500 milli second */
#define USE(X)                  (void)X

/********************************
 * The highest level of logging makes system to work very slowly (up to global hang up)
 * Do not use it without critical necessary
 ********************************/
#ifdef BT_DBG
#undef BT_DBG
#endif

#define BT_DBG(fmt, arg...)
/********************************/

static int hcismd_set;
static DEFINE_MUTEX(hci_smd_enable);
static struct rfkill* btuart_rfkill;

static int hcismd_set_enable(const char *val, struct kernel_param *kp);
module_param_call(hcismd_set, hcismd_set_enable, NULL, &hcismd_set, 0644);

static void hci_dev_smd_open(struct work_struct *worker);
static void hci_dev_restart(struct work_struct *worker);

static int driver_initialized;
module_param_call(driver_initialized, NULL, param_get_int, &driver_initialized, 0644);

#define HCI_SMD_REGISTERED (1<<0) /// HCI SMD driver registered

struct hci_smd_data {
    struct btm_device *bdev;

    struct smd_channel *event_channel;
    struct smd_channel *data_channel;
    struct wake_lock wake_lock_tx;
    struct wake_lock wake_lock_rx_event;
    struct wake_lock wake_lock_rx_data;
    struct timer_list rx_event_q_timer;
    struct timer_list rx_data_q_timer;
    struct tasklet_struct rx_event_task;
    struct tasklet_struct rx_data_task;

    unsigned long flags;
};
static struct hci_smd_data hs;

/* Rx queue monitor timer function */
static int is_rx_event_q_empty(unsigned long arg)
{
    struct btm_device *bdev = (struct btm_device *) arg;
    struct sk_buff_head *list_ = &bdev->ev_pkt_queue.list;
    struct sk_buff *list = ((struct sk_buff *)list_)->next;
    BT_DBG("%s Rx event timer triggered", hdev->name);

    // Check event pkt queue
    if (list == (struct sk_buff *)list_) {
        BT_DBG("%s RX event queue empty", hdev->name);

        return 1;
    } else {
        BT_DBG("%s RX event queue not empty", hdev->name);
        return 0;
    }
}

static void release_rx_event_lock(void)
{
    struct hci_smd_data *hsmd = &hs;
    BT_DBG("Releasing Rx Event Lock");
    if (is_rx_event_q_empty((unsigned long)hsmd->bdev) &&
        wake_lock_active(&hs.wake_lock_rx_event))
            wake_unlock(&hs.wake_lock_rx_event);
}

/* Rx timer callback function */
static void schedule_rx_event_q_timer(unsigned long arg)
{
    struct btm_device *hdev = (struct btm_device *) arg;
    struct hci_smd_data *hsmd = &hs;
    BT_DBG("%s Schedule Rx event timer", hdev->name);

    if (is_rx_event_q_empty(arg) && wake_lock_active(&hs.wake_lock_rx_event)) {
        BT_DBG("%s RX event queue empty", hdev->name);
        /*
         * Since the queue is empty, its ideal
         * to release the wake lock on Rx
         */
        wake_unlock(&hs.wake_lock_rx_event);
    } else{
        BT_DBG("%s RX event queue not empty", hdev->name);
        /*
         * Restart the timer to monitor whether the Rx queue is
         * empty for releasing the Rx wake lock
         */
        mod_timer(&hsmd->rx_event_q_timer,
            jiffies + msecs_to_jiffies(RX_Q_MONITOR));
    }
    USE(hdev);
}

/******************************************
 * Managing rx data queue functions
 ******************************************/
/* Rx data queue monitor timer function */
static int is_rx_data_q_empty(unsigned long arg)
{
    struct btm_device *bdev = (struct btm_device *) arg;
    struct sk_buff_head *list_ = &bdev->acl_pkt_queue.list;
    struct sk_buff *list = ((struct sk_buff *)list_)->next;
    BT_DBG("%s Rx timer triggered", hdev->name);

    // Check event pkt queue
    if (list == (struct sk_buff *)list_) {
        BT_DBG("%s RX data queue empty", hdev->name);

        return 1;
    } else {
        BT_DBG("%s RX data queue not empty", hdev->name);
        return 0;
    }
}

static void release_rx_data_lock(void)
{
    struct hci_smd_data *hsmd = &hs;
    BT_DBG("Releasing Rx Lock");
    if (is_rx_data_q_empty((unsigned long)hsmd->bdev) &&
        wake_lock_active(&hs.wake_lock_rx_data))
            wake_unlock(&hs.wake_lock_rx_data);
}

/* Rx timer callback function */
static void schedule_rx_data_q_timer(unsigned long arg)
{
    struct btm_device *hdev = (struct btm_device *) arg;
    struct hci_smd_data *hsmd = &hs;
    BT_DBG("%s Schedule Rx data timer", hdev->name);

    if (is_rx_data_q_empty(arg) && wake_lock_active(&hs.wake_lock_rx_data)) {
        BT_DBG("%s RX data queue empty", hdev->name);
        /*
         * Since the queue is empty, its ideal
         * to release the wake lock on Rx
         */
        wake_unlock(&hs.wake_lock_rx_data);
    } else{
        BT_DBG("%s RX data queue not empty", hdev->name);
        /*
         * Restart the timer to monitor whether the Rx queue is
         * empty for releasing the Rx wake lock
         */
        mod_timer(&hsmd->rx_data_q_timer,
            jiffies + msecs_to_jiffies(RX_Q_MONITOR));
    }
    USE(hdev);
}


static int hci_smd_open(struct btm_device *hdev)
{
    set_bit(HCI_RUNNING, &hdev->flags);
    return 0;
}


static int hci_smd_close(struct btm_device *hdev)
{
    clear_bit(HCI_RUNNING, &hdev->flags);
    return 0;
//    if (!test_and_clear_bit(HCI_RUNNING, &hdev->flags))
//        return 0;
//    else
//        return -EPERM;
}


//static void hci_smd_destruct(struct btm_device *hdev)
//{
//    if (NULL != hdev->driver_data)
//        kfree(hdev->driver_data);
//}

static void hci_smd_recv_data(void)
{
    int len = 0;
    int rc = 0;
    struct sk_buff *skb = NULL;
    struct hci_smd_data *hsmd = &hs;
    wake_lock(&hs.wake_lock_rx_data);

    len = smd_read_avail(hsmd->data_channel);
    if (len > HCI_MAX_FRAME_SIZE) {
        BT_ERR("Frame larger than the allowed size");
        smd_read(hsmd->data_channel, NULL, len);
        goto out_data;
    }

    if (len <= 0)
        goto out_data;

//    while (len > 0)
    {
        skb = bt_skb_alloc(len, GFP_ATOMIC);
        if (!skb) {
            BT_ERR("Error in allocating  socket buffer");
            smd_read(hsmd->data_channel, NULL, len);
            goto out_data;
        }

        rc = smd_read(hsmd->data_channel, skb_put(skb, len), len);
        if (rc < len) {
            BT_ERR("Error in reading from the channel");
            goto out_data;
        }

        skb->dev = (void *)hsmd->bdev;
        bt_cb(skb)->pkt_type = HCI_ACLDATA_PKT;

        skb_orphan(skb);

        rc = btm_push_to_queue(skb);
        if (rc < 0) {
            BT_ERR("Error in passing the packet to HCI Layer");
            /*
             * skb is getting freed in hci_recv_frame, making it
             * to null to avoid multiple access
             */
            skb = NULL;
            goto out_data;
        }

        /*
         * Start the timer to monitor whether the Rx queue is
         * empty for releasing the Rx wake lock
         */
        BT_DBG("Rx data Timer is starting");
        mod_timer(&hsmd->rx_event_q_timer,
                jiffies + msecs_to_jiffies(RX_Q_MONITOR));
    }
out_data:
    release_rx_data_lock();
    if (rc) {
        if (skb)
            kfree_skb(skb);
    }
}

static void hci_smd_recv_event(void)
{
    int len = 0;
    int rc = 0;
    struct sk_buff *skb = NULL;
    struct hci_smd_data *hsmd = &hs;
    wake_lock(&hs.wake_lock_rx_event);

    len = smd_read_avail(hsmd->event_channel);
    if (len > HCI_MAX_FRAME_SIZE) {
        BT_ERR("Frame larger than the allowed size, flushing frame");
        rc = smd_read(hsmd->event_channel, NULL, len);
        goto out_event;
    }

//    while (len > 0)
    {
        skb = bt_skb_alloc(len, GFP_ATOMIC);
        if (!skb) {
            BT_ERR("Error in allocating  socket buffer");
            smd_read(hsmd->event_channel, NULL, len);
            goto out_event;
        }

        rc = smd_read(hsmd->event_channel, skb_put(skb, len), len);
        if (rc < len) {
            BT_ERR("Error in reading from the event channel");
            goto out_event;
        }

        skb->dev = (void *)hsmd->bdev;
        bt_cb(skb)->pkt_type = HCI_EVENT_PKT;

        skb_orphan(skb);

        rc = btm_push_to_queue(skb);
        if (rc < 0) {
            BT_ERR("Error in passing the packet to HCI Layer");
            /*
             * skb is getting freed in hci_recv_frame, making it
             *  to null to avoid multiple access
             */
            skb = NULL;
            goto out_event;
        }

        len = smd_read_avail(hsmd->event_channel);
        /*
         * Start the timer to monitor whether the Rx queue is
         * empty for releasing the Rx wake lock
         */
        BT_DBG("Rx event Timer is starting");

        mod_timer(&hsmd->rx_event_q_timer,
                jiffies + msecs_to_jiffies(RX_Q_MONITOR));

    }
out_event:
    release_rx_event_lock();
    if (rc) {
        if (skb)
            kfree_skb(skb);
    }
}

static int send_len = 0;

static int hci_smd_send_frame(struct sk_buff *skb)
{
    int len;
    int avail;
    int ret = 0;
    wake_lock(&hs.wake_lock_tx);

    switch (bt_cb(skb)->pkt_type) {
    case HCI_COMMAND_PKT:
        avail = smd_write_avail(hs.event_channel);
        if (!avail) {
            BT_ERR("No space available for smd frame");
            ret =  -ENOSPC;
        }
        len = smd_write(hs.event_channel, skb->data, skb->len);
        if (len < skb->len) {
            BT_ERR("Failed to write Command %d", len);
            ret = -ENODEV;
        }
        send_len += len;
        break;
    case HCI_ACLDATA_PKT:
    case HCI_SCODATA_PKT:
        avail = smd_write_avail(hs.data_channel);
        if (!avail) {
            BT_ERR("No space available for smd frame");
            ret = -ENOSPC;
        }
        len = smd_write(hs.data_channel, skb->data, skb->len);
        if (len < skb->len) {
            BT_ERR("Failed to write Data %d", len);
            ret = -ENODEV;
        }
        break;
    default:
        BT_ERR("Uknown packet type");
        ret = -ENODEV;
        break;
    }

    kfree_skb(skb);
    wake_unlock(&hs.wake_lock_tx);
    return ret;
}

static void hci_smd_rx_data (unsigned long arg)
{
    struct hci_smd_data *hsmd = &hs;

    while ((smd_read_avail(hsmd->data_channel) > 0)) {
        hci_smd_recv_data();
    }
}

static void hci_smd_rx_event (unsigned long arg)
{
    struct hci_smd_data *hsmd = &hs;

    while ((smd_read_avail(hsmd->event_channel) > 0)) {
        hci_smd_recv_event();
    }
}

static void hci_smd_notify_event(void *data, unsigned int event)
{
    struct btm_device *hdev = hs.bdev;
    struct hci_smd_data *hsmd = &hs;
    struct work_struct *reset_worker;
    struct work_struct *open_worker;
    int len = 0;

    if (!hdev) {
        BT_ERR("Frame for unknown HCI device (hdev=NULL)");
        return;
    }

    switch (event) {
    case SMD_EVENT_DATA:
        len = smd_read_avail(hsmd->event_channel);
        if (len > 0)
            tasklet_hi_schedule(&hs.rx_event_task);
        else if (len < 0)
            BT_ERR("Failed to read event from smd %d", len);
        break;
    case SMD_EVENT_OPEN:
        BT_INFO("opening HCI-SMD channel :%s", EVENT_CHANNEL);
        hci_smd_open(hdev);
        open_worker = kzalloc(sizeof(*open_worker), GFP_ATOMIC);
        if (!open_worker) {
            BT_ERR("Out of memory");
            break;
        }
        INIT_WORK(open_worker, hci_dev_smd_open);
        schedule_work(open_worker);
        break;
    case SMD_EVENT_CLOSE:
        BT_INFO("Closing HCI-SMD channel :%s", EVENT_CHANNEL);
        hci_smd_close(hdev);
        reset_worker = kzalloc(sizeof(*reset_worker), GFP_ATOMIC);
        if (!reset_worker) {
            BT_ERR("Out of memory");
            break;
        }
        INIT_WORK(reset_worker, hci_dev_restart);
        schedule_work(reset_worker);
        break;
    default:
        break;
    }
}

static void hci_smd_notify_data(void *data, unsigned int event)
{
    struct btm_device *hdev = hs.bdev;
    struct hci_smd_data *hsmd = &hs;
    int len = 0;

    if (!hdev) {
        BT_ERR("HCI device (hdev=NULL)");
        return;
    }

    switch (event) {
    case SMD_EVENT_DATA:
        len = smd_read_avail(hsmd->data_channel);
        if (len > 0)
            tasklet_hi_schedule(&hs.rx_data_task);
        else if (len < 0)
            BT_ERR("Failed to read data from smd %d", len);
        break;
    case SMD_EVENT_OPEN:
        BT_INFO("opening HCI-SMD channel :%s", DATA_CHANNEL);
        hci_smd_open(hdev);
        break;
    case SMD_EVENT_CLOSE:
        BT_INFO("Closing HCI-SMD channel :%s", DATA_CHANNEL);
        hci_smd_close(hdev);
        break;
    default:
        break;
    }

}

static int hci_smd_hci_register_dev(struct hci_smd_data *hsmd)
{
    struct btm_device* btm_dev;

    btm_dev = hsmd->bdev;

    if (btm_register_dev(btm_dev) < 0) {
        BT_ERR("Can't register btm device");
        btm_free_dev(btm_dev);
        return -ENODEV;
    }

    return 0;
}

static int hci_smd_register_smd(struct hci_smd_data *hsmd)
{
    struct btm_device* btm_dev;
    int rc;

    if (test_bit(HCI_SMD_REGISTERED, &hs.flags))
    {
        BT_ERR("HCI SMD already registered");
        return -EBUSY;
    }

    /* Initialize and register HCI device */
    btm_dev = btm_alloc_dev();
    if (!btm_dev) {
        BT_ERR("Can't allocate btm device");
        return -ENOMEM;
    }

    hsmd->bdev = btm_dev;
    btm_dev->driver_data = NULL;
    //btm_dev->rfkill = btuart_rfkill;
    btm_dev->open  = hci_smd_open;
    btm_dev->close = hci_smd_close;
    btm_dev->send  = hci_smd_send_frame;

    tasklet_init(&hsmd->rx_event_task,
            hci_smd_rx_event, (unsigned long) hsmd);
    tasklet_init(&hsmd->rx_data_task,
            hci_smd_rx_data, (unsigned long) hsmd);
    /*
     * Setup the timer to monitor whether the Rx queue is empty,
     * to control the wake lock release
     */
    setup_timer(&hsmd->rx_event_q_timer, schedule_rx_event_q_timer,
            (unsigned long) hsmd->bdev);
    setup_timer(&hsmd->rx_data_q_timer, schedule_rx_data_q_timer,
            (unsigned long) hsmd->bdev);

    /* Open the SMD Channel and device and register the callback function */
    rc = smd_named_open_on_edge(EVENT_CHANNEL, SMD_APPS_WCNSS,
            &hsmd->event_channel, btm_dev, hci_smd_notify_event);
    if (rc < 0) {
        BT_ERR("Cannot open the command channel");
        btm_free_dev(btm_dev);
        //bdev = NULL;  //freed inside btm_free_dev
        return -ENODEV;
    }

    rc = smd_named_open_on_edge(DATA_CHANNEL, SMD_APPS_WCNSS,
            &hsmd->data_channel, btm_dev, hci_smd_notify_data);
    if (rc < 0) {
        BT_ERR("Failed to open the Data channel");
        btm_free_dev(btm_dev);
        //bdev = NULL; //freed inside btm_free_dev
        return -ENODEV;
    }

    /* Disable the read interrupts on the channel */
    smd_disable_read_intr(hsmd->event_channel);
    smd_disable_read_intr(hsmd->data_channel);

    set_bit(HCI_SMD_REGISTERED, &hs.flags);

    return 0;
}

static void hci_smd_deregister_dev(struct hci_smd_data *hsmd)
{
    if (!test_bit(HCI_SMD_REGISTERED, &hs.flags))
    {
        BT_ERR("HCI SMD not registered yet!");
        return/* -EBUSY*/;
    }

    tasklet_kill(&hs.rx_event_task);
    tasklet_kill(&hs.rx_data_task);

    if (hsmd && hsmd->bdev)
    {
        if (btm_uni_close(hsmd->bdev) < 0)
        {
            BT_ERR("Erron on closing device %s", hsmd->bdev->name);
        }

        if (btm_unregister_dev(hsmd->bdev) < 0)
        {
            BT_ERR("Can't unregister HCI device %s", hsmd->bdev->name);
        }

        btm_free_dev(hsmd->bdev);
        //hsmd->bdev = NULL; //freed inside btm_free_dev
    }
    else
    {
        BT_ERR("hs or hsmd->bdev == null");
    }

    smd_close(hs.event_channel);
    smd_close(hs.data_channel);

    if (wake_lock_active(&hs.wake_lock_rx_event))
        wake_unlock(&hs.wake_lock_rx_event);
    if (wake_lock_active(&hs.wake_lock_rx_data))
        wake_unlock(&hs.wake_lock_rx_data);
    if (wake_lock_active(&hs.wake_lock_tx))
        wake_unlock(&hs.wake_lock_tx);

    /*Destroy the timer used to monitor the Rx queue for emptiness */
    if (hs.rx_event_q_timer.function) {
        del_timer_sync(&hs.rx_event_q_timer);
        hs.rx_event_q_timer.function = NULL;
        hs.rx_event_q_timer.data = 0;
    }
    if (hs.rx_data_q_timer.function) {
        del_timer_sync(&hs.rx_data_q_timer);
        hs.rx_data_q_timer.function = NULL;
        hs.rx_data_q_timer.data = 0;
    }

    clear_bit(HCI_SMD_REGISTERED, &hs.flags);
}


static void hci_dev_restart(struct work_struct *worker)
{
    mutex_lock(&hci_smd_enable);
    hci_smd_deregister_dev(&hs);
    hci_smd_register_smd(&hs);
    mutex_unlock(&hci_smd_enable);
    kfree(worker);
}

static void hci_dev_smd_open(struct work_struct *worker)
{
    mutex_lock(&hci_smd_enable);
    hci_smd_hci_register_dev(&hs);
    mutex_unlock(&hci_smd_enable);
    kfree(worker);
}

int set_driver_state(int on)
{
    driver_initialized = on;
    return 0;
}

static int hcismd_set_enable(const char *val, struct kernel_param *kp)
{
    int ret = 0;

    mutex_lock(&hci_smd_enable);

    ret = param_set_int(val, kp);

    if (ret)
        goto done;

    BT_INFO("Setting parameter: hcismd_set (%d)", hcismd_set);

    switch (hcismd_set) {

    case 1:
        BT_INFO("Registering Bluetooth device");
        hci_smd_register_smd(&hs);
    break;
    case 0:
        BT_INFO("Deregistering Bluetooth device");
        hci_smd_deregister_dev(&hs);
    break;
    default:
        ret = -EFAULT;
    }

done:
    mutex_unlock(&hci_smd_enable);
    return ret;
}

static int btm_rfkill_set_block(void *data, bool blocked)
{
    BT_INFO("BT rfkill state changes to: %d", blocked);
    return 0;
}


static const struct rfkill_ops btm_rfkill_ops = {
    .set_block = btm_rfkill_set_block,
};


static int  __init hci_smd_init(void)
{
    BT_INFO("Bluetooth Driver for Qualcomm with Motorola Solutions UNI Interface");
#ifndef NO_BUILD_CFG_H
    BT_INFO("BUILD VERSION: %s", CFG_BTM_VERSION);
    BT_INFO("BUILD NAME: %s", CFG_BTM_BUILD_NAME);
#endif
    wake_lock_init(&hs.wake_lock_rx_event, WAKE_LOCK_SUSPEND,
             "msm_smd_Rx_event");
    wake_lock_init(&hs.wake_lock_rx_data, WAKE_LOCK_SUSPEND,
             "msm_smd_Rx_data");
    wake_lock_init(&hs.wake_lock_tx, WAKE_LOCK_SUSPEND,
             "msm_smd_Tx");

    clear_bit(HCI_SMD_REGISTERED, &hs.flags);

    btuart_rfkill = rfkill_alloc(DEVICE_NAME, NULL, RFKILL_TYPE_BLUETOOTH, &btm_rfkill_ops, NULL);
    if (btuart_rfkill)
    {
        if (rfkill_register(btuart_rfkill) < 0)
        {
            BT_ERR("Unable to register rfkill");
            rfkill_destroy(btuart_rfkill);
            kfree(btuart_rfkill);
            btuart_rfkill = NULL;
        }
    }
    else
    {
        BT_ERR("Unable to allocate rfkill");
    }

    return 0;
}
module_init(hci_smd_init);

static void __exit hci_smd_exit(void)
{
    if (btuart_rfkill)
    {
        rfkill_destroy(btuart_rfkill);
        kfree(btuart_rfkill);
        btuart_rfkill = NULL;
    }

    wake_lock_destroy(&hs.wake_lock_rx_event);
    wake_lock_destroy(&hs.wake_lock_rx_data);
    wake_lock_destroy(&hs.wake_lock_tx);
}
module_exit(hci_smd_exit);

MODULE_DESCRIPTION("Bluetooth SMD driver");
MODULE_LICENSE("GPL v2");
