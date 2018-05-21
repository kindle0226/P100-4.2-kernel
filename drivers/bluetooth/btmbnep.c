/*
 *  Motorola Solutions Network driver for BNEP
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


#include <linux/netdevice.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/jiffies.h>
#include <asm/param.h>
#include <linux/ip.h>
#include <linux/if_ether.h>
#include <net/net_namespace.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/version.h>

/* Netlink Sockets */
#include <net/netlink.h>
#include <net/sock.h>

#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#ifndef NO_BUILD_CFG_H
#include "build_cfg.h"
#endif

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Motorola Solutions, Inc. http://www.motorolasolutions.com");
MODULE_VERSION("1.0.0");
MODULE_DESCRIPTION("Motorola Solutions Network driver for BNEP");

#define SERVICE_NAME "btmnetsrv"
/* Debug Macros */
#ifndef CFG_BNEP_KERNEL_TRACE_LEVEL
    #define CFG_BNEP_KERNEL_TRACE_LEVEL (0)
#endif

#define DPRINTK(klevel, fmt, args...)  printk(KERN_ ## klevel "%s: %s: " fmt "\n", btmbnep_netdev->name, __FUNCTION__, ## args )

#define MODULE_TRACE_LEVEL CFG_BNEP_KERNEL_TRACE_LEVEL

#if (MODULE_TRACE_LEVEL >= 1)
    #define BT_CRT(fmt, arg...) DPRINTK(CRIT, fmt, ##arg)
    #define BT_ERR(fmt, arg...) DPRINTK(ERR, fmt, ##arg)
#endif
#if (MODULE_TRACE_LEVEL >= 2)
    #define BT_WRN(fmt, arg...) DPRINTK(WARNING, fmt, ##arg)
#endif
#if (MODULE_TRACE_LEVEL >= 3)
    #define BT_NOTICE(fmt, arg...) DPRINTK(NOTICE, fmt, ##arg)
    #define BT_INFO(fmt, arg...) DPRINTK(INFO, fmt, ##arg)
#endif
#if (MODULE_TRACE_LEVEL >= 4)
    #define BT_DBG(fmt, arg...) DPRINTK(DEBUG, fmt, ##arg)
#endif

#ifndef BT_DBG
    #define BT_DBG(fmt, arg...)
#endif
#ifndef BT_WRN
    #define BT_WRN(fmt, arg...)
#endif
#ifndef BT_CRT
    #define BT_CRT(fmt, arg...)
#endif
#ifndef BT_ERR
    #define BT_ERR(fmt, arg...)
#endif
#ifndef BT_NOTICE
    #define BT_NOTICE(fmt, arg...)
#endif
#ifndef BT_INFO
    #define BT_INFO(fmt, arg...)
#endif

#undef PRINT_PACKAGE_DATA

/* Netlink Sockets Protocol */
#define NETLINK_BLUETOOTH       20
    
/* Socket messages */
#define DEFAULT_MSG             101
//#define NEW_MAC_ADDRESS_MSG     102 /*Deprecated. Use SIOCSIFHWADDR instead*/
#define CARRIER_ON_MSG          103
#define CARRIER_OFF_MSG         104
#define GET_STATISTIC_MSG       105
#define IFACE_NAME_MSG          106

/* Queue length to send skb to user space */
#define TX_QUEUE_LENGTH         20
/* Queue length for receive skb from user space */
#define RX_QUEUE_LENGTH         20

#define CLEAR_MAC "\x00\x00\x00\x00\x00\x00"

//TODO add macro dependent on version
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
typedef int netdev_tx_t;
#define SET_NETDEV_DEVTYPE(net, devtype)        ((net)->dev.type = (devtype))
#endif
////////

static struct sock *btmbnep_nl_sk = NULL;
static DEFINE_MUTEX(btmbnep_nl_mutex);
static struct task_struct *queue_task = NULL;

/* btmnetsrv PID */
static int service_pid = 0;

static struct net_device *btmbnep_netdev = NULL;

/* Queue flags */
enum {
    RECEIVE_QUEUE_READY, // receive_queue_ready_event flag
};

struct btmbnep_priv
{
    struct net_device *dev;
    struct net_device_stats stats;

    struct sk_buff_head receive_queue;
    struct sk_buff_head write_queue;

/* Notify that receive_queue ready for new skbuff. */
    wait_queue_head_t receive_queue_ready_event;

    unsigned long flags;
};

static struct device_type btmbnep_type = {
    .name = "bluetooth",
};

/* Find BNEP service */
static int btmbnep_find_service(void)
{
    struct task_struct *task_s;

    BT_DBG( "Start find BNEP service with name: " SERVICE_NAME);
    service_pid = 0;
    for_each_process(task_s)
    {
        if (!strcmp(task_s->comm, SERVICE_NAME))
        {
            BT_DBG( "btmnetsrv pid = %d", task_s->pid);
            service_pid = task_s->pid;
            break;
        }
    }
    if (service_pid == 0)
    {
        BT_WRN( "%s service not found", SERVICE_NAME);
        return -EAGAIN;
    }

    return 0;
}

static int btmbnep_dev_init(struct net_device *dev)
{
    BT_DBG( ">>>>");

    netif_carrier_off(btmbnep_netdev);
    
    BT_DBG( "<<");

    return 0;
}

static void btmbnep_dev_uninit(struct net_device *dev)
{
    BT_DBG( ">>>>");

    BT_DBG( "<<");

    return;
}

static int btmbnep_open(struct net_device *dev)
{

    BT_DBG( ">>>>");

    if (btmbnep_find_service() < 0)
        return -EAGAIN;
    
/* Service sends MAC address automaticaly.
 * This check is needed if service don't sent address.
 */
    if (!memcmp(dev->dev_addr, CLEAR_MAC, ETH_ALEN))
    {
        BT_WRN( "undefined mac address");
        return -EAGAIN;
    }

    netif_start_queue(dev);

    BT_DBG( "<<");

    return 0;
}

static int btmbnep_stop(struct net_device *dev)
{
    BT_DBG( ">>>>");

    netif_stop_queue(dev);

    BT_DBG( "<<");
    return 0;
}

static netdev_tx_t btmbnep_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct btmbnep_priv *priv = netdev_priv(dev);
    int err = NETDEV_TX_OK;
    struct nlmsghdr *nlh;
    struct sk_buff *new_skb = NULL;

#ifdef PRINT_PACKAGE_DATA
    struct iphdr *ip;
    union {
        u8 u_addr[4];
        __be32 b_addr;
    } addr;
    struct ethhdr eth;
#endif

    BT_DBG( ">>>>");

#ifdef PRINT_PACKAGE_DATA
    /* Printing source/destination MAC address and protocol type */

    memcpy(&eth, skb->data, sizeof(eth));
    BT_DBG( "mac daddr = %02x.%02x.%02x.%02x.%02x.%02x"
        , eth.h_dest[0], eth.h_dest[1], eth.h_dest[2]
        , eth.h_dest[3], eth.h_dest[4], eth.h_dest[5]);
    BT_DBG( "mac saddr = %02x.%02x.%02x.%02x.%02x.%02x"
        , eth.h_source[0], eth.h_source[1], eth.h_source[2]
        , eth.h_source[3], eth.h_source[4], eth.h_source[5]);
    BT_DBG( "type = %04x", eth.h_proto);

    /* Printing source/destination IP addresses */

    ip = ip_hdr(skb);
    addr.b_addr = ip->saddr;
    BT_DBG( "Source ip: %d.%d.%d.%d", addr.u_addr[0], addr.u_addr[1], addr.u_addr[2], addr.u_addr[3]);
    addr.b_addr = ip->daddr;
    BT_DBG( "Dest ip: %d.%d.%d.%d", addr.u_addr[0], addr.u_addr[1], addr.u_addr[2], addr.u_addr[3]);
#endif

    new_skb = nlmsg_new(skb->len, GFP_ATOMIC);

    if (!new_skb)
    {
        BT_ERR( "low on mem - packet dropped" );

        priv->stats.rx_dropped ++;
        return NETDEV_TX_BUSY;
    }

    nlh = NLMSG_NEW(new_skb, 0, 0, DEFAULT_MSG, skb->len, 0);

    memcpy(NLMSG_DATA(nlh), skb->data, skb->len);

    new_skb->dev = dev;
    new_skb->ip_summed = CHECKSUM_NONE;

#ifdef PRINT_PACKAGE_DATA
    BT_DBG( "skb->len: %d, must be len: %d, new_skb->len: %d", skb->len, NLMSG_SPACE(skb->len), new_skb->len);
#endif

    /* Note time */
    dev->trans_start = jiffies;

    if (service_pid > 0 && service_pid < PID_MAX_LIMIT)
    {
        skb_queue_tail(&priv->write_queue, new_skb);
        wake_up_process(queue_task);
        if (skb_queue_len(&priv->write_queue) >= TX_QUEUE_LENGTH)
        {
            BT_WRN("write_queue is full");
    
            netif_stop_queue(dev);
        }
    }

    //qjr847: Useless check because skb dereferenced few times before this line
    //        Removed to make KlocWork happy
    //if (skb)
    kfree_skb(skb);

    BT_DBG( "<<");
    return err;

nlmsg_failure:
    BT_ERR( "nlmsg_failure: >>>");
    if (new_skb)
        kfree_skb(new_skb);

    BT_ERR( "nlmsg_failure: <<<");

    return NETDEV_TX_BUSY;
}

static void btmbnep_rx(unsigned char *data, unsigned int length)
{
    struct btmbnep_priv *priv = netdev_priv(btmbnep_netdev);
    struct sk_buff *new_skb;

#ifdef PRINT_PACKAGE_DATA
    struct ethhdr eth;
#endif

    BT_DBG( ">>>>" );

    new_skb = dev_alloc_skb(length + NET_IP_ALIGN);

    if (!new_skb)
    {
        BT_ERR( "low on mem - packet dropped");
        
        priv->stats.rx_dropped++;
        return;
    }

    skb_reserve(new_skb, NET_IP_ALIGN);

    memcpy(skb_put(new_skb, length), data, length);

    new_skb->dev = btmbnep_netdev;
    new_skb->protocol = eth_type_trans( new_skb, btmbnep_netdev );
//    new_skb->ip_summed = CHECKSUM_UNNECESSARY;
//    new_skb->ip_summed = skb_csum_unnecessary(new_skb);
    new_skb->ip_summed = CHECKSUM_NONE;

#ifdef PRINT_PACKAGE_DATA
    BT_DBG( "Received length: %d, new_skb->len: %d", length, new_skb->len );

    memcpy(&eth, eth_hdr( new_skb ), sizeof( eth ));
    BT_DBG( "mac daddr = %02x.%02x.%02x.%02x.%02x.%02x"
        , eth.h_dest[0], eth.h_dest[1], eth.h_dest[2]
        , eth.h_dest[3], eth.h_dest[4], eth.h_dest[5]);
    BT_DBG( "mac saddr = %02x.%02x.%02x.%02x.%02x.%02x"
        , eth.h_source[0], eth.h_source[1], eth.h_source[2]
        , eth.h_source[3], eth.h_source[4], eth.h_source[5]);
    BT_DBG( "type = %04x", eth.h_proto);
#endif

    skb_queue_tail(&priv->receive_queue, new_skb);
    wake_up_process(queue_task);
    if (skb_queue_len(&priv->receive_queue) >= RX_QUEUE_LENGTH)
    {
        BT_WRN("receive_queue is full, wait while sk_receive_queue will be purged");

        wait_event(priv->receive_queue_ready_event, test_bit(RECEIVE_QUEUE_READY, &priv->flags));
        clear_bit(RECEIVE_QUEUE_READY, &priv->flags);

        BT_WRN("receive_queue again is ready");
    }

    BT_DBG( "<<");
}

void btmbnep_tx_timeout (struct net_device *dev)
{
    struct btmbnep_priv *priv = netdev_priv(dev);

    BT_DBG( ">>>>");
    BT_ERR( "Transmit timeout at %ld, latency %ld", jiffies, jiffies - dev->trans_start);

    priv->stats.tx_errors++;
    netif_wake_queue(dev);

    BT_DBG( "<<");
}

static struct net_device_stats *btmbnep_get_stats(struct net_device *dev)
{
    struct btmbnep_priv *priv = netdev_priv(dev);

    return &priv->stats;
}

static void btmbnep_send_stats(void)
{
    struct btmbnep_priv *priv = netdev_priv(btmbnep_netdev);
    struct nlmsghdr *nlh;
    int len;
    struct sk_buff *new_skb;

    BT_DBG( ">>>>");

    /* Copy only 8 first elements of statistic structure */
    len = sizeof(unsigned long) * 8;

    new_skb = nlmsg_new(len, GFP_ATOMIC);

    if (!new_skb)
    {
        BT_ERR( "low on mem - statistic packet dropped" );
        return;
    }

    nlh = NLMSG_NEW(new_skb, service_pid, 0, GET_STATISTIC_MSG, len, 0);

    memcpy(NLMSG_DATA(nlh), &priv->stats, sizeof(struct net_device_stats));

#ifdef PRINTK_PACKAGE_DATA
    BT_DBG( "new_skb->len: %d", new_skb->len);
#endif

    if (service_pid > 0 && service_pid < PID_MAX_LIMIT)
    {
        skb_queue_tail(&priv->write_queue, new_skb);
        wake_up_process(queue_task);
    }

    BT_DBG( "<<");

    return;

nlmsg_failure:
    BT_ERR( "nlmsg_failure: >>>");
    if (new_skb)
        kfree_skb(new_skb);

    BT_ERR( "nlmsg_failure: <<<");
}

/* Sending interface name to service */
static void btmbnep_send_iface_name(void)
{
    struct nlmsghdr *nlh;
    int len;
    struct sk_buff *new_skb;
    struct btmbnep_priv *priv = netdev_priv(btmbnep_netdev);

    BT_DBG( ">>>>");

    len = strlen(btmbnep_netdev->name);

    new_skb = nlmsg_new(len, GFP_ATOMIC);

    if (!new_skb)
    {
        BT_ERR( "low on mem - statistic packet dropped" );
        return;
    }

    nlh = NLMSG_NEW(new_skb, service_pid, 0, IFACE_NAME_MSG, len, 0);
    strncpy(NLMSG_DATA(nlh), btmbnep_netdev->name, len);

#ifdef PRINTK_PACKAGE_DATA
    BT_DBG( "new_skb->len: %d", new_skb->len);
#endif

    if (service_pid > 0 && service_pid < PID_MAX_LIMIT)
    {
        skb_queue_tail(&priv->write_queue, new_skb);
        wake_up_process(queue_task);
    }

    BT_DBG( "<<");

    return;

nlmsg_failure:
    BT_ERR( "nlmsg_failure: >>>");
    if (new_skb)
        kfree_skb(new_skb);

    BT_ERR( "nlmsg_failure: <<<");
}

/* Netlink Sockets Callback */
static void btmbnep_socket_receive(struct sk_buff *skb)
{
    struct nlmsghdr *nlh;

    BT_DBG( ">>>>");

    mutex_lock(&btmbnep_nl_mutex);

    nlh = nlmsg_hdr(skb);

    /*
     * If it`s first connect of btmnetsrv to driver (service_pid = 0)
     * and we are don`t known her PID,
     * pid must be found
     */
    if (service_pid == 0)
        btmbnep_find_service();

    /*
     * In this cause btmnet_srv may be restarted
     * and her PID is changed
     * then need again find btmnetsrv PID
     */
    if (nlh->nlmsg_pid != service_pid && service_pid != 0)
    {
        BT_WRN( "message from unknown process pid: %d", nlh->nlmsg_pid);
        if (btmbnep_find_service() < 0)
            goto unlock;
    }

    if (service_pid == 0)
    {
        BT_WRN( "service_pid not defined");
        goto unlock;
    }

#ifdef PRINTK_PACKAGE_DATA
    BT_DBG( "nlmsg_type = %d", nlh->nlmsg_type);
    BT_DBG( "nlmsg_len = %d", nlh->nlmsg_len);
#endif

    switch (nlh->nlmsg_type)
    {
        case DEFAULT_MSG:
            BT_DBG( "DEFAULT_MSG");
            btmbnep_rx(NLMSG_DATA(nlh), nlh->nlmsg_len);
        break;
        
        case CARRIER_ON_MSG:
            BT_DBG( "CARRIER_ON_MSG");
            netif_carrier_on(btmbnep_netdev);
        break;

        case CARRIER_OFF_MSG:
            BT_DBG( "CARRIER_OFF_MSG");
            netif_carrier_off(btmbnep_netdev);
        break;
        
        case GET_STATISTIC_MSG:
            BT_DBG( "GET_STATISTIC_MSG");
            btmbnep_send_stats();
        break;

        case IFACE_NAME_MSG:
            BT_DBG( "IFACE_NAME_MSG");
            btmbnep_send_iface_name();
        break;

        default:
            BT_WRN( "unknown netlink message type: %d", nlh->nlmsg_type);
        break;
    }

unlock:
    mutex_unlock(&btmbnep_nl_mutex);

    BT_DBG( "<<");
}

static int queue_thread(void *data)
{
    struct sk_buff *skb;
    int res;
    struct btmbnep_priv *priv = netdev_priv(btmbnep_netdev);

    while (!kthread_should_stop())
    {
        set_current_state(TASK_INTERRUPTIBLE);

        BT_DBG( "handle sk_write_queue");
        while ((skb = skb_dequeue(&priv->write_queue)))
        {
            BT_DBG( "sending packet with length: %d", skb->len);
            /* Don't call kfree_skb (or dev_kfree_skb) after netlink_unicast, because
             * it leads to kernel crash. */
            res = netlink_unicast(btmbnep_nl_sk, skb, service_pid, 0);
            if (res != skb->len)
            {
                priv->stats.tx_errors++;
                BT_ERR( "error netlink_unicast: %d", res);
                break;
            }
            else
            {
                BT_DBG( "netlink_unicast transmitted bytes: %d", res );
                priv->stats.tx_bytes += res;
                priv->stats.tx_packets++;
            }
        }

        BT_DBG( "handle sk_receive_queue");
        while ((skb = skb_dequeue(&priv->receive_queue)))
        {
            BT_DBG( "receiving packet with length: %d", skb->len);
            priv->stats.rx_packets++;
            priv->stats.rx_bytes += skb->len;

            if (in_interrupt()) {
                res = netif_rx(skb);
            } else {
                res = netif_rx_ni(skb);
            }

            /* Don't call kfree_skb() (or dev_kfree_skb()) after netif_rx(), because
             * it leads to kernel crash. */
            if (NET_RX_DROP == res)
            {
                BT_ERR( "netif_rx() error: %d, NET_RX_DROP", res);
                priv->stats.rx_dropped++;
            }
            else
            {
                BT_DBG( "netif_rx() result success!");
            }
        }


        BT_DBG( "before netif_wake_queue()");
        netif_wake_queue(btmbnep_netdev);


        BT_DBG( "wake_up_all() on receive_queue_ready_event");
        set_bit(RECEIVE_QUEUE_READY, &priv->flags);
        wake_up_all(&priv->receive_queue_ready_event);

        BT_DBG( "schedule()");
        schedule();
    }

    DPRINTK(ALERT, "### kthread_should_stop() return true! ###");
    set_current_state(TASK_RUNNING);
    
    return 0;
}

static int btmbnep_pernet_init(struct net *net)
{
    BT_DBG( ">>>>");

    btmbnep_nl_sk = netlink_kernel_create(net, NETLINK_BLUETOOTH, 0, btmbnep_socket_receive, &btmbnep_nl_mutex, THIS_MODULE);
    if (!btmbnep_nl_sk)
    {
        DPRINTK(ALERT, "netlink_kernel_create() failed");
        return -ENOMEM;
    }
    
    btmbnep_nl_sk->sk_sndtimeo = MAX_SCHEDULE_TIMEOUT;
    
    queue_task = kthread_run(queue_thread, NULL, "kbnep_queue");
    if (IS_ERR(queue_task))
    {
        BT_ERR( "error when kthread_run: %ld", PTR_ERR(queue_task));

        netlink_kernel_release(btmbnep_nl_sk);
        free_netdev(btmbnep_netdev);
        return PTR_ERR(queue_task);
    }
        
    BT_DBG( "<<");
    return 0;
}

static void btmbnep_pernet_exit(struct net *net)
{
    BT_DBG( ">>>>");

    kthread_stop(queue_task);
    netlink_kernel_release(btmbnep_nl_sk);

    BT_DBG( "<<");
}

static const struct net_device_ops btmbnep_ops =
{
    .ndo_init               = btmbnep_dev_init,
    .ndo_start_xmit         = btmbnep_xmit,
    .ndo_get_stats          = btmbnep_get_stats,
    .ndo_open               = btmbnep_open,
    .ndo_stop               = btmbnep_stop,
    .ndo_tx_timeout         = btmbnep_tx_timeout,
    .ndo_uninit             = btmbnep_dev_uninit,
    .ndo_set_mac_address    = eth_mac_addr,
    .ndo_change_mtu         = eth_change_mtu,
};

static struct pernet_operations btmbnep_pernet_ops = {
    .init = btmbnep_pernet_init,
    .exit = btmbnep_pernet_exit,
};

static void btmbnep_setup( struct net_device *dev )
{
    ether_setup(dev);

    dev->watchdog_timeo     = HZ * 2;
    dev->netdev_ops         = &btmbnep_ops;

    SET_NETDEV_DEVTYPE(dev, &btmbnep_type);
}

static int __init btmbnep_init(void)
{
    struct btmbnep_priv *priv;
    int err = 0;

#ifndef NO_BUILD_CFG_H
    BT_INFO( "BUILD VERSION: %s", CFG_BTM_VERSION);
    BT_INFO( "BUILD NAME: %s", CFG_BTM_BUILD_NAME);
    BT_INFO( "COPYRIGHT: %s", CFG_BTM_COPYRIGHT);
    BT_INFO( "COPYRIGHT YEAR: %s", CFG_BTM_COPYRIGHT_YEAR);
#endif

    btmbnep_netdev = alloc_netdev(sizeof(struct btmbnep_priv), "bnep%d", btmbnep_setup);
    if (!btmbnep_netdev)
    {
        return -ENOMEM;
    }
    
    err = register_netdev(btmbnep_netdev);
    if (err)
    {
        BT_ERR( "error when register_netdev: %d", err);
        free_netdev(btmbnep_netdev);
        return err;
    }

    priv = netdev_priv(btmbnep_netdev);
    memset(priv, 0, sizeof(struct btmbnep_priv));

    init_waitqueue_head(&priv->receive_queue_ready_event);
    skb_queue_head_init(&priv->receive_queue);
    skb_queue_head_init(&priv->write_queue);

    err = register_pernet_subsys(&btmbnep_pernet_ops);
    if (err)
    {
        BT_ERR( "error when register_pernet_device: %d\n", err);
        skb_queue_purge(&priv->receive_queue);
        skb_queue_purge(&priv->write_queue);
        unregister_netdev(btmbnep_netdev);
        free_netdev(btmbnep_netdev);
        return err;
    }

    memset(btmbnep_netdev->dev_addr, 0, ETH_ALEN);

    BT_DBG( "");

    return 0;
}

static void __exit btmbnep_exit(void)
{
    struct btmbnep_priv *priv = netdev_priv(btmbnep_netdev);

    BT_DBG(">>>>");

    skb_queue_purge(&priv->receive_queue);
    skb_queue_purge(&priv->write_queue);

    unregister_pernet_subsys(&btmbnep_pernet_ops);
    unregister_netdev(btmbnep_netdev);

    free_netdev(btmbnep_netdev);

    BT_DBG("<<");
}

module_init(btmbnep_init);
module_exit(btmbnep_exit);
