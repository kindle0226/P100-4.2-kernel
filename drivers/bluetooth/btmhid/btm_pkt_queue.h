/*
 *  drivers/bluetooth/btmhid/btm_pkt_queue.h
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

#ifndef BTM_SKB_QUEUE_H_
#define BTM_SKB_QUEUE_H_

#include <linux/wait.h>
#include <linux/skbuff.h>
#include <linux/types.h>

struct pkt_queue
{
    wait_queue_head_t       data_avail;
    struct sk_buff_head     list;
    atomic_t                is_closing;
};

/**
  * Function that inits packet queue.
  * Called during btm_device allocating
  *
  * @pkt_queue - queue pointer.
  */
int hid_init_pkt_queue(struct pkt_queue *pkt_queue);

/**
  * Function that deinits packet queue.
  * Called on freeing btm_device
  *
  * @pkt_queue - queue pointer.
  */
int hid_deinit_pkt_queue(struct pkt_queue *pkt_queue);

/**
  * Function that adds socket buffer to a packet queue.
  * Called during btm_device allocating
  *
  * @pkt_queue - target queue pointer.
  * @skb - pointer to skb that we want to add
  */
int hid_enqueue_pkt(struct pkt_queue *pkt_queue, struct sk_buff *skb);

/**
  * Function that adds socket buffer into head a packet queue.
  * Called during btm_device allocating
  *
  * @pkt_queue - target queue pointer.
  * @skb - pointer to skb that we want to add
  */
int hid_put_to_head_pkt(struct pkt_queue *pkt_queue, struct sk_buff *skb);

/**
  * Function that gets socket buffer from a packet queue.
  *
  * @pkt_queue - source queue pointer.
  * @skb - pointer to pointer to skb that we eant to get
  *     such structure lets us avoid unneccessary memory copying
  */
int hid_dequeue_pkt(struct pkt_queue *pkt_queue, struct sk_buff **skb);
int hid_async_dequeue_pkt(struct pkt_queue *pkt_queue, struct sk_buff **skb);

/**
  * Function that removes all packets from a queue.
  *
  * @pkt_queue - queue pointer.
  */
int hid_clear_pkt_queue(struct pkt_queue *pkt_queue);

/**
  * Function that kills all waiting for input data threads.
  *
  * @pkt_queue - queue pointer.
  */
int hid_kill_waiters(struct pkt_queue *pkt_queue);

#endif //BTM_SKB_QUEUE_H_
