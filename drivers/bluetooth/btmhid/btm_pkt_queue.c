/*
 *  drivers/bluetooth/btmhid/btm_pkt_queue.c
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

#include "linux/sched.h"

#include "btm_pkt_queue.h"

int hid_init_pkt_queue(struct pkt_queue *pkt_queue)
{
    init_waitqueue_head(&(pkt_queue->data_avail));
    skb_queue_head_init(&pkt_queue->list);
    atomic_set(&pkt_queue->is_closing, 0);

    return 0;
}

int hid_deinit_pkt_queue(struct pkt_queue *pkt_queue) //hope nobody works on the queue during or after execution of this call ;)
{
    wake_up_all(&pkt_queue->data_avail);
    skb_queue_purge(&pkt_queue->list);

    return 0;
}

int hid_enqueue_pkt(struct pkt_queue *pkt_queue, struct sk_buff *skb)
{
    skb_queue_tail(&pkt_queue->list, skb);
    wake_up_interruptible(&pkt_queue->data_avail);

    return 0;
}


int hid_put_to_head_pkt(struct pkt_queue *pkt_queue, struct sk_buff *skb)
{
    skb_queue_head(&pkt_queue->list, skb);
    wake_up_interruptible(&pkt_queue->data_avail);

    return 0;
}

int hid_dequeue_pkt(struct pkt_queue *pkt_queue, struct sk_buff **skb)
{
    struct sk_buff *tmp;
    if (wait_event_interruptible(pkt_queue->data_avail,
                        !skb_queue_empty(&pkt_queue->list) || (atomic_read(&pkt_queue->is_closing) == 1)))
    {
        return -ERESTARTSYS;
    }

    if (atomic_read(&pkt_queue->is_closing) == 1)
    {
        return -ESHUTDOWN;
    }

    tmp = skb_dequeue(&pkt_queue->list);
    if (tmp == NULL)
    {
        return -EFAULT;
    }
    *skb = tmp;
    return (*skb)->len;
}

int hid_async_dequeue_pkt(struct pkt_queue *pkt_queue, struct sk_buff **skb)
{
    struct sk_buff *tmp;
    if (skb_queue_empty(&pkt_queue->list) || (atomic_read(&pkt_queue->is_closing) == 1))
    {
        return 0; // NOTE: Returned zero packet
    }

    if (atomic_read(&pkt_queue->is_closing) == 1)
    {
        return -ESHUTDOWN;
    }

    tmp = skb_dequeue(&pkt_queue->list);
    if (tmp == NULL)
    {
        return -EFAULT;
    }
    *skb = tmp;
    return (*skb)->len;
}

int hid_clear_pkt_queue(struct pkt_queue *pkt_queue)
{
    skb_queue_purge(&pkt_queue->list);
    atomic_set(&pkt_queue->is_closing, 0);

    return 0;
}

int hid_kill_waiters(struct pkt_queue *pkt_queue)
{
    atomic_set(&pkt_queue->is_closing, 1);
    wake_up_all(&pkt_queue->data_avail);

    return 0;
}
