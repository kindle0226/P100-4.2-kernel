/*
 *  drivers/bluetooth/btmuart/btm_log.c
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


#include <linux/proc_fs.h>

#define MODULE_NAME                     "BTM-LOG"
#define MODULE_TRACE_LEVEL              CFG_SYS_LEVEL

#include <linux/mutex.h>
#include <linux/string.h>

#include "btm_log.h"
#include "btm_uni.h"             // for debug logs

int btm_read_proc (char *page, char **start, off_t off, int count, int *eof, void *data);
int btm_write_proc (struct file* file, const char* buffer, unsigned long count, void* data);

int btm_init_dump( struct btm_dump* dump, char* name )
{
    if( !dump )
    {
    BT_ERR("error - dump");
        return -EINVAL;
    }
    
    if( !name || !strlen(name) )
    {
     BT_ERR("bad name");
        return -EINVAL;
    }
  
    dump->name = 0;
    
    // register in proc
    
    if( !( dump->name = kmalloc( strlen(name) + 1, GFP_KERNEL ) ) )
    {
        BT_ERR("can't allocate name");
        return -ENOMEM;
    }
    
    strcpy( dump->name, name );
        
    if ( NULL == (dump->entry = create_proc_entry( dump->name, 0777, NULL )) )
    {
        BT_ERR("can't create /proc/%s", dump->name);
        return -1;
    }
 
    BT_DBG("init /proc/%s", dump->name);
 
    dump->entry->read_proc = btm_read_proc;
    dump->entry->write_proc = btm_write_proc;
    dump->entry->data = dump;
    memset( dump->btm_dump_out, -1, sizeof(dump->btm_dump_out) );
    memset( dump->btm_dump_in, -1, sizeof(dump->btm_dump_in) );
#ifdef MUTEX_BLOCK
    mutex_init( &dump->dump_mutex );
#else
    // spin_lock_init( &dump->dump_spinlock );
#endif
    dump->btm_dump_out_offset = 0;
    dump->btm_dump_in_offset = 0;

    return 0;
    
}

int btm_read_proc (char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int len;
       
    struct btm_dump* dump = data;
  
    BT_DBG("page %p, start %p, off %ld, count %d, eof %p, data_unused %p, buffer offset %d frome %d",
        page, start, off, count, eof, data, dump->btm_dump_out_offset, LOG_SIZE);
    
    *start = page;
    *eof = 1;
#ifdef MUTEX_BLOCK
    mutex_lock( &dump->dump_mutex );
#else
    spin_lock_irqsave( &dump->dump_spinlock, dump->irq_state);
#endif
    if( off > dump->btm_dump_out_offset )
    {
    BT_DBG("off > dump->size");
        len = 0;
    }
    else if( off + count > dump->btm_dump_out_offset )
    {
        BT_DBG("off + count > dump->size");
        len = dump->btm_dump_out_offset - off;
    }
    else
    {
    BT_DBG("else");
        *eof = 0;
        len = count;
    }
    
    memcpy( *start, &dump->btm_dump_out[off], len );
#ifdef MUTEX_BLOCK
    mutex_unlock( &dump->dump_mutex );
#else
    spin_unlock_irqrestore( &dump->dump_spinlock, dump->irq_state );
#endif
    BT_DBG("eof - %d, len %d", *eof, len );
        
    return len;
}

//#include <net/bluetooth/bluetooth.h>
//extern struct btm_device *btm_dev;

// FIXME: Now it's dosn't work
int btm_write_proc (struct file* file, const char* buffer, unsigned long count, void* data)
{
#if 0
    int len;
    int offset = 0;
       
    struct btm_dump *dump = data;
    struct sk_buff *skb;
  
    BT_DBG("file %p, buffer %p, count %ld, data %p", file, buffer, count, data);
    
    spin_lock_irqsave( &dump->dump_spinlock, dump->irq_state);
/*
    if (dump->btm_dump_in_offset + count > LOG_SIZE)
    {
        count = LOG_SIZE - dump->btm_dump_in_offset;
    }
    if (count > 0 && copy_from_user(dump->btm_dump_in, buffer, count))
    {
        BT_DBG("copy_from_user() failed");
        return -EFAULT;
    }
    dump->btm_dump_in_offset += count;
*/
/*
    while (count > 0)
    {
        if (count >= 250)
        {
            len = 250;
            count -= 250;
            offset += 250;
        }
        else
        {
            len = count;
            offset += count;
            count = 0;
        }

        BT_DBG("read from dump");
        skb = bt_skb_alloc(len + SCO_HEADER_SIZE, GFP_ATOMIC);

        memcpy(skb->data, &(btm_dev->sco_handle_out), SCO_HANDLE_SIZE);
        *(uint8_t*)(skb->data + SCO_HANDLE_SIZE) = (uint8_t)len;
                
        skb->len = len + SCO_HEADER_SIZE;
        skb->dev = (void *) btm_dev;
        if (copy_from_user(skb->data + SCO_HEADER_SIZE, buffer + offset, len))
        {
            BT_DBG("Failed copy_from_user :(");
            kfree_skb(skb);
            continue;
        }

        BT_DBG("buf len: %d, skb->len: %d, count: %d, offset: %d", len, skb->len, count, offset);
    
        btm_uni_write(btm_dev, skb, BTIOCTL_SEND_SCODATA);
    }
*/
    spin_unlock_irqrestore( &dump->dump_spinlock, dump->irq_state );
    BT_DBG("count - %d", count);
#endif
    return count;
}

int btm_deinit_dump( struct btm_dump* dump )
{
    BT_DBG("");
        
    if( !dump )
    {
    BT_ERR("error !dump");
        return -EINVAL;
    }
#ifdef MUTEX_BLOCK
    mutex_destroy( &dump->dump_mutex );
#else
    // nothing do
#endif

    // deregister proc
    
    if( dump->entry )
    {
    BT_DBG( "delete /proc/%s", dump->name );
        remove_proc_entry( dump->name, NULL );
    }
    else
    {
        BT_ERR("no entry for %s", dump->name);
    }
    
    kfree( dump->name );
    
    return 0;
}

int btm_add_data_to_dump( struct btm_dump* dump, char* buffer, int length )
{
#ifdef PARANOIC_LOG
    BT_DBG("buffer %p, length %d, offset %d", buffer, length, dump->btm_dump_out_offset);
#endif
  
    if( !dump || !buffer )
    {
        BT_ERR("error - !dump || !buffer");
        return -EINVAL;
    }
    
#ifdef MUTEX_BLOCK
    mutex_lock( &dump->dump_mutex );
#else
    spin_lock_irqsave( &dump->dump_spinlock, dump->irq_state);
#endif
    
    if( LOG_SIZE == dump->btm_dump_out_offset )
    {
    
#ifdef PARANOIC_LOG
        BT_DBG("full");
#endif

#ifdef MUTEX_BLOCK
    mutex_unlock( &dump->dump_mutex );
#else
    spin_unlock_irqrestore( &dump->dump_spinlock, dump->irq_state );
#endif
        return 0;
    }
  
    if( dump->btm_dump_out_offset + length > LOG_SIZE )
    {
        length = LOG_SIZE - dump->btm_dump_out_offset;
#ifdef PARANOIC_LOG
    BT_DBG("last %d bytes", length);
#endif
    }
    
    memcpy( &dump->btm_dump_out[dump->btm_dump_out_offset], buffer, length );
    dump->btm_dump_out_offset += length;
 
#ifdef MUTEX_BLOCK
    mutex_unlock( &dump->dump_mutex );
#else
    spin_unlock_irqrestore( &dump->dump_spinlock, dump->irq_state );
#endif
    
    return length;
}

int btm_read_data_from_dump( struct btm_dump* dump, char* buffer, int length )
{
#ifdef PARANOIC_LOG
    BT_DBG("buffer %p, length %d, offset %d", buffer, length, dump->btm_dump_in_offset);
#endif
  
    if( !dump || !buffer )
    {
        BT_ERR("error - !dump || !buffer");
        return -EINVAL;
    }
    
    spin_lock_irqsave( &dump->dump_spinlock, dump->irq_state);
    
    if (dump->btm_dump_in_offset == 0)
    {
    
#ifdef PARANOIC_LOG
        BT_DBG("empty");
#endif

        spin_unlock_irqrestore( &dump->dump_spinlock, dump->irq_state );
        return 0;
    }
  
    if (dump->btm_dump_in_offset - length < 0)
    {
        length = dump->btm_dump_in_offset;
#ifdef PARANOIC_LOG
           BT_DBG("would be readed %d bytes", length);
#endif
    }
    
    memcpy (buffer, &dump->btm_dump_in[dump->btm_dump_in_offset], length);
    dump->btm_dump_in_offset -= length;
 
    spin_unlock_irqrestore (&dump->dump_spinlock, dump->irq_state);
    
    return length;
}
