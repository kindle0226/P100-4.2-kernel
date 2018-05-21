/*
 *  drivers/bluetooth/btmuart/btm_log.h
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

#ifndef BTM_LOG_H
#define BTM_LOG_H

#define LOG_SIZE 1000000
#define PARANOIC_LOG

#include <linux/mutex.h>

struct btm_dump
{
    char   btm_dump_out[LOG_SIZE];
    char   btm_dump_in[LOG_SIZE]; // From user space to kernel
    int    btm_dump_out_offset;
    int    btm_dump_in_offset;
    struct proc_dir_entry* entry;
    char*  name;
#ifdef MUTEX_BLOCK
    struct mutex dump_mutex;
#else
    spinlock_t dump_spinlock;
    unsigned long irq_state;
#endif
};

int btm_init_dump( struct btm_dump* dump, char* name );
int btm_deinit_dump( struct btm_dump* dump );
int btm_add_data_to_dump( struct btm_dump* dump, char* buffer, int length );
int btm_read_data_from_dump( struct btm_dump* dump, char* buffer, int length);

#endif
