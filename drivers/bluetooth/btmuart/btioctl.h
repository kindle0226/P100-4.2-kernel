/*
 *  drivers/bluetooth/btmuart/btioctl.h
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

#ifndef BTIOCTL_H_
#define BTIOCTL_H_

#include <linux/ioctl.h>

typedef enum
{
    BTIOCTL_OPEN,
    BTIOCTL_CLOSE,
    BTIOCTL_FLUSH_EVENT,
    BTIOCTL_FLUSH_ACLDATA,
    BTIOCTL_FLUSH_SCODATA,
    BTIOCTL_SEND_COMMAND,
    BTIOCTL_SEND_ACLDATA,
    BTIOCTL_SEND_SCODATA,
    BTIOCTL_READ_EVENT,
    BTIOCTL_READ_ACLDATA,
    BTIOCTL_READ_SCODATA,
    BTIOCTL_START_SCOIN,
    BTIOCTL_STOP_SCOIN,
    BTIOCTL_START_SCOOUT,
    BTIOCTL_STOP_SCOOUT,
    BTIOCTL_CLEAR_SCOIN,
    //BTIOCTL_GET_STATECHANGE

} BtUsb_IOCTLtype;

#define BTUSB_MAGIC 237

#define BTUSB_IOCOPEN _IO(BTUSB_MAGIC,0)
#define BTUSB_IOCCLOSE _IO(BTUSB_MAGIC,1)
#define BTUSB_IOCKILLBULKREQ _IO(BTUSB_MAGIC,2)
#define BTUSB_IOCKILLINTRREQ _IO(BTUSB_MAGIC,3)
#define BTUSB_IOCSTARTSCOIN _IO(BTUSB_MAGIC,4)
#define BTUSB_IOCSTARTSCOOUT _IO(BTUSB_MAGIC,5)


#define BTUSB_BLUETOOTH_IOCTL _IOWR(BTUSB_MAGIC, 10, BluetoothIoctlParameters)

#endif //BTIOCTL_H_
