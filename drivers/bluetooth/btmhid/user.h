/*
 *  drivers/bluetooth/btmhid/user.h
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

#ifndef __BTMHID_USER_H__
#define __BTMHID_USER_H__

//
// btmhid commands enum
//

typedef enum
{
    HID_GET_REPORTLEN = 0,
    HID_GET_REPORTBUF,
    HID_SET_REPORT,
    HID_ADD,
    HID_DELETE,
    HID_STOP,
    HID_INIT
} BTMHID_COMMAND_ENUM;

//
// btmhid ioctls parameters
//
typedef enum
{
    OTHER_REPORT = 0x0,
    INPUT_REPORT = 0x1,
    OUTPUT_REPORT = 0x2,
    FEATURE_REPORT = 0x3
} EnumReportType;

#pragma pack (push, 1)
typedef struct device_parameters_t
{
    short           vid;

    short           pid;

    short           version;

    unsigned short  rep_desc_len;

    unsigned short  name_len;

    char            *rep_desc;

    char            *name;

}   device_parameters_t;

typedef struct device_t
{
    device_parameters_t info;

    void                *ptr_to_dev;

}   device_t;

typedef struct set_report_params_t
{
    short               interrupt;

    void                *ptr_to_dev;

    EnumReportType      report_type;

    unsigned short      length;

    unsigned char       *data;

}   set_report_params_t;

typedef struct get_report_params_t
{
    void                *ptr_to_dev;

    EnumReportType      report_type;

    unsigned short      length;

    unsigned char       *data;

}   get_report_params_t;
#pragma pack (pop)

//ioctls
#define BTMHID_IOC_GROUP 0
#define BTMHID_IOCTL_ADD_DEVICE                  _IOWR(BTMHID_IOC_GROUP, HID_ADD, device_t *)
#define BTMHID_IOCTL_DELETE_DEVICE               _IOR(BTMHID_IOC_GROUP, HID_DELETE, void *)
#define BTMHID_IOCTL_STOP                        _IOR(BTMHID_IOC_GROUP, HID_STOP, void *)
#define BTMHID_IOCTL_SET_REPORT                  _IOR(BTMHID_IOC_GROUP, HID_SET_REPORT, set_report_params_t *)
#define BTMHID_IOCTL_GET_REPORT                  _IOW(BTMHID_IOC_GROUP, HID_GET_REPORTLEN, get_report_params_t *)
#define BTMHID_IOCTL_GET_REPORTBUF               _IOW(BTMHID_IOC_GROUP, HID_GET_REPORTBUF, unsigned char *)
#define BTMHID_IOC_MAXNR                          HID_INIT

#endif // __BTMHID_USER_H__
