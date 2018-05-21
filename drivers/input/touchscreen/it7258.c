/*
 * drivers/input/touchscreen/it7258_ts.c
 *
 * it7258 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/firmware.h>
#include <linux/platform_device.h>

#include <linux/slab.h>

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/io.h>

#include <linux/input/mt.h>//slot
#include <linux/input/it7258.h>
#include <linux/regulator/consumer.h>
#include <linux/pm.h>
#include <linux/twl6030-pwrbutton.h> //open while powerkey ok


//#define MOUSE_POINTER

#if IT_DEBUG_MSG
#define IT7258_DBG(format, ...)	printk(""format" \n", ## __VA_ARGS__)//printk(KERN_INFO "it7258 " format "\n", ## __VA_ARGS__)
#else
#define IT7258_DBG(format, ...)
#endif

#define MAX_BUFFER_SIZE		144

#define MAX_X_RESOLUTION	240
#define MAX_Y_RESOLUTION	320

static struct regulator *it7258_2v8;
static struct i2c_client *this_client;
static  struct i2c_driver it7258_ts_driver;
static int tmp_prox;  //disable irq, when close prox func
static int Calibration__success_flag = 0;
static int Upgrade__success_flag = 0;

int init_irq,init_reset,init_en,max_x,max_y;
bool swap,reverse;
struct class *it7258_class;
struct device *it7258_cmd_dev;

static bool is_from_resume = false;

static void it7258_ts_reset(void)
{
	tmp_prox = 0;
	msleep(100);
	gpio_request(init_en,"GPIO_TP_EN");
	gpio_direction_output(init_en,1);   //0:normal state, 1:firmware state
	msleep(10);
	gpio_request(init_reset,"GPIO_TP_RST");
	gpio_direction_output(init_reset,0);
	//msleep(10);
	//gpio_direction_output(init_reset,1);
	//msleep(10);
	//gpio_direction_output(init_reset,0);

	tmp_prox = 1;
	msleep(100);
}

static int it7258_i2c_read(uint8_t *buf, int lenth, uint8_t addr)
{
	int ret;
	struct i2c_msg msgs[2] = {
		{
			.addr = this_client->addr,
			.flags = I2C_M_NOSTART,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = lenth,
			.buf = buf,
		}
	};

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if(ret <= 0)
		IT7258_DBG("it7258_i2c_read_interface error\n");

	return (ret == 2) ? lenth : ret;
}

static int it7258_i2c_write(uint8_t *buf, int lenth)
{
	int ret;

	ret = i2c_master_send(this_client, buf, lenth);
	if(ret <= 0)
		IT7258_DBG("it7258_i2c_write_interface error\n");

	return ret;
}

static int it7258_i2c_write2(uint8_t reg, uint8_t *buf, int lenth)
{
	unsigned char tmpbuf[255];
#if 0
	int ret;

	struct i2c_msg msgs[1] = {
		{
			.addr = this_client->addr,
			.flags = 0, /* default write flag */
			.len = lenth + 1,
			.buf = tmpbuf,
		}
	};
#endif

	tmpbuf[0] = reg;
	memcpy(&tmpbuf[1], buf, lenth);

#if 0
	ret = i2c_transfer(this_client->adapter, msgs, 1);
	if(ret < 0)
		IT7258_DBG("it7258_i2c_write2_interface error\n");

	return (ret == 1) ? sizeof(tmpbuf) : ret;
#else
	return it7258_i2c_write(tmpbuf, lenth+1);
#endif
}

bool waitCommandDone(void)
{
	unsigned char ucQuery = 0xFF;
	unsigned int count = 0;

	do{
		ucQuery = 0xFF;
		it7258_i2c_read( &ucQuery, 1, QUERY_BUF);
		count++;
	}while(ucQuery & 0x01 && count < 500);

	if( !(ucQuery & 0x01) ){
		return  true;
	}else{
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return  false;
	}
}

static int it7258_read_data(struct i2c_client *client)
{
	int ret;
	unsigned char query = 0;
	unsigned char pucPoint[14];

	struct it7258_ts_data *data= i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

	ret = it7258_i2c_read( &query, 1, QUERY_BUF);
	if (!( query & 0x80 || query & 0x01 )){
		IT7258_DBG("No point information\n");
		return 0;
	}
	//IT7258_DBG("Get tp point information!!!!!!!!!!!!!!!\n");

	memset(&pucPoint, 0, sizeof(pucPoint));
	ret = it7258_i2c_read( &pucPoint[0], 14, POINT_INFO_BUF);
	if (ret == 14) {
		if ((pucPoint[0] & 0xf0) == 0x80) {	//gesture
			/*IT7258_DBG("%s:  get gesture, GID=0x%x, [ 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x ]\n"
				, __func__
				, pucPoint[1], pucPoint[2], pucPoint[3], pucPoint[4], pucPoint[5], pucPoint[6], pucPoint[7]
				, pucPoint[8], pucPoint[9], pucPoint[10], pucPoint[11], pucPoint[12], pucPoint[13]);*/
			switch (pucPoint[1]) {
#ifdef MOUSE_POINTER
				case 0x25: //drag
					break;
				case 0x22: //flick
					/*IT7258_DBG("%s:  get gesture, 0x%02x%02x, 0x%02x%02x\n", __func__
						, pucPoint[7], pucPoint[6], pucPoint[9], pucPoint[8]);*/
					{
						// NOTE: The original resolution of the touchpad is 480x800.
						// In other to fit the screen resolution, we need to swap X & Y.
						unsigned char x_abs, y_abs;
						x_abs = (pucPoint[6]>0x7f)? pucPoint[6]-0x7f : 0x7f-pucPoint[6];
						y_abs = (pucPoint[8]>0x7f)? pucPoint[8]-0x7f : 0x7f-pucPoint[8];
						//IT7258_DBG("%s:  x_abs=%d, y_abs=%d\n", __func__, x_abs, y_abs);
						if (x_abs < y_abs) {
							//IT7258_DBG("%s:  get gesture, %s x\n", __func__, pucPoint[7]? "negative" : "positive");
#ifdef DBG_REVERSE_180
							event->gesture = (pucPoint[7]? 5 : 4);
#else
							event->gesture = (pucPoint[7]? 4 : 5);
#endif /*DBG_REVERSE_180*/
						} else {
							//IT7258_DBG("%s:  get gesture, %s y\n", __func__, pucPoint[9]? "negative" : "positive");
#ifdef DBG_REVERSE_180
							event->gesture = (pucPoint[9]? 2 : 3);
#else
							event->gesture = (pucPoint[9]? 3 : 2);
#endif /*DBG_REVERSE_180*/
						}
					}
					break;

				case 0x20: //tap
					//IT7258_DBG("%s: get gesture: tap\n", __func__);
					event->gesture = 1;
					break;
#endif /*MOUSE_POINTER*/
				default:
					return 0;
			}
			return 1;
		} else if ((pucPoint[0] & 0xf0) == 0x0) {
			/*IT7258_DBG("%s:  get point(s), info=0x%x, point, P1(%d,%d), P2(%d,%d), P3(%d,%d)\n", __func__, pucPoint[0]
				, pucPoint[2]+(pucPoint[3]&0xf)*0x100, pucPoint[4]+(pucPoint[3]&0xf0)*0x10
				, pucPoint[6]+(pucPoint[7]&0xf)*0x100, pucPoint[8]+(pucPoint[7]&0xf0)*0x10
				, pucPoint[10]+(pucPoint[11]&0xf)*0x100, pucPoint[12]+(pucPoint[11]&0xf0)*0x10);*/
			// NOTE: The original resolution of the touchpad is 480x800.
			// In other to fit the screen resolution, we need to swap X & Y.
			static u16 curr_x = 0, curr_y = 0;
			switch (pucPoint[0] & 0x7) {
				case 0x1: /*one point*/
					{
						u16 last_x = curr_x, last_y = curr_y;
						curr_x = pucPoint[2]+(pucPoint[3]&0xf)*0x100;
						curr_y = pucPoint[4]+(pucPoint[3]&0xf0)*0x10;

						if (swap) {
							u16 tmp = curr_x;
							curr_x = curr_y;
							curr_y = tmp;
						}
#ifdef MOUSE_POINTER
						if (last_x != 0 || last_y != 0) {
#endif /*MOUSE_POINTER*/
							event->gesture = 6;
#ifdef MOUSE_POINTER
							if (reverse) {
								event->rel_x = 1 * (curr_x - last_x);
								event->rel_y = 1 * (curr_y - last_y);
							} else {
								event->rel_x = -1 * (curr_x - last_x);
								event->rel_y = -1 * (curr_y - last_y);
							}
#else
							event->touch_point = 1;
							if (reverse) {
								event->x1 = curr_x;
								event->y1 = curr_y;
							} else {
								event->x1 = max_x-curr_x;
								event->y1 = max_y-curr_y;
							}
#endif /*MOUSE_POINTER*/
#ifdef MOUSE_POINTER
						} else return 0;
#endif /*MOUSE_POINTER*/
					}
					break;
#ifndef MOUSE_POINTER
				case 0x3: /*two points*/
					{
						curr_x = pucPoint[2]+(pucPoint[3]&0xf)*0x100;
						curr_y = pucPoint[4]+(pucPoint[3]&0xf0)*0x10;

						u16 curr_x2, curr_y2;
						curr_x2 = pucPoint[6]+(pucPoint[7]&0xf)*0x100;
						curr_y2 = pucPoint[8]+(pucPoint[7]&0xf0)*0x10;

						if (swap) {
							u16 tmp = curr_x;
							curr_x = curr_y;
							curr_y = tmp;
							tmp = curr_x2;
							curr_x2 = curr_y2;
							curr_y2 = tmp;
						}

						event->gesture = 6;
						event->touch_point = 2;

						if (reverse) {
							event->x1 = curr_x;
							event->y1 = curr_y;
							event->x2 = curr_x2;
							event->y2 = curr_y2;
						} else {
							event->x1 = max_x-curr_x;
							event->y1 = max_y-curr_y;
							event->x2 = max_x-curr_x2;
							event->y2 = max_y-curr_y2;
						}
					}
					break;
#endif /*MOUSE_POINTER*/
				case 0x0:
					curr_x = curr_y = 0;
#ifndef MOUSE_POINTER
					event->gesture = 6;
					event->touch_point = 0;
					break;
#endif /* MOUSE_POINTER */
				default:
					return 0;
			}
			return 1;
		}
	}
	return 0;
}

static void it7258_report_value(void)
{
	struct it7258_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

	IT7258_DBG("%s, gesture = %d\n", __func__, event->gesture);
	
	if(is_from_resume) {
		
		printk("qmqiu %s:%d\n",__FUNCTION__,__LINE__);
		input_report_key(pbutton.input_dev, pbutton.report_key, 1);
		input_sync(pbutton.input_dev);

		msleep(20);

		input_report_key(pbutton.input_dev, pbutton.report_key, 0);
		input_sync(pbutton.input_dev);
		is_from_resume = false;
		return;
	}
	switch(event->gesture) {
		case 1:
			input_report_key(data->input_dev, BTN_LEFT, 1);		//tap
			input_sync(data->input_dev);
			msleep(1);
			input_report_key(data->input_dev, BTN_LEFT, 0);
			break;
		case 2:
			input_report_key(data->input_dev, KEY_LEFT, 1);		//left
			input_sync(data->input_dev);
			msleep(1);
			input_report_key(data->input_dev, KEY_LEFT, 0);
			break;
		case 3:
			input_report_key(data->input_dev, KEY_RIGHT, 1);	//right
			input_sync(data->input_dev);
			msleep(1);
			input_report_key(data->input_dev, KEY_RIGHT, 0);
			break;
		case 4:
			input_report_key(data->input_dev, KEY_BACK, 1);		//down
			input_sync(data->input_dev);
			msleep(1);
			input_report_key(data->input_dev, KEY_BACK, 0);
			break;
		case 5:
			input_report_key(data->input_dev, KEY_MENU, 1);		//up
			input_sync(data->input_dev);
			msleep(1);
			input_report_key(data->input_dev, KEY_MENU, 0);
			break;
		case 6:
#ifdef MOUSE_POINTER
			input_report_rel(data->input_dev, REL_X, event->rel_x);
			input_report_rel(data->input_dev, REL_Y, event->rel_y);
#else
			input_mt_slot(data->input_dev, 0);
			if (event->touch_point >= 1) {
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
			} else
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			input_mt_slot(data->input_dev, 1);
			if (event->touch_point == 2) {
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
			} else
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
#endif /*MOUSE_POINTER*/
			break;
		default:
			//IT7258_DBG("==touch_point default =\n");
			break;
	}
	input_sync(data->input_dev);
}

static void it7258_ts_irq_work(struct work_struct *work)
{
	if (it7258_read_data(this_client)) {
		it7258_report_value();
	}
	enable_irq(this_client->irq);
}

static irqreturn_t it7258_ts_interrupt(int irq, void *dev_id)
{
	struct it7258_ts_data *it7258_ts = (struct it7258_ts_data *)dev_id;


	if(tmp_prox == 0)
		return IRQ_HANDLED;

	//IT7258_DBG("%s\n", __func__);

	disable_irq_nosync(this_client->irq);
	if (!work_pending(&it7258_ts->event_work)) {
		queue_work(it7258_ts->ts_workqueue, &it7258_ts->event_work);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void it7258_ts_suspend(struct early_suspend *handler)
{
	int ret = 0;
	unsigned char Wrbuf[4] = { CMD_BUF, 0x04, 0x00, 0x01};  //idle mode

	msleep(100);
	ret = it7258_i2c_write(Wrbuf, 4);
	if(ret != 4){
		IT7258_DBG("i2c write communcate error during suspend: 0x%x\n", ret);
	}

	enable_irq_wake(this_client->irq);
	is_from_resume = true;
}

static void it7258_ts_resume(struct early_suspend *handler)
{
	/*
	This will cause " i2c timeout waiting for bus ready".
	tp is working well after resuming even if we don't read buffer from register QUERY_BUF.
	*/
#if 0
	int ret = 0;
	int i = 0;
	char buffer;

	for( i = 0 ; i < 5 ; i++ ){
		buffer = 0xFF;
		ret = it7258_i2c_read(&buffer, 1, QUERY_BUF);
		if(ret <=0)
			msleep(100);
		if( buffer != 0xFF )
			break;
	}
	msleep(100);
#endif
	is_from_resume = false;
	disable_irq_wake(this_client->irq);
}
#endif

bool fnFirmwareReinitialize(void)
{
	u8 pucBuffer[2];
	waitCommandDone();

	pucBuffer[0] = 0x6F;
	if( !it7258_i2c_write2( CMD_BUF, pucBuffer, 1) ){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}
	printk("OOO %s, %d\n", __FUNCTION__, __LINE__);
	return true;
}


static int CalibrationCapSensor(void)
{
	unsigned char pucCmd[20];
	int ret = 0;

	waitCommandDone();
	memset(pucCmd, 0x00, 20);
	pucCmd[0] = 0x13;

	ret = it7258_i2c_write2( CMD_BUF, pucCmd, 5);
	if (ret < 0) {
		printk("[%s:%d] 0x13_write_failed\n", __FUNCTION__, __LINE__);
		return ret;
	}
	mdelay(5000);

	waitCommandDone();
	memset(pucCmd, 0xFF, 20);
	ret = it7258_i2c_write2(CMD_RSP_BUF, pucCmd, 2);

	if( ret >= 0 && ( pucCmd[0] + pucCmd[1] ) == 0x00 ){
		pucCmd[0] = 0x0C;
		it7258_i2c_write2(CMD_BUF, pucCmd, 1);
		ret = 0;
		printk("!! IT7258 fnFirmwareReinitialize success \r\n");
	}else{
		printk("!! IT7258 fnFirmwareReinitialize fail \r\n");
	}

	mdelay(1000);

	if (ret < 0)
		return -1;
	else
		return 0;
}
bool fnEnterFirmwareUpgradeMode(void)
{
	unsigned char pucBuffer[MAX_BUFFER_SIZE];
	char wCommandResponse[2] = { 0xFF, 0xFF };

	waitCommandDone();

	pucBuffer[0] = 0x60;
	pucBuffer[1] = 0x00;
	pucBuffer[2] = 'I';
	pucBuffer[3] = 'T';
	pucBuffer[4] = '7';
	pucBuffer[5] = '2';
	pucBuffer[6] = '6';
	pucBuffer[7] = '0';
	pucBuffer[8] = 0x55;
	pucBuffer[9] = 0xAA;

	printk("before 1 tpd_i2c_write_2 %s, %d\n", __FUNCTION__, __LINE__);
#ifdef HAS_8_BYTES_LIMIT
	if( !it7258_i2c_write2( CMD_BUF, pucBuffer, 6) ){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}
#else
	if( !it7258_i2c_write2( CMD_BUF, pucBuffer, 10) ){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}
#endif
	printk("after 1 tpd_i2c_write_2 %s, %d\n", __FUNCTION__, __LINE__);
	waitCommandDone();
	if(!it7258_i2c_read( wCommandResponse, 2, CMD_RSP_BUF ) ){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}

	if( wCommandResponse[0] | wCommandResponse[1] ){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}
	printk("OOO %s, %d\n", __FUNCTION__, __LINE__);
	return true;
}

bool fnExitFirmwareUpgradeMode(void)
{
	char pucBuffer[MAX_BUFFER_SIZE];
	char wCommandResponse[2] = { 0xFF, 0xFF };

	waitCommandDone();

	pucBuffer[0] = 0x60;
	pucBuffer[1] = 0x80;
	pucBuffer[2] = 'I';
	pucBuffer[3] = 'T';
	pucBuffer[4] = '7';
	pucBuffer[5] = '2';
	pucBuffer[6] = '6';
	pucBuffer[7] = '0';
	pucBuffer[8] = 0xAA;
	pucBuffer[9] = 0x55;

#ifdef HAS_8_BYTES_LIMIT
	if( !it7258_i2c_write2( CMD_BUF, pucBuffer, 6) ){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}
#else
	if( !it7258_i2c_write2( CMD_BUF, pucBuffer, 10) ){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}
#endif

	waitCommandDone();

	if(!it7258_i2c_read( wCommandResponse, 2, CMD_RSP_BUF )){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}

	if(wCommandResponse[0] | wCommandResponse[1]){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}
	printk("OOO %s, %d\n", __FUNCTION__, __LINE__);
	return true;
}

bool fnSetStartOffset(unsigned short wOffset)
{
	u8 pucBuffer[MAX_BUFFER_SIZE];
	char wCommandResponse[2] = { 0xFF, 0xFF };

	waitCommandDone();

	pucBuffer[0] = 0x61;
	pucBuffer[1] = 0;
	pucBuffer[2] = ( wOffset & 0x00FF );
	pucBuffer[3] = (( wOffset & 0xFF00 ) >> 8 );

	if( !it7258_i2c_write2( CMD_BUF, pucBuffer, 4) ){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}

	waitCommandDone();

	if( !it7258_i2c_read( wCommandResponse, 2, CMD_RSP_BUF )  ){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}

	if(wCommandResponse[0] | wCommandResponse[1]){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}

	return true;
}

static bool fnWriteAndCompareFlash(unsigned int nLength, char pnBuffer[], unsigned short nStartOffset)
{
	unsigned int nIndex = 0;
	unsigned char buffer[130] = {0};
	unsigned char bufWrite[130] = {0};
	unsigned char bufRead[130] = {0};
	unsigned char nReadLength;
	int retryCount;
	int i;

	nReadLength = 128;

	while ( nIndex < nLength ) {
		retryCount = 0;
		do {
			if(fnSetStartOffset(nStartOffset + nIndex))
				printk("fnSetStartOffset success %s, %d\n", __FUNCTION__, __LINE__);

			buffer[0] = 0x62;
			buffer[1] = 128;
			for (i = 0; i < 128; i++) {
				bufWrite[i] = buffer[2 + i] = pnBuffer[nIndex + i];
			}

			if(!it7258_i2c_write2( CMD_BUF, buffer, 130)) {
				printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
				return false;
			}
			// Read from Flash
			buffer[0] = 0x63;
			buffer[1] = nReadLength;

			fnSetStartOffset(nStartOffset + nIndex);
			if(!it7258_i2c_write2( CMD_BUF, buffer, 2)){
				printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
				return false;
			}
			waitCommandDone();
			it7258_i2c_read( bufRead, nReadLength, CMD_RSP_BUF);

			// Compare
			for (i = 0; i < 128; i++) {
				if (bufRead[i] != bufWrite[i]) {
					break;
				}
			}
			if (i == 128) break;
		}while ( retryCount++ < 4 );

		if ( retryCount == 4 && i != 128 ){
			printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
			return false;
		}
		nIndex += 128;
	}
	return true;
}

bool fnFirmwareDownload(unsigned int unFirmwareLength, u8* pFirmware, unsigned int unConfigLength, u8* pConfig)
{
	if((unFirmwareLength == 0 || pFirmware == NULL) && (unConfigLength == 0 || pConfig == NULL)){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}
	printk("fnFirmwareDownload %s, %d\n", __FUNCTION__, __LINE__);

	if(!fnEnterFirmwareUpgradeMode()){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}
	printk("fnEnterFirmwareUpgradeMode %s, %d\n", __FUNCTION__, __LINE__);

	if(unFirmwareLength != 0 && pFirmware != NULL){
		// Download firmware
		if(!fnWriteAndCompareFlash(unFirmwareLength, pFirmware, 0)){
			printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
			return false;
		}
	}
	printk("fnWriteAndCompareFlash Fireware %s, %d\n", __FUNCTION__, __LINE__);

	if(unConfigLength != 0 && pConfig != NULL){
		// Download configuration
		unsigned short wFlashSize = 0x8000;
		if( !fnWriteAndCompareFlash(unConfigLength, pConfig, wFlashSize - (unsigned short)unConfigLength)){
			printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
			return false;
		}
	}
	printk("fnWriteAndCompareFlash Config %s, %d\n", __FUNCTION__, __LINE__);

	if(!fnExitFirmwareUpgradeMode()){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}
	printk("fnExitFirmwareUpgradeMode %s, %d\n", __FUNCTION__, __LINE__);

	if(!fnFirmwareReinitialize()){
		printk("XXX %s, %d\n", __FUNCTION__, __LINE__);
		return false;
	}
	printk("OOO %s, %d\n", __FUNCTION__, __LINE__);
	return true;
}

int Upgrade_FW_CFG(void)
{
	unsigned int fw_size = 0;
	unsigned int config_size = 0;
	struct file *fw_fd = NULL;
	struct file *config_fd = NULL;
	mm_segment_t fs={0};
	u8 *fw_buf = kzalloc(0x8000, GFP_KERNEL);
	u8 *config_buf = kzalloc(0x500, GFP_KERNEL);
	if ( fw_buf  == NULL || config_buf == NULL  ){
		printk("kzalloc failed\n");
	}
	printk("Execute Upgrade_FW_CFG()\n");

	fs = get_fs();
	set_fs(get_ds());

	fw_fd = filp_open("/data/upgrade/it7258_FW.bin", O_RDONLY, 0);
	if (!fw_fd){
		printk("open /data/upgrade/it7258_FW failed \n");
		fw_fd = filp_open("/sdcard/upgrade/it7258_FW.bin", O_RDONLY, 0);
		if (!fw_fd){
			printk("open /sdcard/upgrade/it7258_FW failed \n");
			filp_close(fw_fd,NULL);
			return 1;
		}
	}

	fw_size = fw_fd->f_op->read(fw_fd, fw_buf, 0x8000, &fw_fd->f_pos);
	printk("fw_ver : %d,%d,%d,%d\n",fw_buf[8], fw_buf[9], fw_buf[10], fw_buf[11]);
	printk("--------------------- fw_size = %x\n", fw_size);

	config_fd = filp_open("/data/upgrade/it7258_Config.bin", O_RDONLY, 0);
	if(!config_fd){
		printk("open /data/upgrade/it7258_Config failed \n");
		config_fd = filp_open("/sdcard/upgrade/it7258_Config.bin", O_RDONLY, 0);
		if (!config_fd){
			printk("open /sdcard/upgrade/it7258_Config failed \n");
			filp_close(config_fd,NULL);
			return 1;
		}
	}

	config_size = config_fd->f_op->read(config_fd, config_buf, 0x500, &config_fd->f_pos);
	printk("cfg_ver : %d,%d,%d,%d\n",config_buf[config_size-8], config_buf[config_size-7], config_buf[config_size-6], config_buf[config_size-5]);
	printk("--------------------- config_size = %x\n", config_size);

	set_fs(fs);
	filp_close(fw_fd,NULL);
	filp_close(config_fd,NULL);
	if (fnFirmwareDownload(fw_size, fw_buf, config_size, config_buf) == false){
		//fail
		return 1;
	}else{
		//success
		return 0;
	}
}


ssize_t it7258_calibration_show_temp(char *buf)
{
	if( Calibration__success_flag < 0 ){
		return sprintf(buf, "## Calibration Fail [ %d ] ##\n", Calibration__success_flag );
	}else{
		return sprintf(buf, "## Calibration Success [ %d ] ##\n", Calibration__success_flag );
	}
}

ssize_t it7258_upgrade_show_temp(char *buf)
{
	if( Upgrade__success_flag < 0 ){
		return sprintf(buf, "## Upgrade Fail [ %d ] ##\n", Upgrade__success_flag );
	}else{
		return sprintf(buf, "## Upgrade Success [ %d ] ##\n", Upgrade__success_flag );
	}
}

ssize_t it7258_calibration_store_temp(const char *buf)
{
	if(!CalibrationCapSensor()) {
		printk("it7258_calibration_OK\n\n");
		Calibration__success_flag = 1;
		return 0;
	} else {
		printk("it7258_calibration_failed\n");
		Calibration__success_flag = -1;
		return -1;
	}
}

ssize_t it7258_upgrade_store_temp(void)
{
	if(!Upgrade_FW_CFG()) {
		printk("it7258_upgrade_OK\n\n");
		Upgrade__success_flag = 1;
		return 0;
	} else {
		printk("it7258_upgrade_failed\n");
		Upgrade__success_flag = -1;
		return -1;
	}
}

static ssize_t it7258_calibration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("%s():\n", __func__);
	return it7258_calibration_show_temp(buf);
}

static ssize_t it7258_upgrade_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("%s():\n", __func__);
	return it7258_upgrade_show_temp(buf);
}

static ssize_t it7258_calibration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	printk("%s():\n", __func__);
	it7258_calibration_store_temp(buf);
	return count;
}

static ssize_t it7258_upgrade_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	printk("%s():\n", __func__);
	it7258_upgrade_store_temp();
	return count;
}

static DEVICE_ATTR(calibration, 0666, it7258_calibration_show, it7258_calibration_store);
static DEVICE_ATTR(upgrade, 0666, it7258_upgrade_show, it7258_upgrade_store);


static int it7258_check_version(void) {
	char buffer[9];
	int ret = -1;
	int i = 0;

	do {
		buffer[0] = 0xFF;
		ret = it7258_i2c_read( &buffer[0], 1, QUERY_BUF);
		if (5==i++) return -1;
	} while( buffer[0] & 0x01 );

	buffer[0] = CMD_BUF;
	buffer[1] = 0x1;
	buffer[2] = 0x0;
	ret = it7258_i2c_write(buffer, 3);
	if(ret != 3){
		IT7258_DBG("i2c write communcate error in getting FW version : 0x%x\n", ret);
	}
	msleep(10);
	i = 0;
	do{
		buffer[0] = 0xFF;
		ret = it7258_i2c_read(&buffer[0], 1, QUERY_BUF);
		if (5==i++) return -1;
	}while( buffer[0] & 0x01 );

	ret = it7258_i2c_read(&buffer[0], 9, CMD_RSP_BUF);
	if (ret != 0x9){
		IT7258_DBG("i2c read communcate error in getting FW version : 0x%x\n", ret);
	}else{
		IT7258_DBG("ITE7258 Touch Panel Firmware Version %x %x %x %x %x %x %x %x %x\n",
				buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8]);
	}
	return ret;
}

static int it7258_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct it7258_ts_data *it7258_ts;
	struct input_dev *input_dev;
	struct it7258_i2c_platform_data *pdata;
	int err = 0;
	unsigned char Wrbuf[2] = { CMD_BUF, 0x07 };
	char buffer[9];
	int ret = -1;
	int i = 0;

	printk("\n%s\n", __func__);

	this_client = client;

	pdata = client->dev.platform_data;

	max_x = MAX_X_RESOLUTION;
	max_y = MAX_Y_RESOLUTION;
	swap = false;
	reverse = false;

	if (pdata) {
		init_irq = pdata->gpio_irq;
		init_reset= pdata->reset_io;
		init_en= pdata->enable_io;
		swap = pdata->swap;
		reverse = pdata->reverse;
		max_x = (pdata->max_x < MAX_X_RESOLUTION)? MAX_X_RESOLUTION : pdata->max_x;
		max_y = (pdata->max_y < MAX_Y_RESOLUTION)? MAX_Y_RESOLUTION : pdata->max_y;
	}

	if (swap) {
		int tmp = max_x;
		max_x = max_y;
		max_y = tmp;
	}
	printk("I2C addr=%x,client->irq=%d,init_irq=%d,init_reset=%d\n", client->addr,client->irq,init_irq,init_reset);

#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
	it7258_2v8= regulator_get(NULL, "tp_v2v8");
	if (IS_ERR_OR_NULL(it7258_2v8)) {
		printk("Failed to initialize msg21xxa regulator..\n");
		return -ENODEV;
	}
	err = regulator_enable(it7258_2v8);
	if (err) {
		printk("Failed to enabling msg21xxa regulator\n");
		regulator_put(it7258_2v8);
		return err;
	}
#endif

	it7258_ts_reset();
	ret = it7258_check_version();
	if (ret == -1)
	{
		printk("qmqiu exit_check_functionality_failed\n");
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	it7258_ts = kzalloc(sizeof(*it7258_ts), GFP_KERNEL);
	if (!it7258_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	i2c_set_clientdata(client, it7258_ts);
	INIT_WORK(&it7258_ts->event_work, it7258_ts_irq_work);

	it7258_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!it7258_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	//gpio_request(init_irq, "GPIO_TP_INT");
	//gpio_direction_input(init_irq);
	err = gpio_request_one(init_irq,GPIOF_IN, "GPIO_TP_INT");
	if (err)
		printk("%s: Could not get GPIO_TP_INT\n", __func__);
	client->irq = gpio_to_irq(init_irq);
	if(client->irq){
		IT7258_DBG("%s: %s IRQ number is %d", __func__, client->name, client->irq);
		err = request_irq(client->irq, it7258_ts_interrupt, IRQF_TRIGGER_LOW, client->name, it7258_ts); //??? not sure

		if (err < 0) {
			printk("%s: it7258_probe: request irq failed\n", __func__);
			goto exit_irq_request_failed;
		}
	}
	disable_irq(client->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		printk("%s: failed to allocate input device\n", __func__);
		goto exit_input_dev_alloc_failed;
	}

	it7258_ts->input_dev = input_dev;

#ifdef MOUSE_POINTER
	__set_bit(EV_REL, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);

	__set_bit(REL_X, input_dev->relbit);
	__set_bit(REL_Y, input_dev->relbit);
	__set_bit(BTN_LEFT, input_dev->keybit);
	__set_bit(KEY_LEFT, input_dev->keybit);
	__set_bit(KEY_RIGHT, input_dev->keybit);
	__set_bit(KEY_ENTER, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);
	__set_bit(KEY_MENU, input_dev->keybit);
#else
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);

	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	// FIXME: We should use platform data to set slots & abs resolutions
	input_mt_init_slots(input_dev, 2);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
#endif /*MOUSE_POINTER*/

	input_dev->name		= IT7258_TS_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"it7258_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	it7258_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	it7258_ts->early_suspend.suspend = it7258_ts_suspend;
	it7258_ts->early_suspend.resume	= it7258_ts_resume;
	register_early_suspend(&it7258_ts->early_suspend);
#endif
//???? enable tp gesture
	enable_irq(client->irq);

	it7258_class = class_create(THIS_MODULE, "tp-it7258");
	if (IS_ERR (it7258_class))
		pr_err("Failed to create class(it7258)!\n");

	it7258_cmd_dev = device_create(it7258_class, NULL, 0, NULL, "device");
	if (IS_ERR (it7258_cmd_dev))
		pr_err("Failed to create device(it7258_cmd_dev)!\n");

	//upgrade
	if (device_create_file(it7258_cmd_dev, &dev_attr_upgrade) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_upgrade.attr.name);

	//calibrate
	if (device_create_file(it7258_cmd_dev, &dev_attr_calibration) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_calibration.attr.name);

	dev_set_drvdata(it7258_cmd_dev, NULL);

	msleep(100);
	it7258_i2c_write(Wrbuf, 2);   // Clean Queue {0x20, 0x07}
	do{
		it7258_i2c_read(buffer, 1, QUERY_BUF);
		if (5==i++) 
			goto exit_input_register_device_failed;
	}while( buffer[0] & 0x01 );

	it7258_i2c_read(buffer, 2, CMD_RSP_BUF);
	IT7258_DBG("DDD_____ CMD_RSP_BUF : %X, %X\n", buffer[0], buffer[1]);  // add FAE   End

	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, it7258_ts);
exit_irq_request_failed:
	cancel_work_sync(&it7258_ts->event_work);
	destroy_workqueue(it7258_ts->ts_workqueue);
exit_create_singlethread:
	IT7258_DBG("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(it7258_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	gpio_free(init_irq);
	return err;
}


static int __devexit it7258_ts_remove(struct i2c_client *client)
{

	struct it7258_ts_data *it7258_ts = i2c_get_clientdata(client);

	IT7258_DBG("==it7258_ts_remove=\n");

	if (client->irq > 0)
		free_irq(client->irq, it7258_ts);
	if (gpio_is_valid(init_irq))
		gpio_free(init_irq);
	gpio_free(init_reset);
	input_unregister_device(it7258_ts->input_dev);
	kfree(it7258_ts);
	cancel_work_sync(&it7258_ts->event_work);
	destroy_workqueue(it7258_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);

#if defined(CONFIG_REGULATOR_GPIO_SWITCH)
	if(!IS_ERR_OR_NULL(it7258_2v8)){
		regulator_disable(it7258_2v8);
		regulator_put(it7258_2v8);
	}
#endif
	return 0;
}

static const struct i2c_device_id it7258_ts_id[] = {
	{ IT7258_TS_NAME, 0 },{ }
};

#ifndef CONFIG_HAS_EARLYSUSPEND
static int it7258_suspend(struct i2c_client *client, pm_message_t mesg)
{
// disable 2V8 will make touch pad not function during suspend
#if 0 && defined(CONFIG_REGULATOR_GPIO_SWITCH)
	int ret;
	if(it7258_2v8 != NULL){
		ret = regulator_disable(it7258_2v8);
		if (ret < 0)
			printk(KERN_ERR "Failed to turn off msg21xxa 2.8v regulator..\n");
	}
#endif
	return 0;
}

static int it7258_resume(struct i2c_client *client)
{

	int ret = 0;
	int i = 0;
	char buffer;

	is_from_resume = false;
#if 0 && defined(CONFIG_REGULATOR_GPIO_SWITCH)
	if(it7258_2v8 != NULL){
		ret = regulator_enable(it7258_2v8);
		if (ret < 0){
			printk("Failed to enable msg21xxa 2.8v regulator\n");
			return ret;
		}
	}
#endif
	for( i = 0 ; i < 5 ; i++ ){
		buffer = 0xFF;
		ret = it7258_i2c_read(&buffer, 1, QUERY_BUF);

		if( buffer != 0xFF )
			break;
	}
	msleep(100);

	disable_irq_wake(this_client->irq);

	return 0;
}
#endif

MODULE_DEVICE_TABLE(i2c, it7258_ts_id);

static struct i2c_driver it7258_ts_driver = {
	.probe		= it7258_ts_probe,
	.remove		= __devexit_p(it7258_ts_remove),
	.id_table	= it7258_ts_id,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend    = it7258_suspend,
	.resume     = it7258_resume,
#endif
	.driver	= {
		.name	= "IT7258",
		.owner	= THIS_MODULE,
	},
};

static int __init it7258_init_module(void)
{
	printk("%s\n", __func__);

	return i2c_add_driver(&it7258_ts_driver);
}

static void __exit it7258_exit_module(void)
{
	IT7258_DBG("%s\n", __func__);
	i2c_unregister_device(this_client);
	i2c_del_driver(&it7258_ts_driver);
}

module_init(it7258_init_module);
module_exit(it7258_exit_module);

MODULE_LICENSE("GPL");
