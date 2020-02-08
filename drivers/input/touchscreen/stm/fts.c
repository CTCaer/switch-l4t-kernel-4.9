/**
  * fts.c
  *
  * FTS Capacitive touch screen controller (FingerTipS)
  *
  * Copyright (C) 2016, STMicroelectronics Limited.
  * Authors: AMG(Analog Mems Group)
  *
  *		marco.cali@st.com
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  *
  * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
  * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
  * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
  * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM
  * THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
  */


/*!
  * \file fts.c
  * \brief It is the main file which contains all the most important functions
  * generally used by a device driver the driver
  */
#include <linux/device.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spi.h>
#include <linux/completion.h>
#include <linux/device.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/notifier.h>
#include <linux/msm_drm_notify.h>

#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif


#include "fts.h"
#include "fts_lib/ftsCompensation.h"
#include "fts_lib/ftsCore.h"
#include "fts_lib/ftsIO.h"
#include "fts_lib/ftsError.h"
#include "fts_lib/ftsFlash.h"
#include "fts_lib/ftsFrame.h"
#include "fts_lib/ftsGesture.h"
#include "fts_lib/ftsTest.h"
#include "fts_lib/ftsTime.h"
#include "fts_lib/ftsTool.h"



/**
  * Event handler installer helpers
  */
#define event_id(_e)		(EVT_ID_##_e >> 4)
#define handler_name(_h)	fts_##_h##_event_handler

#define install_handler(_i, _evt, _hnd) \
	do { \
		_i->event_dispatch_table[event_id(_evt)] = handler_name(_hnd); \
	} while (0)


/* Use decimal-formatted raw data */
#define RAW_DATA_FORMAT_DEC

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif


extern SysInfo systemInfo;
extern TestToDo tests;
#ifdef GESTURE_MODE
extern struct mutex gestureMask_mutex;
#endif

char tag[8] = "[ FTS ]\0";
char fts_ts_phys[64];	/* /< buffer which store the input device name
			  *	assigned by the kernel */

static u32 typeOfComand[CMD_STR_LEN] = { 0 };	/* /< buffer used to store the
						  * command sent from the MP
						  * device file node */
static int numberParameters;	/* /< number of parameter passed through the MP
				  * device file node */
#ifdef USE_ONE_FILE_NODE
static int feature_feasibility = ERROR_OP_NOT_ALLOW;
#endif
#ifdef GESTURE_MODE
static u8 mask[GESTURE_MASK_SIZE + 2];
extern u16 gesture_coordinates_x[GESTURE_MAX_COORDS_PAIRS_REPORT];
extern u16 gesture_coordinates_y[GESTURE_MAX_COORDS_PAIRS_REPORT];
extern int gesture_coords_reported;
extern struct mutex gestureMask_mutex;
#endif

#ifdef PHONE_KEY
static u8 key_mask;	/* /< store the last update of the key mask
				  * published by the IC */
#endif

extern spinlock_t fts_int;

static void fts_interrupt_enable(struct fts_ts_info *info);
static int fts_init_sensing(struct fts_ts_info *info);
static int fts_mode_handler(struct fts_ts_info *info, int force);


static int fts_chip_initialization(struct fts_ts_info *info, int init_type);


/**
  * Release all the touches in the linux input subsystem
  * @param info pointer to fts_ts_info which contains info about device/hw setup
  */
void release_all_touches(struct fts_ts_info *info)
{
	unsigned int type = MT_TOOL_FINGER;
	int i;

	for (i = 0; i < TOUCH_ID_MAX; i++) {
#ifdef STYLUS_MODE
		if (test_bit(i, &info->stylus_id))
			type = MT_TOOL_PEN;
		else
			type = MT_TOOL_FINGER;
#endif
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, type, 0);
		input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, -1);
	}
	input_sync(info->input_dev);
	info->touch_id = 0;
#ifdef STYLUS_MODE
	info->stylus_id = 0;
#endif
}


/**
  * @defgroup file_nodes Driver File Nodes
  * Driver publish a series of file nodes used to provide several utilities
  * to the host and give him access to different API.
  * @{
  */

/**
  * @defgroup device_file_nodes Device File Nodes
  * @ingroup file_nodes
  * Device File Nodes \n
  * There are several file nodes that are associated to the device and which
  *  are designed to be used by the host to enable/disable features or trigger
  * some system specific actions \n
  * Usually their final path depend on the definition of device tree node of
  * the IC (e.g /sys/devices/soc.0/f9928000.i2c/i2c-6/6-0049)
  * @{
  */
/***************************************** FW UPGGRADE
 * ***************************************************/

/**
  * File node function to Update firmware from shell \n
  * echo path_to_fw X Y > fwupdate   perform a fw update \n
  * where: \n
  * path_to_fw = file name or path of the the FW to burn, if "NULL" the default
  * approach selected in the driver will be used\n
  * X = 0/1 to force the FW update whichever fw_version and config_id;
  * 0=perform a fw update only if the fw in the file is newer than the fw in the
  * chip \n
  * Y = 0/1 keep the initialization data; 0 = will erase the initialization data
  * from flash, 1 = will keep the initialization data
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent an error code (00000000 no
  * error) \n
  * } = end byte
  */
static ssize_t fts_fwupdate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret, mode[2];
	char path[100];
	struct fts_ts_info *info = dev_get_drvdata(dev);

	/* default(if not specified by user) set force = 0 and keep_cx to 1 */
	mode[0] = 0;
	mode[1] = 1;

	/* reading out firmware upgrade parameters */
	sscanf(buf, "%100s %d %d", path, &mode[0], &mode[1]);
	logError(1, "%s fts_fwupdate_store: mode = %s\n", tag, path);


	ret = flashProcedure(path, mode[0], mode[1]);

	info->fwupdate_stat = ret;

	if (ret < OK)
		logError(1, "%s  %s Unable to upgrade firmware! ERROR %08X\n",
			 tag, __func__, ret);
	return count;
}

static ssize_t fts_fwupdate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);

	/* fwupdate_stat: ERROR code Returned by flashProcedure. */
	return scnprintf(buf, PAGE_SIZE, "{ %08X }\n", info->fwupdate_stat);
}


/***************************************** UTILITIES
  * (current fw_ver/conf_id, active mode, file fw_ver/conf_id)
  ***************************************************/
/**
  * File node to show on terminal external release version in Little Endian \n
  * (first the less significant byte) \n
  * cat appid	show the external release version of the FW running in the IC
  */
static ssize_t fts_appid_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int error;
	char temp[100];

	error = scnprintf(buf,
			  PAGE_SIZE,
			  "%s\n",
			  printHex("ST-V",
				   systemInfo.u8_releaseInfo,
				   EXTERNAL_RELEASE_INFO_SIZE,
				   temp,
				   sizeof(temp)));

	return error;
}

/**
  * File node to show on terminal the mode that is active on the IC \n
  * cat mode_active		    to show the bitmask which indicate
  * the modes/features which are running on the IC in a specific instant of time
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1 = 1 byte in HEX format which represent the actual running scan mode
  * (@link scan_opt Scan Mode Options @endlink) \n
  * X2 = 1 byte in HEX format which represent the bitmask on which is running
  * the actual scan mode \n
  * X3X4 = 2 bytes in HEX format which represent a bitmask of the features that
  * are enabled at this moment (@link feat_opt Feature Selection Options
  * @endlink) \n
  * } = end byte
  * @see fts_mode_handler()
  */
static ssize_t fts_mode_active_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError(1, "%s Current mode active = %08X\n", tag, info->mode);
	return scnprintf(buf, PAGE_SIZE, "{ %08X }\n", info->mode);
}

/**
  * File node to show the fw_ver and config_id of the FW file
  * cat fw_file_test			show on the kernel log external release
  * of the FW stored in the fw file/header file
  */
static ssize_t fts_fw_test_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	Firmware fw;
	int ret;
	char temp[100] = { 0 };

	fw.data = NULL;
	ret = readFwFile(info->board->fw_name, &fw, 0);

	if (ret < OK)
		logError(1, "%s Error during reading FW file! ERROR %08X\n",
			 tag, ret);
	else
		logError(1, "%s %s, size = %d bytes\n", tag,
			 printHex("EXT Release = ",
				  systemInfo.u8_releaseInfo,
				  EXTERNAL_RELEASE_INFO_SIZE,
				  temp,
				  sizeof(temp)),
			 fw.data_size);
	kfree(fw.data);
	return 0;
}


#if 0
/**
  * File node to obtain and show strength frame
  * cat strength_frame			to obtain strength data \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent an error code (00000000 no
  *error) \n
  * **** if error code is all 0s **** \n
  * FF = 1 byte in HEX format number of rows \n
  * SS = 1 byte in HEX format number of columns \n
  * N1, ... = the decimal value of each node separated by a coma \n
  * ********************************* \n
  * } = end byte
  */
static ssize_t fts_strength_frame_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	MutualSenseFrame frame;
	int res, count, j, size = (6 * 2) + 1, index = 0;
	char *all_strbuff = NULL;
	/* char buff[CMD_STR_LEN] = {0}; */
	/* struct i2c_client *client = to_i2c_client(dev); */
	struct fts_ts_info *info = dev_get_drvdata(dev);

	frame.node_data = NULL;

	res = fts_disableInterrupt();
	if (res < OK)
		goto END;

	res = senseOn();
	if (res < OK) {
		logError(1, "%s %s: could not start scanning! ERROR %08X\n",
			 tag, __func__, res);
		goto END;
	}
	mdelay(WAIT_FOR_FRESH_FRAMES);
	res = senseOff();
	if (res < OK) {
		logError(1, "%s %s: could not finish scanning! ERROR %08X\n",
			 tag, __func__, res);
		goto END;
	}

	mdelay(WAIT_AFTER_SENSEOFF);
	flushFIFO();

	res = getMSFrame3(MS_STRENGTH, &frame);
	if (res < OK) {
		logError(1, "%s %s: could not get the frame! ERROR %08X\n",
			 tag, __func__, res);
		goto END;
	} else {
		size += (res * 6);
		logError(0, "%s The frame size is %d words\n", tag, res);
		res = OK;
		print_frame_short("MS Strength frame =", array1dTo2d_short(
					  frame.node_data, frame.node_data_size,
					  frame.header.sense_node),
				  frame.header.force_node,
				  frame.header.sense_node);
	}

END:
	flushFIFO();
	release_all_touches(info);
	fts_mode_handler(info, 1);

	all_strbuff = (char *)kzalloc(size * sizeof(char), GFP_KERNEL);

	if (all_strbuff != NULL) {
		snprintf(&all_strbuff[index], 11, "{ %08X", res);

		index += 10;

		if (res >= OK) {
			snprintf(&all_strbuff[index], 3, "%02X",
				 (u8)frame.header.force_node);
			index += 2;
			snprintf(&all_strbuff[index], 3, "%02X",
				 (u8)frame.header.sense_node);

			index += 2;

			for (j = 0; j < frame.node_data_size; j++) {
				snprintf(&all_strbuff[index], 10, "%d,%n",
					 frame.node_data[j], &count);
				index += count;
			}

			kfree(frame.node_data);
		}

		snprintf(&all_strbuff[index], 3, " }");
		index += 2;

		count = snprintf(buf, TSP_BUF_SIZE, "%s\n", all_strbuff);
		kfree(all_strbuff);
	} else
		logError(1,
			 "%s %s: Unable to allocate all_strbuff! ERROR %08X\n",
			 tag,
			 ERROR_ALLOC);

	fts_enableInterrupt();
	return count;
}
#endif

/***************************************** FEATURES
  ***************************************************/

/* TODO: edit this function according to the features policy to allow during
  * the screen on/off, following is shown an example but check always with ST
  * for more details */
/**
  * Check if there is any conflict in enable/disable a particular feature
  * considering the features already enabled and running
  * @param info pointer to fts_ts_info which contains info about the device
  * and its hw setup
  * @param feature code of the feature that want to be tested
  * @return OK if is possible to enable/disable feature, ERROR_OP_NOT_ALLOW
  * in case of any other conflict
  */
int check_feature_feasibility(struct fts_ts_info *info, unsigned int feature)
{
	int res = OK;

/* Example based on the status of the screen and on the feature
  * that is trying to enable */
	/*res=ERROR_OP_NOT_ALLOW;
	  * if(info->resume_bit ==0){
	  *      switch(feature){
	  #ifdef GESTURE_MODE
	  *              case FEAT_SEL_GESTURE:
	  *                      res = OK;
	  *              break;
	  #endif
	  *              default:
	  *                      logError(1,"%s %s: Feature not allowed in this
	  * operating mode! ERROR %08X\n",
	  *				tag,__func__,res);
	  *              break;
	  *
	  *      }
	  * }else{
	  *      switch(feature){
	  #ifdef GESTURE_MODE
	  *              case FEAT_SEL_GESTURE:
	  #endif
	  *              case FEAT__SEL_GLOVE: // glove mode can only activate
	  *during sense on
	  *                      res = OK;
	  *              break;
	  *
	  *              default:
	  *                      logError(1,"%s %s: Feature not allowed in this
	  * operating mode! ERROR %08X\n",tag,__func__,res);
	  *              break;
	  *
	  *      }
	  * }*/


/* Example based only on the feature that is going to be activated */
	switch (feature) {
	case FEAT_SEL_GESTURE:
		if (info->cover_enabled == 1) {
			res = ERROR_OP_NOT_ALLOW;
			logError(1,
				 "%s %s: Feature not allowed when in Cover mode! ERROR %08X\n",
				 tag, __func__, res);
			/* for example here can be placed a code for disabling
			  * the cover mode when gesture is activated */
		}
		break;

	case FEAT_SEL_GLOVE:
		if (info->gesture_enabled == 1) {
			res = ERROR_OP_NOT_ALLOW;
			logError(1,
				 "%s %s: Feature not allowed when Gestures enabled! ERROR %08X\n",
				 tag, __func__, res);
			/* for example here can be placed a code for disabling
			  * the gesture mode when cover is activated
			  * (that means that cover mode has
			  * an higher priority on gesture mode) */
		}
		break;

	default:
		logError(1, "%s %s: Feature Allowed!\n", tag, __func__);
	}

	return res;
}

#ifdef USE_ONE_FILE_NODE
/**
  * File node to enable some feature
  * echo XX 00/01 > feature_enable		to enable/disable XX
  * (possible values @link feat_opt Feature Selection Options @endlink) feature
  * cat feature_enable		to show the result of enabling/disabling process
  * echo 01/00 > feature_enable; cat feature_enable		to perform
  * both actions stated before in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent an error code (00000000 =
  * no error) \n
  * } = end byte
  */
static ssize_t fts_feature_enable_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct fts_ts_info *info = dev_get_drvdata(dev);
	char *p = (char *)buf;
	unsigned int temp;
	int res = OK;

	if ((count - 2 + 1) / 3 != 1)
		logError(1,
			 "%s fts_feature_enable: Number of parameter wrong! %d > %d\n",
			 tag, (count - 2 + 1) / 3, 1);
	else {
		sscanf(p, "%02X ", &temp);
		p += 3;
		res = check_feature_feasibility(info, temp);
		if (res >= OK) {
			switch (temp) {
	#ifdef GESTURE_MODE
			case FEAT_SEL_GESTURE:
				sscanf(p, "%02X ", &info->gesture_enabled);
				logError(1,
					 "%s fts_feature_enable: Gesture Enabled = %d\n",
					 tag, info->gesture_enabled);
				break;
	#endif

	#ifdef GLOVE_MODE
			case FEAT_SEL_GLOVE:
				sscanf(p, "%02X ", &info->glove_enabled);
				logError(1,
					 "%s fts_feature_enable: Glove Enabled = %d\n",
					 tag, info->glove_enabled);

				break;
	#endif

	#ifdef STYLUS_MODE
			case FEAT_SEL_STYLUS:
				sscanf(p, "%02X ", &info->stylus_enabled);
				logError(1,
					 "%s fts_feature_enable: Stylus Enabled = %d\n",
					 tag, info->stylus_enabled);

				break;
	#endif

	#ifdef COVER_MODE
			case FEAT_SEL_COVER:
				sscanf(p, "%02X ", &info->cover_enabled);
				logError(1,
					 "%s fts_feature_enable: Cover Enabled = %d\n",
					 tag, info->cover_enabled);

				break;
	#endif

	#ifdef CHARGER_MODE
			case FEAT_SEL_CHARGER:
				sscanf(p, "%02X ", &info->charger_enabled);
				logError(1,
					 "%s fts_feature_enable: Charger Enabled = %d\n",
					 tag, info->charger_enabled);

				break;
	#endif

	#ifdef GRIP_MODE
			case FEAT_SEL_GRIP:
				sscanf(p, "%02X ", &info->grip_enabled);
				logError(1,
					 "%s fts_feature_enable: Grip Enabled = %d\n",
					 tag, info->grip_enabled);

				break;
	#endif



			default:
				logError(1,
					 "%s fts_feature_enable: Feature %08X not valid! ERROR %08X\n",
					 tag, temp, ERROR_OP_NOT_ALLOW);
				res = ERROR_OP_NOT_ALLOW;
			}
			feature_feasibility = res;
		}
		if (feature_feasibility >= OK)
			feature_feasibility = fts_mode_handler(info, 1);
		else
			logError(1,
				 "%s %s: Call echo XX 00/01 > feature_enable with a correct feature value (XX)! ERROR %08X\n",
				 tag, __func__, res);
	}
	return count;
}



static ssize_t fts_feature_enable_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	int count = 0;

	if (feature_feasibility < OK)
		logError(1,
			 "%s %s: Call before echo XX 00/01 > feature_enable with a correct feature value (XX)! ERROR %08X\n",
			 tag, __func__, feature_feasibility);

	count += scnprintf(buf + count,
			   PAGE_SIZE - count, "{ %08X }\n",
			   feature_feasibility);

	feature_feasibility = ERROR_OP_NOT_ALLOW;
	return count;
}

#else


#ifdef GRIP_MODE
/**
  * File node to set the grip mode
  * echo 01/00 > grip_mode	to enable/disable glove mode \n
  * cat grip_mode		to show the status of the grip_enabled switch \n
  * echo 01/00 > grip_mode; cat grip_mode		to enable/disable grip
  *mode
  * and see the switch status in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent the value
  * info->grip_enabled (1 = enabled; 0= disabled) \n
  * } = end byte
  */
static ssize_t fts_grip_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;

	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError(0, "%s %s: grip_enabled = %d\n", tag, __func__,
		 info->grip_enabled);

	count += scnprintf(buf + count,
			   PAGE_SIZE - count, "{ %08X }\n",
			   info->grip_enabled);

	return count;
}


static ssize_t fts_grip_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	char *p = (char *)buf;
	unsigned int temp;
	int res;
	struct fts_ts_info *info = dev_get_drvdata(dev);


	/* in case of a different elaboration of the input, just modify
	  * this initial part of the code according to customer needs */
	if ((count + 1) / 3 != 1)
		logError(1,
			 "%s %s: Number of bytes of parameter wrong! %d != %d byte\n",
			 tag, __func__, (count + 1) / 3, 1);
	else {
		sscanf(p, "%02X ", &temp);
		p += 3;

/* standard code that should be always used when a feature is enabled! */
/* first step : check if the wanted feature can be enabled */
/* second step: call fts_mode_handler to actually enable it */
/* NOTE: Disabling a feature is always allowed by default */
		res = check_feature_feasibility(info, FEAT_SEL_GRIP);
		if (res >= OK || temp == FEAT_DISABLE) {
			info->grip_enabled = temp;
			res = fts_mode_handler(info, 1);
			if (res < OK)
				logError(1,
					 "%s %s: Error during fts_mode_handler! ERROR %08X\n",
					 tag, __func__, res);
		}
	}

	return count;
}
#endif

#ifdef CHARGER_MODE
/**
  * File node to set the glove mode
  * echo XX/00 > charger_mode		to value >0 to enable
  * (possible values: @link charger_opt Charger Options @endlink),
  * 00 to disable charger mode \n
  * cat charger_mode	to show the status of the charger_enabled switch \n
  * echo 01/00 > charger_mode; cat charger_mode		to enable/disable
  * charger mode and see the switch status in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent the value
  * info->charger_enabled (>0 = enabled; 0= disabled) \n
  * } = end byte
  */
static ssize_t fts_charger_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError(0, "%s %s: charger_enabled = %d\n", tag, __func__,
		 info->charger_enabled);

	count += scnprintf(buf + count,
			   PAGE_SIZE - count, "{ %08X }\n",
			   info->charger_enabled);
	return count;
}


static ssize_t fts_charger_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	char *p = (char *)buf;
	unsigned int temp;
	int res;
	struct fts_ts_info *info = dev_get_drvdata(dev);


/* in case of a different elaboration of the input, just modify this
  * initial part of the code according to customer needs */
	if ((count + 1) / 3 != 1)
		logError(1,
			 "%s %s: Number of bytes of parameter wrong! %d != %d byte\n",
			 tag, __func__, (count + 1) / 3, 1);
	else {
		sscanf(p, "%02X ", &temp);
		p += 3;

/** standard code that should be always used when a feature is enabled!
  * first step : check if the wanted feature can be enabled
  * second step: call fts_mode_handler to actually enable it
  * NOTE: Disabling a feature is always allowed by default
  */
		res = check_feature_feasibility(info, FEAT_SEL_CHARGER);
		if (res >= OK || temp == FEAT_DISABLE) {
			info->charger_enabled = temp;
			res = fts_mode_handler(info, 1);
			if (res < OK)
				logError(1,
					 "%s %s: Error during fts_mode_handler! ERROR %08X\n",
					 tag, __func__, res);
		}
	}

	return count;
}
#endif

#ifdef GLOVE_MODE
/**
  * File node to set the glove mode
  * echo 01/00 > glove_mode	to enable/disable glove mode \n
  * cat glove_mode	to show the status of the glove_enabled switch \n
  * echo 01/00 > glove_mode; cat glove_mode	to enable/disable glove mode and
  *  see the switch status in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent the of value
  * info->glove_enabled (1 = enabled; 0= disabled) \n
  * } = end byte
  */
static ssize_t fts_glove_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError(0, "%s %s: glove_enabled = %d\n", tag, __func__,
		 info->glove_enabled);

	count += scnprintf(buf + count,
			   PAGE_SIZE - count, "{ %08X }\n",
			   info->glove_enabled);

	return count;
}


static ssize_t fts_glove_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	char *p = (char *)buf;
	unsigned int temp;
	int res;
	struct fts_ts_info *info = dev_get_drvdata(dev);


/* in case of a different elaboration of the input, just modify this
  * initial part of the code according to customer needs */
	if ((count + 1) / 3 != 1)
		logError(1,
			 "%s %s: Number of bytes of parameter wrong! %d != %d byte\n",
			 tag, __func__, (count + 1) / 3, 1);
	else {
		sscanf(p, "%02X ", &temp);
		p += 3;

/* standard code that should be always used when a feature is enabled! */
/* first step : check if the wanted feature can be enabled */
/* second step: call fts_mode_handler to actually enable it */
/* NOTE: Disabling a feature is always allowed by default */
		res = check_feature_feasibility(info, FEAT_SEL_GLOVE);
		if (res >= OK || temp == FEAT_DISABLE) {
			info->glove_enabled = temp;
			res = fts_mode_handler(info, 1);
			if (res < OK)
				logError(1,
					 "%s %s: Error during fts_mode_handler! ERROR %08X\n",
					 tag, __func__, res);
		}
	}

	return count;
}
#endif


#ifdef COVER_MODE
/* echo 01/00 > cover_mode     to enable/disable cover mode */
/* cat cover_mode	to show the status of the cover_enabled switch
 * (example output in the terminal = "AA00000001BB" if the switch is enabled) */
/* echo 01/00 > cover_mode; cat cover_mode	to enable/disable cover mode and
  * see the switch status in just one call */
/* NOTE: the cover can be handled also using a notifier, in this case the body
  * of these functions should be copied in the notifier callback */
/**
  * File node to set the cover mode
  * echo 01/00 > cover_mode	to enable/disable cover mode \n
  * cat cover_mode	to show the status of the cover_enabled switch \n
  * echo 01/00 > cover_mode; cat cover_mode	to enable/disable cover mode
  * and see the switch status in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which is the value of info->cover_enabled
  * (1 = enabled; 0= disabled)\n
  * } = end byte \n
  * NOTE: \n
  * the cover can be handled also using a notifier, in this case the body of
  * these functions should be copied in the notifier callback
  */
static ssize_t fts_cover_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError(0, "%s %s: cover_enabled = %d\n", tag, __func__,
		 info->cover_enabled);

	count += scnprintf(buf + count,
			   PAGE_SIZE - count, "{ %08X }\n",
			   info->cover_enabled);

	return count;
}


static ssize_t fts_cover_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	char *p = (char *)buf;
	unsigned int temp;
	int res;
	struct fts_ts_info *info = dev_get_drvdata(dev);


/* in case of a different elaboration of the input, just modify this
  * initial part of the code according to customer needs */
	if ((count + 1) / 3 != 1)
		logError(1,
			 "%s %s: Number of bytes of parameter wrong! %d != %d byte\n",
			 tag, __func__, (count + 1) / 3, 1);
	else {
		sscanf(p, "%02X ", &temp);
		p += 3;

/* standard code that should be always used when a feature is enabled! */
/* first step : check if the wanted feature can be enabled */
/* second step: call fts_mode_handler to actually enable it */
/* NOTE: Disabling a feature is always allowed by default */
		res = check_feature_feasibility(info, FEAT_SEL_COVER);
		if (res >= OK || temp == FEAT_DISABLE) {
			info->cover_enabled = temp;
			res = fts_mode_handler(info, 1);
			if (res < OK)
				logError(1,
					 "%s %s: Error during fts_mode_handler! ERROR %08X\n",
					 tag, __func__, res);
		}
	}

	return count;
}
#endif

#ifdef STYLUS_MODE
/**
  * File node to enable the stylus report
  * echo 01/00 > stylus_mode		to enable/disable stylus mode \n
  * cat stylus_mode	to show the status of the stylus_enabled switch \n
  * echo 01/00 > stylus_mode; cat stylus_mode	to enable/disable stylus mode
  * and see the switch status in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which is the value of info->stylus_enabled
  * (1 = enabled; 0= disabled)\n
  * } = end byte
  */
static ssize_t fts_stylus_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError(0, "%s %s: stylus_enabled = %d\n", tag, __func__,
		 info->stylus_enabled);

	count += scnprintf(buf + count,
			   PAGE_SIZE - count, "{ %08X }\n",
			   info->stylus_enabled);

	return count;
}


static ssize_t fts_stylus_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	char *p = (char *)buf;
	unsigned int temp;
	struct fts_ts_info *info = dev_get_drvdata(dev);


/* in case of a different elaboration of the input, just modify this
  * initial part of the code according to customer needs */
	if ((count + 1) / 3 != 1)
		logError(1,
			 "%s %s: Number of bytes of parameter wrong! %d != %d byte\n",
			 tag, __func__, (count + 1) / 3, 1);
	else {
		sscanf(p, "%02X ", &temp);
		p += 3;


		info->stylus_enabled = temp;
	}

	return count;
}
#endif

#endif

/***************************************** GESTURES
  ***************************************************/
#ifdef GESTURE_MODE
#ifdef USE_GESTURE_MASK	/* if this define is used, a gesture bit mask
			  * is used as method to select the gestures to
			  * enable/disable */

/**
  * File node used by the host to set the gesture mask to enable or disable
  * echo EE X1 X2 ~~ > gesture_mask  set the gesture to disable/enable;
  * EE = 00(disable) or 01(enable) \n
  * X1 ~~  = gesture mask (example 06 00 ~~ 00 this gesture mask represents
  * the gestures with ID = 1 and 2) can be specified
  * from 1 to GESTURE_MASK_SIZE bytes, \n
  * if less than GESTURE_MASK_SIZE bytes are passed as arguments,
  * the omit bytes of the mask maintain the previous settings  \n
  * if one or more gestures is enabled the driver will automatically
  * enable the gesture mode, If all the gestures are disabled the driver
  * automatically will disable the gesture mode \n
  * cat gesture_mask   set inside the specified mask and return an error code
  * for the operation \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent an error code for enabling
  * the mask (00000000 = no error)\n
  * } = end byte \n\n
  * if USE_GESTURE_MASK is not define the usage of the function become: \n\n
  * echo EE X1 X2 ~~ > gesture_mask   set the gesture to disable/enable;
  * EE = 00(disable) or 01(enable) \n
  * X1 ~~ = gesture IDs (example 01 02 05 represent the gestures with ID = 1, 2
  * and 5)
  * there is no limit of the IDs passed as arguments, (@link gesture_opt Gesture
  * IDs @endlink) \n
  * if one or more gestures is enabled the driver will automatically enable
  * the gesture mode. If all the gestures are disabled the driver automatically
  * will disable the gesture mode. \n
  * cat gesture_mask     to show the status of the gesture enabled switch \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which is the value of info->gesture_enabled
  * (1 = enabled; 0= disabled)\n
  * } = end byte
  */
static ssize_t fts_gesture_mask_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0, res, temp;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	if (mask[0] == 0) {
		res = ERROR_OP_NOT_ALLOW;
		logError(1,
			 "%s %s: Call before echo enable/disable xx xx .... > gesture_mask with a correct number of parameters! ERROR %08X\n",
			 tag, __func__, res);
	} else {
		if (mask[1] == FEAT_ENABLE || mask[1] == FEAT_DISABLE)
			res = updateGestureMask(&mask[2], mask[0], mask[1]);
		else
			res = ERROR_OP_NOT_ALLOW;

		if (res < OK)
			logError(1, "%s fts_gesture_mask_store: ERROR %08X\n",
				 tag, res);
	}
	res |= check_feature_feasibility(info, FEAT_SEL_GESTURE);
	temp = isAnyGestureActive();
	if (res >= OK || temp == FEAT_DISABLE)
		info->gesture_enabled = temp;

	logError(1, "%s fts_gesture_mask_store: Gesture Enabled = %d\n", tag,
		 info->gesture_enabled);

	count += scnprintf(buf + count,
			   PAGE_SIZE - count, "{ %08X }\n", res);
	mask[0] = 0;
	return count;
}


static ssize_t fts_gesture_mask_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	char *p = (char *)buf;
	int n;
	unsigned int temp;

	if ((count + 1) / 3 > GESTURE_MASK_SIZE + 1) {
		logError(1,
			 "%s fts_gesture_mask_store: Number of bytes of parameter wrong! %d > (enable/disable + %d )\n",
			 tag, (count + 1) / 3, GESTURE_MASK_SIZE);
		mask[0] = 0;
	} else {
		mask[0] = ((count + 1) / 3) - 1;
		for (n = 1; n <= (count + 1) / 3; n++) {
			sscanf(p, "%02X ", &temp);
			p += 3;
			mask[n] = (u8)temp;
			logError(1, "%s mask[%d] = %02X\n", tag, n, mask[n]);
		}
	}

	return count;
}

#else	/* if this define is not used, to select the gestures to enable/disable
	  * are used the IDs of the gestures */
/* echo EE X1 X2 ... > gesture_mask     set the gesture to disable/enable;
  * EE = 00(disable) or 01(enable); X1 ... = gesture IDs
  * (example 01 02 05... represent the gestures with ID = 1, 2 and 5)
  * there is no limit of the parameters that can be passed,
  * of course the gesture IDs should be valid (all the valid IDs are listed in
  * ftsGesture.h) */
/* cat gesture_mask	enable/disable the given gestures, if one or more
  * gestures is enabled the driver will automatically enable the gesture mode.
  * If all the gestures are disabled the driver automatically will disable the
  * gesture mode.
  * At the end an error code will be printed
  *  (example output in the terminal = "AA00000000BB" if there are no errors) */
/* echo EE X1 X2 ... > gesture_mask; cat gesture_mask	perform in one command
  * both actions stated before */
/**
  * File node used by the host to set the gesture mask to enable or disable
  * echo EE X1 X2 ~~ > gesture_mask	set the gesture to disable/enable;
  * EE = 00(disable) or 01(enable) \n
  * X1 ~ = gesture IDs (example 01 02 05 represent the gestures with ID = 1, 2
  * and 5)
  * there is no limit of the IDs passed as arguments, (@link gesture_opt Gesture
  * IDs @endlink) \n
  * if one or more gestures is enabled the driver will automatically enable
  * the gesture mode, If all the gestures are disabled the driver automatically
  * will disable the gesture mode \n
  * cat gesture_mask	 to show the status of the gesture enabled switch \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which is the value of info->gesture_enabled
  * (1 = enabled; 0= disabled)\n
  * } = end byte
  */
static ssize_t fts_gesture_mask_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int count = 0;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	logError(0, "%s fts_gesture_mask_show: gesture_enabled = %d\n", tag,
		 info->gesture_enabled);

	count += scnprintf(buf + count,
			   PAGE_SIZE - count, "{ %08X }\n",
			   info->gesture_enabled);


	return count;
}


static ssize_t fts_gesture_mask_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	char *p = (char *)buf;
	int n;
	unsigned int temp;
	int res;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	if ((count + 1) / 3 < 2 || (count + 1) / 3 > GESTURE_MASK_SIZE + 1) {
		logError(1,
			 "%s fts_gesture_mask_store: Number of bytes of parameter wrong! %d < or > (enable/disable + at least one gestureID or max %d bytes)\n",
			 tag, (count + 1) / 3, GESTURE_MASK_SIZE);
		mask[0] = 0;
	} else {
		memset(mask, 0, GESTURE_MASK_SIZE + 2);
		mask[0] = ((count + 1) / 3) - 1;
		sscanf(p, "%02X ", &temp);
		p += 3;
		mask[1] = (u8)temp;
		for (n = 1; n < (count + 1) / 3; n++) {
			sscanf(p, "%02X ", &temp);
			p += 3;
			fromIDtoMask((u8)temp, &mask[2], GESTURE_MASK_SIZE);
		}

		for (n = 0; n < GESTURE_MASK_SIZE + 2; n++)
			logError(1, "%s mask[%d] = %02X\n", tag, n, mask[n]);
	}


	if (mask[0] == 0) {
		res = ERROR_OP_NOT_ALLOW;
		logError(1,
			 "%s %s: Call before echo enable/disable xx xx .... > gesture_mask with a correct number of parameters! ERROR %08X\n",
			 tag, __func__, res);
	} else {
		if (mask[1] == FEAT_ENABLE || mask[1] == FEAT_DISABLE)
			res = updateGestureMask(&mask[2], mask[0], mask[1]);
		else
			res = ERROR_OP_NOT_ALLOW;

		if (res < OK)
			logError(1, "%s fts_gesture_mask_store: ERROR %08X\n",
				 tag, res);
	}

	res = check_feature_feasibility(info, FEAT_SEL_GESTURE);
	temp = isAnyGestureActive();
	if (res >= OK || temp == FEAT_DISABLE)
		info->gesture_enabled = temp;
	res = fts_mode_handler(info, 0);

	return count;
}


#endif


/**
  * File node to read the coordinates of the last gesture drawn by the user \n
  * cat gesture_coordinates	to obtain the gesture coordinates \n
  * the string returned in the shell follow this up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent an error code (00000000 =
  *OK) \n
  * \n if error code = 00000000 \n
  * CC = 1 byte in HEX format number of coords (pair of x,y) returned \n
  * XXiYYi ... = XXi 2 bytes in HEX format for x[i] and
  * YYi 2 bytes in HEX format for y[i] (big endian) \n
  * \n
  * } = end byte
  */
static ssize_t fts_gesture_coordinates_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int size = PAGE_SIZE;
	int count = 0, res, i = 0;

	logError(0, "%s %s: Getting gestures coordinates...\n", tag, __func__);

	if (gesture_coords_reported < OK) {
		logError(1, "%s %s: invalid coordinates! ERROR %08X\n", tag,
			 __func__, gesture_coords_reported);
		res = gesture_coords_reported;
	} else {
		size += gesture_coords_reported * 2 * 4 + 2;
		/* coords are pairs of x,y (*2) where each coord is
		  * short(2bytes=4char)(*4) + 1 byte(2char) num of coords (+2)
		  **/
		res = OK;	/* set error code to OK */
	}


	count += scnprintf(buf + count,
			   size - count, "{ %08X", res);

	if (res >= OK) {
		count += scnprintf(buf + count,
				   size - count, "%02X",
				   gesture_coords_reported);

		for (i = 0; i < gesture_coords_reported; i++) {
			count += scnprintf(buf + count,
					   size - count,
					   "%04X",
					   gesture_coordinates_x[i]);
			count += scnprintf(buf + count,
					   size - count,
					   "%04X",
					   gesture_coordinates_y[i]);
		}
	}

	count += scnprintf(buf + count, size - count, " }\n");
	logError(0, "%s %s: Getting gestures coordinates FINISHED!\n",
		 tag, __func__);

	return count;
}
#endif



/***************************************** PRODUCTION TEST
  ***************************************************/

/**
  * File node to execute the Mass Production Test or to get data from the IC
  * (raw or ms/ss init data)
  * echo cmd > stm_fts_cmd	to execute a command \n
  * cat stm_fts_cmd	to show the result of the command \n
  * echo cmd > stm_fts_cmd; cat stm_fts_cmd	to execute and show the result
  * in just one call \n
  * the string returned in the shell is made up as follow: \n
  * { = start byte \n
  * X1X2X3X4 = 4 bytes in HEX format which represent an error_code (00000000 =
  * OK)\n
  * (optional) data = data coming from the command executed represented as HEX
  * string \n
  *                   Not all the command return additional data \n
  * } = end byte \n
  * \n
  * Possible commands (cmd): \n
  * - 00 = MP Test -> return error_code \n
  * - 01 = ITO Test -> return error_code \n
  * - 03 = MS Raw Test -> return error_code \n
  * - 04 = MS Init Data Test -> return error_code \n
  * - 05 = SS Raw Test -> return error_code \n
  * - 06 = SS Init Data Test -> return error_code \n
  * - 13 = Read 1 MS Raw Frame -> return additional data: MS frame row after row
  * \n
  * - 14 = Read MS Init Data -> return additional data: MS init data row after
  * row \n
  * - 15 = Read 1 SS Raw Frame -> return additional data: SS frame,
  * force channels followed by sense channels \n
  * - 16 = Read SS Init Data -> return additional data: SS Init data,
  * first IX for force and sense channels and then CX for force and sense
  * channels \n
  * - F0 = Perform a system reset -> return error_code \n
  * - F1 = Perform a system reset and reenable the sensing and the interrupt
  */
static ssize_t stm_fts_cmd_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int n;
	char *p = (char *)buf;

	memset(typeOfComand, 0, CMD_STR_LEN * sizeof(u32));

	logError(1, "%s\n", tag);
	for (n = 0; n < (count + 1) / 3; n++) {
		sscanf(p, "%02X ", &typeOfComand[n]);
		p += 3;
		logError(1, "%s typeOfComand[%d] = %02X\n", tag, n,
			 typeOfComand[n]);
	}

	numberParameters = n;
	logError(1, "%s Number of Parameters = %d\n", tag, numberParameters);
	return count;
}

static ssize_t stm_fts_cmd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int res, j, doClean = 0, index = 0;
	int size = (6 * 2) + 1;
	int nodes = 0;
	int init_type = SPECIAL_PANEL_INIT;
	u8 *all_strbuff = buf;
	struct fts_ts_info *info = dev_get_drvdata(dev);

	MutualSenseData compData;
	SelfSenseData comData;
	MutualSenseFrame frameMS;
	SelfSenseFrame frameSS;


	if (numberParameters >= 1) {
		res = fts_disableInterrupt();
		if (res < 0) {
			logError(0, "%s fts_disableInterrupt: ERROR %08X\n",
				 tag, res);
			res = (res | ERROR_DISABLE_INTER);
			goto END;
		}

		res = msm_drm_unregister_client(&info->notifier);
		if (res < 0) {
			logError(1, "%s ERROR: unregister notifier failed!\n",
				 tag);
			goto END;
		}


		switch (typeOfComand[0]) {
		/*ITO TEST*/
		case 0x01:
			res = production_test_ito(LIMITS_FILE, &tests);
			break;

		/*PRODUCTION TEST*/
		case 0x02:
			if (systemInfo.u8_cfgAfeVer != systemInfo.u8_cxAfeVer) {
				res = ERROR_OP_NOT_ALLOW;
				logError(0,
					 "%s Miss match in CX version! MP test not allowed with wrong CX memory! ERROR %08X\n",
					 tag, res);
				break;
			}
			res = production_test_initialization(init_type);
			break;

		case 0x00:
			if (systemInfo.u8_cfgAfeVer != systemInfo.u8_cxAfeVer) {
				res = ERROR_OP_NOT_ALLOW;
				logError(0,
					 "%s Miss match in CX version! MP test not allowed with wrong CX memory! ERROR %08X\n",
					 tag, res);
				break;
			}

			res = production_test_main(LIMITS_FILE, 1, init_type,
						   &tests);
			break;

		/*read mutual raw*/
		case 0x13:
			if (numberParameters > 1) {
				logError(0, "%s Get 1 MS Frame\n", tag);
				setScanMode(SCAN_MODE_LOCKED, typeOfComand[1]);
				mdelay(WAIT_FOR_FRESH_FRAMES);
				setScanMode(SCAN_MODE_ACTIVE, 0x00);
				mdelay(WAIT_AFTER_SENSEOFF);
				/* Delete the events related to some touch
				  * (allow to call this function while touching
				  * the screen without having a flooding of the
				  * FIFO)
				  */
				flushFIFO();
				res = getMSFrame3(MS_RAW, &frameMS);
				if (res < 0) {
					logError(0,
						 "%s Error while taking the MS frame... ERROR %08X\n",
						 tag, res);
				} else {
					logError(0,
						 "%s The frame size is %d words\n",
						 tag, res);
#ifdef RAW_DATA_FORMAT_DEC
					size += 3 * 2 +
					    (7 * frameMS.header.sense_node + 1)
					    * frameMS.header.force_node;
#else
					size += (res * sizeof(short) + 2) * 2;
#endif
					/* set res to OK because if getMSFrame
					  * is successful res = number of words
					  * read
					  */
					res = OK;
					print_frame_short(
						"MS frame =",
						array1dTo2d_short(
							frameMS.node_data,
							frameMS.node_data_size,
							frameMS.header.
							sense_node),
						frameMS.header.force_node,
						frameMS.header.sense_node);
				}
			} else {
				logError(1, "%s Wrong number of parameters!\n",
					 tag);
				res = ERROR_OP_NOT_ALLOW;
			}
			break;
		/*read self raw*/
		case 0x15:
			if (numberParameters > 1) {
				logError(0, "%s Get 1 SS Frame\n", tag);
				setScanMode(SCAN_MODE_LOCKED, typeOfComand[1]);
				mdelay(WAIT_FOR_FRESH_FRAMES);
				setScanMode(SCAN_MODE_ACTIVE, 0x00);
				mdelay(WAIT_AFTER_SENSEOFF);
				flushFIFO();
				/* delete the events related to some touch
				  * (allow to call this function while touching
				  * the screen without having a flooding of the
				  * FIFO) */
				res = getSSFrame3(SS_RAW, &frameSS);

				if (res < OK) {
					logError(0,
						 "%s Error while taking the SS frame... ERROR %08X\n",
						 tag, res);
				} else {
					logError(0,
						 "%s The frame size is %d words\n",
						 tag, res);
#ifdef RAW_DATA_FORMAT_DEC
					size += 3 * 2 + 5 +
						(frameSS.header.sense_node +
						 frameSS.header.force_node) * 7;
#else
					size += (res * sizeof(short) + 2) * 2;
#endif
					/* set res to OK because if getMSFrame
					  * is successful res = number of words
					  * read
					  */
					res = OK;
					print_frame_short(
						"SS force frame =",
						array1dTo2d_short(
							frameSS.force_data,
							frameSS.header.
							force_node, 1),
						frameSS.header.force_node, 1);
					print_frame_short(
						"SS sense frame =",
						array1dTo2d_short(
							frameSS.sense_data,
							frameSS.header.
							sense_node,
							frameSS.header.
							sense_node),
						1, frameSS.header.sense_node);
				}
			} else {
				logError(1, "%s Wrong number of parameters!\n",
					 tag);
				res = ERROR_OP_NOT_ALLOW;
			}

			break;

		case 0x14:	/* read mutual comp data */
			logError(0, "%s Get MS Compensation Data\n", tag);
			res = readMutualSenseCompensationData(LOAD_CX_MS_TOUCH,
							      &compData);

			if (res < 0)
				logError(0,
					 "%s Error reading MS compensation data ERROR %08X\n",
					 tag, res);
			else {
				logError(0,
					 "%s MS Compensation Data Reading Finished!\n",
					 tag);
				size += ((compData.node_data_size + 3) *
					 sizeof(u8)) * 2;
				print_frame_i8("MS Data (Cx2) =",
					       array1dTo2d_i8(
						       compData.node_data,
						       compData.
						       node_data_size,
						       compData.header.
						       sense_node),
					       compData.header.force_node,
					       compData.header.sense_node);
			}
			break;

		case 0x16:	/* read self comp data */
			logError(0, "%s Get SS Compensation Data...\n", tag);
			res = readSelfSenseCompensationData(LOAD_CX_SS_TOUCH,
							    &comData);
			if (res < 0)
				logError(0,
					 "%s Error reading SS compensation data ERROR %08X\n",
					 tag, res);
			else {
				logError(0,
					 "%s SS Compensation Data Reading Finished!\n",
					 tag);
				size += ((comData.header.force_node +
					  comData.header.sense_node) * 2 + 6) *
					sizeof(u8) * 2;
				print_frame_u8("SS Data Ix2_fm = ",
					       array1dTo2d_u8(comData.ix2_fm,
							      comData.header.
							      force_node, 1),
					       comData.header.force_node, 1);
				print_frame_i8("SS Data Cx2_fm = ",
					       array1dTo2d_i8(comData.cx2_fm,
							      comData.header.
							      force_node, 1),
					       comData.header.force_node, 1);
				print_frame_u8("SS Data Ix2_sn = ",
					       array1dTo2d_u8(comData.ix2_sn,
							      comData.header.
							      sense_node,
							      comData.header.
							      sense_node), 1,
					       comData.header.sense_node);
				print_frame_i8("SS Data Cx2_sn = ",
					       array1dTo2d_i8(comData.cx2_sn,
							      comData.header.
							      sense_node,
							      comData.header.
							      sense_node), 1,
					       comData.header.sense_node);
			}
			break;
		case 0x17:	/* Read mutual strength */
			logError(0, "%s Get 1 MS Strength\n", tag);
			setScanMode(SCAN_MODE_ACTIVE, 0xFF);
			msleep(WAIT_FOR_FRESH_FRAMES);
			setScanMode(SCAN_MODE_ACTIVE, 0x00);
			msleep(WAIT_AFTER_SENSEOFF);
			/* Flush outstanding touch events */
			flushFIFO();
			nodes = getMSFrame3(MS_STRENGTH, &frameMS);
			if (nodes < 0) {
				res = nodes;
				logError(0,
					 "%s Error while taking the MS strength... ERROR %08X\n",
					 tag, res);
			} else {
				logError(0, "%s The frame size is %d words\n",
					 tag, nodes);
#ifdef RAW_DATA_FORMAT_DEC
				size += 3 * 2 +
				    (7 * frameMS.header.sense_node + 1)
				    * frameMS.header.force_node;
#else
				size += (nodes * sizeof(short) + 2) * 2;
#endif
				print_frame_short("MS strength =",
				    array1dTo2d_short(frameMS.node_data,
						frameMS.node_data_size,
						frameMS.header.sense_node),
				    frameMS.header.force_node,
				    frameMS.header.sense_node);
				res = OK;
			}
			break;
		case 0x03:	/* MS Raw DATA TEST */
			res = fts_system_reset();
			if (res >= OK)
				res = production_test_ms_raw(LIMITS_FILE, 1,
							     &tests);
			break;

		case 0x04:	/* MS CX DATA TEST */
			res = fts_system_reset();
			if (res >= OK)
				res = production_test_ms_cx(LIMITS_FILE, 1,
							    &tests);
			break;

		case 0x05:	/* SS RAW DATA TEST */
			res = fts_system_reset();
			if (res >= OK)
				res = production_test_ss_raw(LIMITS_FILE, 1,
							     &tests);
			break;

		case 0x06:	/* SS IX CX DATA TEST */
			res = fts_system_reset();
			if (res >= OK)
				res = production_test_ss_ix_cx(LIMITS_FILE, 1,
							       &tests);
			break;


		case 0xF0:
		case 0xF1:	/* TOUCH ENABLE/DISABLE */
			doClean = (int)(typeOfComand[0] & 0x01);
			res = cleanUp(doClean);
			break;

		default:
			logError(1,
				 "%s COMMAND NOT VALID!! Insert a proper value ...\n",
				 tag);
			res = ERROR_OP_NOT_ALLOW;
			break;
		}

		doClean = fts_mode_handler(info, 1);
		if (typeOfComand[0] != 0xF0)
			doClean |= fts_enableInterrupt();
		if (doClean < 0)
			logError(0, "%s %s: ERROR %08X\n", tag, __func__,
				 (doClean | ERROR_ENABLE_INTER));
	} else {
		logError(1,
			 "%s NO COMMAND SPECIFIED!!! do: 'echo [cmd_code] [args] > stm_fts_cmd' before looking for result!\n",
			 tag);
		res = ERROR_OP_NOT_ALLOW;
	}

	if (msm_drm_register_client(&info->notifier) < 0)
		logError(1, "%s ERROR: register notifier failed!\n", tag);


END:
	/* here start the reporting phase, assembling the data
	  * to send in the file node */
	size = PAGE_SIZE;
	index = 0;
	index += scnprintf(all_strbuff + index, size - index, "{ %08X", res);

	if (res >= OK) {
		/*all the other cases are already fine printing only the res.*/
		switch (typeOfComand[0]) {
		case 0x13:
		case 0x17:
#ifdef RAW_DATA_FORMAT_DEC
			index += scnprintf(all_strbuff + index, size - index,
					   "%3d",
					   (u8)frameMS.header.force_node);
			index += scnprintf(all_strbuff + index, size - index,
					   "%3d",
					   (u8)frameMS.header.sense_node);
#else
			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   (u8)frameMS.header.force_node);

			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   (u8)frameMS.header.sense_node);
#endif

			for (j = 0; j < frameMS.node_data_size; j++) {
#ifdef RAW_DATA_FORMAT_DEC
				if (j % frameMS.header.sense_node == 0)
					index += scnprintf(all_strbuff + index,
							   size - index, "\n");
				index += scnprintf(all_strbuff + index,
						   size - index, "%7d",
						   frameMS.node_data[j]);
#else
				index += scnprintf(all_strbuff + index,
					   size - index,
					   "%02X%02X",
					   (frameMS.node_data[j] & 0xFF00) >> 8,
					   frameMS.node_data[j] & 0xFF);
#endif
			}

			kfree(frameMS.node_data);
			break;

		case 0x15:
#ifdef RAW_DATA_FORMAT_DEC
			index += scnprintf(all_strbuff + index, size - index,
					   "%3d",
					   (u8)frameSS.header.force_node);
			index += scnprintf(all_strbuff + index, size - index,
					   "%3d",
					   (u8)frameSS.header.sense_node);
			index += scnprintf(all_strbuff + index, size - index,
					   "\n");
#else
			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   (u8)frameSS.header.force_node);

			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   (u8)frameSS.header.sense_node);
#endif

			/* Copying self raw data Force */
			for (j = 0; j < frameSS.header.force_node; j++) {
#ifdef RAW_DATA_FORMAT_DEC
				index += scnprintf(all_strbuff + index,
						   size - index,
						   "%7d",
						   frameSS.force_data[j]);
#else
				index += scnprintf(all_strbuff + index,
					  size - index,
					  "%02X%02X",
					  (frameSS.force_data[j] & 0xFF00) >> 8,
					  frameSS.force_data[j] & 0xFF);
#endif
			}



#ifdef RAW_DATA_FORMAT_DEC
			index += scnprintf(all_strbuff + index, size - index,
					   "\n");
#endif

			/* Copying self raw data Sense */
			for (j = 0; j < frameSS.header.sense_node; j++) {
#ifdef RAW_DATA_FORMAT_DEC
				index += scnprintf(all_strbuff + index,
						   size - index, "%7d",
						   frameSS.sense_data[j]);
#else
				index += scnprintf(all_strbuff + index,
					  size - index,
					  "%02X%02X",
					  (frameSS.sense_data[j] & 0xFF00) >> 8,
					  frameSS.sense_data[j] & 0xFF);
#endif
			}

			kfree(frameSS.force_data);
			kfree(frameSS.sense_data);
			break;

		case 0x14:
			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   (u8)compData.header.force_node);

			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   (u8)compData.header.sense_node);

			/* Cpying CX1 value */
			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   (compData.cx1) & 0xFF);

			/* Copying CX2 values */
			for (j = 0; j < compData.node_data_size; j++) {
				index += scnprintf(all_strbuff + index,
						size - index,
						"%02X",
						(compData.node_data[j]) & 0xFF);
			}

			kfree(compData.node_data);
			break;

		case 0x16:
			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   comData.header.force_node);

			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   comData.header.sense_node);

			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   (comData.f_ix1) & 0xFF);

			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   (comData.s_ix1) & 0xFF);

			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   (comData.f_cx1) & 0xFF);

			index += scnprintf(all_strbuff + index,
					   size - index, "%02X",
					   (comData.s_cx1) & 0xFF);

			/* Copying IX2 Force */
			for (j = 0; j < comData.header.force_node; j++) {
				index += scnprintf(all_strbuff + index,
						   size - index,
						   "%02X",
						   comData.ix2_fm[j] & 0xFF);
			}

			/* Copying IX2 Sense */
			for (j = 0; j < comData.header.sense_node; j++) {
				index += scnprintf(all_strbuff + index,
						   size - index,
						   "%02X",
						   comData.ix2_sn[j] & 0xFF);
			}

			/* Copying CX2 Force */
			for (j = 0; j < comData.header.force_node; j++) {
				index += scnprintf(all_strbuff + index,
						   size - index,
						   "%02X",
						   comData.cx2_fm[j] & 0xFF);
			}

			/* Copying CX2 Sense */
			for (j = 0; j < comData.header.sense_node; j++) {
				index += scnprintf(all_strbuff + index,
						   size - index,
						   "%02X",
						   comData.cx2_sn[j] & 0xFF);
			}

			kfree(comData.ix2_fm);
			kfree(comData.ix2_sn);
			kfree(comData.cx2_fm);
			kfree(comData.cx2_sn);
			break;

		default:
			break;
		}
	}

	index += scnprintf(all_strbuff + index, size - index, " }\n");
	numberParameters = 0;
	/* need to reset the number of parameters in order to wait the
	  * next command, comment if you want to repeat the last command sent
	  * just doing a cat */
	/* logError(0,"%s numberParameters = %d\n",tag, numberParameters); */

	return index;
}

static DEVICE_ATTR(fwupdate, 0664, fts_fwupdate_show,
		   fts_fwupdate_store);
static DEVICE_ATTR(appid, 0444, fts_appid_show, NULL);
static DEVICE_ATTR(mode_active, 0444, fts_mode_active_show, NULL);
static DEVICE_ATTR(fw_file_test, 0444, fts_fw_test_show, NULL);
static DEVICE_ATTR(stm_fts_cmd, 0664, stm_fts_cmd_show,
		   stm_fts_cmd_store);
#ifdef USE_ONE_FILE_NODE
static DEVICE_ATTR(feature_enable, 0664,
		   fts_feature_enable_show, fts_feature_enable_store);
#else


#ifdef GRIP_MODE
static DEVICE_ATTR(grip_mode, 0664, fts_grip_mode_show,
		   fts_grip_mode_store);
#endif

#ifdef CHARGER_MODE
static DEVICE_ATTR(charger_mode, 0664,
		   fts_charger_mode_show, fts_charger_mode_store);
#endif

#ifdef GLOVE_MODE
static DEVICE_ATTR(glove_mode, 0664,
		   fts_glove_mode_show, fts_glove_mode_store);
#endif

#ifdef COVER_MODE
static DEVICE_ATTR(cover_mode, 0664,
		   fts_cover_mode_show, fts_cover_mode_store);
#endif

#ifdef STYLUS_MODE
static DEVICE_ATTR(stylus_mode, 0664,
		   fts_stylus_mode_show, fts_stylus_mode_store);
#endif

#endif

#ifdef GESTURE_MODE
static DEVICE_ATTR(gesture_mask, 0664,
		   fts_gesture_mask_show, fts_gesture_mask_store);
static DEVICE_ATTR(gesture_coordinates, 0664,
		   fts_gesture_coordinates_show, NULL);
#endif

/*  /sys/devices/soc.0/f9928000.i2c/i2c-6/6-0049 */
static struct attribute *fts_attr_group[] = {
	&dev_attr_fwupdate.attr,
	&dev_attr_appid.attr,
	&dev_attr_mode_active.attr,
	&dev_attr_fw_file_test.attr,
	&dev_attr_stm_fts_cmd.attr,
#ifdef USE_ONE_FILE_NODE
	&dev_attr_feature_enable.attr,
#else

#ifdef GRIP_MODE
	&dev_attr_grip_mode.attr,
#endif
#ifdef CHARGER_MODE
	&dev_attr_charger_mode.attr,
#endif
#ifdef GLOVE_MODE
	&dev_attr_glove_mode.attr,
#endif
#ifdef COVER_MODE
	&dev_attr_cover_mode.attr,
#endif
#ifdef STYLUS_MODE
	&dev_attr_stylus_mode.attr,
#endif

#endif

#ifdef GESTURE_MODE
	&dev_attr_gesture_mask.attr,
	&dev_attr_gesture_coordinates.attr,
#endif
	NULL,
};

/** @}*/
/** @}*/


/**
  * @defgroup isr Interrupt Service Routine (Event Handler)
  * The most important part of the driver is the ISR (Interrupt Service Routine)
  * called also as Event Handler \n
  * As soon as the interrupt pin goes low, fts_interrupt_handler() is called and
  * the chain to read and parse the event read from the FIFO start.\n
  * For any different kind of EVT_ID there is a specific event handler
  * which will take the correct action to report the proper info to the host. \n
  * The most important events are the one related to touch information, status
  * update or user report.
  * @{
  */

/**
  * Report to the linux input system the pressure and release of a button
  * handling concurrency
  * @param info pointer to fts_ts_info which contains info about the device
  * and its hw setup
  * @param key_code	button value
  */
void fts_input_report_key(struct fts_ts_info *info, int key_code)
{
	mutex_lock(&info->input_report_mutex);
	input_report_key(info->input_dev, key_code, 1);
	input_sync(info->input_dev);
	input_report_key(info->input_dev, key_code, 0);
	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);
}



/**
  * Event Handler for no events (EVT_ID_NOEVENT)
  */
static void fts_nop_event_handler(struct fts_ts_info *info, unsigned
				  char *event)
{
	logError(1,
		 "%s %s Doing nothing for event = %02X %02X %02X %02X %02X %02X %02X %02X\n",
		 tag, __func__, event[0], event[1], event[2], event[3],
		 event[4],
		 event[5], event[6], event[7]);
}

/**
  * Adjust coordinates based on actual hw margins.
  */
static void fts_coordinates_factor(struct fts_ts_info *info, int *x, int *y)
{
	unsigned int x_work;
	unsigned int y_work;
	unsigned int x_adj;
	unsigned int y_adj;

	x_work = *x;
	y_work = *y;

	// Ensure minimum value.
	x_work = max((unsigned int) x_work, (unsigned int) info->board->x_axis_edge_offset);
	y_work = max((unsigned int) y_work, (unsigned int) info->board->y_axis_edge_offset);
	
	// Ensure maximum value.
	x_work = min((unsigned int) x_work, (unsigned int) info->board->x_axis_real_max);
	y_work = min((unsigned int) y_work, (unsigned int) info->board->y_axis_real_max);
	
	// Adjust with edge offset.
	x_work -= info->board->x_axis_edge_offset;
	y_work -= info->board->y_axis_edge_offset;

	// Calculate coordinates factor.
	x_adj = (info->board->x_axis_max * 1000) /
		(info->board->x_axis_real_max - info->board->x_axis_edge_offset);
	y_adj = (info->board->y_axis_max * 1000) /
		(info->board->y_axis_real_max - info->board->y_axis_edge_offset);
	
	// Calculate the adjusted coordinates.
	x_work = x_work * x_adj / 1000;
	y_work = y_work * y_adj / 1000;

	*x = x_work;
	*y = y_work;
}

/**
  * Event handler for enter and motion events (EVT_ID_ENTER_POINT,
  * EVT_ID_MOTION_POINT )
  * report to the linux input system touches with their coordinated and
  * additional information
  */
static void fts_enter_pointer_event_handler(struct fts_ts_info *info, unsigned
					    char *event)
{
	unsigned char touchId;
	unsigned int touch_condition = 1, tool = MT_TOOL_FINGER;
	int x, y, z, distance;
	u8 touchType;

	if (!info->resume_bit)
		goto no_report;

	touchType = event[1] & 0x0F;
	touchId = (event[1] & 0xF0) >> 4;

	x = (((int)event[3] & 0x0F) << 8) | (event[2]);
	y = ((int)event[4] << 4) | ((event[3] & 0xF0) >> 4);
	/* TODO: check with fw how they will report distance and pressure */
	z = PRESSURE_MAX;
	distance = 0;	/* if the tool is touching the display
			  * the distance should be 0 */

	if (x == info->board->x_axis_max)
		x--;

	if (y == info->board->y_axis_max)
		y--;

	if (info->board->coord_factor)
		fts_coordinates_factor(info, &x, &y);

	input_mt_slot(info->input_dev, touchId);
	switch (touchType) {
#ifdef STYLUS_MODE
	case TOUCH_TYPE_STYLUS:
		logError(0, "%s  %s : It is a stylus!\n", tag, __func__);
		if (info->stylus_enabled == 1) {
			/* if stylus_enabled is not ==1
			  * it will be reported as normal touch */
			tool = MT_TOOL_PEN;
			touch_condition = 1;
			__set_bit(touchId, &info->stylus_id);
			break;
		}
#endif
	/* TODO: customer can implement a different strategy for each kind of
	 * touch */
	case TOUCH_TYPE_FINGER:
	/* logError(0, "%s  %s : It is a finger!\n",tag,__func__); */
	case TOUCH_TYPE_GLOVE:
	/* logError(0, "%s  %s : It is a glove!\n",tag,__func__); */
	case TOUCH_TYPE_PALM:
		/* logError(0, "%s  %s : It is a palm!\n",tag,__func__); */
		tool = MT_TOOL_FINGER;
		touch_condition = 1;
		__set_bit(touchId, &info->touch_id);
		break;


	case TOUCH_TYPE_HOVER:
		tool = MT_TOOL_FINGER;
		touch_condition = 0;	/* need to hover */
		z = 0;	/* no pressure */
		__set_bit(touchId, &info->touch_id);
		distance = DISTANCE_MAX;/* check with fw report the hovering
					  * distance */
		break;

	case TOUCH_TYPE_INVALID:
	default:
		logError(1, "%s  %s : Invalid touch type = %d ! No Report...\n",
			 tag, __func__, touchType);
		goto no_report;
	}

	input_mt_report_slot_state(info->input_dev, tool, 1);

	/* logError(0, "%s  %s : TouchID = %d,Touchcount = %d\n",tag,__func__,
	  *	touchId,touchcount); */
	input_report_key(info->input_dev, BTN_TOUCH, touch_condition);

	/* input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, touchId); */
	input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MINOR, z);
#ifndef SKIP_PRESSURE
	input_report_abs(info->input_dev, ABS_MT_PRESSURE, z);
#endif
#ifndef SKIP_DISTANCE
	input_report_abs(info->input_dev, ABS_MT_DISTANCE, distance);
#endif
	/* logError(0, "%s  %s :  Event 0x%02x - ID[%d], (x, y) = (%3d, %3d)
	 * Size = %d\n",
	  *	tag, __func__, *event, touchId, x, y, touchType); */

no_report:
	return;
}

/**
  * Event handler for leave event (EVT_ID_LEAVE_POINT )
  * Report to the linux input system that one touch left the display
  */
static void fts_leave_pointer_event_handler(struct fts_ts_info *info, unsigned
					    char *event)
{
	unsigned char touchId;
	unsigned int tool = MT_TOOL_FINGER;
	u8 touchType;

	touchType = event[1] & 0x0F;
	touchId = (event[1] & 0xF0) >> 4;



	input_mt_slot(info->input_dev, touchId);
	switch (touchType) {
#ifdef STYLUS_MODE
	case TOUCH_TYPE_STYLUS:
		logError(0, "%s  %s : It is a stylus!\n", tag, __func__);
		if (info->stylus_enabled == 1) {
			/* if stylus_enabled is not ==1 it will be reported as
			 * normal touch */
			tool = MT_TOOL_PEN;
			__clear_bit(touchId, &info->stylus_id);
			break;
		}
#endif

	case TOUCH_TYPE_FINGER:
	/* logError(0, "%s  %s : It is a finger!\n",tag,__func__); */
	case TOUCH_TYPE_GLOVE:
	/* logError(0, "%s  %s : It is a glove!\n",tag,__func__); */
	case TOUCH_TYPE_PALM:
	/* logError(0, "%s  %s : It is a palm!\n",tag,__func__); */
	case TOUCH_TYPE_HOVER:
		tool = MT_TOOL_FINGER;
		__clear_bit(touchId, &info->touch_id);
		break;

	case TOUCH_TYPE_INVALID:
	default:
		logError(1, "%s  %s : Invalid touch type = %d ! No Report...\n",
			 tag, __func__, touchType);
		return;
	}

	input_mt_report_slot_state(info->input_dev, tool, 0);

	/* logError(0, "%s  %s : TouchID = %d, Touchcount = %d\n",tag,__func__,
	  *	touchId,touchcount); */


	input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, -1);
	/* logError(0, "%s  %s : Event 0x%02x - release ID[%d]\n", tag,
	 * __func__,
	  *	event[0], touchId); */
}

/* EventId : EVT_ID_MOTION_POINT */
#define fts_motion_pointer_event_handler fts_enter_pointer_event_handler
/* remap the motion event handler to the same function which handle the enter
 * event */

/**
  * Event handler for error events (EVT_ID_ERROR)
  * Handle unexpected error events implementing recovery strategy and
  * restoring the sensing status that the IC had before the error occurred
  */
static void fts_error_event_handler(struct fts_ts_info *info, unsigned
				    char *event)
{
	int error = 0;

	logError(0,
		 "%s %s Received event %02X %02X %02X %02X %02X %02X %02X %02X\n",
		 tag,
		 __func__, event[0], event[1], event[2], event[3], event[4],
		 event[5],
		 event[6], event[7]);

	switch (event[1]) {
	case EVT_TYPE_ERROR_ESD:/* esd */
	{/* before reset clear all slot */
		release_all_touches(info);

		fts_chip_powercycle(info);

		error = fts_system_reset();
		error |= fts_mode_handler(info, 0);
		error |= fts_enableInterrupt();
		if (error < OK)
			logError(1,
				 "%s %s Cannot restore the device ERROR %08X\n",
				 tag, __func__, error);
	}
	break;
	case EVT_TYPE_ERROR_WATCHDOG:	/* watch dog timer */
	{
		dumpErrorInfo(NULL, 0);
		/* before reset clear all slots */
		release_all_touches(info);
		error = fts_system_reset();
		error |= fts_mode_handler(info, 0);
		error |= fts_enableInterrupt();
		if (error < OK)
			logError(1,
				 "%s %s Cannot reset the device ERROR %08X\n",
				 tag, __func__, error);
	}
	break;
	}
}

/**
  * Event handler for controller ready event (EVT_ID_CONTROLLER_READY)
  * Handle controller events received after unexpected reset of the IC updating
  * the resets flag and restoring the proper sensing status
  */
static void fts_controller_ready_event_handler(struct fts_ts_info *info,
					       unsigned char *event)
{
	int error;

	logError(0,
		 "%s %s Received event %02X %02X %02X %02X %02X %02X %02X %02X\n",
		 tag,
		 __func__, event[0], event[1], event[2], event[3], event[4],
		 event[5],
		 event[6], event[7]);
	release_all_touches(info);
	setSystemResetedUp(1);
	setSystemResetedDown(1);
	error = fts_mode_handler(info, 0);
	if (error < OK)
		logError(1,
			 "%s %s Cannot restore the device status ERROR %08X\n",
			 tag,
			 __func__, error);
}

/**
  * Event handler for status events (EVT_ID_STATUS_UPDATE)
  * Handle status update events
  */
static void fts_status_event_handler(struct fts_ts_info *info, unsigned
				     char *event)
{
	switch (event[1]) {
	case EVT_TYPE_STATUS_ECHO:
		logError(1,
			 "%s %s Echo event of command = %02X %02X %02X %02X %02X %02X\n",
			 tag, __func__, event[2], event[3], event[4], event[5],
			 event[6],
			 event[7]);
		break;

	case EVT_TYPE_STATUS_FORCE_CAL:
		switch (event[2]) {
		case 0x00:
			logError(1,
				 "%s %s Continuous frame drop Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x01:
			logError(1,
				 "%s %s Mutual negative detect Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x02:
			logError(1,
				 "%s %s Mutual calib deviation Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x11:
			logError(1,
				 "%s %s SS negative detect Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x12:
			logError(1,
				 "%s %s SS negative detect Force cal in Low Power mode = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x13:
			logError(1,
				 "%s %s SS negative detect Force cal in Idle mode = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x20:
			logError(1,
				 "%s %s SS invalid Mutual Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x21:
			logError(1,
				 "%s %s SS invalid Self Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x22:
			logError(1,
				 "%s %s SS invalid Self Island soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x30:
			logError(1,
				 "%s %s MS invalid Mutual Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x31:
			logError(1,
				 "%s %s MS invalid Self Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		default:
			logError(1,
				 "%s %s Force cal = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		}
		break;

	case EVT_TYPE_STATUS_FRAME_DROP:
		switch (event[2]) {
		case 0x01:
			logError(1,
				 "%s %s Frame drop noisy frame = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x02:
			logError(1,
				 "%s %s Frame drop bad R = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		case 0x03:
			logError(1,
				 "%s %s Frame drop invalid processing state = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
			break;

		default:
			logError(1,
				 "%s %s Frame drop = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		}
		break;

	case EVT_TYPE_STATUS_SS_RAW_SAT:
		if (event[2] == 1)
			logError(1,
				 "%s %s SS Raw Saturated = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		else
			logError(1,
				 "%s %s SS Raw No more Saturated = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		break;

	case EVT_TYPE_STATUS_WATER:
		if (event[2] == 1)
			logError(1,
				 "%s %s Enter Water mode = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		else
			logError(1,
				 "%s %s Exit Water mode = %02X %02X %02X %02X %02X %02X\n",
				 tag, __func__, event[2], event[3], event[4],
				 event[5],
				 event[6], event[7]);
		break;

	default:
		logError(1,
			 "%s %s Received unhandled status event = %02X %02X %02X %02X %02X %02X %02X %02X\n",
			 tag, __func__, event[0], event[1], event[2], event[3],
			 event[4],
			 event[5], event[6], event[7]);
		break;
	}
}


/* key events reported in the user report */
#ifdef PHONE_KEY
/* TODO: the customer should handle the events coming from the keys according
 * his needs
  * (this is just an sample code that report the click of a button after a
  * press->release action) */
/**
  * Event handler for status events (EVT_TYPE_USER_KEY)
  * Handle keys update events, the third byte of the event is a bitmask,
  * if the bit set means that the corresponding key is pressed.
  */
static void fts_key_event_handler(struct fts_ts_info *info,
				  unsigned char *event)
{
	/* int value; */
	logError(0,
		 "%s %s Received event %02X %02X %02X %02X %02X %02X %02X %02X\n",
		 tag,
		 __func__, event[0], event[1], event[2], event[3], event[4],
		 event[5],
		 event[6], event[7]);

	if (event[0] == EVT_ID_USER_REPORT && event[1] == EVT_TYPE_USER_KEY) {
		/* event[2] contain the bitmask of the keys that are actually
		 * pressed */

		if ((event[2] & FTS_KEY_0) == 0 && (key_mask & FTS_KEY_0) > 0) {
			logError(0,
				 "%s %s: Button HOME pressed and released!\n",
				 tag,
				 __func__);
			fts_input_report_key(info, KEY_HOMEPAGE);
		}

		if ((event[2] & FTS_KEY_1) == 0 && (key_mask & FTS_KEY_1) > 0) {
			logError(0,
				 "%s %s: Button Back pressed and released!\n",
				 tag,
				 __func__);
			fts_input_report_key(info, KEY_BACK);
		}

		if ((event[2] & FTS_KEY_2) == 0 && (key_mask & FTS_KEY_2) > 0) {
			logError(0, "%s %s: Button Menu pressed!\n", tag,
				 __func__);
			fts_input_report_key(info, KEY_MENU);
		}

		key_mask = event[2];
	} else
		logError(1, "%s %s: Invalid event passed as argument!\n", tag,
			 __func__);
}
#endif

/* gesture event must be handled in the user event handler */
#ifdef GESTURE_MODE
/* TODO: Customer should implement their own actions in respond of a gesture
 * event.
  * This is an example that simply print the gesture received and simulate
  * the click on a different button for each gesture. */
/**
  * Event handler for gesture events (EVT_TYPE_USER_GESTURE)
  * Handle gesture events and simulate the click on a different button
  * for any gesture detected (@link gesture_opt Gesture IDs @endlink)
  */
static void fts_gesture_event_handler(struct fts_ts_info *info, unsigned
				      char *event)
{
	int value;
	int needCoords = 0;

	logError(0,
		 "%s  gesture event data: %02X %02X %02X %02X %02X %02X %02X %02X\n",
		 tag, event[0], event[1], event[2], event[3], event[4],
		 event[5],
		 event[6], event[7]);



	if (event[0] == EVT_ID_USER_REPORT && event[1] ==
	    EVT_TYPE_USER_GESTURE) {
		needCoords = 1;
		/* default read the coordinates for all gestures excluding
		 * double tap */

		switch (event[2]) {
		case GEST_ID_DBLTAP:
			value = KEY_WAKEUP;
			logError(0, "%s %s: double tap !\n", tag, __func__);
			needCoords = 0;
			break;

		case GEST_ID_AT:
			value = KEY_WWW;
			logError(0, "%s %s: @ !\n", tag, __func__);
			break;

		case GEST_ID_C:
			value = KEY_C;
			logError(0, "%s %s: C !\n", tag, __func__);
			break;

		case GEST_ID_E:
			value = KEY_E;
			logError(0, "%s %s: e !\n", tag, __func__);
			break;

		case GEST_ID_F:
			value = KEY_F;
			logError(0, "%s %s: F !\n", tag, __func__);
			break;

		case GEST_ID_L:
			value = KEY_L;
			logError(0, "%s %s: L !\n", tag, __func__);
			break;

		case GEST_ID_M:
			value = KEY_M;
			logError(0, "%s %s: M !\n", tag, __func__);
			break;

		case GEST_ID_O:
			value = KEY_O;
			logError(0, "%s %s: O !\n", tag, __func__);
			break;

		case GEST_ID_S:
			value = KEY_S;
			logError(0, "%s %s: S !\n", tag, __func__);
			break;

		case GEST_ID_V:
			value = KEY_V;
			logError(0, "%s %s:  V !\n", tag, __func__);
			break;

		case GEST_ID_W:
			value = KEY_W;
			logError(0, "%s %s:  W !\n", tag, __func__);
			break;

		case GEST_ID_Z:
			value = KEY_Z;
			logError(0, "%s %s:  Z !\n", tag, __func__);
			break;

		case GEST_ID_RIGHT_1F:
			value = KEY_RIGHT;
			logError(0, "%s %s:  -> !\n", tag, __func__);
			break;

		case GEST_ID_LEFT_1F:
			value = KEY_LEFT;
			logError(0, "%s %s:  <- !\n", tag, __func__);
			break;

		case GEST_ID_UP_1F:
			value = KEY_UP;
			logError(0, "%s %s:  UP !\n", tag, __func__);
			break;

		case GEST_ID_DOWN_1F:
			value = KEY_DOWN;
			logError(0, "%s %s:  DOWN !\n", tag, __func__);
			break;

		case GEST_ID_CARET:
			value = KEY_APOSTROPHE;
			logError(0, "%s %s:  ^ !\n", tag, __func__);
			break;

		case GEST_ID_LEFTBRACE:
			value = KEY_LEFTBRACE;
			logError(0, "%s %s:  < !\n", tag, __func__);
			break;

		case GEST_ID_RIGHTBRACE:
			value = KEY_RIGHTBRACE;
			logError(0, "%s %s:  > !\n", tag, __func__);
			break;

		default:
			logError(0, "%s %s:  No valid GestureID!\n", tag,
				 __func__);
			goto gesture_done;
		}

		if (needCoords == 1)
			readGestureCoords(event);

		fts_input_report_key(info, value);

gesture_done:
		return;
	} else
		logError(1, "%s %s: Invalid event passed as argument!\n", tag,
			 __func__);
}
#endif


/**
  * Event handler for user report events (EVT_ID_USER_REPORT)
  * Handle user events reported by the FW due to some interaction triggered
  * by an external user (press keys, perform gestures, etc.)
  */
static void fts_user_report_event_handler(struct fts_ts_info *info, unsigned
					  char *event)
{
	switch (event[1]) {
#ifdef PHONE_KEY
	case EVT_TYPE_USER_KEY:
		fts_key_event_handler(info, event);
		break;
#endif

	case EVT_TYPE_USER_PROXIMITY:
		if (event[2] == 0)
			logError(1, "%s %s No proximity!\n", tag, __func__);
		else
			logError(1, "%s %s Proximity Detected!\n", tag,
				 __func__);
		break;

#ifdef GESTURE_MODE
	case EVT_TYPE_USER_GESTURE:
		fts_gesture_event_handler(info, event);
		break;
#endif
	default:
		logError(1,
			 "%s %s Received unhandled user report event = %02X %02X %02X %02X %02X %02X %02X %02X\n",
			 tag, __func__, event[0], event[1], event[2], event[3],
			 event[4],
			 event[5], event[6], event[7]);
		break;
	}
}

/**
  * Bottom Half Interrupt Handler function
  * This handler is called each time there is at least one new event in the FIFO
  * and the interrupt pin of the IC goes low. It will read all the events from
  * the FIFO and dispatch them to the proper event handler according the event
  * ID
  */
static void fts_event_handler(struct work_struct *work)
{
	struct fts_ts_info *info;
	int error = 0, count = 0;
	unsigned char regAdd = FIFO_CMD_READALL;
	unsigned char data[FIFO_EVENT_SIZE * FIFO_DEPTH];
	unsigned char eventId;
	const unsigned char EVENTS_REMAINING_POS = 7;
	const unsigned char EVENTS_REMAINING_MASK = 0x1F;
	unsigned char events_remaining = 0;
	unsigned char *evt_data;

	event_dispatch_handler_t event_handler;

	info = container_of(work, struct fts_ts_info, work);

	__pm_wakeup_event(&info->wakesrc, jiffies_to_msecs(HZ));

	/* Read the first FIFO event and the number of events remaining */
	error = fts_writeReadU8UX(regAdd, 0, 0, data, FIFO_EVENT_SIZE,
				  DUMMY_FIFO);
	events_remaining = data[EVENTS_REMAINING_POS] & EVENTS_REMAINING_MASK;
	events_remaining = (events_remaining > FIFO_DEPTH - 1) ?
			   FIFO_DEPTH - 1 : events_remaining;

	/* Drain the rest of the FIFO, up to 31 events */
	if (error == OK && events_remaining > 0) {
		error = fts_writeReadU8UX(regAdd, 0, 0, &data[FIFO_EVENT_SIZE],
					  FIFO_EVENT_SIZE * events_remaining,
					  DUMMY_FIFO);
	}
	if (error != OK) {
		logError(1,
			 "Error (%08X) while reading from FIFO in fts_event_handler\n",
			 error);
	} else {
		for (count = 0; count < events_remaining + 1; count++) {
			evt_data = &data[count * FIFO_EVENT_SIZE];

			if (evt_data[0] == EVT_ID_NOEVENT)
				break;

			eventId = evt_data[0] >> 4;

			/* Ensure event ID is within bounds */
			if (eventId < NUM_EVT_ID) {
				event_handler =
					info->event_dispatch_table[eventId];
				event_handler(info, (evt_data));
			}
		}
	}
	input_sync(info->input_dev);

	/* Re-enable interrupts */
	fts_interrupt_enable(info);
}
/** @}*/



/**
  *	Implement the fw update and initialization flow of the IC that should
  *	be executed at every boot up. The function perform a fw update of the
  *	IC in case of crc error or a new fw version and then understand if the
  *	IC need to be re-initialized again.
  *
  *	@return  OK if success or an error code which specify the type of error
  *	encountered
  */
static int fts_fw_update(struct fts_ts_info *info)
{
	u8 error_to_search[4] = { EVT_TYPE_ERROR_CRC_CX_HEAD,
				  EVT_TYPE_ERROR_CRC_CX,
				  EVT_TYPE_ERROR_CRC_CX_SUB_HEAD,
				  EVT_TYPE_ERROR_CRC_CX_SUB };
	int ret = 0;
	int init_type = NO_INIT;

#ifdef PRE_SAVED_METHOD
	int keep_cx = 1;
#else
	int keep_cx = 0;
#endif


	logError(1, "%s Fw Auto Update is starting...\n", tag);

	/* Check CRC status */
	ret = fts_crc_check();
	if (ret > OK) {
		logError(1, "%s %s: CRC Error or NO FW!\n", tag, __func__);
		info->reflash_fw = 1;
	} else {
		logError(1,
			 "%s %s: NO CRC Error or Impossible to read CRC register!\n",
			 tag, __func__);
	}
	ret = flashProcedure(info->board->fw_name, info->reflash_fw, keep_cx);
	if ((ret & 0xF000000F) == ERROR_FILE_NOT_FOUND) {
		logError(1,
			 "%s %s: firmware file not found. Bypassing update.\n",
			 tag, __func__);
		ret = 0;
		goto out;
	} else if ((ret & 0xFF000000) == ERROR_FLASH_PROCEDURE) {
		logError(1,
			 "%s %s: firmware update failed; retrying. ERROR %08X\n",
			 tag, __func__, ret);
		/* Power cycle the touch IC */
		fts_chip_powercycle(info);
		ret = flashProcedure(info->board->fw_name, info->reflash_fw,
				     keep_cx);
		if ((ret & 0xFF000000) == ERROR_FLASH_PROCEDURE) {
			logError(1,
				 "%s %s: firmware update failed again! ERROR %08X\n",
				 tag, __func__, ret);
			logError(1, "%s Fw Auto Update Failed!\n", tag);
			return ret;
		}
	}
	info->reflash_fw = 0;

	logError(1, "%s %s: Verifying if CX CRC Error...\n", tag, __func__);
	ret = fts_system_reset();
	if (ret >= OK) {
		ret = pollForErrorType(error_to_search, 4);
		if (ret < OK) {
			logError(1, "%s %s: No Cx CRC Error Found!\n", tag,
				 __func__);
			logError(1, "%s %s: Verifying if Panel CRC Error...\n",
				 tag, __func__);
			error_to_search[0] = EVT_TYPE_ERROR_CRC_PANEL_HEAD;
			error_to_search[1] = EVT_TYPE_ERROR_CRC_PANEL;
			ret = pollForErrorType(error_to_search, 2);
			if (ret < OK) {
				logError(1,
					 "%s %s: No Panel CRC Error Found!\n",
					 tag, __func__);
				init_type = NO_INIT;
			} else {
				logError(1,
					 "%s %s: Panel CRC Error FOUND! CRC ERROR = %02X\n",
					 tag, __func__, ret);
				init_type = SPECIAL_PANEL_INIT;
			}
		} else {
			logError(1,
				 "%s %s: Cx CRC Error FOUND! CRC ERROR = %02X\n",
				 tag, __func__, ret);

			/** This path of the code is used only in case there is
			  * a CRC error in code or config which not allow the fw
			  * to compute the CRC in the CX before
			  */
			logError(1,
				 "%s %s: Try to recovery with CX in fw file...\n",
				 tag, __func__, ret);
			flashProcedure(info->board->fw_name, CRC_CX, 0);
			logError(1,
				 "%s %s: Refresh panel init data...\n",
				 tag, __func__, ret);
		}
	} else {
		/* Skip initialization because the real state is unknown */
		logError(1,
			 "%s %s: Error while executing system reset! ERROR %08X\n",
			 tag, __func__, ret);
	}

	if (init_type == NO_INIT) {
#ifdef PRE_SAVED_METHOD
		if (systemInfo.u8_cfgAfeVer != systemInfo.u8_cxAfeVer) {
			init_type = SPECIAL_FULL_PANEL_INIT;
			logError(0,
				 "%s %s: Different CX AFE Ver: %02X != %02X... Execute FULL Panel Init!\n",
				 tag, __func__, systemInfo.u8_cfgAfeVer,
				 systemInfo.u8_cxAfeVer);
		} else
#endif
		if (systemInfo.u8_cfgAfeVer != systemInfo.u8_panelCfgAfeVer) {
			init_type = SPECIAL_PANEL_INIT;
			logError(0,
				 "%s %s: Different Panel AFE Ver: %02X != %02X... Execute Panel Init!\n",
				 tag, __func__, systemInfo.u8_cfgAfeVer,
				 systemInfo.u8_panelCfgAfeVer);
		} else
			init_type = NO_INIT;
	}

out:
	/** Reinitialize after a complete FW update or if the initialization
	  * status is not correct.
	  */
	if (init_type != NO_INIT) {
		ret = fts_chip_initialization(info, init_type);
		if (ret < OK) {
			logError(1,
				 "%s %s Cannot initialize the chip ERROR %08X\n",
				 tag, __func__, ret);
		}
	}

	ret = fts_init_sensing(info);
	if (ret < OK) {
		logError(1,
			 "%s Cannot initialize the hardware device ERROR %08X\n",
			 tag, ret);
	}

	logError(1, "%s Fw Update Finished! error = %08X\n", tag, ret);
	return ret;
}

#ifndef FW_UPDATE_ON_PROBE
/**
  *	Function called by the delayed workthread executed after the probe in
  * order to perform the fw update flow
  *	@see  fts_fw_update()
  */
static void fts_fw_update_auto(struct work_struct *work)
{
	struct delayed_work *fwu_work = container_of(work, struct delayed_work,
						     work);
	struct fts_ts_info *info = container_of(fwu_work, struct fts_ts_info,
						fwu_work);

	fts_fw_update(info);
}
#endif

/* TODO: define if need to do the full mp at the boot */
/**
  *	Execute the initialization of the IC (supporting a retry mechanism),
  * checking also the resulting data
  *	@see  production_test_main()
  */
static int fts_chip_initialization(struct fts_ts_info *info, int init_type)
{
	int ret2 = 0;
	int retry;
	int initretrycnt = 0;

	/* initialization error, retry initialization */
	for (retry = 0; retry <= RETRY_INIT_BOOT; retry++) {
		ret2 = production_test_initialization(init_type);
		if (ret2 == OK)
			break;
		initretrycnt++;
		logError(1,
			 "%s initialization cycle count = %04d - ERROR %08X\n",
			 tag,
			 initretrycnt, ret2);
		fts_chip_powercycle(info);
	}
	if (ret2 < OK)	/* initialization error */

		logError(1, "%s fts initialization failed 3 times\n", tag);


	return ret2;
}


/**
  * @addtogroup isr
  * @{
  */
/**
  * Top half Interrupt handler function
  * Respond to the interrupt and schedule the bottom half interrupt handler
  * in its work queue
  * @see fts_event_handler()
  */
static irqreturn_t fts_interrupt_handler(int irq, void *handle)
{
	struct fts_ts_info *info = handle;

	disable_irq_nosync(info->client->irq);

	queue_work(info->event_wq, &info->work);

	return IRQ_HANDLED;
}


/**
  * Initialize the dispatch table with the event handlers for any possible event
  * ID
  * Set IRQ pin behavior (level triggered low)
  * Register top half interrupt handler function.
  * @see fts_interrupt_handler()
  */
static int fts_interrupt_install(struct fts_ts_info *info)
{
	int i, error = 0;

	info->event_dispatch_table = kzalloc(sizeof(event_dispatch_handler_t) *
					     NUM_EVT_ID, GFP_KERNEL);

	if (!info->event_dispatch_table) {
		logError(1, "%s OOM allocating event dispatch table\n", tag);
		return -ENOMEM;
	}

	for (i = 0; i < NUM_EVT_ID; i++)
		info->event_dispatch_table[i] = fts_nop_event_handler;

	install_handler(info, ENTER_POINT, enter_pointer);
	install_handler(info, LEAVE_POINT, leave_pointer);
	install_handler(info, MOTION_POINT, motion_pointer);
	install_handler(info, ERROR, error);
	install_handler(info, CONTROLLER_READY, controller_ready);
	install_handler(info, STATUS_UPDATE, status);
	install_handler(info, USER_REPORT, user_report);

	/* disable interrupts in any case */
	error = fts_disableInterrupt();

	logError(1, "%s Interrupt Mode\n", tag);
	if (request_irq(info->client->irq, fts_interrupt_handler,
			IRQF_TRIGGER_LOW, FTS_TS_DRV_NAME, info)) {
		logError(1, "%s Request irq failed\n", tag);
		kfree(info->event_dispatch_table);
		error = -EBUSY;
	}

	return error;
}

/**
  *	Clean the dispatch table and the free the IRQ.
  *	This function is called when the driver need to be removed
  */
static void fts_interrupt_uninstall(struct fts_ts_info *info)
{
	fts_disableInterrupt();

	kfree(info->event_dispatch_table);

	free_irq(info->client->irq, info);
}

/**
  * Enable the host side interrupt
  */
static void fts_interrupt_enable(struct fts_ts_info *info)
{
	/* logError(0, "%s %s : enable interrupts!\n",tag,__func__); */
	enable_irq(info->client->irq);
}

/**@}*/

/**
  * This function try to attempt to communicate with the IC for the first time
  * during the boot up process in order to read the necessary info for the
  * following stages.
  * The function execute a system reset, read fundamental info (system info)
  * @return OK if success or an error code which specify the type of error
  */
static int fts_init(struct fts_ts_info *info)
{
	int error;


	error = fts_system_reset();
	if (error < OK && isI2cError(error)) {
		logError(1, "%s Cannot reset the device! ERROR %08X\n", tag,
			 error);
		return error;
	} else {
		if (error == (ERROR_TIMEOUT | ERROR_SYSTEM_RESET_FAIL)) {
			logError(1, "%s Setting default Sys INFO!\n", tag);
			error = defaultSysInfo(0);
		} else {
			error = readSysInfo(0);	/* system reset OK */
			if (error < OK) {
				if (!isI2cError(error))
					error = OK;
				logError(1,
					 "%s Cannot read Sys Info! ERROR %08X\n",
					 tag,
					 error);
			}
		}
	}

	return error;
}

/**
  * Execute a power cycle in the IC, toggling the power lines (AVDD and DVDD)
  * @param info pointer to fts_ts_info struct which contain information of the
  * regulators
  * @return 0 if success or another value if fail
  */
int fts_chip_powercycle(struct fts_ts_info *info)
{
	int error = 0;

	logError(1, "%s %s: Power Cycle Starting...\n", tag, __func__);
	logError(1, "%s %s: Disabling IRQ...\n", tag, __func__);
	/** if IRQ pin is short with DVDD a call to the ISR will triggered when
	  * the regulator is turned off if IRQ not disabled */
	fts_disableInterrupt();

	if (info->vdd_reg) {
		error = regulator_disable(info->vdd_reg);
		if (error < 0)
			logError(1, "%s %s: Failed to disable DVDD regulator\n",
				 tag, __func__);
	}

	if (info->avdd_reg) {
		error = regulator_disable(info->avdd_reg);
		if (error < 0)
			logError(1, "%s %s: Failed to disable AVDD regulator\n",
				 tag, __func__);
	}

	if (info->board->reset_gpio != GPIO_NOT_DEFINED)
		gpio_set_value(info->board->reset_gpio, 0);
	else
		mdelay(300);

	/* in FTI power up first the digital and then the analog */
	if (info->vdd_reg) {
		error = regulator_enable(info->vdd_reg);
		if (error < 0)
			logError(1, "%s %s: Failed to enable DVDD regulator\n",
				 tag, __func__);
	}

	mdelay(1);

	if (info->avdd_reg) {
		error = regulator_enable(info->avdd_reg);
		if (error < 0)
			logError(1, "%s %s: Failed to enable AVDD regulator\n",
				 tag, __func__);
	}

	mdelay(5);	/* time needed by the regulators for reaching the regime
			 * values */


	if (info->board->reset_gpio != GPIO_NOT_DEFINED) {
		mdelay(10);	/* time to wait before bring up the reset
				  * gpio after the power up of the regulators */
		gpio_set_value(info->board->reset_gpio, 1);
	}

	release_all_touches(info);

	logError(1, "%s %s: Power Cycle Finished! ERROR CODE = %08x\n", tag,
		 __func__, error);
	setSystemResetedUp(1);
	setSystemResetedDown(1);
	return error;
}


/**
  * Complete the boot up process, initializing the sensing of the IC according
  * to the current setting chosen by the host
  * Register the notifier for the suspend/resume actions and the event handler
  * @return OK if success or an error code which specify the type of error
  */
static int fts_init_sensing(struct fts_ts_info *info)
{
	int error = 0;

	error |= msm_drm_register_client(&info->notifier);/* register the
							   * suspend/resume
							   * function */
	error |= fts_interrupt_install(info);	/* register event handler */
	error |= fts_mode_handler(info, 0);	/* enable the features and
						 * sensing */
	/* error |= fts_enableInterrupt(); */	/* enable the interrupt */
	error |= fts_resetDisableIrqCount();

	if (error < OK)
		logError(1, "%s %s Init after Probe error (ERROR = %08X)\n",
			 tag, __func__, error);


	return error;
}

/* TODO: change this function according with the needs of customer in terms
  * of feature to enable/disable */

/**
  * @ingroup mode_section
  * @{
  */
/**
  * The function handle the switching of the mode in the IC enabling/disabling
  * the sensing and the features set from the host
  * @param info pointer to fts_ts_info which contains info about the device and
  * its hw setup
  * @param force if 1, the enabling/disabling command will be send even
  * if the feature was already enabled/disabled otherwise it will judge if
  * the feature changed status or the IC had a system reset
  * @return OK if success or an error code which specify the type of error
  */
static int fts_mode_handler(struct fts_ts_info *info, int force)
{
	int res = OK;
	int ret = OK;
	u8 settings[4] = { 0 };

	/* disable irq wake because resuming from gesture mode */
	if (IS_POWER_MODE(info->mode, SCAN_MODE_LOW_POWER) &&
	    (info->resume_bit == 1))
		disable_irq_wake(info->client->irq);

	info->mode = MODE_NOTHING;	/* initialize the mode to nothing
					  * in order to be updated depending
					  * on the features enabled */

	logError(0, "%s %s: Mode Handler starting...\n", tag, __func__);
	switch (info->resume_bit) {
	case 0:	/* screen down */
		logError(0, "%s %s: Screen OFF...\n", tag, __func__);
		/* do sense off in order to avoid the flooding of the fifo with
		  * touch events if someone is touching the panel during suspend
		  **/
		logError(0, "%s %s: Sense OFF!\n", tag, __func__);
		/* for speed reason (no need to check echo in this case and
		  * interrupt can be enabled) */
		ret = setScanMode(SCAN_MODE_ACTIVE, 0x00);
		res |= ret;	/* to avoid warning unsused ret variable when a
				  * ll the features are disabled */

#ifdef GESTURE_MODE
		if (info->gesture_enabled == 1) {
			logError(0, "%s %s: enter in gesture mode !\n", tag,
				 __func__);
			res = enterGestureMode(isSystemResettedDown());
			if (res >= OK) {
				enable_irq_wake(info->client->irq);
				fromIDtoMask(FEAT_SEL_GESTURE,
					     (u8 *)&info->mode,
					     sizeof(info->mode));
				MODE_LOW_POWER(info->mode, 0);
			} else
				logError(1,
					 "%s %s: enterGestureMode failed! ERROR %08X recovery in senseOff...\n",
					 tag, __func__, res);
		}
#endif

		setSystemResetedDown(0);
		break;

	case 1:	/* screen up */
		logError(0, "%s %s: Screen ON...\n", tag, __func__);

#ifdef GLOVE_MODE
		if ((info->glove_enabled == FEAT_ENABLE &&
		     isSystemResettedUp()) || force == 1) {
			logError(0, "%s %s: Glove Mode setting...\n", tag,
				 __func__);
			settings[0] = info->glove_enabled;
			/* required to satisfy also the disable case */
			ret = setFeatures(FEAT_SEL_GLOVE, settings, 1);
			if (ret < OK)
				logError(1,
					 "%s %s: error during setting GLOVE_MODE! ERROR %08X\n",
					 tag, __func__, ret);
			res |= ret;

			if (ret >= OK && info->glove_enabled == FEAT_ENABLE) {
				fromIDtoMask(FEAT_SEL_GLOVE, (u8 *)&info->mode,
					     sizeof(info->mode));
				logError(1, "%s %s: GLOVE_MODE Enabled!\n",
					 tag, __func__);
			} else
				logError(1, "%s %s: GLOVE_MODE Disabled!\n",
					 tag, __func__);
		}

#endif

#ifdef COVER_MODE
		if ((info->cover_enabled == FEAT_ENABLE &&
		     isSystemResettedUp()) || force == 1) {
			logError(0, "%s %s: Cover Mode setting...\n", tag,
				 __func__);
			settings[0] = info->cover_enabled;
			ret = setFeatures(FEAT_SEL_COVER, settings, 1);
			if (ret < OK)
				logError(1,
					 "%s %s: error during setting COVER_MODE! ERROR %08X\n",
					 tag, __func__, ret);
			res |= ret;

			if (ret >= OK && info->cover_enabled == FEAT_ENABLE) {
				fromIDtoMask(FEAT_SEL_COVER, (u8 *)&info->mode,
					     sizeof(info->mode));
				logError(1, "%s %s: COVER_MODE Enabled!\n",
					 tag, __func__);
			} else
				logError(1, "%s %s: COVER_MODE Disabled!\n",
					 tag, __func__);
		}
#endif
#ifdef CHARGER_MODE
		if ((info->charger_enabled > 0 && isSystemResettedUp()) ||
		    force == 1) {
			logError(0, "%s %s: Charger Mode setting...\n", tag,
				 __func__);

			settings[0] = info->charger_enabled;
			ret = setFeatures(FEAT_SEL_CHARGER, settings, 1);
			if (ret < OK)
				logError(1,
					 "%s %s: error during setting CHARGER_MODE! ERROR %08X\n",
					 tag, __func__, ret);
			res |= ret;

			if (ret >= OK && info->charger_enabled == FEAT_ENABLE) {
				fromIDtoMask(FEAT_SEL_CHARGER,
					     (u8 *)&info->mode,
					     sizeof(info->mode));
				logError(1, "%s %s: CHARGER_MODE Enabled!\n",
					 tag, __func__);
			} else
				logError(1, "%s %s: CHARGER_MODE Disabled!\n",
					 tag, __func__);
		}
#endif


#ifdef GRIP_MODE
		if ((info->grip_enabled == FEAT_ENABLE &&
		     isSystemResettedUp()) || force == 1) {
			logError(0, "%s %s: Grip Mode setting...\n", tag,
				 __func__);
			settings[0] = info->grip_enabled;
			ret = setFeatures(FEAT_SEL_GRIP, settings, 1);
			if (ret < OK)
				logError(1,
					 "%s %s: error during setting GRIP_MODE! ERROR %08X\n",
					 tag, __func__, ret);
			res |= ret;

			if (ret >= OK && info->grip_enabled == FEAT_ENABLE) {
				fromIDtoMask(FEAT_SEL_GRIP, (u8 *)&info->mode,
					     sizeof(info->mode));
				logError(1, "%s %s: GRIP_MODE Enabled!\n", tag,
					 __func__);
			} else
				logError(1, "%s %s: GRIP_MODE Disabled!\n",
					 tag, __func__);
		}
#endif
		/* If some selective scan want to be enabled can be done
		  * an or of the following options
		  */
		/* settings[0] = ACTIVE_MULTI_TOUCH | ACTIVE_KEY | */
		/*		ACTIVE_HOVER | ACTIVE_PROXIMITY | */
		/*		ACTIVE_FORCE; */
		settings[0] = 0xFF;	/* enable all the possible scans mode
					  * supported by the config */
		logError(0, "%s %s: Sense ON!\n", tag, __func__);
		res |= setScanMode(SCAN_MODE_ACTIVE, settings[0]);
		info->mode |= (SCAN_MODE_ACTIVE << 24);
		MODE_ACTIVE(info->mode, settings[0]);


		setSystemResetedUp(0);
		break;

	default:
		logError(1,
			 "%s %s: invalid resume_bit value = %d! ERROR %08X\n",
			 tag,
			 __func__, info->resume_bit, ERROR_OP_NOT_ALLOW);
		res = ERROR_OP_NOT_ALLOW;
	}


	logError(0, "%s %s: Mode Handler finished! res = %08X mode = %08X\n",
		 tag, __func__, res, info->mode);
	return res;
}

/**
  * Resume work function which perform a system reset, clean all the touches
  * from the linux input system and prepare the ground for enabling the sensing
  */
static void fts_resume_work(struct work_struct *work)
{
	struct fts_ts_info *info;

	info = container_of(work, struct fts_ts_info, resume_work);

	if (!info->sensor_sleep)
		return;

#ifdef CONFIG_TOUCHSCREEN_TBN
	if (info->tbn)
		tbn_request_bus(info->tbn);
#endif

	__pm_wakeup_event(&info->wakesrc, jiffies_to_msecs(HZ));

	info->resume_bit = 1;

	fts_system_reset();

	release_all_touches(info);

	fts_mode_handler(info, 0);

	info->sensor_sleep = false;

	fts_enableInterrupt();
}

/**
  * Suspend work function which clean all the touches from Linux input system
  * and prepare the ground to disabling the sensing or enter in gesture mode
  */
static void fts_suspend_work(struct work_struct *work)
{
	struct fts_ts_info *info;

	info = container_of(work, struct fts_ts_info, suspend_work);

	if (info->sensor_sleep)
		return;

	__pm_wakeup_event(&info->wakesrc, jiffies_to_msecs(HZ));

	info->resume_bit = 0;

	fts_mode_handler(info, 0);

	release_all_touches(info);

	info->sensor_sleep = true;

	fts_disableInterrupt();

#ifdef CONFIG_TOUCHSCREEN_TBN
	if (info->tbn)
		tbn_release_bus(info->tbn);
#endif
}
/** @}*/

/**
  * Callback function used to detect the suspend/resume events generated by
  * clicking the power button.
  * This function schedule a suspend or resume work according to the event
  * received.
  */
static int fts_screen_state_chg_callback(struct notifier_block *nb,
					 unsigned long val, void *data)
{
	struct fts_ts_info *info = container_of(nb, struct fts_ts_info,
						notifier);
	struct msm_drm_notifier *evdata = data;
	unsigned int blank;

	if (val != MSM_DRM_EVENT_BLANK)
		return NOTIFY_DONE;

	if (!info || !evdata || !evdata->data) {
		logError(0, "%s %s: Bad fts notifier call!\n", tag, __func__);
		return NOTIFY_DONE;
	}

	logError(0, "%s %s: fts notifier begin!\n", tag, __func__);

	/* finish processing any events on queue */
	flush_workqueue(info->event_wq);

	blank = *(int *) (evdata->data);
	switch (blank) {
	case MSM_DRM_BLANK_POWERDOWN:
	case MSM_DRM_BLANK_LP:
		if (info->sensor_sleep)
			break;
		logError(0, "%s %s: BLANK\n", tag, __func__);
		queue_work(info->event_wq, &info->suspend_work);
		break;
	case MSM_DRM_BLANK_UNBLANK:
		if (!info->sensor_sleep)
			break;
		logError(0, "%s %s: UNBLANK\n", tag, __func__);
		queue_work(info->event_wq, &info->resume_work);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block fts_noti_block = {
	.notifier_call = fts_screen_state_chg_callback,
};

/**
  * From the name of the power regulator get/put the actual regulator structs
  * (copying their references into fts_ts_info variable)
  * @param info pointer to fts_ts_info which contains info about the device and
  * its hw setup
  * @param get if 1, the regulators are get otherwise they are put (released)
  * back to the system
  * @return OK if success or an error code which specify the type of error
  */
static int fts_get_reg(struct fts_ts_info *info, bool get)
{
	int retval;
	const struct fts_hw_platform_data *bdata = info->board;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}

	if ((bdata->vdd_reg_name != NULL) && (*bdata->vdd_reg_name != 0)) {
		info->vdd_reg = regulator_get(info->dev, bdata->vdd_reg_name);
		if (IS_ERR(info->vdd_reg)) {
			logError(1, "%s %s: Failed to get power regulator\n",
				 tag,
				 __func__);
			retval = PTR_ERR(info->vdd_reg);
			goto regulator_put;
		}
	}

	if ((bdata->avdd_reg_name != NULL) && (*bdata->avdd_reg_name != 0)) {
		info->avdd_reg = regulator_get(info->dev, bdata->avdd_reg_name);
		if (IS_ERR(info->avdd_reg)) {
			logError(1,
				 "%s %s: Failed to get bus pullup regulator\n",
				 tag,
				 __func__);
			retval = PTR_ERR(info->avdd_reg);
			goto regulator_put;
		}
	}

	return OK;

regulator_put:
	if (info->vdd_reg) {
		regulator_put(info->vdd_reg);
		info->vdd_reg = NULL;
	}

	if (info->avdd_reg) {
		regulator_put(info->avdd_reg);
		info->avdd_reg = NULL;
	}

	return retval;
}


/**
  * Enable or disable the power regulators
  * @param info pointer to fts_ts_info which contains info about the device and
  * its hw setup
  * @param enable if 1, the power regulators are turned on otherwise they are
  * turned off
  * @return OK if success or an error code which specify the type of error
  */
static int fts_enable_reg(struct fts_ts_info *info, bool enable)
{
	int retval;

	if (!enable) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (info->vdd_reg) {
		retval = regulator_enable(info->vdd_reg);
		if (retval < 0) {
			logError(1, "%s %s: Failed to enable bus regulator\n",
				 tag, __func__);
			goto exit;
		}
	}

	if (info->avdd_reg) {
		retval = regulator_enable(info->avdd_reg);
		if (retval < 0) {
			logError(1, "%s %s: Failed to enable power regulator\n",
				 tag, __func__);
			goto disable_bus_reg;
		}
	}

	return OK;

disable_pwr_reg:
	if (info->avdd_reg)
		regulator_disable(info->avdd_reg);

disable_bus_reg:
	if (info->vdd_reg)
		regulator_disable(info->vdd_reg);

exit:
	return retval;
}

/**
  * Configure a GPIO according to the parameters
  * @param gpio gpio number
  * @param config if true, the gpio is set up otherwise it is free
  * @param dir direction of the gpio, 0 = in, 1 = out
  * @param state initial value (if the direction is in, this parameter is
  * ignored)
  * return error code
  */
static int fts_gpio_setup(int gpio, bool config, int dir, int state)
{
	int retval = 0;
	unsigned char buf[16];

	if (config) {
		scnprintf(buf, sizeof(buf), "fts_gpio_%u\n", gpio);

		retval = gpio_request(gpio, buf);
		if (retval) {
			logError(1, "%s %s: Failed to get gpio %d (code: %d)",
				 tag,
				 __func__, gpio, retval);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval) {
			logError(1, "%s %s: Failed to set gpio %d direction",
				 tag,
				 __func__, gpio);
			return retval;
		}
	} else
		gpio_free(gpio);

	return retval;
}

/**
  * Setup the IRQ and RESET (if present) gpios.
  * If the Reset Gpio is present it will perform a cycle HIGH-LOW-HIGH in order
  * to assure that the IC has been reset properly
  */
static int fts_set_gpio(struct fts_ts_info *info)
{
	int retval;
	struct fts_hw_platform_data *bdata =
		info->board;

	retval = fts_gpio_setup(bdata->irq_gpio, true, 0, 0);
	if (retval < 0) {
		logError(1, "%s %s: Failed to configure irq GPIO\n", tag,
			 __func__);
		goto err_gpio_irq;
	}

	if (gpio_is_valid(bdata->switch_gpio)) {
		retval = fts_gpio_setup(bdata->switch_gpio, true, 1, 1);
		if (retval < 0)
			logError(1, "%s %s: Failed to configure I2C switch\n",
				 tag, __func__);
	}

	if (bdata->reset_gpio >= 0) {
		retval = fts_gpio_setup(bdata->reset_gpio, true, 1, 0);
		if (retval < 0) {
			logError(1, "%s %s: Failed to configure reset GPIO\n",
				 tag, __func__);
			goto err_gpio_reset;
		}
	}
	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, 0);
		mdelay(10);
		gpio_set_value(bdata->reset_gpio, 1);
	}

	return OK;

err_gpio_reset:
	fts_gpio_setup(bdata->irq_gpio, false, 0, 0);
	bdata->reset_gpio = GPIO_NOT_DEFINED;
err_gpio_irq:
	return retval;
}

/**
  * Retrieve and parse the hw information from the device tree node defined in
  * the system.
  * the most important information to obtain are: IRQ and RESET gpio numbers,
  * power regulator names
  * In the device file node is possible to define additional optional
  * information
  * that can be parsed here.
  */
static int parse_dt(struct device *dev, struct fts_hw_platform_data *bdata)
{
	int retval;
	const char *name;
	struct device_node *np = dev->of_node;
	u32 coords[2];

	bdata->switch_gpio = of_get_named_gpio(np, "st,switch_gpio", 0);
	logError(0, "%s switch_gpio = %d\n", tag, bdata->switch_gpio);

	bdata->irq_gpio = of_get_named_gpio_flags(np, "st,irq-gpio", 0, NULL);

	logError(0, "%s irq_gpio = %d\n", tag, bdata->irq_gpio);


	retval = of_property_read_string(np, "st,regulator_dvdd", &name);
	if (retval == -EINVAL)
		bdata->vdd_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else {
		bdata->vdd_reg_name = name;
		logError(0, "%s pwr_reg_name = %s\n", tag, name);
	}

	retval = of_property_read_string(np, "st,regulator_avdd", &name);
	if (retval == -EINVAL)
		bdata->avdd_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else {
		bdata->avdd_reg_name = name;
		logError(0, "%s bus_reg_name = %s\n", tag, name);
	}

	if (of_property_read_bool(np, "st,reset-gpio")) {
		bdata->reset_gpio = of_get_named_gpio_flags(np,
							    "st,reset-gpio", 0,
							    NULL);
		logError(0, "%s reset_gpio =%d\n", tag, bdata->reset_gpio);
	} else
		bdata->reset_gpio = GPIO_NOT_DEFINED;

	retval = of_property_read_string(np, "st,firmware_name", &name);
	if (retval == -EINVAL)
		bdata->fw_name = PATH_FILE_FW;
	else if (retval >= 0)
		bdata->fw_name = name;
	logError(0, "%s firmware name = %s\n", tag, bdata->fw_name);

	if (of_property_read_u32_array(np, "st,max-coords", coords, 2)) {
		logError(0, "%s st,max-coords not found, using 1440x2560\n");
		coords[0] = 1440;
		coords[1] = 2560;
	}
	bdata->x_axis_max = coords[0];
	bdata->y_axis_max = coords[1];

	bdata->coord_factor = 0;
	if (!of_property_read_u32_array(np, "st,max-real-coords", coords, 2)) {
		bdata->coord_factor = 1;
		bdata->x_axis_real_max = coords[0];
		bdata->y_axis_real_max = coords[1];

		if (!of_property_read_u32_array(np, "st,edge-offset", coords, 2)) {
			bdata->x_axis_edge_offset = coords[0];
			bdata->y_axis_edge_offset = coords[1];
		} else {
			bdata->x_axis_edge_offset = 0;
			bdata->y_axis_edge_offset = 0;
		}
	}

	return OK;
}

/**
  * Probe function, called when the driver it is matched with a device
  * with the same name compatible name
  * This function allocate, initialize all the most important functions and flow
  * those are used by the driver to operate with the IC.
  * It allocates device variables, initialize queues and schedule works,
  * registers the IRQ handler, suspend/resume callbacks, registers the device
  * to the linux input subsystem etc.
  */
#ifdef I2C_INTERFACE
static int fts_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
#else
static int fts_probe(struct spi_device *client)
{
#endif

	struct fts_ts_info *info = NULL;
	int error = 0;
	struct device_node *dp = client->dev.of_node;
	int retval;
	int skip_5_1 = 0;
	u16 bus_type;

	logError(1, "%s %s: driver probe begin!\n", tag, __func__);
	logError(1, "%s driver ver. %s\n", tag, FTS_TS_DRV_VERSION);

	logError(1, "%s SET Bus Functionality :\n", tag);
#ifdef I2C_INTERFACE
	logError(1, "%s I2C interface...\n", tag);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		logError(1, "%s Unsupported I2C functionality\n", tag);
		error = -EIO;
		goto ProbeErrorExit_0;
	}

	logError(1, "%s i2c address: %x\n", tag, client->addr);
	bus_type = BUS_I2C;
#else
	logError(1, "%s SPI interface...\n", tag);
	client->mode = SPI_MODE_0;
#ifndef SPI4_WIRE
	client->mode |= SPI_3WIRE;
#endif

	client->max_speed_hz = SPI_CLOCK_FREQ;
	client->bits_per_word = 8;
	if (spi_setup(client) < 0) {
		logError(1, "%s Unsupported SPI functionality\n", tag);
		error = -EIO;
		goto ProbeErrorExit_0;
	}
	bus_type = BUS_SPI;
#endif


	logError(1, "%s SET Device driver INFO:\n", tag);


	info = kzalloc(sizeof(struct fts_ts_info), GFP_KERNEL);
	if (!info) {
		logError(1,
			 "%s Out of memory... Impossible to allocate struct info!\n",
			 tag);
		error = -ENOMEM;
		goto ProbeErrorExit_0;
	}

	info->client = client;
	info->dev = &info->client->dev;

	dev_set_drvdata(info->dev, info);

#ifdef CONFIG_TOUCHSCREEN_TBN
	info->tbn = tbn_init(info->dev);
	if (!info->tbn) {
		logError(1, "%s ERROR: failed to init tbn context\n", tag);
		error = -ENODEV;
		goto ProbeErrorExit_1;
	}
#endif

	if (dp) {
		info->board = devm_kzalloc(&client->dev,
					   sizeof(struct fts_hw_platform_data),
					   GFP_KERNEL);
		if (!info->board) {
			logError(1, "%s ERROR:info.board kzalloc failed\n",
				 tag);
			goto ProbeErrorExit_1;
		}
		parse_dt(&client->dev, info->board);
	}

	logError(1, "%s SET Regulators:\n", tag);
	retval = fts_get_reg(info, true);
	if (retval < 0) {
		logError(1, "%s ERROR: %s: Failed to get regulators\n", tag,
			 __func__);
		goto ProbeErrorExit_1;
	}

	retval = fts_enable_reg(info, true);
	if (retval < 0) {
		logError(1, "%s %s: ERROR Failed to enable regulators\n", tag,
			 __func__);
		goto ProbeErrorExit_2;
	}

	logError(1, "%s SET GPIOS:\n", tag);
	retval = fts_set_gpio(info);
	if (retval < 0) {
		logError(1, "%s %s: ERROR Failed to set up GPIO's\n", tag,
			 __func__);
		goto ProbeErrorExit_2;
	}
	info->client->irq = gpio_to_irq(info->board->irq_gpio);

	logError(1, "%s SET Event Handler:\n", tag);

	wakeup_source_init(&info->wakesrc, "fts_tp");
	info->event_wq = alloc_workqueue("fts-event-queue", WQ_UNBOUND |
					 WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!info->event_wq) {
		logError(1, "%s ERROR: Cannot create work thread\n", tag);
		error = -ENOMEM;
		goto ProbeErrorExit_4;
	}

	INIT_WORK(&info->work, fts_event_handler);

	INIT_WORK(&info->resume_work, fts_resume_work);
	INIT_WORK(&info->suspend_work, fts_suspend_work);

	logError(1, "%s SET Input Device Property:\n", tag);
	info->dev = &info->client->dev;
	info->input_dev = input_allocate_device();
	if (!info->input_dev) {
		logError(1, "%s ERROR: No such input device defined!\n", tag);
		error = -ENODEV;
		goto ProbeErrorExit_5;
	}
	info->input_dev->dev.parent = &client->dev;
	info->input_dev->name = FTS_TS_DRV_NAME;
	scnprintf(fts_ts_phys, sizeof(fts_ts_phys), "%s/input0",
		 info->input_dev->name);
	info->input_dev->phys = fts_ts_phys;
	info->input_dev->id.bustype = bus_type;
	info->input_dev->id.vendor = 0x0001;
	info->input_dev->id.product = 0x0002;
	info->input_dev->id.version = 0x0100;

	__set_bit(EV_SYN, info->input_dev->evbit);
	__set_bit(EV_KEY, info->input_dev->evbit);
	__set_bit(EV_ABS, info->input_dev->evbit);
	__set_bit(BTN_TOUCH, info->input_dev->keybit);
	/* __set_bit(BTN_TOOL_FINGER, info->input_dev->keybit); */
	/* __set_bit(BTN_TOOL_PEN, info->input_dev->keybit); */

	input_mt_init_slots(info->input_dev, TOUCH_ID_MAX, INPUT_MT_DIRECT);

	/* input_mt_init_slots(info->input_dev, TOUCH_ID_MAX); */

	input_set_abs_params(info->input_dev, ABS_MT_POSITION_X, X_AXIS_MIN,
			     info->board->x_axis_max, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y, Y_AXIS_MIN,
			     info->board->y_axis_max, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR, AREA_MIN,
			     AREA_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MINOR, AREA_MIN,
			     AREA_MAX, 0, 0);
#ifndef SKIP_PRESSURE
	input_set_abs_params(info->input_dev, ABS_MT_PRESSURE, PRESSURE_MIN,
			     PRESSURE_MAX, 0, 0);
#endif
#ifndef SKIP_DISTANCE
	input_set_abs_params(info->input_dev, ABS_MT_DISTANCE, DISTANCE_MIN,
			     DISTANCE_MAX, 0, 0);
#endif

#ifdef GESTURE_MODE
	input_set_capability(info->input_dev, EV_KEY, KEY_WAKEUP);

	input_set_capability(info->input_dev, EV_KEY, KEY_M);
	input_set_capability(info->input_dev, EV_KEY, KEY_O);
	input_set_capability(info->input_dev, EV_KEY, KEY_E);
	input_set_capability(info->input_dev, EV_KEY, KEY_W);
	input_set_capability(info->input_dev, EV_KEY, KEY_C);
	input_set_capability(info->input_dev, EV_KEY, KEY_L);
	input_set_capability(info->input_dev, EV_KEY, KEY_F);
	input_set_capability(info->input_dev, EV_KEY, KEY_V);
	input_set_capability(info->input_dev, EV_KEY, KEY_S);
	input_set_capability(info->input_dev, EV_KEY, KEY_Z);
	input_set_capability(info->input_dev, EV_KEY, KEY_WWW);

	input_set_capability(info->input_dev, EV_KEY, KEY_LEFT);
	input_set_capability(info->input_dev, EV_KEY, KEY_RIGHT);
	input_set_capability(info->input_dev, EV_KEY, KEY_UP);
	input_set_capability(info->input_dev, EV_KEY, KEY_DOWN);

	input_set_capability(info->input_dev, EV_KEY, KEY_F1);
	input_set_capability(info->input_dev, EV_KEY, KEY_F2);
	input_set_capability(info->input_dev, EV_KEY, KEY_F3);
	input_set_capability(info->input_dev, EV_KEY, KEY_F4);
	input_set_capability(info->input_dev, EV_KEY, KEY_F5);

	input_set_capability(info->input_dev, EV_KEY, KEY_LEFTBRACE);
	input_set_capability(info->input_dev, EV_KEY, KEY_RIGHTBRACE);
#endif

#ifdef PHONE_KEY
	/* KEY associated to the touch screen buttons */
	input_set_capability(info->input_dev, EV_KEY, KEY_HOMEPAGE);
	input_set_capability(info->input_dev, EV_KEY, KEY_BACK);
	input_set_capability(info->input_dev, EV_KEY, KEY_MENU);
#endif

	mutex_init(&(info->input_report_mutex));


#ifdef GESTURE_MODE
	mutex_init(&gestureMask_mutex);
#endif

	spin_lock_init(&fts_int);

	/* register the multi-touch input device */
	error = input_register_device(info->input_dev);
	if (error) {
		logError(1, "%s ERROR: No such input device\n", tag);
		error = -ENODEV;
		goto ProbeErrorExit_5_1;
	}

	skip_5_1 = 1;
	/* track slots */
	info->touch_id = 0;
#ifdef STYLUS_MODE
	info->stylus_id = 0;
#endif


	/* init feature switches (by default all the features are disable,
	  * if one feature want to be enabled from the start,
	  * set the corresponding value to 1)*/
	info->gesture_enabled = 0;
	info->glove_enabled = 0;
	info->charger_enabled = 0;
	info->cover_enabled = 0;
	info->grip_enabled = 0;

	info->resume_bit = 1;
	info->notifier = fts_noti_block;

	logError(1, "%s Init Core Lib:\n", tag);
	initCore(info);
	/* init hardware device */
	logError(1, "%s Device Initialization:\n", tag);
	error = fts_init(info);
	if (error < OK) {
		logError(1, "%s Cannot initialize the device ERROR %08X\n", tag,
			 error);
		error = -ENODEV;
		goto ProbeErrorExit_6;
	}

#if defined(FW_UPDATE_ON_PROBE) && defined(FW_H_FILE)
	logError(1, "%s FW Update and Sensing Initialization:\n", tag);
	error = fts_fw_update(info);
	if (error < OK) {
		logError(1,
			 "%s Cannot execute fw upgrade the device ERROR %08X\n",
			 tag,
			 error);
		error = -ENODEV;
		goto ProbeErrorExit_7;
	}

#else
	logError(1, "%s SET Auto Fw Update:\n", tag);
	info->fwu_workqueue = alloc_workqueue("fts-fwu-queue", WQ_UNBOUND |
					      WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!info->fwu_workqueue) {
		logError(1, "%s ERROR: Cannot create fwu work thread\n", tag);
		goto ProbeErrorExit_7;
	}
	INIT_DELAYED_WORK(&info->fwu_work, fts_fw_update_auto);
#endif

	logError(1, "%s SET Device File Nodes:\n", tag);
	/* sysfs stuff */
	info->attrs.attrs = fts_attr_group;
	error = sysfs_create_group(&client->dev.kobj, &info->attrs);
	if (error) {
		logError(1, "%s ERROR: Cannot create sysfs structure!\n", tag);
		error = -ENODEV;
		goto ProbeErrorExit_7;
	}

	error = fts_proc_init();
	if (error < OK)
		logError(1, "%s Error: can not create /proc file!\n", tag);

#ifndef FW_UPDATE_ON_PROBE
	queue_delayed_work(info->fwu_workqueue, &info->fwu_work,
			   msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
#endif

	logError(1, "%s Probe Finished!\n", tag);
	return OK;


ProbeErrorExit_7:
#ifdef FW_UPDATE_ON_PROBE
	msm_drm_unregister_client(&info->notifier);
#endif

ProbeErrorExit_6:
	input_unregister_device(info->input_dev);

ProbeErrorExit_5_1:
	if (skip_5_1 != 1)
		input_free_device(info->input_dev);

ProbeErrorExit_5:
	destroy_workqueue(info->event_wq);

ProbeErrorExit_4:
	/* destroy_workqueue(info->fwu_workqueue); */
	wakeup_source_trash(&info->wakesrc);

	fts_enable_reg(info, false);

ProbeErrorExit_2:
	fts_get_reg(info, false);

ProbeErrorExit_1:
	kfree(info);

ProbeErrorExit_0:
	logError(1, "%s Probe Failed!\n", tag);

	return error;
}


/**
  * Clear and free all the resources associated to the driver.
  * This function is called when the driver need to be removed.
  */
#ifdef I2C_INTERFACE
static int fts_remove(struct i2c_client *client)
{
#else
static int fts_remove(struct spi_device *client)
{
#endif

	struct fts_ts_info *info = dev_get_drvdata(&(client->dev));


#ifdef CONFIG_TOUCHSCREEN_TBN
	tbn_cleanup(info->tbn);
#endif

	fts_proc_remove();

	/* sysfs stuff */
	sysfs_remove_group(&client->dev.kobj, &info->attrs);

	/* remove interrupt and event handlers */
	fts_interrupt_uninstall(info);

	msm_drm_unregister_client(&info->notifier);

	/* unregister the device */
	input_unregister_device(info->input_dev);

	/* input_free_device(info->input_dev ); */

	/* Remove the work thread */
	destroy_workqueue(info->event_wq);
	wakeup_source_trash(&info->wakesrc);
#ifndef FW_UPDATE_ON_PROBE
	destroy_workqueue(info->fwu_workqueue);
#endif

	fts_enable_reg(info, false);
	fts_get_reg(info, false);

	/* free all */
	kfree(info);

	return OK;
}

/**
  * Struct which contains the compatible names that need to match with
  * the definition of the device in the device tree node
  */
static struct of_device_id fts_of_match_table[] = {
	{
		.compatible = "st,fts",
	},
	{},
};

#ifdef I2C_INTERFACE
static const struct i2c_device_id fts_device_id[] = {
	{ FTS_TS_DRV_NAME, 0 },
	{}
};

static struct i2c_driver fts_i2c_driver = {
	.driver			= {
		.name		= FTS_TS_DRV_NAME,
		.of_match_table = fts_of_match_table,
	},
	.probe			= fts_probe,
	.remove			= fts_remove,
	.id_table		= fts_device_id,
};
#else
static struct spi_driver fts_spi_driver = {
	.driver			= {
		.name		= FTS_TS_DRV_NAME,
		.of_match_table = fts_of_match_table,
		.owner		= THIS_MODULE,
	},
	.probe			= fts_probe,
	.remove			= fts_remove,
};
#endif




static int __init fts_driver_init(void)
{
#ifdef I2C_INTERFACE
	return i2c_add_driver(&fts_i2c_driver);
#else
	return spi_register_driver(&fts_spi_driver);
#endif
}

static void __exit fts_driver_exit(void)
{
#ifdef I2C_INTERFACE
	i2c_del_driver(&fts_i2c_driver);
#else
	spi_unregister_driver(&fts_spi_driver);
#endif
}


MODULE_DESCRIPTION("STMicroelectronics MultiTouch IC Driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");

late_initcall(fts_driver_init);
module_exit(fts_driver_exit);
