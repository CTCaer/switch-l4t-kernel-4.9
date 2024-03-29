/*
 * STMicroelectronics st_lsm6dsx sensor driver
 *
 * Copyright 2016 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 * Denis Ciocca <denis.ciocca@st.com>
 *
 * Copyright 2020-2022, CTCaer
 *
 * Licensed under the GPL-2.
 */

#ifndef ST_LSM6DSX_H
#define ST_LSM6DSX_H

#include <linux/device.h>
#include <linux/iio/iio.h>

#define ST_LSM6DS3_DEV_NAME	"lsm6ds3"
#define ST_LSM6DS3H_DEV_NAME	"lsm6ds3h"
#define ST_LSM6DSE_DEV_NAME	"lsm6dse"
#define ST_LSM6DSL_DEV_NAME	"lsm6dsl"
#define ST_LSM6DSM_DEV_NAME	"lsm6dsm"
#define ST_LSM6DSO_DEV_NAME	"lsm6dso"
#define ST_LSM6DSOX_DEV_NAME	"lsm6dsox"
#define ST_LSM6DSOP_DEV_NAME	"lsm6dsop"
#define ST_ISM330DLC_DEV_NAME	"ism330dlc"

enum st_lsm6dsx_hw_id {
	ST_LSM6DS3_ID,
	ST_LSM6DS3H_ID,
	ST_LSM6DSE_ID,
	ST_LSM6DSL_ID,
	ST_LSM6DSM_ID,
	ST_LSM6DSO_ID,
	ST_LSM6DSOX_ID,
	ST_LSM6DSOP_ID,
	ST_ISM330DLC_ID,
	ST_LSM6DSX_MAX_ID,
};

#define ST_LSM6DSX_BUFF_SIZE		400
#define ST_LSM6DSX_CHAN_SIZE		2
#define ST_LSM6DSX_SAMPLE_SIZE		6
#define ST_LSM6DSX_MAX_WORD_LEN		((32 / ST_LSM6DSX_SAMPLE_SIZE) * \
					 ST_LSM6DSX_SAMPLE_SIZE)
#define ST_LSM6DSX_SHIFT_VAL(val, mask)	(((val) << __ffs(mask)) & (mask))

struct st_lsm6dsx_reg {
	u8 addr;
	u8 mask;
};

/**
 * struct st_lsm6dsx_fifo_ops - ST IMU FIFO settings
 * @fifo_th: FIFO threshold register info (addr + mask).
 * @fifo_diff: FIFO diff status register info (addr + mask).
 * @th_wl: FIFO threshold word length.
 */
struct st_lsm6dsx_fifo_ops {
	struct {
		u8 addr;
		u16 mask;
	} fifo_th;
	struct {
		u8 addr;
		u16 mask;
	} fifo_diff;
	u8 th_wl;
};

/**
 * struct st_lsm6dsx_hw_ts_settings - ST IMU hw timer settings
 * @timer_en: Hw timer enable register info (addr + mask).
 * @hr_timer: Hw timer resolution register info (addr + mask).
 * @fifo_en: Hw timer FIFO enable register info (addr + mask).
 * @decimator: Hw timer FIFO decimator register info (addr + mask).
 * @freq_fine: Difference in % of ODR with respect to the typical.
 */
struct st_lsm6dsx_hw_ts_settings {
	struct st_lsm6dsx_reg timer_en;
	struct st_lsm6dsx_reg hr_timer;
	struct st_lsm6dsx_reg fifo_en;
	struct st_lsm6dsx_reg decimator;
	u8 freq_fine;
};

/**
 * struct st_lsm6dsx_settings - ST IMU sensor settings
 * @wai: Sensor WhoAmI default value.
 * @max_fifo_size: Sensor max fifo length in FIFO words.
 * @id: List of hw id supported by the driver configuration.
 * @decimator: List of decimator register info (addr + mask).
 * @fifo_ops: Sensor hw FIFO parameters.
 * @ts_settings: Hw timer related settings.
 */
struct st_lsm6dsx_settings {
	u8 wai;
	u16 max_fifo_size;
	enum st_lsm6dsx_hw_id id[ST_LSM6DSX_MAX_ID];
	struct st_lsm6dsx_reg decimator[ST_LSM6DSX_MAX_ID];
	struct st_lsm6dsx_fifo_ops fifo_ops;
	struct st_lsm6dsx_hw_ts_settings ts_settings;
};

enum st_lsm6dsx_sensor_id {
	ST_LSM6DSX_ID_ACC,
	ST_LSM6DSX_ID_GYRO,
	ST_LSM6DSX_ID_MAX,
};

enum st_lsm6dsx_fifo_mode {
	ST_LSM6DSX_FIFO_BYPASS = 0x0,
	ST_LSM6DSX_FIFO_CONT = 0x6,
};

/**
 * struct st_lsm6dsx_sensor - ST IMU sensor instance
 * @name: Sensor name.
 * @id: Sensor identifier.
 * @hw: Pointer to instance of struct st_lsm6dsx_hw.
 * @gain: Configured sensor sensitivity.
 * @odr: Output data rate of the sensor [Hz].
 * @watermark: Sensor watermark level.
 * @sip: Number of samples in a given pattern.
 * @decimator: FIFO decimation factor.
 * @ts_ref: Sensor timestamp reference for hw one.
 */
struct st_lsm6dsx_sensor {
	char name[32];
	enum st_lsm6dsx_sensor_id id;
	struct st_lsm6dsx_hw *hw;

	u32 gain;
	u16 odr;

	u16 watermark;
	u8 sip;
	u8 decimator;
	s64 ts_ref;
};

/**
 * struct st_lsm6dsx_hw - ST IMU MEMS hw instance
 * @dev: Pointer to instance of struct device (I2C or SPI).
 * @regmap: Register map of the device.
 * @irq: Device interrupt line (I2C or SPI).
 * @fifo_lock: Mutex to prevent concurrent access to the hw FIFO.
 * @conf_lock: Mutex to prevent concurrent FIFO configuration update.
 * @fifo_mode: FIFO operating mode supported by the device.
 * @enable_mask: Enabled sensor bitmask.
 * @ts_gain: Hw timestamp rate after internal calibration.
 * @ts_sip: Total number of timestamp samples in a given pattern.
 * @sip: Total number of samples (acc/gyro/ts) in a given pattern.
 * @buff: Device read buffer.
 * @iio_devs: Pointers to acc/gyro iio_dev instances.
 * @settings: Pointer to the specific sensor settings in use.
 */
struct st_lsm6dsx_hw {
	struct device *dev;
	struct regmap *regmap;
	int irq;

	struct mutex fifo_lock;
	struct mutex conf_lock;

	enum st_lsm6dsx_fifo_mode fifo_mode;
	u8 enable_mask;
	s64 ts_gain;
	u8 ts_sip;
	u8 sip;

	u8 *buff;

	struct iio_dev *iio_devs[ST_LSM6DSX_ID_MAX];
	struct iio_mount_matrix orientation;

	const struct st_lsm6dsx_settings *settings;
};

extern const struct dev_pm_ops st_lsm6dsx_pm_ops;

int st_lsm6dsx_probe(struct device *dev, int irq, int hw_id, const char *name,
		     struct regmap *regmap);
void st_lsm6dsx_shutdown(struct device *dev);
int st_lsm6dsx_sensor_enable(struct st_lsm6dsx_sensor *sensor);
int st_lsm6dsx_sensor_disable(struct st_lsm6dsx_sensor *sensor);
int st_lsm6dsx_fifo_setup(struct st_lsm6dsx_hw *hw);
int st_lsm6dsx_update_watermark(struct st_lsm6dsx_sensor *sensor,
				u16 watermark);
int st_lsm6dsx_flush_fifo(struct st_lsm6dsx_hw *hw);
int st_lsm6dsx_set_fifo_mode(struct st_lsm6dsx_hw *hw,
			     enum st_lsm6dsx_fifo_mode fifo_mode);
const struct iio_mount_matrix *
st_lsm6dsx_get_mount_matrix(struct iio_dev *iio_dev,
			      const struct iio_chan_spec *chan);

#endif /* ST_LSM6DSX_H */
