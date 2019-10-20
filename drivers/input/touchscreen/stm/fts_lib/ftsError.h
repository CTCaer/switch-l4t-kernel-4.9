/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				 **
  **************************************************************************
  *                                                                        *
  *                  FTS error/info kernel log reporting		  *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsError.h
  * \brief Contains all the definitions and structs which refer to Error
  * conditions
  */

#ifndef FTS_ERROR_H
#define FTS_ERROR_H

#include "ftsHardware.h"
#include "ftsSoftware.h"


/** @defgroup error_codes Error Codes
  * Error codes that can be reported by the driver functions.
  * An error code is made up by 4 bytes, each byte indicate a logic error
  * level.\n
  * From the LSB to the MSB, the logic level increase going from a low level
  * error (I2C,TIMEOUT) to an high level error (flashing procedure fail,
  * production test fail etc)
  * @{
  */

/* FIRST LEVEL ERROR CODE */
/** @defgroup first_level	First Level Error Code
  * @ingroup error_codes
  * Errors related to low level operation which are not under control of driver,
  * such as: communication protocol (I2C/SPI), timeout, file operations ...
  * @{
  */
#define OK			(0x00000000)	/* /< No ERROR */
#define ERROR_ALLOC		(0x80000001)	/* /< allocation of memory
						 * failed */
#define ERROR_BUS_R		(0x80000002)	/* /< i2c/spi read failed */
#define ERROR_BUS_W		(0x80000003)	/* /< i2c/spi write failed */
#define ERROR_BUS_WR		(0x80000004)	/* /< i2c/spi write/read
						 * failed */
#define ERROR_BUS_O		(0x80000005)	/* /< error during
						 * opening an i2c device */
#define ERROR_OP_NOT_ALLOW	(0x80000006)	/* /< operation not allowed */
#define ERROR_TIMEOUT		(0x80000007)	/* /< timeout expired!
						 * exceed the max number
						 * of retries or the max
						 * waiting time */
#define ERROR_FILE_NOT_FOUND	(0x80000008)	/* /< the file that i
						 * want to open is not found */
#define ERROR_FILE_PARSE	(0x80000009)	/* /< error during parsing
						 * the file */
#define ERROR_FILE_READ		(0x8000000A)	/* /< error during
						 * reading the file */
#define ERROR_LABEL_NOT_FOUND	(0x8000000B)	/* /< label not found */
#define ERROR_FW_NO_UPDATE	(0x8000000C)	/* /< fw in the chip
						 * newer than the one in
						 * the memmh */
#define ERROR_FLASH_UNKNOWN	(0x8000000D)	/* /< flash status busy
						 * or unknown */
/** @}*/

/* SECOND LEVEL ERROR CODE */
/** @defgroup second_level Second Level Error Code
  * @ingroup error_codes
  * Errors related to simple logic operations in the IC which require one
  * command or which are part of a more complex procedure
  * @{
  */
#define ERROR_DISABLE_INTER		(0x80000200)	/* /< unable to
							 * disable the
							 * interrupt */
#define ERROR_ENABLE_INTER		(0x80000300)	/* /< unable to activate
							 * the interrupt */
#define ERROR_READ_CONFIG		(0x80000400)	/* /< failed to read
							 * config memory */
#define ERROR_GET_OFFSET		(0x80000500)	/* /< unable to
							 * read an offset from
							 * memory */
#define ERROR_GET_FRAME_DATA		(0x80000600)	/* /< unable to
							 * retrieve the data of
							 * a required frame */
#define ERROR_DIFF_DATA_TYPE		(0x80000700)	/* /< FW answers
							 * with an event that
							 * has a different
							 * address respect the
							 * request done */
#define ERROR_WRONG_DATA_SIGN		(0x80000800)	/* /< the signature of
							 * the host data is not
							 * HEADER_SIGNATURE */
#define ERROR_SET_SCAN_MODE_FAIL	(0x80000900)	/* /< setting
							 * the scanning mode
							 * failed
							 * (sense on/off
							 * etc...) */
#define ERROR_SET_FEATURE_FAIL		(0x80000A00)	/* /< setting a specific
							 * feature failed */
#define ERROR_SYSTEM_RESET_FAIL		(0x80000B00)	/* /< the command
							 * SYSTEM RESET
							 * failed */
#define ERROR_FLASH_NOT_READY		(0x80000C00)	/* /< flash status not
							 * ready within a
							 * timeout */
#define ERROR_FW_VER_READ		(0x80000D00)	/* /< unable to retrieve
							 * fw_vers or the
							 * config_id */
#define ERROR_GESTURE_ENABLE_FAIL	(0x80000E00)	/* /< unable to
							 * enable/disable
							 * the gesture */
#define ERROR_GESTURE_START_ADD		(0x80000F00)	/* /< unable to start to
							 * add custom gesture */
#define ERROR_GESTURE_FINISH_ADD	(0x80001000)	/* /< unable to finish
							 * to add custom gesture
							 */
#define ERROR_GESTURE_DATA_ADD		(0x80001100)	/* /< unable to add
							 * custom gesture data
							 * */
#define ERROR_GESTURE_REMOVE		(0x80001200)	/* /< unable to remove
							 * custom gesture data
							 * */
#define ERROR_FEATURE_ENABLE_DISABLE	(0x80001300)	/* /< unable to
							 * enable/disable a
							 * feature mode in
							 * the IC */
#define ERROR_NOISE_PARAMETERS		(0x80001400)	/* /< unable to set/read
							 * noise parameter in
							 * the IC */
#define ERROR_CH_LEN			(0x80001500)	/* /< unable to retrieve
							 * the force and/or
							 * sense length */
/** @}*/

/* THIRD LEVEL ERROR CODE */
/** @defgroup third_level	Third Level Error Code
  * @ingroup error_codes
  * Errors related to logic operations in the IC which require more
  * commands/steps or which are part of a more complex procedure
  * @{
  */
#define ERROR_REQU_COMP_DATA	(0x80010000)	/* /< compensation data
						 * request failed */
#define ERROR_REQU_DATA		(0x80020000)	/* /< data request
						 * failed */
#define ERROR_COMP_DATA_HEADER	(0x80030000)	/* /< unable to retrieve
						 * the compensation data
						 * header */
#define ERROR_COMP_DATA_GLOBAL	(0x80040000)	/* /< unable to retrieve
						 * the global
						 * compensation data */
#define ERROR_COMP_DATA_NODE	(0x80050000)	/* /< unable to retrieve
						 * the compensation data
						 * for each node */
#define ERROR_TEST_CHECK_FAIL	(0x80060000) /* /< check of
					      * production limits or
					      * of fw answers failed */
#define ERROR_MEMH_READ		(0x80070000)	/* /< memh reading
						 * failed */
#define ERROR_FLASH_BURN_FAILED (0x80080000) /* /< flash burn failed */
#define ERROR_MS_TUNING		(0x80090000) /* /< ms tuning failed */
#define ERROR_SS_TUNING		(0x800A0000)	/* /< ss tuning failed */
#define ERROR_LP_TIMER_TUNING	(0x800B0000)	/* /< lp timer
						 * calibration failed */
#define ERROR_SAVE_CX_TUNING	(0x800C0000)	/* /< save cx data to
						 * flash failed */
#define ERROR_HANDLER_STOP_PROC (0x800D0000)	/* /< stop the poll of the FIFO
						 * if particular errors are
						 * found */
#define ERROR_CHECK_ECHO_FAIL	(0x800E0000)	/* /< unable to retrieve
						 * echo event */
#define ERROR_GET_FRAME		(0x800F0000)	/* /< unable to get frame */
/** @}*/

/* FOURTH LEVEL ERROR CODE */
/** @defgroup fourth_level	Fourth Level Error Code
  * @ingroup error_codes
  * Errors related to the highest logic operations in the IC which have an
  * important impact on the driver flow or which require several commands and
  * steps to be executed
  * @{
  */
#define ERROR_PROD_TEST_DATA		(0x81000000)	/* /< production data
							 * test failed */
#define ERROR_FLASH_PROCEDURE		(0x82000000)	/* /< fw update
							 * procedure failed */
#define ERROR_PROD_TEST_ITO		(0x83000000)	/* /< production
							 * ito test failed */
#define ERROR_PROD_TEST_INITIALIZATION	(0x84000000) /* /< production
						      * initialization
						      * test failed */
#define ERROR_GET_INIT_STATUS		(0x85000000) /* /< mismatch of the MS or
						      * SS tuning_version */
/** @}*/

/** @}*/	/* end of error_commands section */

/**
  * Struct which store an ordered list of the errors events encountered during
  *the polling of a FIFO.
  * The max number of error events that can be stored is equal to FIFO_DEPTH
  */
typedef struct {
	u8 list[FIFO_DEPTH * FIFO_EVENT_SIZE];	/* /< byte array which contains
						 * the series of error events
						 * encountered from the last
						 * reset of the list. */
	int count;	/* /< number of error events stored in the list */
	int last_index;	/* /< index of the list where will be stored the next
			 * error event. Subtract -1 to have the index of the
			 * last error event! */
} ErrorList;


void logError(int force, const char *msg, ...);
int isI2cError(int error);
int dumpErrorInfo(u8 *outBuf, int size);
int errorHandler(u8 *event, int size);
int addErrorIntoList(u8 *event, int size);
int getErrorListCount(void);
int resetErrorList(void);
int pollErrorList(int *event_to_search, int event_bytes);
int pollForErrorType(u8 *list, int size);
#endif
