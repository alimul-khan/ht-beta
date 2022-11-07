/*
 * HT_config.h
 *
 *  Created on: Oct 26, 2022
 *      Author: Rakibul
 */

#ifndef INC_HT_CONFIG_H_
#define INC_HT_CONFIG_H_

#include "HT_DS_main.h"
#include "HT_main.h"

/*
 * Throttle switch to enable/disable functions for different HT components
 */
#define HT_HAS_TOF	HT_TRUE
#define HT_HAS_ACCL	HT_TRUE
#define HT_HAS_LTE	HT_TRUE
#define HT_HAS_BLE	HT_FALSE
#define HT_HAS_NFC	HT_FALSE

/*
 * Other log levels are - 	LOG_PRODUCTION,
 *							LOG_ERROR,
 *							LOG_WARNING,
 *							LOG_INFO,
 *							LOG_DEBUG
 * All the logs including and above this DEBUG_LEVEL will be printed.
 * DO NOT use LOG_PRODUCTION for logging. This is only used during the production build.
 */
#define DEBUG_LEVEL LOG_DEBUG

#define HT_ASSET_ID "A1:B2:C3:D4:E5:alimul"

/*
 * delay in second between two consecutive ToF data
 */
#define HT_INTER_MEASUREMENT_DELAY 1000

/*
 * temperature offset for accelerometer reading
 */
#define HT_ACCL_TEMP_OFFSET	6.0

/*
 * We will take the mean of # ToF sensor readings
 */
#define NUMBER_OF_TOF_READING_PER_DATA_BLOCK	4

/*
 * Data interval in hours.
 * e.g we want to take data in every 3 hours, so the value of this derivative will be 3
 * The Maximum value of DATA_INTERVAL can be 24 (h) and minimum value can be 1 (h)
 * 24 must be divisible by DATA_INTERVAL to always take regular number of data
 * e.g. DATA_INTERVAL value cannot be 5 (24%5 != 0)
 */
#define DATA_INTERVAL		3

/*
 * maximum length of Asset ID
 */
#define ASSET_ID_LEN		32

/*
 * Length of latitude and longitude
 */
#define LATITUDE_LEN		8
#define LONGITUDE_LEN		8

/*
 * Deep debug enables some functions that prints detail debugging information
 */
#define DEEP_DEBUG	HT_FALSE

/*
 * Maximum allowed buffer size to be sent using BG95
 * This should limit the total buffer size sent
 */
#define MAX_SENDING_BUFF_SIZE	255

/*
 * On average, the size of each filed in the data block (in bytes)
 * e.g. 1070.55, 23.05, 0.55,
 * So, the total size of the data block will be sBlk times this size
 */
#define NUMBER_OF_BYTES_PER_FIELD_IN_DB	8

/*
 * We need to wait for at least MINIMUM_CHARGING_TIME time (in millisecond)
 * between two consecutive cycle of LTE
 */
#define MINIMUM_CHARGING_TIME	120000

/*
 * Timeout for BG95 status to be ok (in milliseconds)
 */
#define HT_BG95_STATUS_TIMEOUT	30000

/*
 * Timeout for network search (in milliseconds)
 */
#define HT_BG95_NETWORK_SEARCH_TIMEOUT	30000


/*
 * Maximum number of tries, in case -
 * 		1. The ToF sensor reading is not successful - (Error)
 * 		2. The ToF sensor reading is invalid
 * 		3. The initialization of BG95 fails
 *
 */
#define HT_MAXIMUM_NUMBER_OF_TRIES		5


/*
 * maximum allowed standard deviation for ToF reading
 */
#define HT_SD_THRESHOLD		5.0

#endif /* INC_HT_CONFIG_H_ */
