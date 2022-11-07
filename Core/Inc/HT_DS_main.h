/*
 * HT_DS_main.h
 *
 *  Created on: Oct 24, 2022
 *      Author: Rakibul
 */

#ifndef INC_HT_DS_MAIN_H_
#define INC_HT_DS_MAIN_H_

#include "HT_config.h"
#include "iis2dh_reg.h"

#if (DATA_INTERVAL < 1) || (DATA_INTERVAL > 24)
#error "DATA_INTERVAL is out of bound"
#elif 24%DATA_INTERVAL != 0
#error "24 must be divisible by DATA_INTERVAL"
#else
#define TOTAL_DATA_BLOCKS	24/DATA_INTERVAL
#endif


#define HT_TRUE 	1
#define HT_FALSE 	0

typedef enum {
	HT_BG95_POWER_OFF,
	HT_BG95_POWER_ON,
	HT_BG95_STATUS_ERROR,
	HT_BG95_STATUS_OK,
	HT_BG95_NOT_CONNECTED_TO_NETWORK,
	HT_BG95_CONNECTED_TO_NETWORK,
	HT_BG95_SLEEP,
	HT_BG95_AIRPLANE_MODE,
	HT_BG95_MIN_FUNCTION_MODE,
}bg95_state_t;

typedef struct _tof {
	VL53L1_DEV ctxt;
}tof_t;

typedef struct _bg95 {
	char buff[MAX_SENDING_BUFF_SIZE];
	bg95_state_t state;
}bg95_t;

typedef struct accl_t {
	stmdev_ctx_t ctxt;
}accl_t;


typedef struct _ht_comp {
	tof_t		tof;
	bg95_t		bg95;
	accl_t		accl;
	uint8_t		nCurReading;					// current reading number
	uint32_t	lastDischargingStartTime;		// when the super capacitor were charged last (in milliseconds)
	uint32_t	lastDischargingEndTime;
}ht_ctxt_t;

typedef struct _date_time {
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t hh;				// in 24 hour format
	uint8_t mm;
	uint8_t ss;
}date_time_t;


typedef struct _nmea {
	char 		latitude[LATITUDE_LEN];
	char 		longitude[LONGITUDE_LEN];
	date_time_t	date_time;
}nmea_t;

typedef struct _data_block {
	float	 	range_mm;		// distance range in mm
	float		temperature;	// temperature in degree celsius
	float		std;			// standard deviation of the range_mm readings
}data_block_t;

typedef struct _ht_data {
#ifndef ASSET_ID_LEN
#error "ASSET_ID_LEN is not defined"
#else
	char 			assetID[ASSET_ID_LEN]; 	// A1:B2:C3:D4:E5:id3-rakibul
#endif
	uint8_t			ver;					// version of data packet
	float 			vbat;					// battery voltage
	uint8_t			csq;					// LTE signal quality
	uint8_t 		flags;					// Event/Error Flag
	nmea_t			nmea;					// Latitude, Longitude, Date-time
	uint8_t 		nBlk;					// number of blocks
	uint8_t 		sBlk;					// number of fields in each block
#ifdef TOTAL_DATA_BLOCKS
	data_block_t	data[TOTAL_DATA_BLOCKS];// sensor data block
#endif
}ht_data_t;


typedef enum {
	HT_ERROR = -1,
	HT_SUCCESS,
	HT_INVALID_PARAM,
	HT_ERROR_TOF,
	HT_ERROR_ACCL,
	HT_MEM_ALLOC_ERROR
}ht_status;

enum {
	HT_DISABLE,
	HT_ENABLE
};


enum {
	LOG_PRODUCTION, // DO NOT use it for logging. This is only used during the production build
	LOG_ERROR,
	LOG_WARNING,
	LOG_INFO,
	LOG_DEBUG
};

#endif /* INC_HT_DS_MAIN_H_ */
