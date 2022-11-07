/*
 * BG95M2.h
 *
 *  Created on: Oct 24, 2022
 *      Author: Rakibul
 */

#ifndef BG95M2_INC_BG95M2_H_
#define BG95M2_INC_BG95M2_H_

#include "HT_DS_main.h"

#define BG95M2_MAX_CNT_LEN		28
#define BG95M2_AT_HTTP_POST_LEN	27
#define BG95M2_GENERIC_POST_LEN	146
#define BG95M2_REQ_TIME			80
#define BG95M2_RSP_TIME			80


typedef enum {
	AIRPLANE_MODE,
	MIN_FUNCTION_MODE,
	SLEEP_MODE,
	POWER_SAVE_MODE,
	TOTAL_BG95M2_OP_MODE
}BG95M2_operating_mode_t;


typedef enum {
	BG95M2_SUCCESS,
	BG95M2_ERROR,
}BG95M2_Status;


ht_status init_BG95M2_LTE(ht_ctxt_t *htCtxt);
int init_send_data(void);
BG95M2_Status send_data(char *buf);
int bg95m2_sleep(BG95M2_operating_mode_t op);
int bg95m2_wakeup(BG95M2_operating_mode_t op);
ht_status turn_off_lte(ht_ctxt_t *HtCtxt);

#endif /* BG95M2_INC_BG95M2_H_ */
