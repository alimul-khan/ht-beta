/*
 * BG95M2.c
 *
 *  Created on: Oct 24, 2022
 *      Author: Rakibul
 */

#include <HT_main.h>
#include "BG95M2.h"

ht_status init_BG95M2_LTE(ht_ctxt_t *htCtxt)
{
	if (htCtxt == NULL) {
		HT_LOG(LOG_ERROR, "htCtxt is NULL\r\n");
		return HT_INVALID_PARAM;
	}

	HAL_GPIO_WritePin(CEL_PWRKEY_GPIO_Port, CEL_PWRKEY_Pin, GPIO_PIN_SET); //PWRKEY
	HAL_GPIO_WritePin(CEL_RST_N_GPIO_Port, CEL_RST_N_Pin, GPIO_PIN_SET); //RESET_N

	HT_LOG(LOG_DEBUG, "4.2V on\r\n");
	HAL_GPIO_WritePin(CEL_PWR_EN_GPIO_Port, CEL_PWR_EN_Pin, GPIO_PIN_SET); //V_BG95 (power enable)

	HT_LOG(LOG_DEBUG, "Will be waiting for 1.5 minutes\r\n");
	for (uint8_t i = 0; i<90; i++) {
		HAL_Delay(1000);
	}

	htCtxt->lastDischargingStartTime = HAL_GetTick();

	HT_LOG(LOG_DEBUG, "Init BG95M2...\r\n");

	HAL_GPIO_WritePin(CEL_PWRKEY_GPIO_Port, CEL_PWRKEY_Pin, GPIO_PIN_RESET); //PWRKEY
	HAL_GPIO_WritePin(CEL_RST_N_GPIO_Port, CEL_RST_N_Pin, GPIO_PIN_RESET); //RESET_N
	HAL_Delay (750);

	HAL_GPIO_WritePin(CEL_PWRKEY_GPIO_Port, CEL_PWRKEY_Pin, GPIO_PIN_SET); //PWRKEY
	HAL_GPIO_WritePin(CEL_RST_N_GPIO_Port, CEL_RST_N_Pin, GPIO_PIN_SET); //RESET_N

	htCtxt->bg95.state = HT_BG95_POWER_ON;

	uint32_t elapsedTime = 0;
	uint32_t initTime = HAL_GetTick();

	do {
		if (HAL_GPIO_ReadPin(CEL_STATUS_GPIO_Port, CEL_STATUS_Pin) == GPIO_PIN_SET) {
			htCtxt->bg95.state = HT_BG95_STATUS_OK;
			HT_LOG(LOG_DEBUG, "BG95 Status OK\r\n");
		} else {
			htCtxt->bg95.state = HT_BG95_STATUS_ERROR;
			HAL_Delay(250);
		}
		elapsedTime = HAL_GetTick() - initTime;
	} while (elapsedTime <= HT_BG95_STATUS_TIMEOUT && htCtxt->bg95.state == HT_BG95_STATUS_ERROR );

	if (htCtxt->bg95.state == HT_BG95_STATUS_ERROR) {
		HT_LOG(LOG_ERROR, "BG95 Status is NOT OK\r\n");
		turn_off_lte(htCtxt);
		htCtxt->lastDischargingEndTime = HAL_GetTick();
		htCtxt->bg95.state = HT_BG95_POWER_OFF;
		return HT_ERROR;
	}

	HAL_Delay (2000); // minimum 1.75s for time to stabilize

	uint8_t atChangeNetOpMode[] = "AT+QCFG=\"ledmode\",1\r\n";
	HAL_UART_Transmit(&huart3, atChangeNetOpMode , strlen((const char *)atChangeNetOpMode), 1000);
	HAL_Delay(1000);

	elapsedTime = 0;
	initTime = HAL_GetTick();

	do {
		if (HAL_GPIO_ReadPin(CEL_NET_STATUS_GPIO_Port, CEL_NET_STATUS_Pin) == GPIO_PIN_SET) {
			htCtxt->bg95.state = HT_BG95_CONNECTED_TO_NETWORK;
			HT_LOG(LOG_DEBUG, "BG95 Network Connected\r\n");
		} else {
			htCtxt->bg95.state = HT_BG95_NOT_CONNECTED_TO_NETWORK;
			HAL_Delay(250);
		}
		elapsedTime = HAL_GetTick() - initTime;
	} while (elapsedTime <= HT_BG95_NETWORK_SEARCH_TIMEOUT && htCtxt->bg95.state == HT_BG95_NOT_CONNECTED_TO_NETWORK );

	if (htCtxt->bg95.state == HT_BG95_NOT_CONNECTED_TO_NETWORK) {
		HT_LOG(LOG_ERROR, "BG95 Network Not Found!\r\n");
		turn_off_lte(htCtxt);
		htCtxt->lastDischargingEndTime = HAL_GetTick();
		htCtxt->bg95.state = HT_BG95_POWER_OFF;
		return HT_ERROR;
	}

	HT_LOG(LOG_DEBUG, "done.\r\n");
	return HT_SUCCESS;
}


int init_send_data(void)
{
	HT_LOG(LOG_DEBUG, "Initializing sending data...\r\n");

	uint8_t atCSGP[]="AT+QICSGP=1,1,\"mnet.bell.ca.ioe\",\"\",\"\",1\r\n";
	uint8_t atACT1[]="AT+QIACT=1\r\n";
	uint8_t atCfgCtx[]="AT+QHTTPCFG=\"contextid\",1\r\n";
	uint8_t atCfgHead[]="AT+QHTTPCFG=\"requestheader\",1\r\n";
	uint8_t atCfgSsl[]="AT+QHTTPCFG=\"sslctxid\",1\r\n";
	uint8_t atCfgCnt[]="AT+QHTTPCFG=\"contenttype\",1\r\n";
	uint8_t atUrl[]="AT+QHTTPURL=83,80\r\n";
	uint8_t url[]="http://haztrackerapi-env.eba-zdf4njwq.us-west-1.elasticbeanstalk.com/sensor/publish\r\n";

	HAL_UART_Transmit(&huart3, atCSGP , strlen((const char *)atCSGP), 1000);
	HAL_Delay(1000);

	HAL_UART_Transmit(&huart3, atACT1 , strlen((const char *)atACT1), 1000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart3, atCfgCtx , strlen((const char *)atCfgCtx), 1000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart3, atCfgHead , strlen((const char *)atCfgHead), 1000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart3, atCfgSsl , strlen((const char *)atCfgSsl), 1000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart3, atCfgCnt , strlen((const char *)atCfgCnt), 1000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart3, atUrl , strlen((const char *)atUrl), 1000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart3, url , strlen((const char *)url), 1000);
	HAL_Delay(1000);

	HT_LOG(LOG_DEBUG, "Done\r\n");
	return 0;
}


BG95M2_Status send_data(char *buff)
{
	if (buff == NULL) {
		HT_LOG(LOG_ERROR, "Buff is Null\r\n");
		return HT_INVALID_PARAM;
	}
	size_t buffLen = strlen((const char *)buff) - 2; //excluding \r\n from the length
	size_t buffLenLen = 0;
	if (buffLen > 0 && buffLen < 10) {
		buffLenLen = 1;
	} else if (buffLen > 9 && buffLen < 100 ) {
		buffLenLen = 2;
	} else if (buffLen > 99 && buffLen < 1000) {
		buffLenLen = 3;
	} else { // assuming the highest buffer length will be 9999
		buffLenLen = 4;
	}

	HT_LOG(LOG_DEBUG, "Sending Data...\r\n");

	uint8_t atHttpPost[BG95M2_AT_HTTP_POST_LEN];

	uint8_t post[] = "POST /sensor/publish HTTP/1.1\r\n"
			"Host: haztrackerapi-env.eba-zdf4njwq.us-west-1.elasticbeanstalk.com\r\n"
			"Content-Type: text/plain\r\n";
	uint8_t cntLen[BG95M2_MAX_CNT_LEN];

	sprintf((char *)cntLen,"Content-Length: %d\r\n\r\n", buffLen);
	sprintf((char *)atHttpPost,"AT+QHTTPPOST=%d,%d,%d\r\n", buffLen + BG95M2_GENERIC_POST_LEN + buffLenLen,
															BG95M2_REQ_TIME,
															BG95M2_RSP_TIME);

	HAL_UART_Transmit(&huart3, atHttpPost, strlen((const char *)atHttpPost), 1000);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart3, post, strlen((const char *)post), 1000);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart3, cntLen, strlen((const char *)cntLen), 1000);
	HAL_Delay(500);
	HAL_UART_Transmit(&huart3, (uint8_t *)buff, buffLen, 1000);
	HAL_Delay(2000);

	HT_LOG(LOG_DEBUG, "DONE\r\n");
	return BG95M2_SUCCESS;
}


int bg95m2_sleep(BG95M2_operating_mode_t op)
{
	if (op >= TOTAL_BG95M2_OP_MODE) {
		return -1;
	}

	switch(op) {
	case AIRPLANE_MODE:
		HAL_UART_Transmit(&huart3, (uint8_t *)"AT+CFUN=4\r\n", 12, 1000);
		HAL_Delay(1500);
		break;
	case MIN_FUNCTION_MODE:
		HAL_UART_Transmit(&huart3, (uint8_t *)"AT+CFUN=0\r\n", 12, 1000);
		HAL_Delay(1500);
		break;
	case SLEEP_MODE:
		HAL_UART_Transmit(&huart3, (uint8_t *)"AT+QSCLK=1\r\n", 13, 1000);
		HAL_Delay(1500);
		HAL_GPIO_WritePin(CEL_MAIN_DTR_GPIO_Port, CEL_MAIN_DTR_Pin, GPIO_PIN_SET); //Sleep
		break;
	case POWER_SAVE_MODE:
		printf("not implemented yet\r\n");
		break;
	default:
		break;
	}

	return 0;
}


int bg95m2_wakeup(BG95M2_operating_mode_t op)
{
	if (op >= TOTAL_BG95M2_OP_MODE) {
		return -1;
	}

	switch(op) {
	case AIRPLANE_MODE:
	case MIN_FUNCTION_MODE:
		HAL_UART_Transmit(&huart3, (uint8_t *)"AT+CFUN=1\r\n", 12, 1000);
		HAL_Delay(1500);
		break;
	case SLEEP_MODE:
		HAL_GPIO_WritePin(CEL_MAIN_DTR_GPIO_Port, CEL_MAIN_DTR_Pin, GPIO_PIN_RESET);
		HAL_UART_Transmit(&huart3, (uint8_t *)"AT+QSCLK=0\r\n", 13, 1000);
		HAL_Delay(1500);
		break;
	case POWER_SAVE_MODE:
		printf("not implemented yet\r\n");
		break;
	default:
		break;
	}

	return 0;
}

ht_status turn_off_lte(ht_ctxt_t *htCtxt)
{
	if (htCtxt == NULL) {
		HT_LOG(LOG_ERROR, "htCtxt is NULL\r\n");
		return HT_INVALID_PARAM;
	}

	HT_LOG(LOG_DEBUG, "Turn Off BG95M2...\r\n");

	HAL_GPIO_WritePin(CEL_PWRKEY_GPIO_Port, CEL_PWRKEY_Pin, GPIO_PIN_RESET); //PWRKEY
	HAL_GPIO_WritePin(CEL_RST_N_GPIO_Port, CEL_RST_N_Pin, GPIO_PIN_RESET); //RESET_N
	HAL_Delay (1250);

	HAL_GPIO_WritePin(CEL_PWRKEY_GPIO_Port, CEL_PWRKEY_Pin, GPIO_PIN_SET); //PWRKEY
	HAL_GPIO_WritePin(CEL_RST_N_GPIO_Port, CEL_RST_N_Pin, GPIO_PIN_SET); //RESET_N

	HAL_Delay(2000);

	htCtxt->bg95.state = HT_BG95_POWER_OFF;
	htCtxt->lastDischargingEndTime = HAL_GetTick();

	HT_LOG(LOG_DEBUG, "Turn off 4.2V...\r\n");
	HAL_GPIO_WritePin(CEL_PWR_EN_GPIO_Port, CEL_PWR_EN_Pin, GPIO_PIN_RESET); //V_BG95 (power enable)

	HAL_Delay(1000);

	HAL_GPIO_WritePin(CEL_PWRKEY_GPIO_Port, CEL_PWRKEY_Pin, GPIO_PIN_RESET); //PWRKEY
	HAL_GPIO_WritePin(CEL_RST_N_GPIO_Port, CEL_RST_N_Pin, GPIO_PIN_RESET); //RESET_N

	HAL_Delay(2000);

	HT_LOG(LOG_DEBUG, "done.\r\n");
	return HT_SUCCESS;
}
