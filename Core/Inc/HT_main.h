/*
 * HT_main.h
 *
 *  Created on: Sep 26, 2022
 *      Author: Rakibul
 */

#ifndef INC_HT_MAIN_H_
#define INC_HT_MAIN_H_

#include "main.h"
#include "vl53l1_api.h"
#include "BG95M2.h"
#include "HT_DS_main.h"
#include "iis2dh_reg.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <rtc.h>
#include <spi.h>
#include <i2c.h>
#include <usart.h>

extern DMA_HandleTypeDef hdma_usart3_rx;


#define TOF_SHUT_N_Pin 				GPIO_PIN_3
#define TOF_SHUT_N_GPIO_Port	 	GPIOE
#define TOF_INT_Pin 				GPIO_PIN_1
#define TOF_INT_GPIO_Port	 		GPIOE
#define LED_RD_Pin 					GPIO_PIN_7
#define LED_RD_GPIO_Port 			GPIOD
#define ACC_CS_N_Pin 				GPIO_PIN_15
#define ACC_CS_N_GPIO_Port 			GPIOA
#define CEL_RST_N_Pin 				GPIO_PIN_12
#define CEL_RST_N_GPIO_Port 		GPIOA
#define ACCL_INT2_Pin 				GPIO_PIN_4
#define ACCL_INT2_GPIO_Port 		GPIOE
#define GNSS_RX_Pin 				GPIO_PIN_7
#define GNSS_RX_GPIO_Port 			GPIOB
#define GNSS_TX_Pin 				GPIO_PIN_6
#define GNSS_TX_GPIO_Port 			GPIOB
#define CEL_MAIN_DTR_Pin 			GPIO_PIN_11
#define CEL_MAIN_DTR_GPIO_Port	 	GPIOA
#define LED_GR_Pin 					GPIO_PIN_5
#define LED_GR_GPIO_Port 			GPIOE
#define BLE_RST_N_Pin 				GPIO_PIN_0
#define BLE_RST_N_GPIO_Port 		GPIOE
#define CEL_NET_STATUS_Pin 			GPIO_PIN_10
#define CEL_NET_STATUS_GPIO_Port	GPIOA
#define CEL_PON_TRG_Pin 			GPIO_PIN_8
#define CEL_PON_TRG_GPIO_Port 		GPIOA
#define CEL_USB_BOOT_Pin 			GPIO_PIN_9
#define CEL_USB_BOOT_GPIO_Port 		GPIOC
#define CEL_MAIN_RI_Pin 			GPIO_PIN_6
#define CEL_MAIN_RI_GPIO_Port 		GPIOC
#define CEL_MAIN_DCD_Pin 			GPIO_PIN_14
#define CEL_MAIN_DCD_GPIO_Port 		GPIOD
#define CEL_PSM_Pin 				GPIO_PIN_13
#define CEL_PSM_GPIO_Port 			GPIOD
#define CEL_AP_RDY_Pin 				GPIO_PIN_1
#define CEL_AP_RDY_GPIO_Port 		GPIOC
#define CEL_PWRKEY_Pin 				GPIO_PIN_2
#define CEL_PWRKEY_GPIO_Port 		GPIOC
#define CEL_STATUS_Pin 				GPIO_PIN_10
#define CEL_STATUS_GPIO_Port 		GPIOD
#define GNSS_PWR_EN_Pin 			GPIO_PIN_5
#define GNSS_PWR_EN_GPIO_Port 		GPIOC
#define TOF_P_EN_Pin 				GPIO_PIN_1
#define TOF_P_EN_GPIO_Port 			GPIOB
#define ACC_P_EN_N_Pin 				GPIO_PIN_7
#define ACC_P_EN_N_GPIO_Port 		GPIOE
#define CEL_PWR_EN_Pin 				GPIO_PIN_11
#define CEL_PWR_EN_GPIO_Port 		GPIOE
#define ACC_CS_GPIO_Port			GPIOA
#define ACC_CS_GPIO_Pin				GPIO_PIN_15

#define ACC_SENSOR_BUS 				hspi1

#define UART_DATA_IN 				USART3
#define UART_DATA_IN_HANDLER 		huart3
#define UART_DATA_IN_DMA			hdma_usart3_rx

#define RxBuf_SIZE		255
extern uint8_t RxBuf[RxBuf_SIZE];

#define MINUTES_IN_AN_HOUR	60

void HT_setup(void *pUserData);
void HT_loop(void *pUserData);

ht_status init_ht_ctxt(ht_ctxt_t *htCtxt, void *pUserData);
ht_status init_ht_data(ht_data_t *htData, void *pUserData);
ht_status get_ht_data_block(data_block_t *db, void *pUserData);
ht_status make_buff_from_data(ht_data_t *htData, char* buff);
void HT_Sleep(uint16_t minute);

#if HT_HAS_TOF
// function related with ToF
int16_t get_tof_data(VL53L1_DEV *Dev, void* pUserData);
ht_status init_tof(VL53L1_DEV *Dev, void *pUserData);
ht_status calibrate_tof(VL53L1_DEV *Dev, void* pUserData);
ht_status setup_tof(VL53L1_DEV *Dev, void *pUserData);
#endif

#if HT_HAS_ACCL
ht_status init_accl(stmdev_ctx_t *devCtxt, void *pUserData);
float get_accl_data(stmdev_ctx_t *devCtxt, void *pUserData);
// Accl platform specific functions
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
#endif

// Mathematical functions
uint16_t calc_mode(uint16_t data[], uint8_t n);
float calc_sd(uint16_t data[], float mean, uint8_t n);


#define HT_LOG(ll, ...)  do { if (ll<=DEBUG_LEVEL) { printf ("%s:%d %s()# ", __FILE__, __LINE__, __func__); printf ( __VA_ARGS__); } } while(0)


void printLimitCheckValue(VL53L1_DEV Dev);
void printCalibrationData(VL53L1_DEV Dev);
void printMeasurementStatus(VL53L1_RangingMeasurementData_t RangingData);


#endif /* INC_HT_MAIN_H_ */
