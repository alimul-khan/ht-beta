/*
 * HT_main.c
 *
 *  Created on: Sep 26, 2022
 *      Author: Rakibul
 *
 *
 *      STM32CubeIDE
 * Version: 1.10.1
 * Build: 12716_20220707_0928 (UTC)
 * OS: Windows 11, v.10.0, x86_64 / win32
 * Java vendor: Eclipse Adoptium
 * Java runtime version: 11.0.14.1+1
 * Java version: 11.0.14.1
 *
 */

#include "HT_main.h"

ht_data_t	gHtData;
ht_ctxt_t	gHtCtxt;
int delay_ =10000;
uint8_t __io_putchar(uint8_t ch) {
	HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 100);
	return ch;
}

void HT_setup(void *pUserData)
{
	HAL_Delay(5000);
	HT_LOG(LOG_INFO, "My Asset ID: %s\r\n", HT_ASSET_ID);


	init_ht_ctxt(&gHtCtxt, NULL);
	init_ht_data(&gHtData, NULL);

	init_tof(&(gHtCtxt.tof.ctxt), NULL);
	setup_tof(&(gHtCtxt.tof.ctxt), NULL);
//	calibrate_tof(&(gHtCtxt.tof.ctxt));

	init_accl(&(gHtCtxt.accl.ctxt), NULL);

	// charge up the super capacitor before starting



	HAL_GPIO_WritePin(CEL_PWRKEY_GPIO_Port, CEL_PWRKEY_Pin, GPIO_PIN_SET); //PWRKEY
	HAL_GPIO_WritePin(CEL_RST_N_GPIO_Port, CEL_RST_N_Pin, GPIO_PIN_SET); //RESET_N
	HAL_Delay (750);
	HAL_GPIO_WritePin(CEL_PWR_EN_GPIO_Port, CEL_PWR_EN_Pin, GPIO_PIN_SET); // V_BG95 (power enable)

	HT_LOG(LOG_INFO, "Will be charging SC for a minute\r\n");
	for (uint8_t i = 0; i<60; i++) {
		HAL_Delay(100);
	}

	HAL_GPIO_WritePin(CEL_PWR_EN_GPIO_Port, CEL_PWR_EN_Pin, GPIO_PIN_RESET); // V_BG95 (power disable)
	HAL_Delay (750);
	HAL_GPIO_WritePin(CEL_PWRKEY_GPIO_Port, CEL_PWRKEY_Pin, GPIO_PIN_RESET); //PWRKEY
	HAL_GPIO_WritePin(CEL_RST_N_GPIO_Port, CEL_RST_N_Pin, GPIO_PIN_RESET); //RESET_N


	HAL_UARTEx_ReceiveToIdle_DMA(&UART_DATA_IN_HANDLER, RxBuf, RxBuf_SIZE);
    __HAL_DMA_DISABLE_IT(&UART_DATA_IN_DMA, DMA_IT_HT);

    HAL_Delay (delay_); //    HT_Sleep(1);
}


void HT_loop(void *pUserData)
{
	if(gHtCtxt.nCurReading == TOTAL_DATA_BLOCKS) {
		uint32_t curTime = HAL_GetTick();
		if (curTime - gHtCtxt.lastDischargingEndTime >= MINIMUM_CHARGING_TIME) {
			HT_LOG(LOG_DEBUG, "Minimum Charging Time elapsed. BG95 state = %d\r\n", gHtCtxt.bg95.state);
			switch(gHtCtxt.bg95.state) {
			case HT_BG95_POWER_OFF:
				;
				uint8_t numberOfTries = 0;
				ht_status status;
				do {
					numberOfTries++;
					status = init_BG95M2_LTE(&gHtCtxt);
					if (status != HT_SUCCESS) {
						HAL_Delay (delay_); //HT_Sleep(1);
					}
				} while(status != HT_SUCCESS && numberOfTries<HT_MAXIMUM_NUMBER_OF_TRIES);

				if (status != HT_SUCCESS) {
					HT_LOG(LOG_ERROR, "BG95 init failed\r\n");
					HAL_Delay (delay_);// HT_Sleep(MINUTES_IN_AN_HOUR);
				}
				break;
			case HT_BG95_POWER_ON:
				break;
			case HT_BG95_STATUS_ERROR:
			case HT_BG95_NOT_CONNECTED_TO_NETWORK:
				turn_off_lte(&gHtCtxt);
				break;
			case HT_BG95_CONNECTED_TO_NETWORK:
				make_buff_from_data(&gHtData, gHtCtxt.bg95.buff);
				init_send_data();
				send_data(gHtCtxt.bg95.buff);
				turn_off_lte(&gHtCtxt);
				gHtCtxt.nCurReading = 0;
				HAL_Delay (delay_); //HT_Sleep(1); // because after turning off, we don't want to take the reading immediately
				break;
			case HT_BG95_AIRPLANE_MODE:
			case HT_BG95_SLEEP:
			case HT_BG95_MIN_FUNCTION_MODE:
				// todo: in future
				break;
			default:
				break;
			}
		} else {HAL_Delay(2000);}
	} else {
		uint8_t numberOfTries = 0;
		ht_status status;
		do {
			HT_LOG(LOG_DEBUG, "Taking data %d\r\n", gHtCtxt.nCurReading);

			gHtData.data[gHtCtxt.nCurReading].range_mm = -1.0;
			gHtData.data[gHtCtxt.nCurReading].std = 0.0;
			gHtData.data[gHtCtxt.nCurReading].temperature = 0.0;

			numberOfTries++;
			HT_LOG(LOG_DEBUG, "Numer of Tries %d\r\n", numberOfTries);

			status = get_ht_data_block(&(gHtData.data[gHtCtxt.nCurReading]), NULL);

			// because in case status is not success,
			// we don't want to turn off everything and turn back on again the next moment
			if(status != HT_SUCCESS) {
				HAL_Delay (2000); //HT_Sleep(1);
			}
		} while (status != HT_SUCCESS && numberOfTries < HT_MAXIMUM_NUMBER_OF_TRIES);

		HAL_Delay (delay_); //HT_Sleep(DATA_INTERVAL*MINUTES_IN_AN_HOUR - numberOfTries);
//		HT_Sleep(1);
	}
}


ht_status init_ht_data(ht_data_t *htData, void *pUserData)
{
	if (htData == NULL) {
		HT_LOG(LOG_ERROR, "htData is NULL\r\n");
		return HT_INVALID_PARAM;
	}

	strcpy(htData->assetID, HT_ASSET_ID);
	htData->nBlk = 24/DATA_INTERVAL;
	htData->sBlk = 3;
	htData->ver = 1;
	htData->vbat = 0;
	htData->csq = 0;
	htData->flags = 0;
	strcpy(htData->nmea.latitude, "0");
	strcpy(htData->nmea.longitude, "0");
	htData->nmea.date_time.date = 0;
	htData->nmea.date_time.month = 0;
	htData->nmea.date_time.year = 0;
	htData->nmea.date_time.hh = 0;
	htData->nmea.date_time.mm = 0;
	htData->nmea.date_time.ss = 0;

	return HT_SUCCESS;
}

ht_status init_ht_ctxt(ht_ctxt_t *htCtxt, void *pUserData)
{
	if (htCtxt == NULL) {
		HT_LOG(LOG_ERROR, "htCtxt is NULL\r\n");
		return HT_INVALID_PARAM;
	}

	htCtxt->tof.ctxt = (VL53L1_Dev_t *)malloc(sizeof(VL53L1_Dev_t));
	if (htCtxt->tof.ctxt == NULL) {
		HT_LOG(LOG_ERROR, "ToF Ctxt memory allocation failed\r\n");
		return HT_MEM_ALLOC_ERROR;
	}

	htCtxt->bg95.state = HT_BG95_POWER_OFF;

	htCtxt->nCurReading = 0;

	htCtxt->lastDischargingEndTime = HAL_GetTick();

	return HT_SUCCESS;
}

#if HT_HAS_TOF
ht_status init_tof(VL53L1_DEV *Dev, void *pUserData)
{
	if (Dev == NULL) {		HT_LOG(LOG_ERROR, "Dev is NULL\r\n"); return HT_INVALID_PARAM;}
	(*Dev)->I2cHandle = &hi2c1;
	(*Dev)->I2cDevAddr = 0x52;
	/*** VL53L1X Initialization Bootup ***/
	HAL_GPIO_WritePin(TOF_P_EN_GPIO_Port, TOF_P_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(TOF_SHUT_N_GPIO_Port, TOF_SHUT_N_Pin, GPIO_PIN_SET);
	HAL_Delay(500);

	if (VL53L1_WaitDeviceBooted((*Dev)) != VL53L1_ERROR_NONE) {HT_LOG(LOG_ERROR, "VL53L1_WaitDeviceBooted failed.\r\n");return HT_ERROR_TOF;	}
	// initialize vl53l1x communication parameters
	if (VL53L1_DataInit((*Dev)) != VL53L1_ERROR_NONE) {HT_LOG(LOG_ERROR, "VL53L1_DataInit failed.\r\n");return HT_ERROR_TOF;	}
	if (VL53L1_StaticInit((*Dev)) != VL53L1_ERROR_NONE) {HT_LOG(LOG_ERROR, "VL53L1_StaticInit failed.\r\n");return HT_ERROR_TOF;	}
	return HT_SUCCESS;
}

ht_status setup_tof(VL53L1_DEV *Dev, void *pUserData)
{
	if (Dev == NULL) {
		HT_LOG(LOG_ERROR, "Dev is NULL\r\n");
		return HT_INVALID_PARAM;
	}

	/*
	 * Possible options VL53L1_DISTANCEMODE_SHORT	MaxDistance: 1.3m
	 * 					VL53L1_DISTANCEMODE_MEDIUM	MaxDistance: 3m
	 * 					VL53L1_DISTANCEMODE_LONG	MaxDistance: 4m
	 */
	if (VL53L1_SetDistanceMode( (*Dev), VL53L1_DISTANCEMODE_MEDIUM ) != VL53L1_ERROR_NONE) {
		HT_LOG(LOG_ERROR, "VL53L1_SetDistanceMode failed.\r\n");
		return HT_ERROR_TOF;
	}

	/*
	 * Highest value of Timing Budget in the function is 10s
	 * Higher the timing budget, better the accuracy and also higher the power consumption
	 */
	if (VL53L1_SetMeasurementTimingBudgetMicroSeconds( (*Dev), 700000 ) != VL53L1_ERROR_NONE) {
		HT_LOG(LOG_ERROR, "VL53L1_SetMeasurementTimingBudgetMicroSeconds failed.\r\n");
		return HT_ERROR_TOF;
	}

	/*
	 * Set Region
	 * Give the coordinate of topLeft and BottomRight corner
	 * Minimum region possible is 4X4
	 * Maximum is 16X16 which is full ROI
	 */
	VL53L1_UserRoi_t roiConfig;
	// top (6,9)
	roiConfig.TopLeftX = 6;
	roiConfig.TopLeftY = 9;
	// bottom (9,6)
	roiConfig.BotRightX = 9;
	roiConfig.BotRightY = 6;

	if (VL53L1_SetUserROI((*Dev), &roiConfig) != VL53L1_ERROR_NONE) {
		HT_LOG(LOG_ERROR, "VL53L1_SetUserROI failed.\r\n");
		return HT_ERROR_TOF;
	}

	return HT_SUCCESS;
}

ht_status calibrate_tof(VL53L1_DEV *Dev, void* pUserData)
{
	if (Dev == NULL) {
		HT_LOG(LOG_ERROR, "Dev is NULL\r\n");
		return HT_INVALID_PARAM;
	}

//	if (VL53L1_PerformRefSpadManagement((*Dev)) != VL53L1_ERROR_NONE) {
//		HT_LOG(LOG_ERROR, "VL53L1_PerformRefSpadManagement failed.\r\n");
//		return HT_ERROR_TOF;
//	}
//	HAL_Delay(1000);
//
//
//	if (VL53L1_PerformOffsetSimpleCalibration((*Dev),730) != VL53L1_ERROR_NONE) {
//		HT_LOG(LOG_ERROR, "VL53L1_PerformOffsetSimpleCalibration failed.\r\n");
//		return HT_ERROR_TOF;
//	}
//	HAL_Delay(2000);


//	if (VL53L1_PerformSingleTargetXTalkCalibration((*Dev), 40) != VL53L1_ERROR_NONE) {
//		HT_LOG(LOG_ERROR, "VL53L1_PerformSingleTargetXTalkCalibration failed.\r\n");
//		return HT_ERROR_TOF;
//	}
//	HAL_Delay(2000);

	printCalibrationData((*Dev));

	/*
	 * enable setupToF if any of the above calibration is performed.
	 */
//	if (setup_tof(Dev, NULL) != HT_SUCCESS) {
//		HT_LOG(LOG_ERROR, "setup_tof failed.\r\n");
//		return HT_ERROR_TOF;
//	}


	/*
	 * Print Sigma and Signal
	 * Default value for sigma = 1.5 and signal = 90
	 */
	printLimitCheckValue((*Dev));

	/*
	 * As you increase the distance, the Signal will decrease.
	 * From Rakiul's experiment, we found that for 1m, the signal value is close to 15% in lab
	 */
	if (VL53L1_SetLimitCheckValue((*Dev), VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.15*65536) != VL53L1_ERROR_NONE) {
		HT_LOG(LOG_ERROR, "VL53L1_SetLimitCheckValue failed.\r\n");
		return HT_ERROR_TOF;
	}

	if (VL53L1_SetLimitCheckValue((*Dev), VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, 80*65536) != VL53L1_ERROR_NONE) {
		HT_LOG(LOG_ERROR, "VL53L1_SetLimitCheckValue failed.\r\n");
		return HT_ERROR_TOF;
	}

	/*
	 * Enable/Disable Signal/Sigma Checking
	 */
	if (VL53L1_SetLimitCheckEnable((*Dev), VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, HT_DISABLE) != VL53L1_ERROR_NONE) {
		HT_LOG(LOG_ERROR, "VL53L1_SetLimitCheckEnable failed.\r\n");
		return HT_ERROR_TOF;
	}

	if (VL53L1_SetLimitCheckEnable((*Dev), VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, HT_DISABLE) != VL53L1_ERROR_NONE) {
		HT_LOG(LOG_ERROR, "VL53L1_SetLimitCheckEnable failed.\r\n");
		return HT_ERROR_TOF;
	}

	return HT_SUCCESS;
}


void printLimitCheckValue(VL53L1_DEV Dev)
{
	uint32_t signalVal, sigmaVal;
	VL53L1_GetLimitCheckValue(Dev, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, &signalVal);
	VL53L1_GetLimitCheckValue(Dev, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, &sigmaVal);
	HT_LOG(LOG_INFO, "\r\n---------------------------Printing Sigma/Signal Value-----------------------------------\r\n");
	HT_LOG(LOG_INFO, "VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE = %0.2f\r\nVL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE = %0.2f\n", sigmaVal/65536.0, signalVal/65536.0);
	HT_LOG(LOG_INFO, "---------------------------Sigma/Signal Value End-----------------------------------\r\n");
}

void printCalibrationData(VL53L1_DEV Dev)
{
	VL53L1_CalibrationData_t pCalData;
	VL53L1_GetCalibrationData(Dev, &pCalData);
	HT_LOG(LOG_INFO, "\r\n---------------------------Printing Calibration Value-----------------------------------\r\n");
	HT_LOG(LOG_INFO, "\t----customer----\r\n");
	HT_LOG(LOG_INFO, "algo__crosstalk_compensation_plane_offset_kcps = %ld\r\n", pCalData.customer.algo__crosstalk_compensation_plane_offset_kcps);
	HT_LOG(LOG_INFO, "algo__crosstalk_compensation_x_plane_gradient_kcps = %d\r\n", pCalData.customer.algo__crosstalk_compensation_x_plane_gradient_kcps);
	HT_LOG(LOG_INFO, "algo__crosstalk_compensation_y_plane_gradient_kcps = %d\r\n", pCalData.customer.algo__crosstalk_compensation_y_plane_gradient_kcps);
	HT_LOG(LOG_INFO, "algo__part_to_part_range_offset_mm = %d\r\n", pCalData.customer.algo__part_to_part_range_offset_mm);
	HT_LOG(LOG_INFO, "mm_config__inner_offset_mm = %d\r\n", pCalData.customer.mm_config__inner_offset_mm);
	HT_LOG(LOG_INFO, "mm_config__outer_offset_mm = %d\r\n", pCalData.customer.mm_config__outer_offset_mm);
	HT_LOG(LOG_INFO, "\t----add_off_cal_data----\r\n");
	HT_LOG(LOG_INFO, "result__mm_inner_actual_effective_spads = %d\r\n", pCalData.add_off_cal_data.result__mm_inner_actual_effective_spads);
	HT_LOG(LOG_INFO, "result__mm_inner_peak_signal_count_rtn_mcps = %d\r\n", pCalData.add_off_cal_data.result__mm_inner_peak_signal_count_rtn_mcps);
	HT_LOG(LOG_INFO, "result__mm_outer_actual_effective_spads = %d\r\n", pCalData.add_off_cal_data.result__mm_outer_actual_effective_spads);
	HT_LOG(LOG_INFO, "result__mm_outer_peak_signal_count_rtn_mcps = %d\r\n", pCalData.add_off_cal_data.result__mm_outer_peak_signal_count_rtn_mcps);
	HT_LOG(LOG_INFO, "\t----optical_centre----\r\n");
	HT_LOG(LOG_INFO, "x_centre = %d\r\n", pCalData.optical_centre.x_centre);
	HT_LOG(LOG_INFO, "y_centre = %d\r\n", pCalData.optical_centre.y_centre);
	HT_LOG(LOG_INFO, "\t----gain_cal----\r\n");
	HT_LOG(LOG_INFO, "standard_ranging_gain_factor = %d\r\n", pCalData.gain_cal.standard_ranging_gain_factor);
	HT_LOG(LOG_INFO, "\t----cal_peak_rate_map----\r\n");
	HT_LOG(LOG_INFO, "cal_distance_mm = %d\r\n", pCalData.cal_peak_rate_map.cal_distance_mm);
	HT_LOG(LOG_INFO, "max_samples = %d\r\n", pCalData.cal_peak_rate_map.max_samples);
	HT_LOG(LOG_INFO, "height = %d\r\n", pCalData.cal_peak_rate_map.height);
	HT_LOG(LOG_INFO, "width = %d\r\n", pCalData.cal_peak_rate_map.width);
	HT_LOG(LOG_INFO, "---------------------------Calibration Value End-----------------------------------\r\n");
}

void printMeasurementStatus(VL53L1_RangingMeasurementData_t RangingData)
{
	HT_LOG(LOG_INFO, "%0.2f,", (RangingData.AmbientRateRtnMegaCps / 65536.0));
	HT_LOG(LOG_INFO, "%0.2f,", RangingData.EffectiveSpadRtnCount / 256.0);
	HT_LOG(LOG_INFO, "%d,", RangingData.RangeStatus);
	HT_LOG(LOG_INFO, "%0.2f,", (RangingData.SigmaMilliMeter / 65536.0));
	HT_LOG(LOG_INFO, "%0.2f,", (RangingData.SignalRateRtnMegaCps / 65536.0));
	HT_LOG(LOG_INFO, "%d,", RangingData.RangeMilliMeter);
}

int16_t get_tof_data(VL53L1_DEV *Dev, void* pUserData)
{
	if (Dev == NULL) {
		HT_LOG(LOG_ERROR, "Dev is NULL\r\n");
		return HT_INVALID_PARAM;
	}

	VL53L1_Error status;
	VL53L1_RangingMeasurementData_t RangingData;

	status = VL53L1_StartMeasurement( (*Dev) );
	if (status != VL53L1_ERROR_NONE) {
		HT_LOG(LOG_ERROR, "VL53L1_StartMeasurement failed. status  = %d\r\n", status);
		return HT_ERROR;
	}

	status = VL53L1_WaitMeasurementDataReady( (*Dev) );
	if (status != VL53L1_ERROR_NONE) {
		HT_LOG(LOG_ERROR, "VL53L1_WaitMeasurementDataReady failed. status  = %d\r\n", status);
		return HT_ERROR;
	}

	status = VL53L1_GetRangingMeasurementData( (*Dev), &RangingData );
	if (status != VL53L1_ERROR_NONE) {
		HT_LOG(LOG_ERROR, "VL53L1_GetRangingMeasurementData failed. status  = %d\r\n", status);
		return HT_ERROR;
	}

	status = VL53L1_StopMeasurement((*Dev));
	if (status != VL53L1_ERROR_NONE) {
		HT_LOG(LOG_ERROR, "VL53L1_StopMeasurement failed. status  = %d\r\n", status);
		return HT_ERROR;
	}

	return RangingData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID ? RangingData.RangeMilliMeter : HT_ERROR;
}
#endif //HT_HAS_TOF

#if HT_HAS_ACCL
ht_status init_accl(stmdev_ctx_t *devCtxt, void *pUserData)
{
	if (devCtxt == NULL) {
		HT_LOG(LOG_ERROR, "devCtxt is NULL\r\n");
		return HT_INVALID_PARAM;
	}
	static uint8_t whoamI;
	ht_status status = HT_SUCCESS;

	devCtxt->write_reg = platform_write;
	devCtxt->read_reg = platform_read;
	devCtxt->handle = &ACC_SENSOR_BUS;

    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACC_P_EN_N_GPIO_Port, ACC_P_EN_N_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);

	status |= iis2dh_device_id_get(devCtxt, &whoamI);			/* Check device ID */

	if (whoamI != IIS2DH_ID) {									/* manage here device not found */
		HT_LOG(LOG_ERROR, "Accl not Found\r\n");
		return HT_ERROR;
	}

	status |= iis2dh_block_data_update_set(devCtxt, PROPERTY_ENABLE);	/* Enable Block Data Update */
	status |= iis2dh_data_rate_set(devCtxt, IIS2DH_ODR_1Hz);			/* Set Output Data Rate to 1Hz */
	status |= iis2dh_full_scale_set(devCtxt, IIS2DH_2g);				/* Set full scale to 2g */
	status |= iis2dh_temperature_meas_set(devCtxt, IIS2DH_TEMP_ENABLE);/* Enable temperature sensor */
	status |= iis2dh_operating_mode_set(devCtxt, IIS2DH_HR_12bit);		/* Set device in continuous mode with 12 bit resol. */

	return status == HT_SUCCESS ? HT_SUCCESS : HT_ERROR_ACCL;
}

float get_accl_data(stmdev_ctx_t *devCtxt, void *pUserData)
{
	int16_t rawTemperature = 0;
	float temperature_degC;
	iis2dh_reg_t reg;
	iis2dh_temp_data_ready_get(devCtxt, &reg.byte);

	if (reg.byte) {
		if (iis2dh_temperature_raw_get(devCtxt, &rawTemperature) != HT_SUCCESS) {
			HT_LOG(LOG_ERROR, "Temperature reading error\r\n");
			// todo: need to handle this situation in future
		}
		temperature_degC = iis2dh_from_lsb_hr_to_celsius(rawTemperature);
	}
	return (temperature_degC - HT_ACCL_TEMP_OFFSET);
}

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,uint16_t len)
{
  reg |= 0x40;
  HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_GPIO_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t *)bufp, len, 1000);
  HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_GPIO_Pin, GPIO_PIN_SET);
  return 0;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
  reg |= 0xC0;
  HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_GPIO_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_GPIO_Pin, GPIO_PIN_SET);
  return 0;
}
#endif // HT_HAS_ACCL

uint16_t calc_mode(uint16_t data[], uint8_t n) {
   uint16_t maxValue = 0, maxCount = 0;

   for (uint8_t i = 0; i < n; ++i) {
      int count = 0;

      for (uint8_t j = 0; j < n; ++j) {
         if (data[j] == data[i])
         ++count;
      }

      if (count > maxCount) {
         maxCount = count;
         maxValue = data[i];
      }
   }

   return maxValue;
}

float calc_sd (uint16_t data[], float mean, uint8_t n)
{
    float SD = 0.0;
    for (uint8_t i = 0; i < n; ++i) {
        SD += pow(data[i] - mean, 2);
    }
    return sqrt(SD / n * 1.0);
}

ht_status get_ht_data_block(data_block_t *db, void *pUserData)
{
	if (db == NULL) {
		HT_LOG(LOG_ERROR, "db is NULL\r\n");
		return HT_INVALID_PARAM;
	}

	uint16_t *data = (uint16_t *)malloc(NUMBER_OF_TOF_READING_PER_DATA_BLOCK*sizeof(uint16_t));
	if (data == NULL) {
		HT_LOG(LOG_ERROR, "data Memory allocation error\r\n");
		return HT_MEM_ALLOC_ERROR;
	}
	memset(data, 0, NUMBER_OF_TOF_READING_PER_DATA_BLOCK*sizeof(data));

	uint32_t sumD = 0;
	int16_t tmpD = -1;
	float meanD = 0.0;
	float sd = 0.0;

	uint8_t numberOfTries = 0;
	for (uint8_t i = 0; i < NUMBER_OF_TOF_READING_PER_DATA_BLOCK && numberOfTries < HT_MAXIMUM_NUMBER_OF_TRIES; i++) {
		tmpD = get_tof_data(&(gHtCtxt.tof.ctxt), NULL);
		if(tmpD == -1) {
			i--;
			HT_LOG(LOG_ERROR, "Received invalid data\r\n");
			HAL_Delay(HT_INTER_MEASUREMENT_DELAY);
			numberOfTries++;
			continue;
		}
		data[i] = tmpD;
		sumD += tmpD;
		HAL_Delay(HT_INTER_MEASUREMENT_DELAY);
	}

	if (numberOfTries >= HT_MAXIMUM_NUMBER_OF_TRIES) {
		HT_LOG(LOG_ERROR, "Invalid data for a long time\r\n");
		return HT_ERROR;
	}

	meanD = sumD*1.0/NUMBER_OF_TOF_READING_PER_DATA_BLOCK;
	sd = calc_sd(data, meanD, NUMBER_OF_TOF_READING_PER_DATA_BLOCK);

	if (sd > HT_SD_THRESHOLD) {
		HT_LOG(LOG_ERROR, "High Standard Deviation\r\n");
		return HT_ERROR;
	}

	db->range_mm = meanD;
	db->std = sd;
	db->temperature = get_accl_data(&(gHtCtxt.accl.ctxt), NULL);

	gHtCtxt.nCurReading++;
	free(data);

	return HT_SUCCESS;
}


ht_status make_buff_from_data(ht_data_t *htData, char* buff)
{
	if (htData == NULL || buff == NULL) {
		HT_LOG(LOG_ERROR, "htData or buff is NULL\r\n");
		return HT_INVALID_PARAM;
	}

	char *tmpBuffAllData;
	char *tmpBuffData;

	size_t tmpBuffDataSize = (htData->sBlk)*NUMBER_OF_BYTES_PER_FIELD_IN_DB;
	size_t tmpBuffAllDataSize = (htData->nBlk)*tmpBuffDataSize;

	tmpBuffAllData = (char *)malloc(tmpBuffAllDataSize);
	if (tmpBuffAllData == NULL) {
		HT_LOG(LOG_ERROR, "tmpBuffAllData Memory Allocation Fail\r\n");
		return HT_MEM_ALLOC_ERROR;
	}

	tmpBuffData = (char *)malloc(tmpBuffDataSize);
	if (tmpBuffData == NULL) {
		HT_LOG(LOG_ERROR, "tmpBuffData Memory Allocation Fail\r\n");
		return HT_MEM_ALLOC_ERROR;
	}

	memset(tmpBuffAllData, '\0', (htData->nBlk)*htData->sBlk*NUMBER_OF_BYTES_PER_FIELD_IN_DB);
	for (uint8_t i = 0; i<(htData->nBlk-1); i++) {
		memset(tmpBuffData, '\0', tmpBuffDataSize);
		sprintf(tmpBuffData, "%0.2f, %0.2f, %0.2f, ", htData->data[i].range_mm, htData->data[i].temperature, htData->data[i].std);
		strcat(tmpBuffAllData, tmpBuffData);
	}
	uint8_t lastBlock = htData->nBlk-1;
	memset(tmpBuffData, '\0', tmpBuffAllDataSize);
	sprintf(tmpBuffData, "%0.2f, %0.2f, %0.2f", htData->data[lastBlock].range_mm, htData->data[lastBlock].temperature, htData->data[lastBlock].std);
	strcat(tmpBuffAllData, tmpBuffData);


	sprintf( buff, "%s, %d, %0.2f, %d, %d, %sN, %sW, %d/%d/%d - %d:%d:%d, %d, %d, %s\r\n",	htData->assetID,
																							htData->ver,
																							htData->vbat,
																							htData->csq,
																							htData->flags,
																							htData->nmea.latitude,
																							htData->nmea.longitude,
																							htData->nmea.date_time.date,
																							htData->nmea.date_time.month,
																							htData->nmea.date_time.year,
																							htData->nmea.date_time.hh,
																							htData->nmea.date_time.mm,
																							htData->nmea.date_time.ss,
																							htData->nBlk,
																							htData->sBlk,
																							tmpBuffAllData);

	HT_LOG(LOG_DEBUG, "%s\r\n", tmpBuffAllData);
	free(tmpBuffAllData);
	free(tmpBuffData);

	return HT_SUCCESS;
}

void HT_Sleep(uint16_t minute)
{
	HT_LOG(LOG_INFO, "Sleeping for %d minutes\r\n", minute);
	for(uint8_t i = 0; i < minute * 2; i++) {
		HAL_SuspendTick();
		HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0xEF32, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
		SystemClock_Config();
		HAL_ResumeTick();
	}
}
