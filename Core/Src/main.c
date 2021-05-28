/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Switch includes based on the step to run
// STEP 1
#include "lr1110_modem_1.0.7_part-1.h"
// #include "lr_1110_transceiver_304_part-1.h"

// STEP 2
#include "lr1110_modem_1.0.7_part-2.h"
// #include "lr_1110_transceiver_304_part-2.h"

#include "configuration.h"
#include "lr1110_bootloader.h"
#include "lr1110_system.h"
#include "lr1110_system_types.h"
#include "system.h"

#include "lr1110_wifi.h"
#include "lr1110_wifi_types.h"
#include "lr1110_gnss.h"
#include "lr1110_gnss_types.h"

#include "lr1110_modem_system.h"
#include "lr1110_modem_lorawan.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  radio_t radio = {
	&hspi1,
	{ NSS_GPIO_Port, NSS_Pin },
	{ LR_NRESET_GPIO_Port, LR_NRESET_Pin },
    { LR_IRQ__DIO9_GPIO_Port, LR_IRQ__DIO9_Pin },
    { BUSY_GPIO_Port, BUSY_Pin },
  };

  lr1110_bootloader_version_t version_system = { 0 };
  lr1110_modem_version_t modem_version = { 0 };

  //  lr1110_modem_status_t modem_status = { 0 };
//  lr1110_modem_response_code_t modem_return;
//  lr1110_modem_event_fields_t modem_event;

  lr1110_system_stat1_t lr_stat1;
  lr1110_system_stat2_t lr_stat2;
  lr1110_system_irq_mask_t lr_irq_status;
  lr1110_system_errors_t lr_errors;

  lr1110_status_t lr_status = LR1110_STATUS_ERROR;

  // lr1110_modem_chip_eui_t chip_eui;


/*
  lr1110_hal_reset( &radio );
  lr1110_bootloader_reboot(&radio, false);
  HAL_Delay(2500);
  lr1110_bootloader_get_version(&radio, &version_system);
  lr1110_modem_get_version( &radio, &modem_version );
  modem_return = lr1110_modem_suspend( &radio, LR1110_MODEM_RESUMED);
  modem_return = lr1110_modem_get_status( &radio, &modem_status );
  modem_return = lr1110_modem_get_chip_eui( &radio, &chip_eui );
  lr1110_modem_system_reboot( &radio, false );
  lr1110_bootloader_get_version(&radio, &version_system);
  lr1110_modem_get_version( &radio, &modem_version );
*/

  /* START GNSS test */
  /*
  lr1110_hal_reset( &radio );
  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
  HAL_Delay(500);
  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
  // lr1110_modem_get_version(&radio, &modem_version);
  lr1110_bootloader_get_version(&radio, &version_system);
  lr_status = lr1110_system_get_version(&radio, &version);

  lr_status = lr1110_system_get_status(&radio, &lr_stat1, &lr_stat2, &lr_irq_status);
  lr_status = lr1110_system_get_errors(&radio, &lr_errors);

  lr_status = lr1110_system_set_reg_mode(&radio, LR1110_SYSTEM_REG_MODE_DCDC);

  lr1110_system_rfswitch_cfg_t rf_switch_setup = { 0 };
  rf_switch_setup.enable                       = LR1110_SYSTEM_RFSW2_HIGH;
  rf_switch_setup.standby                      = 0xFF;
  rf_switch_setup.tx                           = 0x00;
  rf_switch_setup.tx_hp                        = 0x00;
  rf_switch_setup.rx                           = 0x00;
  rf_switch_setup.wifi                         = 0x00;
  rf_switch_setup.gnss                         = LR1110_SYSTEM_RFSW2_HIGH;

  lr_status = lr1110_system_set_dio_as_rf_switch(&radio, &rf_switch_setup);

  lr_status = lr1110_system_cfg_lfclk(&radio, LR1110_SYSTEM_LFCLK_XTAL, true);
  lr_status = lr1110_system_set_tcxo_mode(&radio, LR1110_SYSTEM_TCXO_CTRL_1_8V, 500);
  lr_status = lr1110_system_calibrate(&radio, 0x3F);

  uint32_t local_time = 1614157841;
  uint16_t gnss_result_size = 0;
  uint8_t gnss_results[LR1110_GNSS_MAX_SIZE_ARRAY] = { 0 };
  uint8_t nb_sv_detected = 0;
  lr1110_gnss_detected_satellite_t list_of_sv[32] = { 0 };

  lr_status = lr1110_gnss_set_constellations_to_use(&radio, LR1110_GNSS_GPS_MASK + LR1110_GNSS_BEIDOU_MASK);

  while (!nb_sv_detected)
  {
	  HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	  lr_status = lr1110_gnss_scan_autonomous(&radio, local_time, LR1110_GNSS_OPTION_BEST_EFFORT, LR1110_GNSS_BIT_CHANGE_MASK | LR1110_GNSS_DOPPLER_MASK | LR1110_GNSS_IRQ_PSEUDO_RANGE_MASK, 0);
	  lr_status = lr1110_gnss_get_result_size(&radio, &gnss_result_size);
	  lr_status = lr1110_gnss_read_results(&radio, gnss_results, gnss_result_size);
	  lr_status = lr1110_gnss_get_nb_detected_satellites(&radio, &nb_sv_detected );
	  lr_status = lr1110_gnss_get_detected_satellites(&radio, nb_sv_detected, list_of_sv);

	  HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
	  HAL_Delay(5000);
  }

  int i;

  for (i = 0; i < nb_sv_detected; ++i)
  {
	  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
  }



  HAL_Delay(5000);
  */

  /* END GNSS test */

  /*
  // lr1110_wifi_channel_mask_t channels = LR1110_WIFI_CHANNEL_1_MASK + LR1110_WIFI_CHANNEL_2_MASK + LR1110_WIFI_CHANNEL_3_MASK + LR1110_WIFI_CHANNEL_4_MASK + LR1110_WIFI_CHANNEL_5_MASK + LR1110_WIFI_CHANNEL_6_MASK;
  lr1110_wifi_channel_mask_t channels = 0b0011111111111111;
  lr_status = lr1110_wifi_scan(&radio, LR1110_WIFI_TYPE_SCAN_B_G_N, channels, LR1110_WIFI_SCAN_MODE_FULL_BEACON, 16, 4, 110, false);

  lr_status = lr1110_system_get_errors(&radio, &lr_errors);

  uint8_t nb_results = 0;
  lr1110_wifi_get_nb_results(&radio, &nb_results);
  lr1110_wifi_extended_full_result_t all_results[LR1110_WIFI_MAX_RESULTS] = {0};
  lr1110_wifi_read_extended_full_results(&radio, 0, nb_results, all_results);
  */




  // lr1110_hal_wakeup( &radio );
//  HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
//  HAL_Delay(500);
//  HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
//  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
//  HAL_Delay(500);
//  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);

  // lr1110_system_set_tcxo_mode(&radio, LR1110_SYSTEM_TCXO_CTRL_1_6V, 10000);

  // lr1110_hal_bootstrap( &radio );


  /* START LR1110 Firmware Update */
  // Rebooting into bootstrap
  lr1110_hal_bootstrap( &radio );
  lr1110_hal_reset( &radio );
  HAL_Delay(500);
  lr1110_bootloader_get_version(&radio, &version_system);

  if( version_system.type != 0xDF )
  {
	lr1110_bootloader_reboot( &radio, true );
	HAL_Delay(500);
  }

  /*
  lr1110_bootloader_get_version(&radio, &version_system);
  lr1110_hal_bootstrap(&radio);
  lr1110_bootloader_get_version(&radio, &version_system);
  lr1110_modem_get_version(&radio, &modem);
  */

  // STEP 1
  lr1110_bootloader_erase_flash( &radio );
  lr1110_bootloader_get_version(&radio, &version_system);

  lr_status = lr1110_bootloader_write_flash_encrypted_full( &radio, 0, lr1110_firmware_image, ( uint32_t ) LR1110_FIRMWARE_IMAGE_SIZE );
  lr1110_hal_reset( &radio );
  lr1110_bootloader_get_version(&radio, &version_system);

  // STEP 2 - offset is taken from STEP 1
  lr_status = lr1110_bootloader_write_flash_encrypted_full( &radio, 163328, lr1110_firmware_image, ( uint32_t ) LR1110_FIRMWARE_IMAGE_SIZE );


  // Rebooting out of bootstrap
  lr1110_bootloader_reboot( &radio, false );
  lr1110_modem_system_reboot(&radio, false);
  HAL_Delay(2500);

  /*
  lr1110_hal_reset( &radio );
  lr1110_bootloader_reboot( &radio, false );
  lr1110_bootloader_get_version(&radio, &version_system);
  */
  lr1110_modem_get_version(&radio, &modem_version);


  // lr1110_bootloader_read_pin( &radio, pin );
  // lr1110_bootloader_read_chip_eui( &radio, chip_eui );
  // lr1110_bootloader_read_join_eui( &radio, join_eui );
  /**/

  // lr1110_system_get_temp(&radio, &lr_temp);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NSS_Pin|GREEN_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(QI_EN_GPIO_Port, QI_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RED_LED_Pin|LR_NRESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QI_EN_Pin */
  GPIO_InitStruct.Pin = QI_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(QI_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : QI_INT_Pin QI_PDETB_Pin */
  GPIO_InitStruct.Pin = QI_INT_Pin|QI_PDETB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ACC_INT1_Pin ACC_INT2_Pin */
  GPIO_InitStruct.Pin = ACC_INT1_Pin|ACC_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BATT_CHARGING_Pin */
  GPIO_InitStruct.Pin = BATT_CHARGING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BATT_CHARGING_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LR_NRESET_Pin */
  GPIO_InitStruct.Pin = LR_NRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LR_NRESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HALL_SOUTH_Pin */
  GPIO_InitStruct.Pin = HALL_SOUTH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HALL_SOUTH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LR_IRQ__DIO9_Pin */
  GPIO_InitStruct.Pin = LR_IRQ__DIO9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LR_IRQ__DIO9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VBAT_MCU_Pin */
  GPIO_InitStruct.Pin = VBAT_MCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBAT_MCU_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
