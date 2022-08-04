#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "bno055.h"
#include "string.h"
#include "CANSPI.h"
#include "MCP2515.h"
#include <stdlib.h>
#include "i2c-lcd.h"
#include "stm32_tm1637.h"
#include "MY_NRF24.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart6;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef htim1;
/* USER CODE END Includes */

const uint64_t TxpipeAddrs = 0xE8E8F0F0E1LL; // RF pipe to send
struct {
	int rpm;//for Motor Data
	float motor_torq,motor_torque_demand;//for IMU and Motor Data
	uint16_t motor_temp,motor_vol,batt_vol,motor_curr,batt_curr;//for telemetry
}mxTxData;//To send data
char RPM_RR[10];
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
int row=0;
int col=0;
void SystemClock_Config(void);


int main(void)
{
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_SPI3_Init();
  MX_I2C2_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();

  memset(txMessage, 0x0, sizeof(txMessage));
  txMessage.frame.dlc = 8;// Data length
//------------------------------------ **** TRANSMIT - ACK ****------------------------------------//
  nrf24_DebugUART_Init(huart2);
  NRF24_begin(GPIOB, GPIO_PIN_6, GPIO_PIN_4, hspi3); //어떤 핀에서 데이터 받아올지 정하기
  NRF24_setPALevel(RF24_PA_0dB);
	NRF24_setAutoAck(false);
	NRF24_setChannel(80);
	NRF24_setPayloadSize(16);
	NRF24_setDataRate(RF24_2MBPS);
	NRF24_openWritingPipe(TxpipeAddrs);
	NRF24_stopListening();
//------------------------------------ **** MCP2515 Setting ****------------------------------------//
  CANSPI_Initialize();
  while (1)
  {
//------------------------------------ **** CAN_Motor ****------------------------------------//
  	txMessage.frame.id = 0x205;// To Listen Motor PDO data
	  CANSPI_Transmit(&txMessage);// Send Tx Message to get PDO data

	  txMessage.frame.id = 0x114;// To Listen Motor PDO data
	  CANSPI_Transmit(&txMessage);// Send Tx Message to get PDO data

	  txMessage.frame.id = 0x112;// To Listen Motor PDO data
	  CANSPI_Transmit(&txMessage);// Send Tx Message to get PDO data

	  int nRecvMsg = 0;
	  while (nRecvMsg < 3)// If Message sent well
	  {
	  	if (CANSPI_Receive(&rxMessage))
	  	{
	  		nRecvMsg += 1;
	  		if (rxMessage.id == 0x205)
	  		{
	  			uint16_t RPM_1 = ((uint16_t)rxMessage.frame.data1 << 8) | rxMessage.frame.data0;
	  			uint16_t RPM_2 = ((uint16_t)rxMessage.frame.data3 << 8) | rxMessage.frame.data2;
	  			int RPM = ((int)RPM_2 << 16) | RPM_1; // Motor RPM data 4bytes
	  			uint16_t torque_buff = ((uint16_t)rxMessage.frame.data5 << 8) | rxMessage.frame.data4;// Motor Torque data 2bytes
	  			uint16_t temp_buff = ((uint16_t)rxMessage.frame.data7 << 8) | rxMessage.frame.data6;// Motor Temperature 2bytes
	  			sprintf(RPM_RR,"%d",RPM);
	  		}
	  		else if (rxMessage.id == 0x114)
	  		{
	  			uint16_t motor_vol1= ((uint16_t)rxMessage.frame.data1 << 8) | rxMessage1.frame.data0; //Motor_vol data 2bytes
	  			uint16_t motor_curr1= ((uint16_t)rxMessage.frame.data3 << 8) | rxMessage1.frame.data2; //Motor_curr data 2bytes
	  			uint16_t batt_vol1= ((uint16_t)rxMessage.frame.data5 << 8) | rxMessage1.frame.data4; //Batt vol data 2bytes
	  			uint16_t batt_curr1= ((uint16_t)rxMessage.frame.data7 << 8) | rxMessage1.frame.data6; //Batt_curr data 2bytes
	  		}
	  		else if (rxMessage.id == 0x112)
	  		{
					uint16_t torque_demand1= ((uint16_t)rxMessage2.frame.data1 << 8) | rxMessage.frame.data0; //torque_demand data 2bytes
					uint16_t throttle_input_vol1= ((uint16_t)rxMessage.frame.data3 << 8) | rxMessage.frame.data2; //throttle_input_vol data 2bytes
					uint8_t heatsink_temp1= (uint8_t)rxMessage.frame.data4; // heatsink_temp data 1bytes
					uint16_t motor_curr1= ((uint16_t)rxMessage.frame.data7 << 8) | rxMessage.frame.data6; //Motor current data 2bytes
	  		}
	  	}
	  	else
	  	{
	  		HAL_Delay(10);
	  	}
	  }
//------------------------------------ **** 7Segment ****------------------------------------//

  tm1637DisplayDecimal(RPM_RR,1);//Display 7-segment for motor data

//------------------------------------ **** RF_Trans ****------------------------------------//
  mxTxData.rpm = RPM;
  mxTxData.motor_temp = temp_buff;
  mxTxData.motor_torq = torque_buff * 0.1;
  mxTxData.motor_torque_demand = motor_torque_demand1*0.1;
  mxTxData.motor_vol= motor_vol1;
  mxTxData.batt_vol= batt_vol1;
  mxTxData.motor_curr= motor_curr1;
  mxTxData.batt_curr= batt_curr1;

  NRF24_write(mxTxData, sizeof(mxTxData));//
	}
}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
