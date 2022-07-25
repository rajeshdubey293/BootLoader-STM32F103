/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stm32f1xx_hal_flash_ex.h>
#include "usbd_cdc_if.h"
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
 UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define BL_DEBUG_MSG_EN 1

#define D_UART &huart1
//#define C_UART huart2

#define FLASH_ADDRESS_APP 		0x08008000U
#define FLASH_ADDRESS_UPDATE	0x08018000U
#define INVALID_PAGE 0x04
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Print_Msg(char *Format,...);
void Show_Bootloader_Menu(void);
void Bootloader_Jump_To_User_App(void);
HAL_StatusTypeDef Execute_Flash_Erase(uint8_t Number_Of_Sector);
HAL_StatusTypeDef Execute_Flash_Write(uint32_t *Data_Buffer, uint32_t Flash_Address, uint32_t Length);
HAL_StatusTypeDef Execute_Flash_Read(uint32_t Flash_Address, uint32_t *Rx_Buffer, uint32_t Number_Of_Words);
void Get_Binary_File_From_UART(void);
uint8_t Get_UART_Byte(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
GPIO_PinState Pin_Status = GPIO_PIN_SET;
HAL_StatusTypeDef Ret_Code;

uint8_t UART_Buffer = 0;
uint8_t UART_Data_Available = 0;
char Test_Write_Buffer[] = "Testing Flash Writing\n";
char Test_Read_Buffer[24] = {0};
uint8_t Length = 24;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&UART_Buffer, sizeof(UART_Buffer));
	Print_Msg("Press Button Within 3 Seconds to Boot Into Bootloader\r\n");
	Print_Msg("3\r\n");
	HAL_Delay(1000);
	Print_Msg("2\r\n");
	HAL_Delay(1000);
	Print_Msg("1\r\n");
	HAL_Delay(1000);
	if( Pin_Status == GPIO_PIN_RESET)
	{
		Print_Msg("********** Welcome to STM32F1xx Bootloader **********\r\n");
		Show_Bootloader_Menu();
	}
	else
	{
		Print_Msg("Booting into Application\r\n");
		Bootloader_Jump_To_User_App();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(UART_Data_Available == 1)
		{
			switch(UART_Buffer)
			{
			case '1':
				Print_Msg("Send Application Binary File\r\n");
				Get_Binary_File_From_UART();
				break;
			case '2':
				Print_Msg("Send Bootloader Binary File\r\n");
				break;
			case '3':
				Print_Msg("Booting into Application\r\n");
				Bootloader_Jump_To_User_App();
				break;
			default:
				Print_Msg("Invalid option, try again\r\n");
				Show_Bootloader_Menu();
				break;

			}
			UART_Data_Available = 0;
		}


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void Bootloader_Jump_To_User_App(void)
{
	//just a function pointer to hold the address of the reset handler of the user app.
	void (*App_Reset_Handler)(void);

	// 1. configure the MSP by reading the value from the base address of the sector 2
	uint32_t MSP_Value = *(volatile uint32_t *)FLASH_ADDRESS_APP;

	//This function comes from CMSIS.
	__set_MSP(MSP_Value);

	/* 2. Now fetch the reset handler address of the user application
	 * from the location FLASH_SECTOR2_BASE_ADDRESS+4
	 */
	uint32_t Reset_Handler_Address = *(volatile uint32_t *) (FLASH_ADDRESS_APP + 4);

	App_Reset_Handler = (void*) Reset_Handler_Address;

	//3. jump to reset handler of the user application
	App_Reset_Handler();

}
void  (char *Format,...)
{
#ifdef BL_DEBUG_MSG_EN
	char Str[80];

	/*Extract the the argument list using VA apis */
	va_list Args;
	va_start(Args, Format);
	vsprintf(Str, Format, Args);
	//HAL_UART_Transmit(D_UART,(uint8_t *)Str, strlen(Str),HAL_MAX_DELAY);
	CDC_Transmit_FS((uint8_t *)Str, strlen(Str));
	va_end(Args);
#endif
}
void Show_Bootloader_Menu(void)
{
	Print_Msg("1. Upgrade Application\n");
	Print_Msg("2. Upgrade Bootloader\n");
	Print_Msg("3. Boot into Application\n");
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		Pin_Status = GPIO_PIN_RESET;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&UART_Buffer, sizeof(UART_Buffer));
	UART_Data_Available = 1;
}
HAL_StatusTypeDef Execute_Flash_Erase(uint8_t Number_Of_Page)
{
	FLASH_EraseInitTypeDef FlashErase_Handle;
	uint32_t Sector_Error;
	HAL_StatusTypeDef Status;

	if( Number_Of_Page > 128 )
	{
		return INVALID_PAGE;
	}
	FlashErase_Handle.TypeErase = FLASH_TYPEERASE_PAGES;
	FlashErase_Handle.PageAddress = FLASH_ADDRESS_APP; // this is the initial page
	FlashErase_Handle.NbPages = Number_Of_Page;

	/*Get access to touch the flash registers */
	HAL_FLASH_Unlock();
	Status = HAL_FLASHEx_Erase(&FlashErase_Handle, &Sector_Error);
	HAL_FLASH_Lock();
	return Status;
}
HAL_StatusTypeDef Execute_Flash_Write(uint32_t *Data_Buffer, uint32_t Flash_Address, uint32_t Length)
{
	HAL_StatusTypeDef Status = HAL_OK;
	uint32_t Data_Buffer_Bytes_Count = 0;

	//We have to unlock flash module to get control of registers
	HAL_FLASH_Unlock();
	for(Data_Buffer_Bytes_Count = 0; Data_Buffer_Bytes_Count < Length ; Data_Buffer_Bytes_Count++)
	{
		//Here we program the flash word by word
		Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
				Flash_Address, (uint32_t)Data_Buffer[Data_Buffer_Bytes_Count] );
		Flash_Address = Flash_Address + 4;
	}
	HAL_FLASH_Lock();

	return Status;
}
HAL_StatusTypeDef Execute_Flash_Read(uint32_t Flash_Address, uint32_t *Rx_Buffer, uint32_t Number_Of_Words)
{
	while (1)
	{
		*Rx_Buffer = *(__IO uint32_t*)Flash_Address;
		Flash_Address += 4;
		Rx_Buffer++;
		if (!(Number_Of_Words--))
			break;
	}
	return HAL_OK;
}
void Get_Binary_File_From_UART(void)
{
	uint8_t Temp_Buffer[100] = {0x00};
	uint8_t UART_Byte_Count = 0;
	uint32_t Flash_Address = FLASH_ADDRESS_UPDATE;
	while(1)
	{
		if(UART_Data_Available == 1)
		{
			if(UART_Buffer == '\n')
				break;
			Temp_Buffer[UART_Byte_Count++] = UART_Buffer;
			if(UART_Byte_Count == 100)
				break;
#if 0
		if(UART_Byte_Count == 3)
		{
			UART_Byte_Count = 0;
			Execute_Flash_Write(Temp_Buffer, Flash_Address, 4);
			Flash_Address = Flash_Address + 4;
		}
#endif
			UART_Data_Available = 0;
		}

	}
}
uint8_t Get_UART_Byte(void)
{
	uint8_t UART_Temp_Buffer = 0;;
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&UART_Temp_Buffer, sizeof(UART_Temp_Buffer));
	return UART_Temp_Buffer;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
