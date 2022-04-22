/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "functions.h"
#include "mcp23017.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart1;
extern MCP23017_HandleTypeDef hmcp;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	//extern const uint8_t MCP3008_CH[8];
	//char uart_buf[50];
	extern char spi_buf[256];
	//int uart_buf_len;
	extern const uint8_t MCP23017_A[16];
	extern const uint8_t MCP23017_B[16];

	// Say something
	/*
	uart_buf_len = sprintf(uart_buf, "SPI Test\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);
	*/


   // Read ADC value register
	/*
   HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);
   HAL_SPI_Transmit(&hspi2, (uint8_t *)&MCP3008_CH[1], 1, 100);
   HAL_SPI_Receive(&hspi2, (uint8_t *)spi_buf, 1, 100);
   HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
   */

	//spi_buf[0]= read_ch_MCP3008(0);

   // Print out read values
	/*
   uart_buf_len = sprintf(uart_buf,
                           "Received_1 %u\r\n\n",
                           (unsigned int)spi_buf[0]);
   HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);
	*/
	//print_value_uart_1((int)spi_buf[0]);
	/*
	hmcp.gpio[MCP23017_PORTB] = MCP23017_B[0];
	hmcp.gpio[MCP23017_PORTA] = MCP23017_A[0];

	mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
	mcp23017_write_gpio(&hmcp, MCP23017_PORTB);
	*/


	for (int k=0;k<16;k++){


		hmcp.gpio[MCP23017_PORTB] = MCP23017_B[k];
		hmcp.gpio[MCP23017_PORTA] = MCP23017_A[k];

		mcp23017_write_gpio(&hmcp, MCP23017_PORTA);
		mcp23017_write_gpio(&hmcp, MCP23017_PORTB);


		for (int p=0;p<16;p++){
			spi_buf[16*k+p] = read_ch_MCP3008(p);

			//uart_buf_len = sprintf(uart_buf,"%c",0b00000000spi_buf[16*k+p]*);
			//HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);

		}



		//print_char_uart_1("\r\n");
	}


	//print_char_uart_1("\r\n");
	send_matrix_uart_1();
	//print_char_uart_1("\r\n\n");





  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
