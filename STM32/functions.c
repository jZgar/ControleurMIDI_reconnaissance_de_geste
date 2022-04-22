#include "functions.h"
#include "main.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi2;

extern UART_HandleTypeDef huart1;

char read_ch_MCP3008(int ch){
/* read the value of the channel "ch" of the MCP3008 an return a value between 0 and 254*/

	extern const uint8_t MCP3008_CH[8];
	unsigned char spi_buf[2];


	if (ch<8){// pour le premier MCP3008
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t *)&MCP3008_CH[ch], 1, 100);
		HAL_SPI_Receive(&hspi2, (uint8_t *)spi_buf, 1, 100);
		HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
	}
	else{// pour le second MCP3008
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t *)&MCP3008_CH[ch-8], 1, 100);
		HAL_SPI_Receive(&hspi2, (uint8_t *)spi_buf, 1, 100);
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_6, GPIO_PIN_SET);
	}


	if(spi_buf[0]==(char)255){spi_buf[0]=(char)254;}

	return spi_buf[0];
	}


void print_value_uart_1(int value){
	/*envoie des données contenues dans le tableau sur le bus uart1(st link) */

	int uart_buf_len;
	char uart_buf[50];

	uart_buf_len = sprintf(uart_buf,"Value : %d\r\n\n",value);
	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);
}

void print_char_uart_1(char* chaine){
		int uart_buf_len;
		char uart_buf[256];

		uart_buf_len = sprintf(uart_buf, chaine);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);
}

void send_matrix_uart_1(void){

	extern char spi_buf[256];
	int uart_buf_len;
	char uart_buf[256];

	uart_buf_len = sprintf(uart_buf, "%c", 255);
	HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);

	for(int k=0; k<256;k++){
		uart_buf_len = sprintf(uart_buf,"%c", (unsigned char)spi_buf[k]);
		HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);
	}


}


