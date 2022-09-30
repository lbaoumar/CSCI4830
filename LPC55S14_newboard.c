/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    LPC55S14_newboard.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC55S14.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "fsl_power.h"
#include "fsl_spi.h"
#include <stdbool.h>
#include "fsl_usart.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_I2C_MASTER_BASE    (I2C1_BASE)
#define I2C_MASTER_CLOCK_FREQUENCY (12000000)
#define WAIT_TIME                  20U
#define EXAMPLE_I2C_MASTER ((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)

//#define I2C_MASTER_SLAVE_ADDR_7BIT  0x60U		//for camera is 30 and for TOF ix 60
#define I2C_BAUDRATE               100000U
#define I2C_DATA_LENGTH            2U


#define EXAMPLE_SPI_MASTER          SPI3
#define EXAMPLE_SPI_MASTER_IRQ      FLEXCOMM3_IRQn
#define EXAMPLE_SPI_MASTER_CLK_SRC  kCLOCK_Flexcomm3
#define EXAMPLE_SPI_MASTER_CLK_FREQ CLOCK_GetFlexCommClkFreq(3U)
#define EXAMPLE_SPI_SSEL            1
#define EXAMPLE_SPI_SPOL            kSPI_SpolActiveAllLow

#define DEMO_USART          USART2
#define DEMO_USART_CLK_SRC  kCLOCK_Flexcomm2
#define DEMO_USART_CLK_FREQ CLOCK_GetFlexCommClkFreq(2U)

 /* Variables
 ******************************************************************************/
uint8_t I2C_MASTER_SLAVE_ADDR_7BIT = 0x60U;		//for camera is 30 and for TOF ix 60
uint8_t g_master_txBuff[I2C_DATA_LENGTH ]; //changed from data length to 1 to check if camera send works
uint8_t g_master_rxBuff[I2C_DATA_LENGTH ];
uint8_t tof_value = 0x00;					//this variable might be used to send the value of tof through uart'
uint8_t g_master_txBuff_cam[1];

//uart variables
uint8_t txbuff[6]; //for testing
 uint8_t ch;
 usart_config_t config;
 uint8_t cam_data[2];
 uint8_t cam_data_img[5];

uint8_t rxbuff[20] = {0};

//SPI variables
#define BUFFER_SIZE (5)
static uint8_t srcBuff[BUFFER_SIZE];
static uint8_t destBuff[BUFFER_SIZE];


/*******************************************************************************
 * Code
 ******************************************************************************/
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
i2c_master_config_t masterConfig;
status_t reVal        = kStatus_Fail;

//global variable for spi
spi_master_config_t userConfig = {0};
uint32_t srcFreq               = 0;
uint32_t i                     = 0;
//uint32_t err                   = 0;
spi_transfer_t xfer            = {0};

int tof_tx( status_t reVal, uint8_t deviceAddress)
{


	if (kStatus_Success == I2C_MasterStart(EXAMPLE_I2C_MASTER, I2C_MASTER_SLAVE_ADDR_7BIT, kI2C_Write))
	         {
	             /* subAddress = 0x01, data = g_master_txBuff - write to slave.
	               start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop */
	             reVal = I2C_MasterWriteBlocking(EXAMPLE_I2C_MASTER, &deviceAddress, 1, kI2C_TransferNoStopFlag);
	             if (reVal != kStatus_Success)
	                    {
	                        return -1;
	                    }
	             reVal = I2C_MasterWriteBlocking(EXAMPLE_I2C_MASTER, g_master_txBuff, I2C_DATA_LENGTH  , kI2C_TransferNoStopFlag);
	             if (reVal != kStatus_Success)
	                    {
	                        return -1;
	                    }
	             reVal = I2C_MasterStop(EXAMPLE_I2C_MASTER);
	             if (reVal != kStatus_Success)
	                    {
	                        return -1;
	                    }
	         }
	return 0;
}

int tof_tx_cam( status_t reVal, uint8_t deviceAddress)
{


	if (kStatus_Success == I2C_MasterStart(EXAMPLE_I2C_MASTER, I2C_MASTER_SLAVE_ADDR_7BIT, kI2C_Write))
	         {
	             /* subAddress = 0x01, data = g_master_txBuff - write to slave.
	               start + slaveaddress(w) + subAddress + length of data buffer + data buffer + stop */
	             reVal = I2C_MasterWriteBlocking(EXAMPLE_I2C_MASTER, &deviceAddress, 1, kI2C_TransferNoStopFlag);
	             if (reVal != kStatus_Success)
	                    {
	                        return -1;
	                    }
	             reVal = I2C_MasterWriteBlocking(EXAMPLE_I2C_MASTER, g_master_txBuff_cam, I2C_DATA_LENGTH - 1  , kI2C_TransferNoStopFlag);
	             if (reVal != kStatus_Success)
	                    {
	                        return -1;
	                    }
	             reVal = I2C_MasterStop(EXAMPLE_I2C_MASTER);
	             if (reVal != kStatus_Success)
	                    {
	                        return -1;
	                    }
	         }
	return 0;
}

int tof_rd(status_t reVal, uint8_t deviceAddress)
{

	 /* Receive blocking data from slave */
	            /* subAddress = 0x01, data = g_master_rxBuff - read from slave.
	              start + slaveaddress(w) + subAddress + repeated start + slaveaddress(r) + rx data buffer + stop */
	// I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);
	if (kStatus_Success == I2C_MasterStart(EXAMPLE_I2C_MASTER, I2C_MASTER_SLAVE_ADDR_7BIT, kI2C_Write))
	{


		reVal = I2C_MasterWriteBlocking(EXAMPLE_I2C_MASTER, &deviceAddress, 1, kI2C_TransferNoStopFlag);

		if (reVal != kStatus_Success)
		{
			return -1;
		}

		reVal = I2C_MasterRepeatedStart(EXAMPLE_I2C_MASTER, I2C_MASTER_SLAVE_ADDR_7BIT, kI2C_Read);
		if (reVal != kStatus_Success)
		{
			return -1;
		}

		reVal = I2C_MasterReadBlocking(EXAMPLE_I2C_MASTER, g_master_rxBuff, I2C_DATA_LENGTH  , kI2C_TransferNoStopFlag);
		if (reVal != kStatus_Success)
		{
			return -1;
		}

		reVal = I2C_MasterStop(EXAMPLE_I2C_MASTER);
		if (reVal != kStatus_Success)
		{
		   return -1;
		}

	}
	return 0;
}

/**this function is for the Time of flight */
int tof()
{
	/*
	  * masterConfig.debugEnable = false;
	  * masterConfig.ignoreAck = false;
	  * masterConfig.pinConfig = kI2C_2PinOpenDrain;
	  * masterConfig.baudRate_Bps = 100000U;
	  * masterConfig.busIdleTimeout_ns = 0;
	  * masterConfig.pinLowTimeout_ns = 0;
	  * masterConfig.sdaGlitchFilterWidth_ns = 0;
	  * masterConfig.sclGlitchFilterWidth_ns = 0;
	  */
	 I2C_MasterGetDefaultConfig(&masterConfig);

	 /* Change the default baudrate configuration */
	 masterConfig.baudRate_Bps = I2C_BAUDRATE;

	 /* Initialize the I2C master peripheral */
	 I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);
	 //Send master blocking data to slave register 4
	 //for some reasons, i am getting a nak at 1st call of rd or tx(fixed)
	 //need to call twice to work at first
	 uint8_t deviceAddress = 0x04U;
	 g_master_txBuff[0] = 0x47U;	//low byte
	 g_master_txBuff[1] = 0x00U;	//high byte

	 //send data to command register
	 tof_tx( reVal, deviceAddress);

	 /* Send master blocking data to slave register 3 */
	// I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);
	deviceAddress = 0x03U;
	g_master_txBuff[0] = 0x02U;	//low byte
	g_master_txBuff[1] = 0x04U;	//high byte
	//send data to command register
	 tof_tx( reVal, deviceAddress);

   //read the data from device address
	deviceAddress = 0x08U;	//CMD that i will read data from
	//this variable must be global. test if it gets the value of tof

	while(1)
	{
	 tof_rd( reVal, deviceAddress);
	 for (uint32_t i = 0U; i < I2C_DATA_LENGTH ; i++)
				 {

					 PRINTF("0x%2x  ", g_master_rxBuff[i]);
					 //PRINTF("\r\n\r\n");
					tof_value = g_master_rxBuff[0];
					if (tof_value == 0xFF)
					{
						I2C_MASTER_SLAVE_ADDR_7BIT = 0x30U;	//camera address
						camera();
						return 0;
					}
					//PRINTF("0x%2x  ", tof_value);
					// PRINTF("%d", value);
					else if( i == 0)
					{
						//setting the value in uart tx buffer
						//to display the value of tof to seven segment
						txbuff[0] = 'T';
						txbuff[1] = 'O';
						txbuff[2] = 'F';
						txbuff[3] = 0;
						txbuff[4] = 1;
						txbuff[5] = tof_value;
						uart();
					}
				 }
				 PRINTF("\r\n\r\n");
	}
	return 0;
}

//TODO sending the data from camera to uart
int camera()
{
	//initializing the camera
	/*
		  * masterConfig.debugEnable = false;
		  * masterConfig.ignoreAck = false;
		  * masterConfig.pinConfig = kI2C_2PinOpenDrain;
		  * masterConfig.baudRate_Bps = 100000U;
		  * masterConfig.busIdleTimeout_ns = 0;
		  * masterConfig.pinLowTimeout_ns = 0;
		  * masterConfig.sdaGlitchFilterWidth_ns = 0;
		  * masterConfig.sclGlitchFilterWidth_ns = 0;
		  */
		 I2C_MasterGetDefaultConfig(&masterConfig);

		 /* Change the default baudrate configuration */
		 masterConfig.baudRate_Bps = I2C_BAUDRATE;

		 /* Initialize the I2C master peripheral */
		 I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);
		 //Send master blocking data to slave register 4
		 //for some reasons, i am getting a nak at 1st call of rd or tx(fixed)
		 //need to call twice to work at first
		 uint8_t deviceAddress = 0xffU;
		// g_master_txBuff[0] = 0x01U;
		// tof_tx( reVal, deviceAddress);
//
//		 deviceAddress = 0x12U;
//		 g_master_txBuff[0] = 0x80U;
//		 tof_tx( reVal, deviceAddress);
		 int count = 0;
		 uint8_t cam[196][2] = {{0xff, 0x01},{0x12, 0x80},
				 {0xff, 0x0},{0x2c, 0xff},
				 {0x2e, 0xdf},
				 {0xff, 0x1},
				 {0x3c, 0x32},
				 {0x11, 0x0},
				 {0x09, 0x2},
				 {0x04, 0xa8},
				 {0x13, 0xe5},
				 {0x14, 0x48},
				 {0x2c, 0xc},
				 {0x33, 0x78},
				 {0x3a, 0x33},
				 {0x3b, 0xfb},
				 {0x3e, 0x0},
				 {0x43, 0x11},
				 {0x16, 0x10},
				 {0x39, 0x2},
				 {0x35, 0x88},
				 {0x22, 0xa},
				 {0x37, 0x40},
				 {0x23, 0x0},
				 {0x34, 0xa0},
				 {0x06, 0x2},
				 {0x06, 0x88},
				 {0x07, 0xc0},
				 {0x0d, 0xb7},
				 {0x0e, 0x1},
				 {0x4c, 0x0},
				 {0x4a, 0x81},
				 {0x21, 0x99},
				 {0x24, 0x40},
				 {0x25, 0x38},
				 {0x26, 0x82},
				 {0x5c, 0x0},
				 {0x63, 0x0},
				 {0x46, 0x22},
				 {0x0c, 0x3a},
				 {0x5d, 0x55},
				 {0x5e, 0x7d},
				 {0x5f, 0x7d},
				 {0x60, 0x55},
				 {0x61, 0x70},
				 {0x62, 0x80},
				 {0x7c, 0x5},
				 {0x20, 0x80},
				 {0x28, 0x30},
				 {0x6c, 0x0},
				 {0x6d, 0x80},
				 {0x6e, 0x0},
				 {0x70, 0x2},
				 {0x71, 0x94},
				 {0x73, 0xc1},
				 {0x3d, 0x34},
				 {0x12, 0x4},
				 {0x5a, 0x57},
				 {0x4f, 0xbb},
				 {0x50, 0x9c},
				 {0xff, 0x0},
				 {0xe5, 0x7f},
				 {0xf9, 0xc0},
				 {0x41, 0x24},
				 {0xe0, 0x14},
				 {0x76, 0xff},
				 {0x33, 0xa0},
				 {0x42, 0x20},
				 {0x43, 0x18},
				 {0x4c, 0x0},
				 {0x87, 0xd0},
				 {0x88, 0x3f},
				 {0xd7, 0x3},
				 {0xd9, 0x10},
				 {0xd3, 0x82},
				 {0xc8, 0x8},
				 {0xc9, 0x80},
				 {0x7c, 0x0},
				 {0x7d, 0x0},
				 {0x7c, 0x3},
				 {0x7d, 0x48},
				 {0x7d, 0x48},
				 {0x7c, 0x8},
				 {0x7d, 0x20},
				 {0x7d, 0x10},
				 {0x7d, 0xe},
				 {0x90, 0x0},
				 {0x91, 0xe},
				 {0x91, 0x1a},
				 {0x91, 0x31},
				 {0x91, 0x5a},
				 {0x91, 0x69},
				 {0x91, 0x75},
				 {0x91, 0x7e},
				 {0x91, 0x88},
				 {0x91, 0x8f},
				 {0x91, 0x96},
				 {0x91, 0xa3},
				 {0x91, 0xaf},
				 {0x91, 0xc4},
				 {0x91, 0xd7},
				 {0x91, 0xe8},
				 {0x91, 0x20},
				 {0x92, 0x0},
				 {0x93, 0x6},
				 {0x93, 0xe3},
				 {0x93, 0x3},
				 {0x93, 0x3},
				 {0x93, 0x0},
				 {0x93, 0x2},
				 {0x93, 0x0},
				 {0x93, 0x0},
				 {0x93, 0x0},
				 {0x93, 0x0},
				 {0x93, 0x0},
				 {0x93, 0x0},
				 {0x93, 0x0},
				 {0x96, 0x0},
				 {0x97, 0x8},
				 {0x97, 0x19},
				 {0x97, 0x2},
				 {0x97, 0xc},
				 {0x97, 0x24},
				 {0x97, 0x30},
				 {0x97, 0x28},
				 {0x97, 0x26},
				 {0x97, 0x2},
				 {0x97, 0x98},
				 {0x97, 0x80},
				 {0x97, 0x0},
				 {0x97, 0x0},
				 {0xa4, 0x0},
				 {0xa8, 0x0},
				 {0xc5, 0x11},
				 {0xc6, 0x51},
				 {0xbf, 0x80},
				 {0xc7, 0x10},
				 {0xb6, 0x66},
				 {0xb8, 0xa5},
				 {0xb7, 0x64},
				 {0xb9, 0x7c},
				 {0xb3, 0xaf},
				 {0xb4, 0x97},
				 {0xb5, 0xff},
				 {0xb0, 0xc5},
				 {0xb1, 0x94},
				 {0xb2, 0xf},
				 {0xc4, 0x5c},
				 {0xa6, 0x0},
				 {0xa7, 0x20},
				 {0xa7, 0xd8},
				 {0xa7, 0x1b},
				 {0xa7, 0x31},
				 {0xa7, 0x0},
				 {0xa7, 0x18},
				 {0xa7, 0x20},
				 {0xa7, 0xd8},
				 {0xa7, 0x19},
				 {0xa7, 0x31},
				 {0xa7, 0x0},
				 {0xa7, 0x18},
				 {0xa7, 0x20},
				 {0xa7, 0xd8},
				 {0xa7, 0x19},
				 {0xa7, 0x31},
				 {0xa7, 0x0},
				 {0xa7, 0x18},
				 {0x7f, 0x0},
				 {0xe5, 0x1f},
				 {0xe1, 0x77},
				 {0xdd, 0x7f},
				 {0xc2, 0xe},
				 {0xff, 0x0},
				 {0xe0, 0x4},
				 {0xc0, 0xc8},
				 {0xc1, 0x96},
				 {0x86, 0x3d},
				 {0x51, 0x90},
				 {0x52, 0x2c},
				 {0x53, 0x0},
				 {0x54, 0x0},
				 {0x55, 0x88},
				 {0x57, 0x0},
				 {0x50, 0x92},
				 {0x5a, 0x50},
				 {0x5b, 0x3c},
				 {0x5c, 0x0},
				 {0xd3, 0x4},
				 {0xe0, 0x0},
				 {0xff, 0x0},
				 {0x05, 0x0},
				 {0xda, 0x8},
				 {0xd7, 0x3},
				 {0xe0, 0x0},
				 {0x05, 0x0},
				 {0xff,0xff}};
		 //initializing the camera
		 //uint8_t g_master_txBuff[];
		 while(count < 196)
		 {
			 deviceAddress = cam[count][0];
			 g_master_txBuff_cam[0] = cam[count][1];
			 tof_tx_cam( reVal, deviceAddress);
			 count++;
			 if(count == 2)
			 {
				 SDK_DelayAtLeastUs(150000UL, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);	//delay
			 }

		 }
		 //writing in camera through SPI
		 uint8_t CMD_reg[2] = {0x84,0x02};
		 xfer.txData      = CMD_reg;
		xfer.rxData      = destBuff;
		xfer.dataSize    = 2; //i changed destBuff to srcBuff just to check
		xfer.configFlags = kSPI_FrameAssert;
		SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer);
		// reading value from camera fifo flag
		 SDK_DelayAtLeastUs(150000UL, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
		 uint8_t CMD_read[2] = {0x41};
		xfer.txData      = CMD_read;
		xfer.rxData      = destBuff;
		xfer.dataSize    = 2; //i changed destBuff to srcBuff just to check
		xfer.configFlags = kSPI_FrameAssert;
		SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer);

		//read value from reg 44 to 42 to get 02 28 00
		uint8_t CMD_read_reg_fourfour[2] = {0x44};
		xfer.txData      = CMD_read_reg_fourfour;
		xfer.rxData      = destBuff;
		xfer.dataSize    = 2; //i changed destBuff to srcBuff just to check
		xfer.configFlags = kSPI_FrameAssert;
		SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer);

		uint8_t CMD_read_reg_fourthree[1] = {0x43};
		xfer.txData      = CMD_read_reg_fourthree;
		xfer.rxData      = destBuff;
		xfer.dataSize    = 2; //i changed destBuff to srcBuff just to check
		xfer.configFlags = kSPI_FrameAssert;
		SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer);

		uint8_t CMD_read_reg_fourtwo[2] = {0x42};
		xfer.txData      = CMD_read_reg_fourtwo;
		xfer.rxData      = destBuff;
		xfer.dataSize    = 2; //i changed destBuff to srcBuff just to check
		xfer.configFlags = kSPI_FrameAssert;
		SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer);

		//read data from the bus burst read register(call it multiple time and it increment
		//the pointer for the data
		uint8_t CMD_read_reg_burst[2] = {0x3C};
		count = 0;
		cam_data_img[0] = 'I';
		cam_data_img[1] = 'M';
		cam_data_img[2] = 'G';
		cam_data_img[3] = 0;
		cam_data_img[4] = 0;


		uint8_t data_array[641];

		for(count = 0; count < 240; count++)
		{
			xfer.txData      = CMD_read_reg_burst;
			xfer.rxData      = destBuff;
			xfer.dataSize    = 2; //i changed destBuff to srcBuff just to check
			xfer.configFlags = kSPI_FrameAssert;
			SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer);
			if(count == 0)
			{
				uart_img();
			}
			if(count < 200 && count <= 40)
			{
				for(int i = 201; i <441; i = i+2)
				{
				cam_data[0] = destBuff[0];
				cam_data[1] = destBuff[1];
				uart_cam();
				}

			}
			count++;

		}
	return 0;
}

/** this the uart function**/
int uart_cam()
{
	//uart code

		 /*
	 * config.baudRate_Bps = 115200U;
	 * config.parityMode = kUSART_ParityDisabled;
	 * config.stopBitCount = kUSART_OneStopBit;
	 * config.loopback = false;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */
	USART_GetDefaultConfig(&config);
	config.baudRate_Bps = 921600;
	config.enableTx     = true;
	config.enableRx     = true;
	USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);

	USART_WriteBlocking(DEMO_USART, cam_data, sizeof(cam_data)); //was minus 1
//get stock after sending img. need to be fixed
	while (1)
		{

		 USART_ReadBlocking(DEMO_USART, &ch, 1);	//ch holds the value from esp tx to rx of NXP
		//PRINTF("0x%2x", ch);
		 if(ch == 'R')
		 {
			 break;	//if the esp is ready to receive the next byte, get out of the loop
		 }
		//USART_WriteBlocking(DEMO_USART, &ch, 1);
		}

	return 0;
}

int uart_img()
{
	//uart code

		 /*
	 * config.baudRate_Bps = 115200U;
	 * config.parityMode = kUSART_ParityDisabled;
	 * config.stopBitCount = kUSART_OneStopBit;
	 * config.loopback = false;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */
	USART_GetDefaultConfig(&config);
	config.baudRate_Bps = 921600;
	config.enableTx     = true;
	config.enableRx     = true;
	USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);

	USART_WriteBlocking(DEMO_USART, cam_data_img, sizeof(cam_data_img)); //was minus 1

	while (1)
		{

		 USART_ReadBlocking(DEMO_USART, &ch, 1);	//ch holds the value from esp tx to rx of NXP
		// PRINTF("0x%2x", ch);
		 if(ch == 'R')
		 {
			 return 0;	//if the esp is ready to receive the next byte, get out of the loop
		 }
		//USART_WriteBlocking(DEMO_USART, &ch, 1);
		}
//	PRINTF("\n");

	return 0;
}


/** this the uart function**/
int uart()
{
	//uart code

		 /*
	 * config.baudRate_Bps = 115200U;
	 * config.parityMode = kUSART_ParityDisabled;
	 * config.stopBitCount = kUSART_OneStopBit;
	 * config.loopback = false;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */
	USART_GetDefaultConfig(&config);
	config.baudRate_Bps = 921600;
	config.enableTx     = true;
	config.enableRx     = true;
	USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);

	// PRINTF("0x%2x", txbuff[5]);
	USART_WriteBlocking(DEMO_USART, txbuff, sizeof(txbuff)); //was minus 1

	while (1)
		{

		 USART_ReadBlocking(DEMO_USART, &ch, 1);	//ch holds the value from esp tx to rx of NXP
		// PRINTF("0x%2x", ch);
		 if(ch == 'R')
		 {
			 return 0;	//if the esp is ready to receive the next byte, get out of the loop
		 }
//		//USART_WriteBlocking(DEMO_USART, &ch, 1);
		}
	PRINTF("\n");

	return 0;
}


int main(void) {


//	spi_master_config_t userConfig = {0};
//	uint32_t srcFreq               = 0;
//	uint32_t i                     = 0;
//	//uint32_t err                   = 0;
//	spi_transfer_t xfer            = {0};

	/* attach 12 MHz clock to FLEXCOMM1 (I2C master) */
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM1);
	/* reset FLEXCOMM for I2C */
	RESET_PeripheralReset(kFC1_RST_SHIFT_RSTn);

	//SPI config
	 /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
	//CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

	/* attach 12 MHz clock to SPI3 */
	CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
	/* reset FLEXCOMM for SPI */
	RESET_PeripheralReset(kFC3_RST_SHIFT_RSTn);

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

   // PRINTF("\r\nI2C board2board polling example -- Master transfer.\r\n");


         //call the time of flight function.
    /*
  	  * userConfig.enableLoopback = false;
  	  * userConfig.enableMaster = true;
  	  * userConfig.polarity = kSPI_ClockPolarityActiveHigh;
  	  * userConfig.phase = kSPI_ClockPhaseFirstEdge;
  	  * userConfig.direction = kSPI_MsbFirst;
  	  * userConfig.baudRate_Bps = 500000U;
  	  */
  	 SPI_MasterGetDefaultConfig(&userConfig);
  	 srcFreq            = EXAMPLE_SPI_MASTER_CLK_FREQ;
  	 userConfig.sselNum = (spi_ssel_t)EXAMPLE_SPI_SSEL;
  	 userConfig.sselPol = (spi_spol_t)EXAMPLE_SPI_SPOL;
  	 SPI_MasterInit(EXAMPLE_SPI_MASTER, &userConfig, srcFreq);

      tof();


//
//             /* Init Buffer*/
//		for (i = 0; i < BUFFER_SIZE; i++)
//		{
//			if(i == 0)
//			{
//				srcBuff[i] = 2;
//			}
//			if(i == 1)
//			{
//				srcBuff[i] = 1;	//address
//			}
//			if(i == 2)
//			{
//				srcBuff[i] = 0;	//address
//			}
//			if(i == 3)
//			{
//				srcBuff[i] = 0;	//address
//			}
//			if(i == 4)
//			{
//				srcBuff[i] = 0x15;	//data to write to memory
//			}
//		}
//
//
//		/*Start Transfer*/
//		//send CMD 6 first
//		uint8_t CMD_six[1] = {6};
//		xfer.txData      = CMD_six;
//		xfer.rxData      = destBuff;
//		xfer.dataSize    = 1; //i changed destBuff to srcBuff just to check
//		xfer.configFlags = kSPI_FrameAssert;
//	//	SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer);
//
//		//SPI_send(xfer, CMD_six);
//
//		//write data to address 000000
//		//SPI_send(xfer, srcBuff);
//		xfer.txData      = srcBuff;
//		xfer.rxData      = destBuff;
//		xfer.dataSize    = 5; //i changed destBuff to srcBuff just to check
//		xfer.configFlags = kSPI_FrameAssert;
//	//	SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer);
//
//		//send CMD to read data at adrr 000000
//		uint8_t CMD_rd[4];
//		for (i = 0; i < 4; i++)
//		{
//			if(i == 0)
//			{
//				CMD_rd[i] = 3;
//			}
//			if(i == 1)
//			{
//				CMD_rd[i] = 1;	//address
//			}
//			if(i == 2)
//			{
//				CMD_rd[i] = 0;	//address
//			}
//			if(i == 3)
//			{
//				CMD_rd[i] = 0;	//address
//			}
//			if(i == 4)
//			{
//				CMD_rd[i] = 0;	//address
//			}
//		}
//
//		//SPI_send(xfer, CMD_rd);
//		xfer.txData      = CMD_rd;
//		xfer.rxData      = destBuff;
//		xfer.dataSize    = 5; //i changed destBuff to srcBuff just to check
//		xfer.configFlags = kSPI_FrameAssert;
	//	SPI_MasterTransferBlocking(EXAMPLE_SPI_MASTER, &xfer);
//		for(i = 0; i < 5; i++)
//		{
//			PRINTF("%d\n", destBuff[i]);
//		}

/**** Camera code*/
		//camera();

//uart();
    while(1) {
        //i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
        PRINTF("DONE");
    }
    return 0 ;
}
