/*
 ## Cypress FX3 Camera Kit source file (sensor.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2012,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file implements the I2C based driver for the MT9M114 image sensor used
   in the FX3 HD 720p camera kit.

   Please refer to the Aptina MT9M114 sensor datasheet for the details of the
   I2C commands used to configure the sensor.
 */

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3i2c.h>
#include <cyu3spi.h> //Added by Daniel 4_9_2015
#include <cyu3types.h>
#include <cyu3gpio.h>
#include <cyu3utils.h>

#include "sensor.h"


/* global and externals */
uint16_t  CyPythonID;
extern CyU3PSpiConfig_t spiConfig;								/* make it global to change it in the transmission */

//static void Step1_Post_Reset (void);
//static void Step2_PLL_Timing (void);
//static void Step3_Recommended (void);
//static void Step4_APGA (void);
//static void Step5_AWB_CCM (void);
//static void Step7_PIPE_Preference (void);
//static void Step8_Features (void);

/* This function inserts a delay between successful I2C transfers to prevent
   false errors due to the slave being busy.
 */
static void
SensorI2CAccessDelay (
        CyU3PReturnStatus_t status)
{
    /* Add a 4us delay if the I2C operation that preceded this call was successful. */
    if (status == CY_U3P_SUCCESS)
        CyU3PBusyWait (4);
}


void
CyFxSpiPythonWord ( uint16_t SpiPyRegister , uint16_t SpiPyData16 )
{

	uint8_t bufferTemp[2];

	SpiPyRegister <<= 1;				//shift to add WRITE/READ bit
	SpiPyRegister |= 0x0001;			// add WRITE bit

	bufferTemp[0] = SpiPyRegister; //|= 0x40;		//A6 - A0 - READ/WRITE -> READ = 0 ~ WRITE = 1
	bufferTemp[1] = SpiPyRegister >> 8;			    //A8 - A7

	CyU3PSpiSetSsnLine (CyFalse);

	CyU3PSpiTransmitWords (bufferTemp, 2);

	spiConfig.wordLen = 16;
	spiConfig.ssnPol     = CyTrue; //False is active low
	CyU3PSpiSetConfig (&spiConfig, NULL);

	bufferTemp[0] = SpiPyData16;//buffer[1];		//D7  - D0
	bufferTemp[1] = SpiPyData16>>8;//buffer[0];		//D15 - D8

	CyU3PSpiTransmitWords (bufferTemp, 2);


	spiConfig.wordLen = 10;
	spiConfig.ssnPol     = CyFalse; //False is active low
	spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
	CyU3PSpiSetConfig (&spiConfig, NULL);

	for( int CyWait = 0 ; CyWait < 100 ; CyWait++)
	{
		__nop();
	}



}

uint16_t
CyFxSpiPythonRWord ( uint16_t SpiPyCmd )
{

	uint8_t 	bufferTemp[2];
	uint16_t 	SpiPythonRdWord;

	SpiPyCmd <<= 1;				//shift to add WRITE/READ bit
	//SpiPyCmd |= 0x0000;			// add READ bit

	bufferTemp[0] = SpiPyCmd; //|= 0x40;		//A6 - A0 - READ/WRITE -> READ = 0 ~ WRITE = 1
	bufferTemp[1] = SpiPyCmd >> 8;			    //A8 - A7


	CyU3PSpiSetSsnLine (CyFalse);

	CyU3PSpiTransmitWords (bufferTemp, 2);

	spiConfig.wordLen = 16;
	spiConfig.ssnPol     = CyTrue; //False is active low
	spiConfig.cpha       = CyTrue;
	CyU3PSpiSetConfig (&spiConfig, NULL);

	//bufferTemp[0] = SpiPyData16;//buffer[1];		//D7  - D0
	//bufferTemp[1] = SpiPyData16>>8;//buffer[0];		//D15 - D8

	CyU3PSpiReceiveWords (bufferTemp, 2);


	spiConfig.wordLen = 10;
	spiConfig.ssnPol     = CyFalse; //False is active low
	spiConfig.cpha       = CyFalse;
	spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
	CyU3PSpiSetConfig (&spiConfig, NULL);


	for( int CyWait = 0 ; CyWait < 100 ; CyWait++)
	{
		__nop();
	}


	SpiPythonRdWord = bufferTemp[1];
	SpiPythonRdWord <<= 8;
	SpiPythonRdWord |= bufferTemp[0];

	return SpiPythonRdWord;



}

/* Write to an I2C slave with two bytes of data. */
CyU3PReturnStatus_t
SensorWrite2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t highData,
        uint8_t lowData)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t  preamble;
    uint8_t buf[2];

    /* Validate the I2C slave address. */
    if ((slaveAddr != SENSOR_ADDR_WR) && (slaveAddr != I2C_MEMORY_ADDR_WR))
    {
      //  CyU3PDebugPrint (4, "I2C Slave address is not valid!\n");
        return 1;
    }

    /* Set the parameters for the I2C API access and then call the write API. */
    preamble.buffer[0] = slaveAddr;
    preamble.buffer[1] = highAddr;
    preamble.buffer[2] = lowAddr;
    preamble.length    = 3;             /*  Three byte preamble. */
    preamble.ctrlMask  = 0x0000;        /*  No additional start and stop bits. */

    buf[0] = highData;
    buf[1] = lowData;

    apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, 2, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return apiRetStatus;
}


CyU3PReturnStatus_t
SensorWrite (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t count,
        uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    /* Validate the I2C slave address. */
    if ((slaveAddr != SENSOR_ADDR_WR) && (slaveAddr != I2C_MEMORY_ADDR_WR))
    {
       // CyU3PDebugPrint (4, "I2C Slave address is not valid!\n");
        return 1;
    }

    if (count > 64)
    {
       // CyU3PDebugPrint (4, "ERROR: SensorWrite count > 64\n");
        return 1;
    }

    /* Set up the I2C control parameters and invoke the write API. */
    preamble.buffer[0] = slaveAddr;
    preamble.buffer[1] = highAddr;
    preamble.buffer[2] = lowAddr;
    preamble.length    = 3;
    preamble.ctrlMask  = 0x0000;

    apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, count, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return apiRetStatus;
}


// Writing To the serializer deserializer and devices attached to the I2C bus through the FPDLink
// Added Andres
CyU3PReturnStatus_t
FPDLinkWrite (
        uint8_t slaveAddr,
        uint8_t Addr,
        uint8_t count,
        uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    if (count > 64)
    {
       // CyU3PDebugPrint (4, "ERROR: SensorWrite count > 64\n");
        return 1;
    }

    /* Set up the I2C control parameters and invoke the write API. */
    preamble.buffer[0] = slaveAddr;
    preamble.buffer[1] = Addr;
    preamble.length    = 2;
    preamble.ctrlMask  = 0x0000;

    apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, count, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return apiRetStatus;
}

// Writing To the serializer deserializer and devices attached to the I2C bus through the FPDLink
// Added Andres
CyU3PReturnStatus_t
FPDLinkWriteSingle (
        uint8_t slaveAddr,
        //uint8_t Addr,
        uint8_t count,
        uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    if (count > 64)
    {
       // CyU3PDebugPrint (4, "ERROR: SensorWrite count > 64\n");
        return 1;
    }

    /* Set up the I2C control parameters and invoke the write API. */
    preamble.buffer[0] = slaveAddr;
    //preamble.buffer[1] = Addr;
    preamble.length    = 1;
    preamble.ctrlMask  = 0x0000;

    apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, count, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return apiRetStatus;
}



CyU3PReturnStatus_t CyFxDUALLEDDriver (
        uint8_t  VarCyFxDUALLEDDriver)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	uint8_t buf[2];
	if(VarCyFxDUALLEDDriver != 0 )
	{


		buf[0] = VarCyFxDUALLEDDriver;
		FPDLinkWrite(LEDDUALDRV_ADDR_WR,0x06,1,buf);

		buf[0] = VarCyFxDUALLEDDriver;
				FPDLinkWrite(LEDDUALDRV_ADDR_WR,0x05,1,buf);

		buf[0] = 0x0A;
		FPDLinkWrite(LEDDUALDRV_ADDR_WR,0x01,1,buf);
	}
	else
	{
		buf[0] = 0x00;
		FPDLinkWrite(LEDDUALDRV_ADDR_WR,0x01,1,buf);
	}

    return apiRetStatus;
}

CyU3PReturnStatus_t  CyFxLENSHV892_Update(
        uint8_t  VarCyLensUpdt)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	uint8_t buf[2];

	buf[0] = VarCyLensUpdt;
	FPDLinkWrite(LENS_HV892_ADDR_WR,buf[0],1,buf);

	return apiRetStatus;
}

CyU3PReturnStatus_t CyFxLM36011Brightness (
        uint8_t  VarCyFxLM36011Brightness)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	uint8_t buf[2];

		buf[0] = VarCyFxLM36011Brightness;
		FPDLinkWrite(LEDDRV_ADDR_WR,0x03,1,buf);


    return apiRetStatus;
}


uint8_t CyFxLM36011_Status_RD  (
        void)
{
	uint8_t buf[2];
	FPDLinkRead(LEDDRV_ADDR_RD,0x05,1,buf);
	return buf[0];
}



CyU3PReturnStatus_t
CyFxLSM6DSLTR  (
        uint8_t *buf)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	FPDLinkRead(GSENS_ADDR_RD,0x28,2,buf);

	return apiRetStatus;
}

//0    ODR_FIFO_3 ODR_FIFO_2 ODR_FIFO_1 ODR_FIFO_0   FIFO_MODE_2 FIFO_MODE_1 FIFO_MODE_0


//0000 FIFO disabled
//0001 FIFO ODR is set to 12.5 Hz
//0010 FIFO ODR is set to 26 Hz
//0011 FIFO ODR is set to 52 Hz
//0100 FIFO ODR is set to 104 Hz
//0101 FIFO ODR is set to 208 Hz
//0110 FIFO ODR is set to 416 Hz
//0111 FIFO ODR is set to 833 Hz
//1000 FIFO ODR is set to 1.66 kHz
//1001 FIFO ODR is set to 3.33 kHz
//1010 FIFO ODR is set to 6.66 kHz

void
CyFxLSM6DSLTR_EN  (
        void)
{
	uint8_t buf[2];
	//init LSM6DSLTR
	buf[0] = 0b00100110;						//FIFO mode : Continuous mode , FIFO ODR : 52 Hz
	FPDLinkWrite(GSENS_ADDR_WR,0x0A,1,buf);		//FIFO_CTRL5

}

void
CyFxLSM6DSLTR_DIS  (
        void )
{
	uint8_t buf[2];
	//init LSM6DSLTR
	buf[0] = 0b00000000;						//FIFO mode : Off
	FPDLinkWrite(GSENS_ADDR_WR,0x0A,1,buf);		//FIFO_CTRL5

}


uint8_t
CyFxLSM6DSLTR_FIFO_RD  (
        uint8_t *buf)
{
	uint8_t cnt;
	uint8_t RetCnt;
	FPDLinkRead(GSENS_ADDR_RD,0x3A,2,buf);

	cnt = buf[0];
	cnt = cnt / 3;
	cnt--;
	RetCnt = cnt;
	uint8_t index = 0;
	while(cnt)
	{
		FPDLinkRead(GSENS_ADDR_RD,0x3E,2,buf+index);
		index += 2;
		FPDLinkRead(GSENS_ADDR_RD,0x3E,2,buf+index);
		index += 2;
		FPDLinkRead(GSENS_ADDR_RD,0x3E,2,buf+index);
		index += 2;
		cnt--;

	}

	return RetCnt;
}


uint16_t CyFxLSM6DSLTR_TEMP_RD  (
        void)
{
	uint8_t buf[2];
	FPDLinkRead(GSENS_ADDR_RD,0x20,2,buf);
	return CY_U3P_MAKEWORD(buf[1],buf[0]);
}





CyU3PReturnStatus_t
SensorRead2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    if ((slaveAddr != SENSOR_ADDR_RD) && (slaveAddr != I2C_MEMORY_ADDR_RD))
    {
      //  CyU3PDebugPrint (4, "I2C Slave address is not valid!\n");
        return 1;
    }

    preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK;        /*  Mask out the transfer type bit. */
    preamble.buffer[1] = highAddr;
    preamble.buffer[2] = lowAddr;
    preamble.buffer[3] = slaveAddr;
    preamble.length    = 4;
    preamble.ctrlMask  = 0x0004;                                /*  Send start bit after third byte of preamble. */

    apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, 2, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return apiRetStatus;
}

CyU3PReturnStatus_t
SensorRead (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t count,
        uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    /* Validate the parameters. */
    if ((slaveAddr != SENSOR_ADDR_RD) && (slaveAddr != I2C_MEMORY_ADDR_RD))
    {
       // CyU3PDebugPrint (4, "I2C Slave address is not valid!\n");
        return 1;
    }
    if ( count > 64 )
    {
       // CyU3PDebugPrint (4, "ERROR: SensorWrite count > 64\n");
        return 1;
    }

    preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK;        /*  Mask out the transfer type bit. */
    preamble.buffer[1] = highAddr;
    preamble.buffer[2] = lowAddr;
    preamble.buffer[3] = slaveAddr;
    preamble.length    = 4;
    preamble.ctrlMask  = 0x0004;                                /*  Send start bit after third byte of preamble. */

    apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, count, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return apiRetStatus;
}


void
SensorReset (
        void)
{
    CyU3PReturnStatus_t apiRetStatus;

    /* Drive the GPIO low to reset the sensor. */
    apiRetStatus = CyU3PGpioSetValue (SENSOR_RESET_GPIO, CyFalse);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
       // CyU3PDebugPrint (4, "GPIO Set Value Error, Error Code = %d\n", apiRetStatus);
        return;
    }

    /* Wait for some time to allow proper reset. */
    CyU3PThreadSleep (10);

    /* Drive the GPIO high to bring the sensor out of reset. */
    apiRetStatus = CyU3PGpioSetValue (SENSOR_RESET_GPIO, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
       // CyU3PDebugPrint (4, "GPIO Set Value Error, Error Code = %d\n", apiRetStatus);
        return;
    }

    /* Delay the allow the sensor to power up. */
    CyU3PThreadSleep (10);
    return;
}


void
CyPythonProgmmUpload ( void )
{
	please contact for Program Upload

}

void
CyPythonManageMPart1 ( void )
{

		  CyFxSpiPythonWord(2,  0x0000);// Monochrome sensor
		  CyFxSpiPythonWord(17, 0x2113);// Configure PLL
		  CyFxSpiPythonWord(20, 0x0000);// Configure clock management
		  CyFxSpiPythonWord(26, 0x2280);// Configure PLL lock detector
		  CyFxSpiPythonWord(27, 0x3D2D);// Configure PLL lock detector
		  CyFxSpiPythonWord(32, 0x2004);// PLL input clock
		  CyFxSpiPythonWord(8,  0x0000);// Release PLL soft reset
		  CyFxSpiPythonWord(16, 0x0003);// Enable PLL

}



void
CyPythonManageMPart2 ( void )
{
		CyFxSpiPythonWord(9,  0x0000);	// Release clock generator Soft Reset
		CyFxSpiPythonWord(32, 0x7006);	// Enable logic clock
		CyFxSpiPythonWord(34, 0x0001);	// Enable logic block
}


void
CyPythonManageMPart1_PLL_bypass ( void )
{
			CyFxSpiPythonWord(2,  0x0002);	//# Monochrome sensor - for color use 0x0003
			CyFxSpiPythonWord(32, 0x300C);	//# Configure clock management
			CyFxSpiPythonWord(20, 0x0000);	//# Configure clock management
			CyFxSpiPythonWord(16, 0x0007);	//# Configure PLL bypass mode
}


void
CyPythonManageMPart2_PLL_bypass  ( void )
{
		CyFxSpiPythonWord(9,  0x0000);		// Release clock generator Soft Reset
		CyFxSpiPythonWord(32, 0x700F);	    // Enable logic clock
		CyFxSpiPythonWord(34, 0x0001);	    // Enable logic blocks
}



void
CyPythonSpiUploads( void )
{

	  //IMAGE CORE
	  //CyFxSpiPythonWord(40, 0x0003);	//image_core_config0 DEF 0x0000	IDS 0x0003 SCR 0x0003
	  	  	  	  	  	  	  	  	    //[0] imc_pwd_n 0x0 0 Image Core Power Down ‘0’: powered down,	  ‘1’: powered up
	  	  	  	  	  	  	  	  	  	//[1] mux_pwd_n 0x0 0 Column Multiplexer Power Down ‘0’: powered down, ‘1’: powered up
	  	  	  	  	  	  	  	  	  	//[2] colbias_enable 0x0 0 Bias Enable	  ‘0’: disabled	  ‘1’: enabled

	  CyFxSpiPythonWord(41, 0x085F);	//image_core_config1 DEF 0x085A IDS 0x085C SCR 0x085F
	  	  	  	  	  	  	  	  	  	//[3:0]   dac_ds 0xA 10 Double Slope Reset Level
	  	  	  	  	  	  	  	  	  	//[7:4]   dac_ts 0x5 5 Triple Slope Reset Level
										//[10:8]  reserved 0x3 3 Reserved
										//[12:11] reserved 0x1 1 Reserved
										//[13]    reserved 0x0 0 Reserved
										//[14]    reserved 0x0 0 Reserved
										//[15]    reserved 0x0 0 Reserved


	  CyFxSpiPythonWord(42, 0x4113);	//reserved DEF 0x0003 IDS 0x4113 SCR 0x4113
										//[0] reserved 0x1 1 Reserved
										//[1] reserved 0x1 1 Reserved
										//[6:4] reserved 0x0 0 Reserved
										//[10:8] reserved 0x0 0 Reserved
										//[15:12] reserved 0x0 0 Reserved

	  CyFxSpiPythonWord(43, 0x0518);	//reserved DEF 0x0508 IDS 0x051C SCR 0x0518
										//[0]    reserved 0x0 0 Reserved
										//[1]    reserved 0x0 0 Reserved
										//[2]    reserved 0x0 0 Reserved
										//[3]    reserved 0x0 0 Reserved
										//[6:4]  reserved 0x0 0 Reserved
										//[15:7] reserved 0x0 0 Reserved


	  //AFE
	  CyFxSpiPythonWord(48, 0x0001);	//AFE Configuration DEF 0x0000 IDS 0x0001 SCR 0x0001
	  	  	  	  	  	  	  	  	    //[0] pwd_n 0x0 0 Power down for AFE’s ‘0’: powered down,‘1’: powered up


	  //BIAS
	  CyFxSpiPythonWord(64, 0x0001);	//Bias Power Down Configuration DEF 0x0000 IDS 0x0001 SCR 0x0001
	  	  	  	  	  	  	  	  	  	//[0] pwd_n 0x0 0 Power down bandgap ‘0’: powered down, ‘1’: powered up


	  CyFxSpiPythonWord(65, 0x282B);    //Bias Configuration DEF 0xF8CB IDS 0x382B SCR 0x382B
	  	  	  	  	  	  	  	  	  	//[0]     extres   0x1 1 External Resistor Selection ‘0’: internal resistor,‘1’: external resistor
										//[3:1]   reserved 0x5 5 Reserved
										//[7:4]   imc_colpc_ibias - Column Precharge ibias Configuration	//P1300
										//[11:8]  imc_colbias_ibias - Column Bias ibias Configuration		//P1300
										//[15:12] cp_ibias - Charge Pump Bias								//P1300

	  CyFxSpiPythonWord(66, 0x51A2);	//AFE Bias Configuration DEF 0x53C8 IDS 0x53C8  SCR 0x53C8
										//[3:0]  afe_ibias    0x8  8  AFE ibias Configuration	//P1300
										//[7:4]  afe_adc_iref 0xC  12 ADC iref Configuration	//P1300
										//[14:8] afe_pga_iref 0x53 83 PGA iref Configuration	//P1300

	  CyFxSpiPythonWord(67, 0x0332);	//Column Multiplexer Bias Configuration DEF 0x8788 IDS 0x0565 SCR 0x0665
										//[3:0]   mux_25u_stage1 0x8 8 Column Multiplexer Stage 1 Bias Configuration 	//P1300
										//[7:4]   mux_25u_stage2 0x8 8 Column Multiplexer Stage 2 Bias Configuration	//P1300
										//[11:8]  mux_25u_delay  0x8 8 Column Multiplexer Delay Bias Configuration		//P1300
	  	  	  	  	  	  	  	  	  	//[15:12] reserved       0x8 8 Reserved											//P1300

	  CyFxSpiPythonWord(68, 0x0000);    //LVDS Bias Configuration DEF 0x0085 IDS 0x0085 SCR 0x0085
										//[3:0] lvds_ibias 0x5 5 LVDS Ibias //P1300
										//[7:4] lvds_iref  0x8 8 LVDS Iref  //P1300

	  CyFxSpiPythonWord(69, 0x0024);    //LVDS Bias Configuration DEF 0x0088 IDS 0x0088 SCR 0x0888
										//[3:0] imc_vsfdmed_ibias 0x8 8 VSFD Medium Bias //P1300
										//[7:4] adcref_ibias 0x8 8 ADC Reference Bias    //P1300


	  CyFxSpiPythonWord(70, 0x4800);	//reserved DEF 0x4111 IDS 0x4800 SCR 0x4800
										//[3:0]   reserved 0x1 1 Reserved
										//[7:4]   reserved 0x1 1 Reserved
										//[11:8]  reserved 0x1 1 Reserved
										//[15:12] reserved 0x4 4 Reserved

	  CyFxSpiPythonWord(71, 0x8888);	//reserved DEF 0x9788 IDS 0x8888 SCR 0x8888
	  	  	  	  	  	  	  	  	  	//[15:0]  reserved 0x9788 38792 Reserved

	  //CHARGE PUMP
	  CyFxSpiPythonWord(72, 0x0127);	//Charge Pump Configuration DEF 0x2220 IDS 0x127 SCR 0x0117
										//[0] trans_pwd_n         0x0 0 PD Trans Charge Pump Enable ‘0’: disabled, ‘1’: enabled
										//[1] resfd_calib_pwd_n   0x0 0 FD Charge Pump Enable ‘0’: disabled, ‘1’: enabled
										//[2] sel_sample_pwd_n    0x0 0 Select/Sample Charge Pump Enable ‘0’: disabled ‘1’: enabled
										//[6:4] trans_trim        0x2 2 PD Trans Charge Pump Trim
										//[10:8] resfd_calib_trim 0x2 2 FD Charge Pump Trim
										//[14:12] sel_sample_trim 0x2 2 Select/Sample Charge Pump Trim


	  //DATA
	  CyFxSpiPythonWord(128, 0x4700  ); //Black Calibration Configuration DEF 0x4008 IDS 0x4714 SCR 0x470A
										//[7:0] black_offset 	0x08 8 Desired black level at output
										//[10:8] black_samples 	0x0  0 Black pixels taken into account for black calibration.Total samples = 2**black_samples
										//[14:11] reserved 		0x8  8 Reserved
										//[15] crc_seed 		0x0  0 CRC Seed ‘0’: All-0 ‘1’: All-1

	  CyFxSpiPythonWord(129, 0xA001 );	//Black Calibration and Data Formating Configuration DEF 0x0001 IDS 0x8001 SCR 0x8001
										//[0]   auto_blackcal_enable 0x1 1 Automatic blackcalibration is enabled when 1, bypassed when 0
										//[9:1] blackcal_offset 0x00 0 Black Calibration offset used when auto_black_cal_en = ‘0’.
										//[10]  blackcal_offset_dec 0x0 0 blackcal_offset is added when 0, subtracted when 1
										//[11]  reserved 0x0 0 Reserved
										//[12]  reserved 0x0 0 Reserved
										//[13]  Mode ‘0’: 8-bit ‘1’: 10bit
										//[14]  ref_mode 0x0 0 Data contained on reference lines: ‘0’: reference pixels ‘1’: black average for the corresponding data channel
										//[15]  ref_bcal_enable 0x0 0 Enable black calibration on reference lines ‘0’: Disabled ‘1’: Enabled


	  CyFxSpiPythonWord(130, 0x0015);   //Data Formating - Training Pattern DEF 0x000F IDS 0x000F SCR 0x0001 - changed for GPIF block USB controller
										//[0] bl_frame_valid_enable	  0x1 1 Assert frame_valid for black lines when ‘1’,	  gate frame_valid for black lines when ‘0’.	  Parallel output mode only.
										//[1] bl_line_valid_enable    0x1 1 Assert line_valid for black lines when ‘1’, gate	  line_valid for black lines when ‘0’.	  Parallel output mode only.
										//[2] ref_frame_valid_enable  0x1 1 Assert frame_valid for ref lines when ‘1’, gate	  frame_valid for black lines when ‘0’.	  Parallel output mode only.
										//[3] ref_line_valid_enable   0x1 1 Assert line_valid for ref lines when ‘1’, gate	  line_valid for black lines when ‘0’.	  Parallel output mode only.
										//[4] frame_valid_mode        0x0 0 Behaviour of frame_valid strobe between	  overhead lines when [0] and/or [1] is	  deasserted:	  ‘0’: retain frame_valid deasserted between	  lines	  ‘1’: assert frame_valid between lines
										//[5] invert_bitstream        0x0 0 Negative Image	  ‘0’: Normal	  ‘1’: Negative
										//[8] data_negedge            0x0 0 Clock−Data Relation	  ‘0’: data is clocked out on the rising edge of	  the related clock	  ‘1’: data is clocked out on the falling edge of	  the related clock
										//[9] reserved                0x0 0 Reserved

	  //IDS 175 0x0080 DEF 0x0080	reserved
	  //IDS 177 0x0400 DEF 0x0100   reserved

	  CyFxSpiPythonWord(177, 0x0400);


	  //SEQUENCER
	  CyFxSpiPythonWord(192, 0x0801);	//Sequencer General Configuration DEF 0x0002 IDS 0x0002 SCR 0x0801
										//[0] enable             0x0 0 Enable sequencer	  ‘0’: Idle,	  ‘1’: enabled
										//[1] fast_startup       0x1 1 Fast startup  ‘0’: First frame is full frame (blanked out)	  ‘1’: Reduced startup time
										//[2] reserved           0x0 0 Reserved
										//[3] reserved           0x0 0 Reserved
										//[4] triggered_mode     0x0 0 Triggered Mode Selection	  ‘0’: Normal Mode,	  ‘1’: Triggered Mode
										//[5] slave_mode         0x0 0 Master/Slave Selection	  ‘0’: master,	  ‘1’: slave
										//[6] reserved           0x0 0 Reserved
										//[7] subsampling        0x0 0 Subsampling mode selection	  ‘0’: no subsampling,	  ‘1’: subsampling
										//[8] reserved           0x0 0 Reserved
										//[10] roi_aec_enable    0x0 0 Enable windowing for AEC Statistics.	  ‘0’: Subsample all windows	  ‘1’: Subsample configured window
										//[13:11] monitor_select 0x0 0 Control of the monitor pins
										//[14] reserved          0x0 0 Reserved
										//[15] sequence          0x0 0 Enable a sequenced readout with different	  parameters for even and odd frames

	  CyFxSpiPythonWord(194, 0x00E4);   // Integration Control DEF 0x00E4 IDS 0x00E4 SCR 0x03E4
										//[0]     reserved         0x0 0 Reserved
										//[1]     reserved         0x0 0 Reserved
										//[2]     fr_mode          0x1 1 Representation of fr_length. ‘0’: reset length  ‘1’: frame length
										//[3]     reserved         0x0 0 Reserved
										//[4]     int_priority     0x0 0 Integration Priority ‘0’: Frame readout has priority over integration  ‘1’: Integration End has priority over frame readout
										//[5]     halt_mode        0x1 1 The current frame will be completed when the sequencer is disabled and halt_mode = ‘1’. When ‘0’, the sensor stops immediately when disabled, without finishing the current frame.
										//[6]     fss_enable       0x1 1 Generation of Frame Sequence Start Sync  code (FSS)  ‘0’: No generation of FSS  ‘1’: Generation of FSS
										//[7]     fse_enable       0x1 1 Generation of Frame Sequence End Sync  code (FSE) ‘0’: No generation of FSE  ‘1’: Generation of FSE
										//[8]     reverse_y        0x0 0 Reverse readout  ‘0’: bottom to top readout  ‘1’: top to bottom readout
										//[9] 	  reverse_x        0x0 0 Reverse readout (X−direction)  ‘0’: left to right  ‘1’: right to left
										//[11:10] subsampling_mode 0x0 0 Subsampling mode  “00”: Subsampling in x and y (VITA  compatible)  “01”: Subsampling in x, not y  “10”: Subsampling in y, not x  “11”: Subsampling in x an y
										//[13:12] reserved         0x0 0 Reserved
										//[14]    reserved         0x0 0 Reserved
										//[15]    reserved         0x0 0 Reserved


	  CyFxSpiPythonWord(197, 0x030A);  //Black Line Configuration DEF 0x0104 IDS 0x0105 SCR 0x030A
									   //[7:0]  black_lines     0x04 4 Number of black lines. Minimum is 1.	  Range 1-255
									   //[12:8] gate_first_line 0x1  1 Blank out first lines	  0: no blank	  1-31: blank 1-31 lines

	  CyFxSpiPythonWord(199, 0x0299);  // Exposure/Frame Rate Configuration DEF 0x0001 IDS 0x069E SCR 0x0299
	  	  	  	  	  	  	  	  	   //[15:0] mult_timer0 0x0001 1 Mult Timer (Global shutter only)	  Defines granularity (unit = 1/PLL clock) of	  exposure and reset_length

	  CyFxSpiPythonWord(200, 0x0350);  //Exposure/Frame Rate Configuration DEF 0x0000 IDS 0x026E SCR 0x0350
	  	  	  	  	  	  	  	  	   //[15:0] fr_length0 0x0000 0 Frame/Reset length (Global shutter only)	  Reset length when fr_mode = ‘0’,	  Frame Length when fr_mode = ‘1’

	  CyFxSpiPythonWord(201, 0x01F4);  //Exposure/Frame Rate Configuration DEF 0x0000 IDS 0x0028 SCR 0x01F4
	  	  	  	  	  	  	  	  	   //[15:0] exposure0 0x0000 0 Exposure Time  Granularity defined by mult_timer

	  CyFxSpiPythonWord(204, 0x00C1);  //Gain Configuration DEF 0x01E1 IDS 0x00E3 SCR 0x00E1
									   //[4:0]  mux_gainsw0   0x01 1  Column Gain Setting
									   //[12:5] afe_gain0     0xF  15 AFE Programmable Gain Setting  # (gain 2x : 0x00E4 // gain 3.5x : 0x0024) new
									   //[13]   gain_lat_comp 0x0  0  Postpone gain update by 1 frame when ‘1’ to compensate for exposure time updates latency. Gain is applied at start of next frame if ‘0’

	  CyFxSpiPythonWord(207, 0x0000); //Reference Line Configuration DEF 0x0000 IDS 0x0000 SCR 0x0014
	  	  	  	  	  	  	  	  	  //[7:0] ref_lines 0x00 0 Number of Reference Lines 0-255

	  CyFxSpiPythonWord(208, 0xC900); //reserved DEF 0xC900 IDS	0xC900 SCR NOT ASS	- Readout delay or something like it	//Frame overhead time steps 10,3us
	  	  	  	  	  	  	  	  	  //[7:0]  reserved 0x00 0   Reserved
	  	  	  	  	  	  	  	  	  //[15:8] reserved 0xC9 201 Reserved

	  //IDS 209 0x0004 DEF 0x0004
	  //IDS 211 0x0049 DEF 0x0049

	  CyFxSpiPythonWord(214, 0x0100); //Reserved DEF 0x0100 IDS 0x0100 SCR 0x0100   overhead time end ~12 us
	  	  	  	  	  	  	  	  	  //[7:0] reserved 0x00 0 Reserved

	  CyFxSpiPythonWord(215, 0x191F );	//Reserved DEF 0x191F IDS 0x191F SCR 0x101F
										//[0] reserved 0x1 1 Reserved
										//[1] reserved 0x1 1 Reserved
										//[2] reserved 0x0 0 Reserved
										//[3] reserved 0x0 0 Reserved
										//[4] reserved 0x0 0 Reserved
										//[5] reserved 0x0 0 Reserved
										//[6] reserved 0x0 0 Reserved
										//[7] reserved 0x0 0 Reserved
										//[8] reserved 0x1 1 Reserved
										//[9] reserved 0x0 0 Reserved
										//[10] reserved 0x0 0 Reserved
										//[11] reserved 0x0 0 Reserved
										//[12] reserved 0x0 0 Reserved
										//[13] reserved 0x0 0 Reserved
										//[14] reserved 0x0 0 Reserved

	  CyFxSpiPythonWord(216, 0x0000);  //Reserved DEF 0x0000 IDS 0x0000 SCR 0x0000
	  	  	  	  	  	  	  	  	   //[6:0] reserved 0x00 0 Reserved

	  CyFxSpiPythonWord(219, 0x0023); //Reserved DEF 0x005C IDS 0x0022 SCR 0x0023
									  //[6:0]  reserved 0x05C 92 Reserved
									  //[14:8] reserved 0x00  0 Reserved


	  CyFxSpiPythonWord(220,0x3C2B); //Reserved DEF 0x3624 IDS 0x3B2A  SCR 0x3C2B  E-black calibration image 0x3C2B  E-gray calibration 0x3C4D
									 //[6:0]  reserved 0x24 36 Reserved
									 //[14:8] reserved 0x36 54 Reserved


	  CyFxSpiPythonWord(221,0x2B4D); //Reserved DEF 0x6245 IDS 0x624A SCR 0x2B4D
	  	  	  	  	  	  	  	  	 //[6:0]   reserved 0x45 69 Reserved
	  	  	  	  	  	  	  	  	 //[14:8]  reserved 0x62 98 Reserved

	  CyFxSpiPythonWord(222,0x6230); //Reserved DEF 0x6230 IDS 0x624A SCR NOTASS
	  	  	  	  	  	  	  	  	 //[6:0]  reserved 0x30 48 Reserved
	  	  	  	  	  	  	  	  	 //[14:8] reserved 0x62 98 Reserved

	  CyFxSpiPythonWord(224, 0x3E01); //Reserved DEF 0x3E01 IDS 0x3EEE SCR 0x3E5E  //space in between lines
									 //[3:0] reserved 0x1  1 Reserved
									 //[7:4] reserved 0x00 0 Reserved
									 //[8]   reserved 0x0  0 Reserved
									 //[9]   reserved 0x1  1 Reserved
									 //[10]  reserved 0x1  1 Reserved
									 //[11]  reserved 0x1  1 Reserved
									 //[12]  reserved 0x1  1 Reserved
									 //[13]  reserved 0x1  1 Reserved

	  //IDS 227 0x0000 DEF 0x0000

	  CyFxSpiPythonWord(211,0x0049); //Reserved DEF 0x0049 IDS NOTASS SCR 0x0049
									 //[0] reserved 0x1 1 Reserved
									 //[1] reserved 0x0 0 Reserved
									 //[2] reserved 0x0 0 Reserved
									 //[3] reserved 0x1 1 Reserved
									 //[6:4] reserved 0x4 4 Reserved
									 //[15:8] reserved 0x0 0 Reserved

	  CyFxSpiPythonWord(216,0x0000); //Reserved DEF 0x0000 IDS NOTASS SCR 0x0000
	  	  	  	  	  	  	  	  	 //[6:0] reserved 0x00 0 Reserved

	  CyFxSpiPythonWord(219,0x0023); //Reserved DEF 0x005C IDS NOTASS SCR 0x0023
									 //[6:0]  reserved 0x05C 92 Reserved
									 //[14:8] reserved 0x00 0 Reserved

	  CyFxSpiPythonWord(220,0x3C2B); //Reserved DEF 0x3624 IDS 0x3B2A  SCR 0x3C2B  E-black calibration image 0x3C2B  E-gray calibration 0x3C4D
									 //[6:0]  reserved 0x24 36 Reserved
									 //[14:8] reserved 0x36 54 Reserved


	  CyFxSpiPythonWord(221,0x2B4D); //Reserved DEF 0x6245 IDS 0x624A SCR 0x2B4D
	  	  	  	  	  	  	  	  	 //[6:0]   reserved 0x45 69 Reserved
	  	  	  	  	  	  	  	  	 //[14:8]  reserved 0x62 98 Reserved


	  CyFxSpiPythonWord(230,0x0299); //Reserved DEF 0x0001
	  CyFxSpiPythonWord(231,0x0350); //Reserved DEF 0x0000
	  CyFxSpiPythonWord(232,0x01F4); //Reserved DEF 0x0000
	  CyFxSpiPythonWord(235,0x00E1); //Reserved DEF 0x0000

	  CyFxSpiPythonWord(96, 0x0001); //Temperature Sensor Configuration DEF 0x0000
									 //[0]    enable   0x0 0 Temperature Diode Enable ‘0’: disabled, ‘1’: enabled
									 //[1]    reserved 0x0 0 Reserved
									 //[2]    reserved 0x0 0 Reserved
									 //[3]    reserved 0x0 0 Reserved
									 //[4]    reserved 0x0 0 Reserved
									 //[5]    reserved 0x0 0 Reserved
									 //[13:8] offset   0x0 0 Temperature Offset (signed)
}



void
CyPythonPowerDown (void )
{
	  //CyFxSpiPythonWord(112, 0x0999); // Soft reset
	  CyFxSpiPythonWord(72, 0x7006);  // Disable analog clock
	  CyFxSpiPythonWord(64, 0x0000);  // Disable column multiplexer
	  CyFxSpiPythonWord(42, 0x4110);  // Image core config
	  CyFxSpiPythonWord(48, 0x0000);  // Disable AFE
	  CyFxSpiPythonWord(40, 0x0000);  // Disable biasing block
	  CyFxSpiPythonWord(32, 0x0010);  // Disable charge pump
	  CyFxSpiPythonWord(10, 0x0000);  // Disable LVDS transmitters

	  CyFxSpiPythonWord(34, 0x0000);//1 34 0x0000 Soft reset clock generator
	  CyFxSpiPythonWord(32, 0x7004);//2 32 0x7004 Disable logic clock
	  CyFxSpiPythonWord(9,  0x0000);//3 9 0x0000 Disable logic blocks

	  CyFxSpiPythonWord(16, 0x0099);//1 16 0x0099 Soft reset PLL
	  CyFxSpiPythonWord(8,  0x0000);//2 8 0x0000 Disable PLL

}


void
CyPythonSoftPowerUp ( void )
{


	  CyFxSpiPythonWord(10, 0x0000);  // Release soft reset state
	  CyFxSpiPythonWord(32, 0x700F);  // Divide by 5 off// CyFxSpiPythonWord(32, 0x7007);  // Enable analog clock
	  CyFxSpiPythonWord(40, 0x0003);  // Enable column multiplexer
	  CyFxSpiPythonWord(42, 0x4113);  // Configure image core
	  CyFxSpiPythonWord(48, 0x0001);  // Enable AFE
	  CyFxSpiPythonWord(64, 0x0001);  // Enable biasing block
	  CyFxSpiPythonWord(72, 0x0127);  // Enable charge pump
	  //CyFxSpiPythonWord(112, 0x0000); // Disable LVDS transmitters

}

void
CyPythonFrameSet_PLL_Bypass(void )
{

	// MultiTimer @ 66.6Mhz/PLL Bypass(/4)) = 60.06nS x 20 = 1.2uS
	CyFxSpiPythonWord(199, 20);//14); // Multi - Timer CyFxSpiPythonWord(199, 0x0048); // Multi - Timer
	// frame lenght for 30fps = 33.33333333mS / 1.2012uS = 27750 = 0x‭6C66‬
	CyFxSpiPythonWord(200,27609);//0x6AE0);// 0x6AF4);//0x61A8);
}

void
CyPythonFrameSet(void )
{
// 30 FPS
	CyFxSpiPythonWord(199, 0x0048); // Mult- Timer
	CyFxSpiPythonWord(200, 0x7871); // fr lenght
}

void
CyPythonROISet ( void )
{

		CyFxSpiPythonWord(256,  0xC207); 		//roi0_configuration0 - ROI Configuration  (194,07)
												//[7:0]  x_start 0x00 0 ROI 0 − X Start Configuration (bits 8..1)
												//[15:8] x_end   0xC9 201 ROI 0 − X End Configuration (bits 8..1)
		CyFxSpiPythonWord(257,  0x8810); 		//roi0_configuration1 - ROI Configuration  (136,16)
												//[7:0]  y_start 0x00 0 ROI 0 − Y Start Configuration (bits 9..2)
												//[15:8] y_end   0x97 151 ROI 0 − Y End Configuration (bits 9..2)

		CyFxSpiPythonWord(258,  0x3B14); 		//
		CyFxSpiPythonWord(259,  0xBB44); 		//

		CyFxSpiPythonWord(260, 0); //
		CyFxSpiPythonWord(261, 0); //
		CyFxSpiPythonWord(262, 0); //
		CyFxSpiPythonWord(263, 0); //

		CyFxSpiPythonWord(263, 0); //
		CyFxSpiPythonWord(264, 20);//C4); //
		CyFxSpiPythonWord(265, 0); //
}




void
CyPythonReadID ( void )
{
	CyPythonID = CyFxSpiPythonRWord(0);

}

CyU3PReturnStatus_t
FPDLinkRead (
        uint8_t slaveAddr,
        uint8_t Addr,
        uint8_t count,
        uint8_t *buf)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;

    if ( count > 64 )
    {
       // CyU3PDebugPrint (4, "ERROR: SensorWrite count > 64\n");
        return 1;
    }

    preamble.buffer[0] = slaveAddr & I2C_SLAVEADDR_MASK;        /*  Mask out the transfer type bit. */
    preamble.buffer[1] = Addr;
    preamble.buffer[2] = slaveAddr;
    preamble.length    = 3;
    preamble.ctrlMask  = 0x0002;                                /*  Send start bit after third byte of preamble. */

    apiRetStatus = CyU3PI2cReceiveBytes (&preamble, buf, count, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return apiRetStatus;
}

void
SensorInit (
        void)
{

	uint8_t buf[2];

	/* Wait for some time to allow proper reset. */
	CyU3PThreadSleep (500);


	//Added Andres
	buf[0] = SER_ADDR_WR; //set address of serializer allowable i2c addresses to send through FPDLink
	//FPDLinkWrite (SER_ADDR_WR, 0x06, 1, buf);
	FPDLinkWrite (DESER_ADDR_WR, 0x07, 1, buf);


	buf[0] = 0x55; //set GPIO3 as remote GPIO control on the serializer
	FPDLinkWrite (SER_ADDR_WR, 0x0E, 1, buf);

	buf[0] = 0x19; //SCL High Time	400kHz
	FPDLinkWrite (SER_ADDR_WR, 0x11, 1, buf);

	buf[0] = 0x19; //SCL Low Time	400kHz
	FPDLinkWrite (SER_ADDR_WR, 0x12, 1, buf);


	buf[0] = SER_ADDR_WR; //sets allowable i2c addresses to send through serializer
    //SensorWrite (SER_ADDR_WR, 0x06, 1, buf);
	FPDLinkWrite (DESER_ADDR_WR, 0x07, 1, buf);

    buf[0] = 0x55;			//Andres set GPIO3 as remote GPIO control
    FPDLinkWrite (SER_ADDR_WR, 0x0E, 1, buf);

	buf[0] = LEDDRV_ADDR_WR; //sets allowable i2c addresses to send through serializer
	FPDLinkWrite (DESER_ADDR_WR, 0x0B, 1, buf);
	FPDLinkWrite (DESER_ADDR_WR, 0x13, 1, buf);		//LM36011 - SINGLE LED driver with STROBE input via GPO3  DS90UB913A serializer

	buf[0] = LEDDUALDRV_ADDR_WR; //sets allowable i2c addresses to send through serializer
	FPDLinkWrite (DESER_ADDR_WR, 0x08, 1, buf);
	FPDLinkWrite (DESER_ADDR_WR, 0x10, 1, buf);		//LM3643  - DUAL LED driver

	buf[0] = EXPA_ADDR_WR; //sets allowable i2c addresses to send through serializer
	FPDLinkWrite (DESER_ADDR_WR, 0x09, 1, buf);
	FPDLinkWrite (DESER_ADDR_WR, 0x11, 1, buf);		//FXL6408 - I2C I/O expander

	buf[0] = GSENS_ADDR_WR; //sets allowable i2c addresses to send through serializer
	FPDLinkWrite (DESER_ADDR_WR, 0x0A, 1, buf);
	FPDLinkWrite (DESER_ADDR_WR, 0x12, 1, buf);		//LSM6DS3H - Gsensor

	buf[0] = LENS_HV892_ADDR_WR; //sets allowable i2c addresses to send through serializer
	FPDLinkWrite (DESER_ADDR_WR, 0x0C, 1, buf);
	FPDLinkWrite (DESER_ADDR_WR, 0x14, 1, buf);		//HV892 - Lens

	//init I/O Expander
	buf[0] = 0x0D;
	FPDLinkWrite(EXPA_ADDR_WR,0x03,1,buf);

	buf[0] = 0xF2;
	FPDLinkWrite(EXPA_ADDR_WR,0x07,1,buf);

	//init LSM6DSLTR
	buf[0] = 0b01000000;						//ODR Output Data Rate 52 Hz
	FPDLinkWrite(GSENS_ADDR_WR,0x10,1,buf);		//CTRL1_XL

	//init LSM6DSLTR
	buf[0] = 0b01000100;						//Block update enabled and IF_INC automatic
	FPDLinkWrite(GSENS_ADDR_WR,0x12,1,buf);		//CTRL3_C

	//init LSM6DSLTR
	buf[0] = 0b00000001;						//  Accelerometer in FIFO no Decimation
	FPDLinkWrite(GSENS_ADDR_WR,0x08,1,buf);		//	FIFO_CTRL3

	buf[0] = 0b00000101;						// IR Drive and Strobe Enabled
	FPDLinkWrite(LEDDRV_ADDR_WR,0x01,1,buf);	// Strobe

	//buf[0] = 0b00000001;							// Lens power up driver
	//FPDLinkWrite(LENS_HV892_ADDR_WR,0x00,1,buf);	//


	 //first fire  the power
	 buf[0] = 0x0C;			// this is V_EN signal to enable the LM3880 and start power sequence and oscillator
	 	 	 	 	 	 	// turn LED on
	 FPDLinkWrite(EXPA_ADDR_WR,0x05,1,buf);
	 CyU3PThreadSleep (100);

	 //LED off and nRESET HIGH
	 buf[0] = 0x05;
	 FPDLinkWrite(EXPA_ADDR_WR,0x05,1,buf);
	 CyU3PThreadSleep (100);

	 //LED on
	 buf[0] = 0x0D;
	 FPDLinkWrite(EXPA_ADDR_WR,0x05,1,buf);		//60mA

	 //start Python480 setup sequence

	 CyPythonManageMPart1_PLL_bypass();
	 CyU3PThreadSleep (500);

	 //LED off
	 buf[0] = 0x05;
	 FPDLinkWrite(EXPA_ADDR_WR,0x05,1,buf);		//58mA

	 CyPythonManageMPart2_PLL_bypass();			//58mA
	 CyU3PThreadSleep (100);

	 //LED on
	 buf[0] = 0x0D;
	 FPDLinkWrite(EXPA_ADDR_WR,0x05,1,buf);


	 CyPythonSpiUploads();		//100mA		bypassed PLL in behuizing 75 graden
	 CyPythonProgmmUpload(); 	 	 	 	 	//normal mode 130mA 	 83 degrees in house without 60 graden
	 CyPythonSoftPowerUp();
	 CyPythonROISet();			//Set frame Sizes to 752x460;
	 CyPythonFrameSet_PLL_Bypass();		//Set frame rate to 30 FPS

	 CyFxSpiPythonWord( 201, 27532);	//EXPOSURE_DEF );
	 CyFxSpiPythonWord( 205, 384 );		//103mA




}

/*
 * Verify that the sensor can be accessed over the I2C bus from FX3.
 */
uint8_t
SensorI2cBusTest (
        void)
{
    /* The sensor ID register can be read here to verify sensor connectivity. */
    uint8_t buf[2];

    /* Reading sensor ID */
    if (SensorRead2B (SENSOR_ADDR_RD, 0x00, 0x00, buf) == CY_U3P_SUCCESS)
    {
        if ((buf[0] == 0x24) && (buf[1] == 0x81))
        {
            return CY_U3P_SUCCESS;
        }
    }
    return 1;
}

/* Post sensor reset settings. */
//}

/* Update sensor PLL and timing settings.
   The procedure is adapted from Aptina's sensor initialization scripts. Please
   refer to the MT9M114 sensor datasheet for details.
 */
//static void
//Step2_PLL_Timing (
//        void)
//{
//    /* Default PLL_settings */
//    SensorWrite2B (SENSOR_ADDR_WR, 0x09, 0x8E, 0, 0);   /* set XDMA to logical addressing */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x7E, 0, 1);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x80, 0x02, 0x25);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x82, 0x07, 0);
//
//    /* Timing settings */
//    SensorWrite2B (SENSOR_ADDR_WR, 0x09, 0x8E, 0, 0);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x00, 0x00, 0x7C);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x02, 0x00, 0x04);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x04, 0x03, 0x53);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x06, 0x05, 0x0B);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x08, 0x02, 0x34);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x0A, 0x93, 0x40);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x0C, 0x00, 0x01);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x0E, 0x00, 0xDB);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x10, 0x05, 0xAD);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x12, 0x02, 0xFE);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x14, 0x06, 0x50);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x16, 0x00, 0x60);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x18, 0x02, 0xD3);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x34, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x54, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x56, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x58, 0x05, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x5A, 0x02, 0xD0);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x5C, 0x00, 0x03);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x68, 0x05, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x6A, 0x02, 0xD0);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x8C, 0x1E, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x8E, 0x0F, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x14, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x16, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x18, 0x04, 0xFF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x1A, 0x02, 0xCF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x1C, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x1E, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x20, 0x00, 0xFF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x22, 0x00, 0x8F);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xE8, 0x01, 0x00, 0x00);
//}

/* Patch, errata and optimization settings from Aptina.
   The procedure is adapted from Aptina's sensor initialization scripts. Please
   refer to the MT9M114 sensor datasheet for details.
 */
//static void
//Step3_Recommended (
//        void)
//{
//    uint8_t buf[2];
//
//    /* Sensor optimization. */
//    SensorWrite2B (SENSOR_ADDR_WR, 0x31, 0x6A, 0x82, 0x70);
//    SensorWrite2B (SENSOR_ADDR_WR, 0x31, 0x6C, 0x82, 0x70);
//    SensorWrite2B (SENSOR_ADDR_WR, 0x3E, 0xD0, 0x23, 0x05);
//    SensorWrite2B (SENSOR_ADDR_WR, 0x3E, 0xD2, 0x77, 0xCF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0x31, 0x6E, 0x82, 0x02);
//    SensorWrite2B (SENSOR_ADDR_WR, 0x31, 0x80, 0x87, 0xFF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0x30, 0xD4, 0x60, 0x80);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xA8, 0x02, 0x00, 0x08);
//
//    /* Errata item 1: REG = 0x3E14, 0xFF39. */
//    SensorWrite2B (SENSOR_ADDR_WR, 0x3E, 0x14, 0xFF, 0x39);
//
//    /* Errata item 2: BITFIELD = 0x301A, 0x0400, 0x1 */
//    SensorRead2B (SENSOR_ADDR_RD, 0x30, 0x1A, buf);
//    buf[0] |= (1 << 2);
//    SensorWrite2B (SENSOR_ADDR_WR, 0x30, 0x1A, buf[0], buf[1]);
//}

/*
   The procedure is adapted from Aptina's sensor initialization scripts. Please
   refer to the MT9M114 sensor datasheet for details.
 */
//static void
//Step4_APGA (
//        void)
//{
//    /* LOAD_PROM = 0xA8, PGA, FACTORY, ELSELOAD = PROM_PROMPT */
//    SensorWrite2B (SENSOR_ADDR_WR, 0x09, 0x8E, 0x00, 0x00); /* set XDMA to logical addressing */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x5E, 0x00, 0x03); /* enable PGA and APGA */
//}

/*
   The procedure is adapted from Aptina's sensor initialization scripts. Please
   refer to the MT9M114 sensor datasheet for details.
 */
//static void
//Step5_AWB_CCM (
//        void)
//{
//    /* CCM */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x92, 0x02, 0x6C);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x94, 0xFF, 0x1A);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x96, 0xFF, 0xB3);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x98, 0xFF, 0x80);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x9A, 0x01, 0x66);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x9C, 0x00, 0x03);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x9E, 0xFF, 0x9A);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xA0, 0xFE, 0xB4);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xA2, 0x02, 0x4D);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xA4, 0x01, 0xBF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xA6, 0xFF, 0x01);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xA8, 0xFF, 0xF3);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xAA, 0xFF, 0x75);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xAC, 0x01, 0x98);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xAE, 0xFF, 0xFD);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xB0, 0xFF, 0x9A);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xB2, 0xFE, 0xE7);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xB4, 0x02, 0xA8);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xB6, 0x01, 0xD9);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xB8, 0xFF, 0x26);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xBA, 0xFF, 0xF3);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xBC, 0xFF, 0xB3);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xBE, 0x01, 0x32);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xC0, 0xFF, 0xE8);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xC2, 0xFF, 0xDA);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xC4, 0xFE, 0xCD);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xC6, 0x02, 0xC2);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xC8, 0x00, 0x75);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xCA, 0x01, 0x1C);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xCC, 0x00, 0x9A);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xCE, 0x01, 0x05);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xD0, 0x00, 0xA4);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xD2, 0x00, 0xAC);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xD4, 0x0A, 0x8C);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xD6, 0x0F, 0x0A);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xD8, 0x19, 0x64);
//    /* Load factory RG and BG gains, if any */
//
//    /* AWB */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x14, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x16, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x18, 0x04, 0xFF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x1A, 0x02, 0xCF);
//
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x04, 0x00, 0x33); /*  CAM_AWB_AWB_XSHIFT_PRE_ADJ */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x06, 0x00, 0x40); /*  CAM_AWB_AWB_YSHIFT_PRE_AD */
//
//    /* VAR8 = 18, 0xF2, 0x3 */
//    /* VAR8 = 18, 0xF3, 0x2 */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xF2, 0x03, 0x02);
//
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x06, 0x00, 0x3C);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xF4, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xF6, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xF8, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xFA, 0xE7, 0x24);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xFC, 0x15, 0x83);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0xFE, 0x20, 0x45);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x00, 0x05, 0xDC);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x02, 0x00, 0x7C);
//
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x0C, 0x80, 0x80);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x0E, 0x80, 0x88);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x10, 0x80, 0x80);
//}

/*
   The procedure is adapted from Aptina's sensor initialization scripts. Please
   refer to the MT9M114 sensor datasheet for details.
 */
//static void
//Step7_PIPE_Preference (
//        void)
//{
//    uint8_t buf[2];
//
//    /* Color pipeline */
//    /* VAR = 18, 0x126, 0x126, 0x0020 */
//    /* VAR= 18, 0x128, 0x128, 0x009A */
//    /* VAR= 18, 0x146, 0x146, 0x0070 */
//    /* VAR= 18, 0x148, 0x148, 0x00F3 */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x26, 0x00, 0x20);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x28, 0x00, 0x9A);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x46, 0x00, 0x70);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x48, 0x00, 0xF3);
//
//    /* VAR= 18, 0x152, 0x152, 0x0020 */
//    /* VAR= 18, 0x154, 0x154, 0x009A */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x52, 0x00, 0x20);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x54, 0x00, 0x9A);
//
//    /* VAR8 = 18, 0x12A, 0x12A, 0x80 */
//    /* VAR8 = 18, 0x12B, 0x12B, 0x4B */
//    /* VAR8 = 18, 0x12C, 0x12C, 0x00 */
//    /* VAR8 = 18, 0x12D, 0x12D, 0xFF */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x2A, 0x80, 0x4B);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x2C, 0x00, 0xFF);
//
//    /* VAR8 = 18, 0x12E, 0x12E, 0x3C */
//    /* VAR8 = 18, 0x12F, 0x12F, 0x02 */
//    /* VAR8 = 18, 0x130, 0x130, 0x06 */
//    /* VAR8 = 18, 0x131, 0x131, 0x64 */
//    /* VAR8 = 18, 0x132, 0x132, 0x01 */
//    /* VAR8 = 18, 0x133, 0x133, 0x0C */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x2E, 0x3C, 0x02);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x30, 0x06, 0x64);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x32, 0x01, 0x0C);
//
//    /* VAR8 = 18, 0x134, 0x134, 0x3C */
//    /* VAR8 = 18, 0x135, 0x135, 0x3C */
//    /* VAR8 = 18, 0x136, 0x136, 0x3C */
//    /* VAR8 = 18, 0x137, 0x137, 0x19 */
//    /* VAR8 = 18, 0x138, 0x138, 0x64 */
//    /* VAR8 = 18, 0x139, 0x139, 0x64 */
//    /* VAR8 = 18, 0x13A, 0x13A, 0x64 */
//    /* VAR8 = 18, 0x13B, 0x13B, 0x32 */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xc9, 0x34, 0x3C, 0x3C);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x36, 0x3C, 0x19);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x38, 0x64, 0x64);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x3A, 0x64, 0x32);
//
//    /* VAR= 18, 0x13C, 0x13C, 0x0010 */
//    /* VAR= 18, 0x13E, 0x13E, 0x0070 */
//    /* VAR= 18, 0x140, 0x140, 0x00DC */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x3C, 0x00, 0x10);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x3E, 0x00, 0x70);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x40, 0x00, 0xDC);
//
//    /* VAR8 = 18, 0x142, 0x142, 0x38 */
//    /* VAR8 = 18, 0x143, 0x143, 0x30 */
//    /* VAR8 = 18, 0x144, 0x144, 0x50 */
//    /* VAR8 = 18, 0x145, 0x145, 0x19 */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x42, 0x38, 0x30);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x44, 0x50, 0x19);
//
//
//    /* VAR= 18, 0x14A, 0x14A, 0x0230 */
//    /* VAR= 18, 0x14C, 0x14C, 0x0010 */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x4A, 0x02, 0x30);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x4C, 0x00, 0x10);
//
//    /* VAR= 18, 0x14E, 0x14E, 0x01CD */
//    /* VAR8 = 18, 0x150, 0x150, 0x05 */
//    /* VAR8 = 18, 0x151, 0x151, 0x40 */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x4E, 0x01, 0xCD);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x50, 0x05, 0x40);
//
//    /* REG= 0xC87B, 0x1B   : CAM_AET_TARGET_AVERAGE_LUMA_DARK */
//    /* REG= 0xC878, 0x0E   : CAM_AET_AEMODE */
//    /* REG= 0xC890, 0x0080 : CAM_AET_TARGET_GAIN */
//    /* REG= 0xC886, 0x0100 : CAM_AET_AE_MAX_VIRT_AGAIN */
//    /* REG= 0xC87C, 0x005A : CAM_AET_BLACK_CLIPPING_TARGET */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x7B, 0x00, 0x1B);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x78, 0x00, 0x0E);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x90, 0x00, 0x80);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x86, 0x01, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x7C, 0x00, 0x5A);
//
//    /* VAR8 = 13, 0x2A, 0x2A, 0x05 */
//    /* VAR8 = 10, 0xA, 0xA, 0x20 */
//    buf[0] = 0x05;
//    SensorWrite (SENSOR_ADDR_WR, 0xB4, 0x2A, 1, buf);
//
//    buf[0] = 0x05;
//    SensorWrite (SENSOR_ADDR_WR, 0xA8, 0x0A, 1, buf);
//}

/*
   The procedure is adapted from Aptina's sensor initialization scripts. Please
   refer to the MT9M114 sensor datasheet for details.
 */
//static void
//Step8_Features (
//        void)
//{
//    /* REG = 0x098E, 0x0000 : Set XDMA to logical addressing */
//    /* REG = 0xC984, 0x8040 : cam_port_output_control = 32832 */
//    /* REG = 0x001E, 0x0777 : PAD SLEW CONTROL */
//    /* IF_SERIAL=0xCA, 0x08, 0xFF, 8:16, ==0x10, LOAD=MIPI settings */
//    SensorWrite2B (SENSOR_ADDR_WR, 0x09, 0x8E, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x84, 0x80, 0x40);
//    SensorWrite2B (SENSOR_ADDR_WR, 0x00, 0x1E, 0x07, 0x77);
//}

/*
   The procedure is adapted from Aptina's sensor initialization scripts. Please
   refer to the MT9M114 sensor datasheet for details.
 */
//void
//SensorScaling_VGA (
//        void)
//{
//    uint8_t buf[2];
//
//    /* PLL */
//    SensorWrite2B (SENSOR_ADDR_WR, 0x09, 0x8E, 0x10, 0x00);
//    buf[0] = 0x01;
//    SensorWrite (SENSOR_ADDR_WR, 0xC9, 0x7E, 1, buf);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x80, 0x03, 0x3B);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x82, 0x0D, 0x00);
//
//    /* 720P */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x84, 0x80, 0x40);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x00, 0x00, 0x04);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x02, 0x00, 0x04);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x04, 0x03, 0xCB);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x06, 0x05, 0x0B);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x08, 0x01, 0x81); /* pixclk config */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x0A, 0xD4, 0x52); /* pixclk config */
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x0C, 0x00, 0x01);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x0E, 0x00, 0xDB);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x10, 0x05, 0xB2);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x12, 0x03, 0xEF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x14, 0x06, 0x35);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x16, 0x00, 0x60);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x18, 0x03, 0xC3);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x26, 0x00, 0x20);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x34, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x54, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x56, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x58, 0x05, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x5A, 0x03, 0xC0);
//    buf[0] = 0x03;
//    SensorWrite (SENSOR_ADDR_WR, 0xC8, 0x5C, 1, buf);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x68, 0x02, 0x80);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x6A, 0x01, 0xE0);
//    buf[0] = 0x00;
//    SensorWrite (SENSOR_ADDR_WR, 0xC8, 0x78, 1, buf);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x8C, 0x0F, 0xCD);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x8E, 0x0F, 0xCD);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x14, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x16, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x18, 0x02, 0x7F);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x1A, 0x01, 0xDF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x1C, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x1E, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x20, 0x00, 0x7F);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x22, 0x00, 0x5F);
//
//    SensorChangeConfig();
//}

/*
   The procedure is adapted from Aptina's sensor initialization scripts. Please
   refer to the MT9M114 sensor datasheet for details.
 */
//void
//SensorScaling_HD720p_30fps (
//        void)
//{
//    uint8_t buf[2];
//
//    SensorWrite2B (SENSOR_ADDR_WR, 0x09, 0x8E, 0x10, 0x00);
//
//    buf[0] = 0x01;
//    SensorWrite (SENSOR_ADDR_WR, 0xC9, 0x7E, 1, buf);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x80, 0x01, 0x20);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x82, 0x07, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x84, 0x80, 0x40);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x00, 0x00, 0x04);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x02, 0x00, 0x04);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x04, 0x03, 0xCB);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x06, 0x05, 0x0B);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x08, 0x02, 0xDC);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x0A, 0x6C, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x0C, 0x00, 0x01);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x0E, 0x00, 0xDB);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x10, 0x05, 0xB3);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x12, 0x03, 0xEE);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x14, 0x06, 0x36);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x16, 0x00, 0x60);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x18, 0x03, 0xC3);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x26, 0x00, 0x20);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x34, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x54, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x56, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x58, 0x05, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x5A, 0x03, 0xC0);
//
//    buf[0] = 0x03;
//    SensorWrite (SENSOR_ADDR_WR, 0xC8, 0x5C, 1, buf);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x68, 0x05, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x6A, 0x02, 0xD0);
//
//    buf[0] = 0x00;
//    SensorWrite (SENSOR_ADDR_WR, 0xC8, 0x78, 1, buf);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x8C, 0x1E, 0x02);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC8, 0x8E, 0x1E, 0x02);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x14, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x16, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x18, 0x04, 0xFF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x1A, 0x02, 0xCF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x1C, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x1E, 0x00, 0x00);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x20, 0x00, 0xFF);
//    SensorWrite2B (SENSOR_ADDR_WR, 0xC9, 0x22, 0x00, 0x8F);
//
//    SensorChangeConfig();
//}

/*
   The procedure is adapted from Aptina's sensor initialization scripts. Please
   refer to the MT9M114 sensor datasheet for details.
 */
//void
//SensorChangeConfig (
//        void)
//{
//    uint8_t buf[2];
//
//    /* 1. set next state is change_config */
//    buf[0] = 0x28;  /* sys_state_enter_config_change */
//    SensorWrite (SENSOR_ADDR_WR, 0xDC, 0x00, 1, buf);
//
//    /* 2. poll command register */
//    while (SensorRead2B(SENSOR_ADDR_RD, 0x00, 0x80, buf))
//    {
//        if (!(buf[1]& 0x02))
//            break;
//    }
//
//    /* 3. issue command */
//    SensorWrite2B (SENSOR_ADDR_WR, 0x00, 0x80, 0x80, 0x02);
//
//    /* 4. waiting for firmware updating new configuration */
//    while (SensorRead2B(SENSOR_ADDR_RD, 0x00, 0x80, buf))
//    {
//        if (!(buf[1]& 0x02))
//            break;
//    }
//
//    /* 5. error handling */
//    SensorRead2B (SENSOR_ADDR_RD, 0x00, 0x80, buf);
//
//    /* 6. refresh command */
//    SensorWrite2B (SENSOR_ADDR_WR, 0x00, 0x80, 0x80, 0x04);
//    while (SensorRead2B(SENSOR_ADDR_RD, 0x00, 0x80, buf))
//    {
//        if (!(buf[1]& 0x04))
//            break;
//    }
//
//    SensorRead2B (SENSOR_ADDR_RD, 0xDC, 0x01, buf);
//}

/*
   Get the current brightness setting from the MT9M114 sensor.
 */
uint8_t
SensorGetBrightness (
        void)
{
    uint8_t buf[2];

    SensorRead2B (SENSOR_ADDR_RD, 0xCC, 0x0A, buf);
    return (uint8_t)buf[1];
}





/*
   Update the brightness setting for the MT9M114 sensor.
 */
void
SensorSetBrightness (
        uint8_t input)
{
    SensorWrite2B (SENSOR_ADDR_WR, 0xCC, 0x0A, 0x00, input);
}




