/*
 ## Cypress FX3 Camera Kit header file (sensor.h)
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

/* This file defines the parameters and the interface for the NOIP1SN0480A image
   sensor driver.
 */

#ifndef _INCLUDED_SENSOR_H_
#define _INCLUDED_SENSOR_H_

#include <cyu3types.h>

/* I2C Slave address for the Ser/Deser sensor. DS90UB913/14*/
// Added Andres
#define DESER_ADDR_WR	0xC0
#define DESER_ADDR_RD   0xC1
#define SER_ADDR_WR		0xB0
#define SER_ADDR_RD		0xB1


/* I2C address for the DAC. */
//Added by Daniel 8_10_2015
//#define DAC_ADDR_WR		0b10011000 //For DAC5571

#define GSENS_ADDR_WR	0xD4
#define GSENS_ADDR_RD	0xD5	//LSM6DS3H

#define EXPA_ADDR_WR	0x88	//FXL6408
#define EXPA_ADDR_RD	0x89

#define LEDDRV_ADDR_WR		0xC8	//LM36011
#define LEDDRV_ADDR_RD		0xC9

#define LEDDUALDRV_ADDR_WR		0xC6	//LM3643
#define LEDDUALDRV_ADDR_RD		0xC7

#define LENS_HV892_ADDR_WR  0x46		//write only

/* I2C Slave address for the image sensor. */
#ifdef SADDR_HIGH
#define SENSOR_ADDR_WR 0xBA             /* Slave address used to write sensor registers. */
#define SENSOR_ADDR_RD 0xBB             /* Slave address used to read from sensor registers. */
#else
#define SENSOR_ADDR_WR 0x90             /* Slave address used to write sensor registers. */
#define SENSOR_ADDR_RD 0x91             /* Slave address used to read from sensor registers. */
#endif

#define I2C_SLAVEADDR_MASK 0xFE         /* Mask to get actual I2C slave address value without direction bit. */

#define I2C_MEMORY_ADDR_WR 0xA0         /* I2C slave address used to write to an EEPROM. */
#define I2C_MEMORY_ADDR_RD 0xA1         /* I2C slave address used to read from an EEPROM. */

/* GPIO 22 on FX3 is used to reset the Image sensor. */
#define SENSOR_RESET_GPIO 22

/* GPIO 22 on FX3 is used to reset the Image sensor. */
//#define SENSOR_RESET_GPIO 22 //Not used anymore. Daniel 4_9_2015
#define STROBE_LM36011				17		//	 TP4	Connected through FPDLink GPIO
#define TESTPIN3_GPIO				18		//   TP3
#define FRAME_OUT_GPIO				20		//   J5
#define TRIG_RECORD_EXT_GPIO		21

/* Function    : SensorWrite2B
   Description : Write two bytes of data to image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 highData  - High byte of data to be written.
                 lowData   - Low byte of data to be written.
 */
extern CyU3PReturnStatus_t
SensorWrite2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t highData,
        uint8_t lowData);

/* Function    : SensorWrite
   Description : Write arbitrary amount of data to image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 count     - Size of write data in bytes. Limited to a maximum of 64 bytes.
                 buf       - Pointer to buffer containing data.
 */
extern CyU3PReturnStatus_t
SensorWrite (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t count,
        uint8_t *buf);

/* Function    : SensorRead2B
   Description : Read 2 bytes of data from image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 buf       - Buffer to be filled with data. MSB goes in byte 0.
 */
extern CyU3PReturnStatus_t
SensorRead2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t *buf);

/* Function    : SensorRead
   Description : Read arbitrary amount of data from image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 count     = Size of data to be read in bytes. Limited to a max of 64.
                 buf       - Buffer to be filled with data.
 */
extern CyU3PReturnStatus_t
SensorRead (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t count,
        uint8_t *buf);

/* Function    : SensorInit
   Description : Initialize the MT9M114 sensor.
   Parameters  : None
 */
extern void
SensorInit (
        void);

/* Function    : SensorReset
   Description : Reset the MT9M114 image sensor using FX3 GPIO.
   Parameters  : None
 */
extern void
SensorReset (
        void);

/* Function    : SensorChangeConfig
   Description : Update sensor configuration based on selected video parameters.
   Parameters  : None
 */
extern void
SensorChangeConfig (
        void);

/* Function     : SensorScaling_HD720p_30fps
   Description  : Configure the MT9M114 sensor for 720p 30 fps video stream.
   Parameters   : None
 */
extern void
SensorScaling_HD720p_30fps (
        void);

/* Function     : SensorScaling_VGA
   Description  : Configure the MT9M114 sensor for VGA video stream.
   Parameters   : None
 */
extern void
SensorScaling_VGA (
        void);

/* Function    : SensorI2cBusTest
   Description : Test whether the MT9M114 sensor is connected on the I2C bus.
   Parameters  : None
 */
extern uint8_t
SensorI2cBusTest (
        void);

/* Function    : SensorGetBrightness
   Description : Get the current brightness setting from the MT9M114 sensor.
   Parameters  : None
 */
extern uint8_t
SensorGetBrightness (
        void);

/* Function    : SensorSetBrightness
   Description : Set the desired brightness setting on the MT9M114 sensor.
   Parameters  :
                 brightness - Desired brightness level.
 */
extern void
SensorSetBrightness (
        uint8_t input);


/* Function    : CyFxSpiPythonWord
   Description : Write a data word to any register of the Python480 set with SpiPyRegister
   Parameters  :
                 SpiPyRegister - Desired Register
                 SpiPyData16 - Value to set
 */
extern void
CyFxSpiPythonWord (
		uint16_t SpiPyRegister ,
		uint16_t SpiPyData16 );

extern
uint16_t
CyFxSpiPythonRWord ( uint16_t SpiPyCmd );

extern CyU3PReturnStatus_t
CyFxDUALLEDDriver (
        uint8_t  VarCyFxDUALLEDDriver);

extern CyU3PReturnStatus_t
CyFxLM36011Brightness (
        uint8_t  VarCyFxLM36011Brightness);

CyU3PReturnStatus_t
CyFxLSM6DSLTR  (
        uint8_t *buf);

uint8_t
CyFxLSM6DSLTR_FIFO_RD  (
        uint8_t *buf);

uint16_t
CyFxLSM6DSLTR_TEMP_RD  (
        void);


void
CyFxLSM6DSLTR_EN  (
        void);

void
CyFxLSM6DSLTR_DIS  (
        void);

CyU3PReturnStatus_t
FPDLinkWrite (
        uint8_t slaveAddr,
        uint8_t Addr,
        uint8_t count,
        uint8_t *buf);


CyU3PReturnStatus_t
FPDLinkRead (
        uint8_t slaveAddr,
        uint8_t Addr,
        uint8_t count,
        uint8_t *buf);

CyU3PReturnStatus_t  CyFxLENSHV892_Update(
        uint8_t  VarCyLensUpdt);


#endif /* _INCLUDED_SENSOR_H_ */

/*[]*/

