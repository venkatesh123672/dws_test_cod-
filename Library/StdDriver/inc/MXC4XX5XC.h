/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information and source code
 *  contained herein is confidential. The software including the source code
 *  may not be copied and the information contained herein may not be used or
 *  disclosed except with the written permission of MEMSIC Inc. (C) 2020
 *****************************************************************************/

/**
 * @brief
 * This file implement magnetic sensor driver APIs. 
 * Modified history: 
 * V1.1: Add the interrupt function on 20190214
 * V1.2: Add the DEVICE_ID_2 checking on 20190215
 * V1.3: Bug fix when converting register data on 20190821
 * V1.4: Open the API of z-axis offset compensation for user on 20200212
 */
 
typedef   signed char  int8_t; 		// signed 8-bit number    (-128 to +127)
typedef unsigned char  uint8_t; 	// unsigned 8-bit number  (+0 to +255)
typedef   signed short int16_t; 	// signed 16-bt number    (-32768 to +32767)
typedef unsigned short uint16_t; 	// unsigned 16-bit number (+0 to +65535)
//typedef   signed int   int32_t; 	// signed 32-bt number    (-2,147,483,648 to +2,147,483,647)
//typedef unsigned int   uint32_t; 	// unsigned 32-bit number (+0 to +4,294,967,295)

#define MXC4XX5XC_ADDRESS			0x15	

#define MXC4XX5XC_REG_DATA			0x03
#define MXC4XX5XC_REG_CTRL			0x0D
#define MXC4XX5XC_REG_DEVICE_ID		0x0E


#define MXC4XX5XC_REG_INT_SRC0		0x00
#define MXC4XX5XC_REG_INT_SRC1		0x01
#define MXC4XX5XC_REG_INT_CLR0		0x00
#define MXC4XX5XC_REG_INT_CLR1		0x01
#define MXC4XX5XC_REG_INT_MASK0		0x0A
#define MXC4XX5XC_REG_INT_MASK1		0x0B
#define MXC4XX5XC_REG_DETECTION		0x0C

#define MXC4XX5XC_CMD_8G_POWER_ON	0x40
#define MXC4XX5XC_CMD_4G_POWER_ON	0x20
#define MXC4XX5XC_CMD_2G_POWER_ON	0x00	
#define MXC4XX5XC_CMD_POWER_DOWN	0x01


#define MXC4XX5XC_DEVICE_ID_1		0x02
#define MXC4XX5XC_DEVICE_ID_2		0x03

#define MXC4XX5XC_2G_SENSITIVITY	1024
#define MXC4XX5XC_4G_SENSITIVITY	512
#define MXC4XX5XC_8G_SENSITIVITY	256
#define MXC4XX5XC_T_ZERO			25
#define MXC4XX5XC_T_SENSITIVITY		0.586

/**
 * @brief Enable the sensor
 */
int MXC4XX5XC_Enable(void);

/**
 * @brief Disable the sensor
 */
int MXC4XX5XC_Disable(void);


/**
 * @brief Get sensor data
 * @param acc_out is the accelerometer sensor vector, unit is g
 * @param acc_out is the temperature output, unit is degree Celsius 
 */
void MXC4XX5XC_GetData(float *acc_out, float *t_out);
		   

