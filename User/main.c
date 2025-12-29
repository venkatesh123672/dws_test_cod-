/******************************************************************************
 * ICM-20948 WHO_AM_I + Wake-Up Test
 * Using Nuvoton M0564 + I2C interrupt driver
 *****************************************************************************/

#include <stdio.h>
#include "M0564.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000

/***************LPF_CONFIG_REGISTER***********/

#define ICM20948_LP_CONFIG_REG        0x05

/**********bank_sel_register****************/
#define ICM20948_REG_BANK_SEL         0x7F

/*****************bank_selcted_macro**************/
#define ICM20948_BANK_0               0x00
#define ICM20948_BANK_1               0x10
#define ICM20948_BANK_2               0x20
#define ICM20948_BANK_3               0x30

/***********************WHO_AM_I********************/

#define ICM20948_WHO_AM_I             0x00

/*******************Power Management****************/
#define ICM20948_PWR_MGMT_1           0x06
#define ICM20948_PWR_MGMT_2           0x07


/*******************Interrupt Registers**************/

#define ICM20948_INT_PIN_CFG          0x0F
#define ICM20948_INT_ENABLE_1         0x10
#define ICM20948_INT_ENABLE_2         0x11
#define ICM20948_ACCEL_INTEL_CTRL     0x12
#define ICM20948_INT_STATUS_1         0x1A


/* ---------------- IC Address ---------------- */
#define ICM20948_ADDR      0x69  // AD0 = LOW

/*************Accelerometer Output Registers***********/

#define ICM20948_ACCEL_XOUT_H         0x2D
#define ICM20948_ACCEL_XOUT_L         0x2E
#define ICM20948_ACCEL_YOUT_H         0x2F
#define ICM20948_ACCEL_YOUT_L         0x30
#define ICM20948_ACCEL_ZOUT_H         0x31
#define ICM20948_ACCEL_ZOUT_L         0x32


/*************Gyroscope Output Registers************/

#define ICM20948_GYRO_XOUT_H          0x33
#define ICM20948_GYRO_XOUT_L          0x34
#define ICM20948_GYRO_YOUT_H          0x35
#define ICM20948_GYRO_YOUT_L          0x36
#define ICM20948_GYRO_ZOUT_H          0x37
#define ICM20948_GYRO_ZOUT_L          0x38

/*****************Temperature Output Registers**********/

#define ICM20948_TEMP_OUT_H           0x39
#define ICM20948_TEMP_OUT_L           0x3A

/***External Sensor Data (Magnetometer routed output)*****/

#define ICM20948_EXT_SENS_DATA_00     0x3B
#define ICM20948_EXT_SENS_DATA_01     0x3C
#define ICM20948_EXT_SENS_DATA_02     0x3D
#define ICM20948_EXT_SENS_DATA_03     0x3E
#define ICM20948_EXT_SENS_DATA_04     0x3F
#define ICM20948_EXT_SENS_DATA_05     0x40
#define ICM20948_EXT_SENS_DATA_06     0x41
#define ICM20948_EXT_SENS_DATA_07     0x42


/*********BANK 1 — Gyro/Accel Configuration
🔸 Accelerometer Configuration******************/


#define ICM20948_ACCEL_CONFIG         0x14
#define ICM20948_ACCEL_SMPLRT_DIV_1   0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2   0x11


/********Gyroscope Configuration************/
#define ICM20948_GYRO_CONFIG_1        0x01
#define ICM20948_GYRO_CONFIG_2        0x02
#define ICM20948_GYRO_SMPLRT_DIV      0x00


/***********BANK 2 — More Sensor Config
Accelerometer low power********************/

#define ICM20948_ACCEL_INTEL_CTRL     0x12

/*****FIFO + Low Noise / Low Power Mode********/

#define ICM20948_FIFO_RST             0x68
#define ICM20948_FIFO_MODE            0x69

/***********BANK 3 — I²C Master / Magnetometer (AK09916) Control***************/

#define ICM20948_I2C_MST_CTRL         0x01

/***************Slave 0 (AK09916) Configuration*************/

#define ICM20948_I2C_SLV0_ADDR        0x03
#define ICM20948_I2C_SLV0_REG         0x04
#define ICM20948_I2C_SLV0_CTRL        0x05


/*******AK09916 Magnetometer Registers (External Sensor)**********/

#define AK09916_I2C_ADDRESS           0x0C

#define AK09916_WIA1                  0x00
#define AK09916_WIA2                  0x01

#define AK09916_STATUS_1              0x10
#define AK09916_HXL                   0x11
#define AK09916_HXH                   0x12
#define AK09916_HYL                   0x13
#define AK09916_HYH                   0x14
#define AK09916_HZL                   0x15
#define AK09916_HZH                   0x16
#define AK09916_STATUS_2              0x18

#define AK09916_CNTL2                 0x31   // operation mode
#define AK09916_CNTL3                 0x32   // soft reset

/***********  Magnetometer Power / Reset Registers*********/

#define ICM20948_USER_CTRL            0x03
#define ICM20948_LP_CONFIG            0x05

/********** threshold value for the Wake on Motion Interrupt for ACCEL
x/y/z axes. LSB = 4 mg. Range is 0 mg to 1020 mg. */

#define ICM20948_threshold_register  0x13


/* ---------------- Global Buffers ------------- */
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[6];
volatile uint8_t g_au8MstRxData[10];
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;
volatile uint8_t count = 0;

 void calibrtion_of_the_geo_scope();

volatile uint8_t g_u8MstRxData[50];

uint8_t buffer[10];

uint8_t  data=1;

uint8_t lenth_data=0;
uint8_t calibrtion_counter=0;

uint8_t caibrtion_value=1;
uint8_t who=0;

void i2c_acc_lo_meter();

void imu_enable_motion_interrupt();

uint8_t pwr_after;
/**************acc_metro_16_bit data**************/
int16_t acc_x_axis=0;

int16_t acc_y_axis=0;

int16_t acc_z_axis=0;

/**************geo_metro_16_bit data**************/

int16_t geo_x_axis=0;

int16_t geo_y_axis=0;

int16_t geo_z_axis=0;

int16_t geo_x_axis_x=0;

int16_t geo_y_axis_y=0;

int16_t geo_z_axis_z=0;


/*************temprature********************/

uint16_t temprature=0;

uint8_t buffer1[3];

/* ---------------- Function Pointer ------------ */
typedef void (*I2C_FUNC)(uint32_t u32Status);
static volatile I2C_FUNC s_I2C0HandlerFn = NULL;


void acceleomter_glass_brak_calication();
void acceleomter_motion_human_calication();
void acceleomter_curtian_motion_calication();
void ICM_WriteReg(uint8_t , uint8_t ,uint8_t );


uint8_t int_status=0;
/* --------------------------------------------------------
   I2C Interrupt Handler
-------------------------------------------------------- */
void I2C0_IRQHandler(void)
{
    uint32_t u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

void UART2_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS->IPRST1 |=  SYS_IPRST1_UART2RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART2RST_Msk;

    /* Configure UART1 and set UART1 baud rate */
    UART2->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC,9600);
    UART2->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

}

/* --------------------------------------------------------
   I2C Master RX Handler
-------------------------------------------------------- */
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08)
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1));
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x18)
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x28)
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
    }
    else if (u32Status == 0x10)
    {
        I2C_SET_DATA(I2C0, ((g_u8DeviceAddr << 1) | 1));
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if (u32Status == 0x40)
     {
         if(lenth_data > 1)
             I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI | I2C_CTL_AA);  // ACK for multi-byte
         else
             I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);               // NACK if only 1 byte
     }
    else if (u32Status == 0x50)   // data received, ACK returned
    {
        g_u8MstRxData[count++] = I2C_GET_DATA(I2C0);

        if(count <lenth_data)
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI | I2C_CTL_AA); // ACK → continue
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI); // NACK for last byte
        }
    }
    else if (u32Status == 0x58)    // last byte
    {
        g_u8MstRxData[count++] = I2C_GET_DATA(I2C0);

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI); // STOP
        g_u8MstEndFlag = 1;
        lenth_data=0;
    }
}


/* --------------------------------------------------------
   I2C Master TX Handler
-------------------------------------------------------- */
void I2C_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
      //  I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen <= lenth_data)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
           I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8MstEndFlag = 1;

        }
    }

}


/********************** System Init *******************/
void SYS_Init(void)
{
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK_SetCoreClock(PLL_CLOCK);

//    CLK_EnableModuleClock(UART0_MODULE);
//    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Enable UART module clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART1CKEN_Msk | CLK_APBCLK0_UART2CKEN_Msk);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UARTDIV_Msk)) | CLK_CLKDIV0_UART(1);

    CLK_EnableModuleClock(TMR0_MODULE);
           CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

    SYS->GPD_MFPL = SYS_GPD_MFPL_PD0MFP_UART0_RXD |
                    SYS_GPD_MFPL_PD1MFP_UART0_TXD;

    SYS->GPE_MFPH &= ~(SYS_GPE_MFPH_PE12MFP_Msk | SYS_GPE_MFPH_PE13MFP_Msk);
    SYS->GPE_MFPH |= (SYS_GPE_MFPH_PE12MFP_I2C0_SCL | SYS_GPE_MFPH_PE13MFP_I2C0_SDA);

    /**********************************uart2****************************************/
        SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC2MFP_Msk |SYS_GPC_MFPL_PC3MFP_Msk);
        SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC2MFP_UART2_TXD | SYS_GPC_MFPL_PC3MFP_UART2_RXD);


//    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC5MFP_Msk );
//    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC5MFP_GPIO);

    GPIO_SetMode(PC, BIT5, GPIO_MODE_INPUT);
    GPIO_EnableInt(PC, 5, GPIO_INT_BOTH_EDGE);



}


/* --------------------------------------------------------
   I2C Init
-------------------------------------------------------- */
void I2C0_Init(void)
{
    CLK_EnableModuleClock(I2C0_MODULE);
    I2C_Open(I2C0, 400000);
    I2C_SetSlaveAddr(I2C0, 0, ICM20948_ADDR, 0);
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

/* --------------------------------------------------------
   Register WRITE
-------------------------------------------------------- */
uint8_t ICM_ReadReg(uint8_t reg,uint8_t data_lenth)
{
	count=0;

	lenth_data=data_lenth;

    g_u8DeviceAddr = ICM20948_ADDR;
    g_au8MstTxData[0] = reg;
    g_au8MstTxData[1] =0;
    g_u8MstDataLen = 0;
    g_u8MstEndFlag = 0;

    s_I2C0HandlerFn = I2C_MasterTx;
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    while(!g_u8MstEndFlag);


    g_u8DeviceAddr = ICM20948_ADDR;
    g_au8MstTxData[0] = reg;
    g_u8MstDataLen = 0;
    g_u8MstEndFlag = 0;


    // read register value
    g_u8MstEndFlag = 0;
  //  count = 0;
    s_I2C0HandlerFn = I2C_MasterRx;
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);
    while(!g_u8MstEndFlag);



    if(data)
    {
    return g_u8MstRxData[0];
    data=0;
    }
}

void ICM_WriteReg(uint8_t reg, uint8_t value,uint8_t data_lenth)
{
	lenth_data=data_lenth;

    g_u8DeviceAddr = ICM20948_ADDR;
    g_au8MstTxData[0] = reg;
    g_au8MstTxData[1] = value;
    g_u8MstDataLen = 0;
    g_u8MstEndFlag = 0;

    s_I2C0HandlerFn = I2C_MasterTx;
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

    while(!g_u8MstEndFlag);
    //	 lenth_data=0;
}
void GPCDEF_IRQHandler(void)
{

    if(GPIO_GET_INT_FLAG(PC, BIT5))
    {
       GPIO_CLR_INT_FLAG(PC, BIT5);
 	   if(PC5)
 		{
 		// ICM_ReadReg(0x1A);   // THIS CLEARS THE INTERRUPT
 		}
    }
}
/* --------------------------------------------------------
   Main
-------------------------------------------------------- */
int main()
{
    SYS_UnlockReg();
    SYS_Init();
    SYS_LockReg();

    NVIC_EnableIRQ(GPCDEF_IRQn);

    UART2_Init();
      UART_EnableInt(UART2, (UART_INTEN_RDAIEN_Msk ));

    I2C0_Init();

//    uint8_t who, pwr_before, pwr_after;

    who = ICM_ReadReg(ICM20948_WHO_AM_I,1);


//    uint8_t pwr_before = ICM_ReadReg(ICM20948_PWR_MGMT_1);
//    printf("PWR before = 0x%02X\r\n", pwr_before);

    UART_Write(UART2, "edel_smart_devies",strlen("edel_smart_devies") );

    /*GPIO ENABLE */
//      NVIC_EnableIRQ(GPCDEF_IRQn);

//
//    // Configure INT pin
//    ICM_WriteReg(0x0F, 0x30);
//
//    // Enable RAW DATA READY interrupt
//    ICM_WriteReg(0x10, 0x01);

//    ICM_WriteReg(ICM20948_REG_BANK_SEL, ICM20948_BANK_0);  // Ensure Bank 0

	// THIS THE FIRST STEP OTHER WISE CHIP IS NOT WORKING and disiable the tempertaure_sensor and enable the lower power mode
//	ICM_WriteReg(ICM20948_PWR_MGMT_1, 0x29,1); // wake IMU

	ICM_WriteReg(ICM20948_PWR_MGMT_1, 0x01,1); // wake IMU
//

//	TIMER_Delay(TIMER0,10000);
/////****this_register _is uesd the enable and dissable the gyoscope and enable the acc meter**********/
////	ICM_WriteReg(ICM20948_PWR_MGMT_2, 0x07, 1);
//	ICM_WriteReg(ICM20948_PWR_MGMT_2, 0x00, 1);


//    /*****accelometr_register_+2g_configration********/
//
//    ICM_WriteReg(ICM20948_REG_BANK_SEL, ICM20948_BANK_2);
//
//    ICM_WriteReg(ICM20948_ACCEL_CONFIG, 0x1B); // ±2g, LPF enabled
//
//    ICM_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_1, 0x00); // high byte
//    ICM_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_2, 0x09); // low byte


 //  acceleomter_glass_brak_calication();

//   TIMER_Delay(TIMER0,10000);
//
//    acceleomter_motion_human_calication();
//
//    TIMER_Delay(TIMER0,10000);
//
//	imu_enable_motion_interrupt();
//
//    TIMER_Delay(TIMER0,10000);
//
//    acceleomter_curtian_motion_calication();
//
//    TIMER_Delay(TIMER0,10000);
//
//   ICM_WriteReg(ICM20948_REG_BANK_SEL, 0x00,1);  // Ensure Bank 0
//
//    TIMER_Delay(TIMER0,10000);


	/***************CHIP_WEAKUP_REGISTER*************/
		// THIS THE FIRST STEP OTHER WISE CHIP IS NOT WORKING
	//	ICM_WriteReg(ICM20948_PWR_MGMT_1, 0x01); // wake IMU

		/*******the_out_put _data_ 1 is the correct _the-chip is not entring the sleep mode ***/
		//	 pwr_after= ICM_ReadReg(ICM20948_PWR_MGMT_1,1);


//    TIMER_Delay(TIMER0,1000);

  //  ICM_WriteReg(ICM20948_LP_CONFIG_REG, 0x20,1);  // Ensure Bank 0


    while(1)
    {
      i2c_acc_lo_meter();

      TIMER_Delay(TIMER0,100000);

      int_status = ICM_ReadReg(0x19,1);

      UART_Write(UART2,itoa(int_status,buffer1,10) ,sizeof(buffer1));

      memset(buffer1,0,sizeof(buffer1));
    }
}
void i2c_acc_lo_meter()
   {

//		/***************CHIP_WEAKUP_REGISTER*************/
//		// THIS THE FIRST STEP OTHER WISE CHIP IS NOT WORKING
//		ICM_WriteReg(ICM20948_PWR_MGMT_1, 0x01); // wake IMU


//		/*******the_out_put _data_ 1 is the correct _the-chip is not entring the sleep mode ***/
//		 pwr_after= ICM_ReadReg(ICM20948_PWR_MGMT_1);

		/*********************SELCET-THE _BANK _REGISTER****************/
//		 ICM_WriteReg(ICM20948_REG_BANK_SEL, 0x00);  // Ensure Bank 0
//
//		ICM_ReadReg(ICM20948_REG_BANK_SEL);

      /***************ACCLOMETER_REGISTER*************************/


          ICM_ReadReg(ICM20948_ACCEL_XOUT_H,12);

//          ICM_ReadReg(ICM20948_ACCEL_XOUT_L);
//
//
//          ICM_ReadReg(ICM20948_ACCEL_YOUT_H);
//
//		  ICM_ReadReg(ICM20948_ACCEL_YOUT_L);
//
//
//		  ICM_ReadReg(ICM20948_ACCEL_ZOUT_H);
//
//          ICM_ReadReg(ICM20948_ACCEL_ZOUT_L);


//         /*************Gyroscope _ Registers************/
//
//
//		ICM_ReadReg(ICM20948_GYRO_XOUT_H);
//
//		ICM_ReadReg(ICM20948_GYRO_XOUT_L);
//
//
//		ICM_ReadReg(ICM20948_GYRO_YOUT_H);
//
//		ICM_ReadReg(ICM20948_GYRO_YOUT_L);
//
//
//		ICM_ReadReg(ICM20948_GYRO_ZOUT_H);
//
//		ICM_ReadReg(ICM20948_GYRO_ZOUT_L);
//
//
//        /*******TEMPERATURE_REGISTERS**************/
//
//         ICM_ReadReg(ICM20948_TEMP_OUT_H);
//
//         ICM_ReadReg(ICM20948_TEMP_OUT_L);
//
//
//
//      //   CLK_SysTickDelay(20000000); // 200ms
//         TIMER_Delay(TIMER0,10000000);
         /****************acc_metro_data***************/

//         UART_Write(UART2," ACC ",strlen(" ACC "));
//
//         acc_x_axis=(int16_t)((g_u8MstRxData[0]<<8)|g_u8MstRxData[1]);
//
//
//
//         UART_Write(UART2,itoa(acc_x_axis,buffer,10) ,strlen(buffer) );
//         UART_Write(UART2,"  ",strlen("  "));
//
//         acc_y_axis=(int16_t)((g_u8MstRxData[2]<<8)|g_u8MstRxData[3]);
//
//         UART_Write(UART2,itoa(acc_y_axis,buffer,10) ,strlen(buffer) );
//         UART_Write(UART2,"  ",strlen("  "));
//
//         acc_z_axis=(int16_t)((g_u8MstRxData[4]<<8)|g_u8MstRxData[5]);
//
//         UART_Write(UART2,itoa(acc_z_axis,buffer,10) ,strlen(buffer) );
//         UART_Write(UART2,"  ",strlen("  "));


      //   CLK_SysTickDelay(2000000000);
      //   TIMER_Delay(TIMER0,100000000);

//         /****************geo_metro_data***************/

         UART_Write(UART2,"\n\r",strlen("\n\r"));

         geo_x_axis=(int16_t)((g_u8MstRxData[6]<<8)|g_u8MstRxData[7]);

         UART_Write(UART2," GEO ",strlen(" GEO "));

     //    geo_x_axis_x=geo_x_axis;

         UART_Write(UART2,itoa(geo_x_axis,buffer,10) ,sizeof(buffer) );
         UART_Write(UART2,"  ",strlen("  "));


         geo_y_axis=(int16_t)((g_u8MstRxData[8]<<8)|g_u8MstRxData[9]);

      //   geo_y_axis_y=geo_y_axis;

         UART_Write(UART2,itoa(geo_y_axis,buffer,10) ,sizeof(buffer) );
         UART_Write(UART2,"  ",strlen("  "));

         geo_z_axis=(int16_t)((g_u8MstRxData[10]<<8)|g_u8MstRxData[11]);

       //  geo_z_axis_z=geo_z_axis;

         UART_Write(UART2,itoa(geo_z_axis,buffer,10) ,sizeof(buffer) );
         UART_Write(UART2,"  ",strlen("  "));


//         CLK_SysTickDelay(2000000000);

  //       TIMER_Delay(TIMER0,10000000);

         /********************temp************************/

//         UART_Write(UART2,"\n\r",strlen("\n\r"));
//
//         UART_Write(UART2," T ",strlen(" T "));
//
//         temprature=(g_u8MstRxData[14]<<8)|g_u8MstRxData[15];
//
//         UART_Write(UART2,itoa(temprature,buffer,10) ,sizeof(buffer) );
//           UART_Write(UART2,"\r",1);

//         CLK_SysTickDelay(2000000000);



           if((calibrtion_counter<=100)&&(caibrtion_value))
           {
        	   calibrtion_of_the_geo_scope();
        	   if(calibrtion_counter==100)
			   caibrtion_value=0;
        	   calibrtion_counter++;

           }
           else
           {
        	   UART_Write(UART2,"calibrtio_values_of_geo_meter",strlen("calibrtio_values_of_geo_meter"));

			   geo_x_axis=(int16_t)((g_u8MstRxData[6]<<8)|g_u8MstRxData[7]);

			   UART_Write(UART2," GEO ",strlen(" GEO "));

			//   geo_x_axis_x-=geo_x_axis;

			   geo_x_axis=geo_x_axis_x-geo_x_axis;

			 //  geo_x_axis=geo_x_axis-geo_x_axis_x

			   UART_Write(UART2,itoa(geo_x_axis,buffer,10) ,sizeof(buffer) );
			   UART_Write(UART2,"  ",strlen("  "));


			   geo_y_axis=(int16_t)((g_u8MstRxData[8]<<8)|g_u8MstRxData[9]);

			//   geo_y_axis_y-=geo_y_axis;

			   geo_y_axis=geo_y_axis_y-geo_y_axis;

			   UART_Write(UART2,itoa(geo_y_axis,buffer,10) ,sizeof(buffer) );
			   UART_Write(UART2,"  ",strlen("  "));

			   geo_z_axis=(int16_t)((g_u8MstRxData[10]<<8)|g_u8MstRxData[11]);

			  // geo_z_axis_z-=geo_z_axis;

			   geo_z_axis=geo_z_axis_z-geo_z_axis;

			   UART_Write(UART2,itoa(geo_z_axis,buffer,10) ,sizeof(buffer) );
			   UART_Write(UART2,"  ",strlen("  "));

           }

           memset(g_u8MstRxData,0,50);

                 TIMER_Delay(TIMER0,10000);

                   UART_Write(UART2,"\n\r",strlen("\n\r"));

    }

void calibrtion_of_the_geo_scope()
{
	 geo_x_axis_x+=geo_x_axis;

//	 if(calibrtion_counter==100)
//		 geo_x_axis_x=geo_x_axis_x/100;

	 geo_y_axis_y+=geo_y_axis;
//	 if(calibrtion_counter==100)
//		 geo_y_axis_y=geo_y_axis_y/100;

	 geo_z_axis_z+=geo_z_axis;

	 if(calibrtion_counter==100)
	 {
		 geo_x_axis_x=geo_x_axis_x/100;
		 geo_y_axis_y=geo_y_axis_y/100;
		 geo_z_axis_z=geo_z_axis_z/100;
	 }
}

void acceleomter_motion_human_calication()
{
	    ICM_WriteReg(ICM20948_REG_BANK_SEL, ICM20948_BANK_2,1);

	    TIMER_Delay(TIMER0,1000);

	    ICM_WriteReg(ICM20948_ACCEL_CONFIG, 0x11,1); // ±2g, LPF enabled

	    /*±2g is fine for normal walking/running, ±4g if you expect faster motion or impacts*/
	//    ICM_WriteReg(ICM20948_ACCEL_CONFIG, 0x13); // ±4g, LPF enabled

	    TIMER_Delay(TIMER0,1000);

	    ICM_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_1, 0x00,1); // high byte

	    TIMER_Delay(TIMER0,1000);

	    ICM_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_2, 0x04,1); // low byte

	    TIMER_Delay(TIMER0,1000);

	 //   ICM_WriteReg(ICM20948_REG_BANK_SEL, 0x00,1);  // Ensure Bank 0
}
void acceleomter_curtian_motion_calication()
{
	    ICM_WriteReg(ICM20948_REG_BANK_SEL, ICM20948_BANK_2,1);

	    TIMER_Delay(TIMER0,1000);

	    ICM_WriteReg(ICM20948_ACCEL_CONFIG, 0x19,1); // ±2g, LPF enabled

	    TIMER_Delay(TIMER0,1000);

	    ICM_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_1, 0x00,1); // high byte

	    TIMER_Delay(TIMER0,1000);

	    ICM_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_2, 0x09,1); // low byte

	    TIMER_Delay(TIMER0,1000);

	//    ICM_WriteReg(ICM20948_REG_BANK_SEL, 0x00,1);  // Ensure Bank 0
}
void acceleomter_glass_brak_calication()
{
	    ICM_WriteReg(ICM20948_REG_BANK_SEL, ICM20948_BANK_2,1);

	    TIMER_Delay(TIMER0,1000);

	    ICM_WriteReg(ICM20948_ACCEL_CONFIG, 0x03,1); // ±2g, LPF enabled

	    TIMER_Delay(TIMER0,1000);

	    ICM_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_1, 0x00,1); // high byte

	    TIMER_Delay(TIMER0,1000);

	    ICM_WriteReg(ICM20948_ACCEL_SMPLRT_DIV_2, 0x99,1); // low byte

	    TIMER_Delay(TIMER0,1000);

	//    ICM_WriteReg(ICM20948_REG_BANK_SEL, 0x00);  // Ensure Bank 0
}
void imu_enable_motion_interrupt()
{
	/***to selscted the bank 2 iniliztion of the interupt***/
	 ICM_WriteReg(ICM20948_REG_BANK_SEL, ICM20948_BANK_2,1);

     TIMER_Delay(TIMER0,1000);
/** to selscted the intrupt register in bank 2************/
     ICM_WriteReg(ICM20948_ACCEL_INTEL_CTRL, 0x03,1); // ACCEL_INTEL_EN=1, WOM_INTEL_EN=1

     TIMER_Delay(TIMER0,1000);
/*based on the Threshold value interupt is genertsed(sensitivity range)*****/
     ICM_WriteReg(ICM20948_threshold_register, 0x05,1); // Threshold = 20

     TIMER_Delay(TIMER0,1000);
     /***to chnged the  registerin bank 0******/
     ICM_WriteReg(ICM20948_REG_BANK_SEL, ICM20948_BANK_0,1);

     TIMER_Delay(TIMER0,1000);
     /***to enble the intrupr registerin bank 0******/

     ICM_WriteReg(ICM20948_INT_PIN_CFG, 0x80,1);

     TIMER_Delay(TIMER0,1000);

     /***in this register different intrupt is their to selscted the paticuler intrupt*****/

     ICM_WriteReg(ICM20948_INT_ENABLE_1, 0x08,1);

     //✅ 3. If you want to set specific values
     //Threshold (mg)	Register value (decimal)	Hex
     //20 mg    20/4 = 5	        0x05
     //40 mg  	40/4 = 10	        0x0A
     //50 mg	50/4 = 12.5 → 13	0x0D
     //100 mg	100/4 = 25	        0x19
     //200 mg	200/4 = 50	        0x32
     //500 mg	500/4 = 125	        0x7D
     //1000 mg	1000/4 = 250	    0xFA
     /* depending this values to changed the Threshold value sensitivity range */

}





