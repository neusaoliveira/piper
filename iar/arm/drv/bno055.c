/******************************************************************************/
/*                                                                            */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  drv\bno055.c                                                              */
/*  adafru.it/2472                                                            */
/*                                                                            */
/*  Driver para leitura da IMU BNO055                                         */
/*                                                                            */
/*  2015-07-14 Marcelo Henrique Santos                                        */
/*                                                                            */
/******************************************************************************/

#include "etc/defines.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "lib/i2c.h"
#include "drv/bno055.h"
#include "utl/utils.h"
#include "conf.h"

#include <string.h>

/******************************************************************************/
/*                                                                            */
/*  Operation mode settings                                                   */
/*                                                                            */
/******************************************************************************/
typedef enum
{
	OPERATION_MODE_CONFIG                   = 0X00,
	OPERATION_MODE_ACCONLY                  = 0X01,
	OPERATION_MODE_MAGONLY                  = 0X02,
	OPERATION_MODE_GYRONLY                  = 0X03,
	OPERATION_MODE_ACCMAG                   = 0X04,
	OPERATION_MODE_ACCGYRO                  = 0X05,
	OPERATION_MODE_MAGGYRO                  = 0X06,
	OPERATION_MODE_AMG                      = 0X07,
	OPERATION_MODE_IMUPLUS                  = 0X08,
	OPERATION_MODE_COMPASS                  = 0X09,
	OPERATION_MODE_M4G                      = 0X0A,
	OPERATION_MODE_NDOF_FMC_OFF             = 0X0B,
	OPERATION_MODE_NDOF                     = 0X0C
} operation_mode;


/******************************************************************************/
/*                                                                            */
/*  Power modes settings                                                      */
/*                                                                            */
/******************************************************************************/
typedef enum
{
	POWER_MODE_NORMAL                       = 0X00,
	POWER_MODE_LOWPOWER                     = 0X01,
	POWER_MODE_SUSPEND                      = 0X02
} powermode;

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Data */
StBnoEuler BnoData;


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte  Bno055ReadByte(byte Addr);
void  Bno055WriteByte(byte Addr, byte Data);
short BnoReadShort(byte Addr);


/******************************************************************************/
/*                                                                            */
/* Make sure we have the right device                               	      */
/*                                                                            */
/******************************************************************************/
byte Bno085RevInfo(void)
{
	
	memset(StBnoData.info, 2, sizeof(StRevInfo));
	
	/* Check the accelerometer revision */
	StBnoData.info.accel_rev = Bno055ReadByte(BNO055_ACCEL_REV_ID_ADDR);

	/* Check the magnetometer revision */
	StBnoData.info.mag_rev   = Bno055ReadByte(BNO055_MAG_REV_ID_ADDR);

	/* Check the gyroscope revision */
	StBnoData.info.gyro_rev  = Bno055ReadByte(BNO055_GYRO_REV_ID_ADDR);

	/* Check the SW revision */
	StBnoData.info.bl_rev    = Bno055ReadByte(BNO055_BL_REV_ID_ADDR);	
	StBnoData.info.sw_rev    = BnoReadShort(BNO055_SW_REV_ID_LSB_ADDR);	
	
	return SUCESSO;        
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte BnoInit(void)
{
    /* Switch to config mode (just in case since this is the default) */
    Bno055WriteByte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);

    /* Reset */
    Bno055WriteByte(BNO055_SYS_TRIGGER_ADDR, 0x20);	
    //delay(50);

    /* Set to normal power mode */
    Bno055WriteByte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    //delay(10);

    Bno055WriteByte(BNO055_PAGE_ID_ADDR, 0);

    /* Set the output units */
    /*
    uint8_t unitsel = (0 << 7) | // Orientation = Android
                      (0 << 4) | // Temperature = Celsius
                      (0 << 2) | // Euler = Degrees
                      (1 << 1) | // Gyro = Rads
                      (0 << 0);  // Accelerometer = m/s^2
    Bno055WriteByte(BNO055_UNIT_SEL_ADDR, unitsel);
    */

    Bno055WriteByte(BNO055_SYS_TRIGGER_ADDR, 0x0);
    //delay(10);	
    
    /* Switch to NDOF mode (just in case since this is the default) */
    Bno055WriteByte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    //delay(20);
    
    /* IMU initialized */
    return(SUCESSO);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte BnoReadAcc(void)
{
    	
	BnoData.AccRaw[0] = BnoReadShort(BNO055_ACCEL_DATA_X_LSB_ADDR);
    BnoData.AccRaw[1] = BnoReadShort(BNO055_ACCEL_DATA_Y_LSB_ADDR);
	BnoData.AccRaw[2] = BnoReadShort(BNO055_ACCEL_DATA_Z_LSB_ADDR);
	
    #if BNO_BIG_ENDIAN
        Swap16(&BnoData.AccRaw[0]);
        Swap16(&BnoData.AccRaw[1]);
        Swap16(&BnoData.AccRaw[2]);
    #endif

    // Acc data read
    return(SUCESSO);
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte Bno055ReadByte(byte Addr)
{
    /* wait bus busy */
    //while(I2CMasterBusBusy(BNO_I2C_BASE));

    /* Configure slave address (BNO055) for write */
    I2CMasterSlaveAddrSet(BNO_I2C_BASE , BNO055_ADDRESS_A , false);

    /* Send register Addr */
    I2CMasterDataPut(BNO_I2C_BASE, Addr);
    I2CMasterControl(BNO_I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if(I2CMasterWait1K(BNO_I2C_BASE)) { return(ERRO); }

    /* Configure slave address (BNO055) for read */
    I2CMasterSlaveAddrSet(BNO_I2C_BASE , BNO055_ADDRESS_A , true);

    /* Read 1 byte of data */
    I2CMasterControl(BNO_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    if(I2CMasterWait1K(BNO_I2C_BASE)) { return(ERRO); }
    return((byte)I2CMasterDataGet(BNO_I2C_BASE));
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
short BnoReadShort(byte Addr)
{
	auto short Ret;
    auto byte *p;
    
    p = (byte*)&Ret;

    /* wait bus busy */
    //while(I2CMasterBusBusy(BNO_I2C_BASE));

    /* Configure slave address (BNO055) for write */
    I2CMasterSlaveAddrSet(BNO_I2C_BASE , BNO055_ADDRESS_A , false);

    /* Send register Addr */
    I2CMasterDataPut(BNO_I2C_BASE, Addr);
    I2CMasterControl(BNO_I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(BNO_I2C_BASE));// { return(0x7FFF); }

    /* Configure slave address (BNO055) for read */
    I2CMasterSlaveAddrSet(BNO_I2C_BASE , BNO055_ADDRESS_A , true);

    /* Read L byte of data */
    I2CMasterControl(BNO_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(BNO_I2C_BASE));// { return(0x7FFF); }
    *p = (byte)I2CMasterDataGet(BNO_I2C_BASE);
    p++;

    /* Read H byte of data */
    I2CMasterControl(BNO_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(BNO_I2C_BASE));// { return(0x7FFF); }
    *p = (byte)I2CMasterDataGet(BNO_I2C_BASE);
    
    return(Ret);
	
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void Bno055WriteByte(byte Addr, byte Data)
{
    /* wait bus busy */
    //while(I2CMasterBusBusy(BNO_I2C_BASE));

    /* Configure slave address (BNO055) for write */
    I2CMasterSlaveAddrSet(BNO_I2C_BASE , BNO055_ADDRESS_A , false);

    /* Send register Addr */
    I2CMasterDataPut(BNO_I2C_BASE, Addr);
    I2CMasterControl(BNO_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if(I2CMasterWait1K(BNO_I2C_BASE)) { return; }

    /* Write 1 byte of data */
    I2CMasterDataPut(BNO_I2C_BASE, Data);
    I2CMasterControl(BNO_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    I2CMasterWait1K(BNO_I2C_BASE);
}

