/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  drv\adxl345.c                                                             */
/*                                                                            */
/*  Driver para leitura do accelerometro integrado 'a imu                     */
/*                                                                            */
/*  2012-04-10 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#include "etc/defines.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "lib/i2c.h"
#include "drv/adxl345.h"
#include "app/imu/axis.h"
#include "utl/math.h"
#include "utl/utils.h"
#include "conf.h"

// Accelerometer address
#define ACCEL_ADDRESS                   0x53 //0x3A

#ifndef ACC_BIG_ENDIAN
    #define ACC_BIG_ENDIAN              FALSE
#endif

// filter enabled?
#ifndef ACC_FILTRO
    #define ACC_FILTRO                  0
#endif
#if ACC_FILTRO
    #define ACC_THRESH                  10 // Raw
    #define ACC_WGHT_MAX                85 // %
    #define ACC_WGHT_MIN                10 // %
    #define ACC_WGHT_GAIN               15 // %/step
#endif


// I2c hardware
#ifndef ACC_I2C_BASE
    #define ACC_I2C_BASE                I2C0_MASTER_BASE
#endif

// Outtput Accel
short Acc[3];
short AccRaw[3];
short AccOfst[3];

// Axis alignment constant
const int AxisAlign[9]                  = CNF_IMU_AXIS;

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte AccInit(void)
{
    /* Bias previamente calculado */
    AccOfst[0] = ACC_BIAS_X; //-36; // -7;
    AccOfst[1] = ACC_BIAS_Y; // -8; //  4;
    AccOfst[2] = ACC_BIAS_Z; //149; //-25;

    /* wait bus busy */
    //while(I2CMasterBusBusy(ACC_I2C_BASE));

    /* Configure slave address (ADXL345) for write */
    I2CMasterSlaveAddrSet(ACC_I2C_BASE , ACCEL_ADDRESS , false);

    /* Configura Power */
    I2CMasterDataPut(ACC_I2C_BASE, 0x2D);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(ACC_I2C_BASE, 0x08);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }

    /* Delay a while... */
    //for(i=0;i<1000;i++);

    /* Configure slave address (ADXL345) for write */
    I2CMasterSlaveAddrSet(ACC_I2C_BASE , ACCEL_ADDRESS , false);

    /* Configura Data Format */
    I2CMasterDataPut(ACC_I2C_BASE, 0x31);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(ACC_I2C_BASE, 0x08);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }

    /* Delay a while... */
    //for(i=0;i<1000;i++);

    /* Configure slave address (ADXL345) for write */
    I2CMasterSlaveAddrSet(ACC_I2C_BASE , ACCEL_ADDRESS , false);

    /* Configura Output rate */
    I2CMasterDataPut(ACC_I2C_BASE, 0x2C);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(ACC_I2C_BASE, 0x09);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }

    // Accelerometer initialized
    return(SUCESSO);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte AccRead(void)
{
    auto byte *pData;
    auto byte  i;

    #if ACC_FILTRO
        auto   short AccTmp[3];
        static short AccWgt[3];
    #endif

    /* points to raw data */
    pData = (byte*)AccRaw;

    /* wait bus busy */
    //while(I2CMasterBusBusy(ACC_I2C_BASE));

    /* Configure slave address (ADXL345) for write */
    I2CMasterSlaveAddrSet(ACC_I2C_BASE , ACCEL_ADDRESS , false);

    /* Request accel */
    I2CMasterDataPut(ACC_I2C_BASE, 0x32);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }

    /* Configure slave address (ADXL345) for read */
    I2CMasterSlaveAddrSet(ACC_I2C_BASE , ACCEL_ADDRESS , true);

    /* Read 1 of 6 bytes of data */
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    *pData = (byte)I2CMasterDataGet(ACC_I2C_BASE);
    pData++;

    for(i = 0; i < 4; i++)
    {
        /* Read 2~5 of 6 bytes of data */
        I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
        *pData = (byte)I2CMasterDataGet(ACC_I2C_BASE);
        pData++;
    }

    /* Read 6 of 6 bytes of data */
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    *pData = (byte)I2CMasterDataGet(ACC_I2C_BASE);


    #if ACC_BIG_ENDIAN
        Swap16(&AccRaw[0]);
        Swap16(&AccRaw[1]);
        Swap16(&AccRaw[2]);
    #endif

    /* Devemos usar filtro? */
    #if ACC_FILTRO

        // Calibrated Data
        AccTmp[0] = AxisAlign[3] * (AccRaw[ACC_X] - AccOfst[0]);
        AccTmp[1] = AxisAlign[4] * (AccRaw[ACC_Y] - AccOfst[1]);
        AccTmp[2] = AxisAlign[5] * (AccRaw[ACC_Z] - AccOfst[2]);

        // check variationess
        if(abss(AccTmp[0] - Acc[0]) < ACC_THRESH) { AccWgt[0] += ACC_WGHT_GAIN; }
        else                                      { AccWgt[0] -= ACC_WGHT_GAIN; }
        if(abss(AccTmp[1] - Acc[1]) < ACC_THRESH) { AccWgt[1] += ACC_WGHT_GAIN; }
        else                                      { AccWgt[1] -= ACC_WGHT_GAIN; }
        if(abss(AccTmp[2] - Acc[2]) < ACC_THRESH) { AccWgt[2] += ACC_WGHT_GAIN; }
        else                                      { AccWgt[2] -= ACC_WGHT_GAIN; }

        // limit values
        AccWgt[0] = lims(AccWgt[0], ACC_WGHT_MIN, ACC_WGHT_MAX);
        AccWgt[1] = lims(AccWgt[1], ACC_WGHT_MIN, ACC_WGHT_MAX);
        AccWgt[2] = lims(AccWgt[2], ACC_WGHT_MIN, ACC_WGHT_MAX);

        // Update variables (Sequence is 1-0-2 for alignment!)
        Acc[0] = (((AccTmp[0] * (100-AccWgt[0])) + (Acc[0] * AccWgt[0])) + 50) / 100;
        Acc[1] = (((AccTmp[1] * (100-AccWgt[1])) + (Acc[1] * AccWgt[1])) + 50) / 100;
        Acc[2] = (((AccTmp[2] * (100-AccWgt[2])) + (Acc[2] * AccWgt[2])) + 50) / 100;
    #else
        // Update variables (Sequence is 1-0-2 for alignment!)
        Acc[0] = AxisAlign[3] * (AccRaw[ACC_X] - AccOfst[0]);
        Acc[1] = AxisAlign[4] * (AccRaw[ACC_Y] - AccOfst[1]);
        Acc[2] = AxisAlign[5] * (AccRaw[ACC_Z] - AccOfst[2]);
    #endif

    // Accel read
    return(SUCESSO);
}

