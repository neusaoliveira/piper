/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  drv\bma180.c                                                              */
/*                                                                            */
/*  Driver para leitura do accelerometro integrado 'a imu                     */
/*                                                                            */
/*  2012-04-10 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#include <string.h>
#include "etc/defines.h"
#include "etc/system.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "lib/i2c.h"
#include "drv/bma180.h"
#include "app/imu/axis.h"
#include "utl/math.h"
#include "utl/utils.h"
#include "conf.h"

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

// Accelerometer address
#define ACCEL_ADDRESS                   0x40 //0x80

// data endian
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

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte AccInit(void)
{
    auto byte BwTcs;

    // clear offset
    AccOfst[0] = 0;
    AccOfst[1] = 0;
    AccOfst[2] = 0;

    /* wait bus busy */
    //while(I2CMasterBusBusy(ACC_I2C_BASE));

    /* Configure slave address (BMA180) for write */
    I2CMasterSlaveAddrSet(ACC_I2C_BASE , ACCEL_ADDRESS , false);

    /* Reseta o camarada */
    I2CMasterDataPut(ACC_I2C_BASE, 0x10);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(ACC_I2C_BASE, 0xB6);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }

    /* Delay a while... */
    Halt(1);

    /* Configure slave address (BMA180) for read */
    I2CMasterSlaveAddrSet(ACC_I2C_BASE , ACCEL_ADDRESS , true);

    /* Le Bandwidth e Temperature Compensation */
    I2CMasterDataPut(ACC_I2C_BASE, 0x20);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    BwTcs = (byte)I2CMasterDataGet(ACC_I2C_BASE);

    /* zera o bandwidth (10Hz) */
    BwTcs &= 0x0F;

    /* Configure slave address (BMA180) for write */
    I2CMasterSlaveAddrSet(ACC_I2C_BASE, ACCEL_ADDRESS, false);

    /* Configura Bw e Tcs */
    I2CMasterDataPut(ACC_I2C_BASE, 0x20);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(ACC_I2C_BASE, BwTcs);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }

    /* Zera interrupcoes */
    I2CMasterDataPut(ACC_I2C_BASE, 0x24);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(ACC_I2C_BASE, 0x00);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(ACC_I2C_BASE, 0x25);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(ACC_I2C_BASE, 0x00);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }

    /* Desabilita mais umas interrupcoes ai */
    I2CMasterDataPut(ACC_I2C_BASE, 0x21);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(ACC_I2C_BASE, 0x00);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }

    /* Desabilita fine tunning */
    I2CMasterDataPut(ACC_I2C_BASE, 0x22);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(ACC_I2C_BASE, 0x14);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }

    /* Faz primeira leitura dos dados */
    AccRead();

    /* ignora os dados lidos */
    memset(Acc,    0x00, 3 * sizeof(short));
    memset(AccRaw, 0x00, 3 * sizeof(short));

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

    /* Configure slave address (BMA180) for write */
    I2CMasterSlaveAddrSet(ACC_I2C_BASE , ACCEL_ADDRESS , false);

    /* Accx Address */
    I2CMasterDataPut(ACC_I2C_BASE, 0x02);
    I2CMasterControl(ACC_I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if(I2CMasterWait1K(ACC_I2C_BASE)) { return(ERRO); }

    /* Configure slave address (BMA180) for read */
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

    // converte de 15..2 -> 13..0
    // Divsao pra manter o sinal
    AccRaw[0] = AccRaw[0] / 4;
    AccRaw[1] = AccRaw[1] / 4;
    AccRaw[2] = AccRaw[2] / 4;

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

