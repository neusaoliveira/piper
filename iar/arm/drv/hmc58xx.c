/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  drv\hmc58xx.c                                                             */
/*                                                                            */
/*  Driver para leitura do magnetometro integrado 'a imu                      */
/*                                                                            */
/*  2012-04-13 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/


#include "etc/defines.h"
#include "etc/system.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "lib/i2c.h"
#include "drv/hmc58xx.h"
#include "utl/utils.h"
#include "app/imu/axis.h"
#include "conf.h"

/* Magnetometer I2C address */
#define MAGNET_ADDRESS                  0x1E //0x3C

/* Depending on compiler, it needs to swap bytes */
#ifndef MAG_BIG_ENDIAN
    #define MAG_BIG_ENDIAN              FALSE
#endif

/* Using backup */
#ifndef MAG_BACKUP
    #define MAG_BACKUP                  FALSE
#endif

/* Auto calibration */
#ifndef MAG_AUTOCAL
    #define MAG_AUTOCAL                 FALSE
#endif

// I2c hardware
#ifndef MAG_I2C_BASE
    #define MAG_I2C_BASE                I2C0_MASTER_BASE
#endif

// CRA config
#define MAG_CRA                         0x70
#define MAG_CRA_POS                     (MAG_CRA + 1)
#define MAG_CRA_NEG                     (MAG_CRA + 2)

// Outtput magnetometer
short Mag[3];
short MagRaw[3];
short MagOfst[3];

short MagPos[3];
short MagNeg[3];

#if MAG_BACKUP
    short eeprom MagBkk[3];
#endif

#if MAG_AUTOCAL
    short MagX[2];
    short MagY[2];
    short MagZ[2];
#endif

// Axis alignment constant
static int AxisAlign[9]                  = CNF_IMU_AXIS;
    
/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte MagInit(void)
{
    #if MAG_BACKUP
        // check eeprom for default values...
        if((short)MagBkk[0] == 0xFFFF)
        {
            // save defaults
            MagBkk[0] =   88;
            MagBkk[1] = -110;
            MagBkk[2] = -244;
        }

        // load offset
        MagOfst[0] = MagBkk[0];
        MagOfst[1] = MagBkk[1];
        MagOfst[2] = MagBkk[2];
    #else
        // default offset
        MagOfst[0] = -130; //-33;
        MagOfst[1] = -104; //-43;
        MagOfst[2] =  -11; // 11;
    #endif

    #if MAG_AUTOCAL
        MagX[0] = 32000; MagX[1] = -32000;
        MagY[0] = 32000; MagY[1] = -32000;
        MagZ[0] = 32000; MagZ[1] = -32000;
    #endif

    /* wait bus busy */
    //while(I2CMasterBusBusy(MAG_I2C_BASE));

    /* Configure slave address (HMC5800) for write */
    I2CMasterSlaveAddrSet(MAG_I2C_BASE , MAGNET_ADDRESS , false);

    /* Endereço do primeiro registrador CRA */
    I2CMasterDataPut(MAG_I2C_BASE, 0x00);
    I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }

    /* configura 8 amostras por leitura, a 15Hz, normal */
    I2CMasterDataPut(MAG_I2C_BASE, MAG_CRA);
    I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }

    /* configura o ganho */
    I2CMasterDataPut(MAG_I2C_BASE, 0x80);
    I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }

    /* modo de operacao */
    I2CMasterDataPut(MAG_I2C_BASE, 0x00);
    I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }

    // Magnetometer initialized
    return(SUCESSO);
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte MagRead(void)
{
    auto byte *pData;
    auto byte  i;

    /* points to raw data */
    pData = (byte*)MagRaw;

    /* wait bus busy */
    //while(I2CMasterBusBusy(MAG_I2C_BASE));

    /* Configure slave address (HMC5800) for write */
    I2CMasterSlaveAddrSet(MAG_I2C_BASE , MAGNET_ADDRESS , false);

    /* Request accel */
    I2CMasterDataPut(MAG_I2C_BASE, 0x03);
    I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }

    /* Configure slave address (HMC5800) for read */
    I2CMasterSlaveAddrSet(MAG_I2C_BASE , MAGNET_ADDRESS , true);

    /* Read 1 of 6 bytes of data */
    I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }
    *pData = (byte)I2CMasterDataGet(MAG_I2C_BASE);
    pData++;

    for(i = 0; i < 4; i++)
    {
        /* Read 2~5 of 6 bytes of data */
        I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }
        *pData = (byte)I2CMasterDataGet(MAG_I2C_BASE);
        pData++;
    }

    /* Read 6 of 6 bytes of data */
    I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }
    *pData = (byte)I2CMasterDataGet(MAG_I2C_BASE);

    #if MAG_BIG_ENDIAN
        Swap16((unsigned short*)&MagRaw[0]);
        Swap16((unsigned short*)&MagRaw[1]);
        Swap16((unsigned short*)&MagRaw[2]);
    #endif

    // Update variables (Sequence is 1-0-2 for alignment!)
    Mag[0] = AxisAlign[6] * (MagRaw[MAG_X] - MagOfst[0]);
    Mag[1] = AxisAlign[7] * (MagRaw[MAG_Y] - MagOfst[1]);
    Mag[2] = AxisAlign[8] * (MagRaw[MAG_Z] - MagOfst[2]);

    // auto calibration
    #if MAG_AUTOCAL
        // limits of X
        if(MagRaw[MAG_X] < MagX[0]) {MagX[0] = MagRaw[MAG_X]; }
        if(MagRaw[MAG_X] > MagX[1]) {MagX[1] = MagRaw[MAG_X]; }

        // limits of Y
        if(MagRaw[MAG_Y] < MagY[0]) {MagY[0] = MagRaw[MAG_Y]; }
        if(MagRaw[MAG_Y] > MagY[1]) {MagY[1] = MagRaw[MAG_Y]; }

        // limits of Z
        if(MagRaw[MAG_Z] < MagZ[0]) {MagZ[0] = MagRaw[MAG_Z]; }
        if(MagRaw[MAG_Z] > MagZ[1]) {MagZ[1] = MagRaw[MAG_Z]; }

        // calculates centroid
        MagOfst[0] = (MagX[0] + MagX[1]) / 2;
        MagOfst[1] = (MagY[0] + MagY[1]) / 2;
        MagOfst[2] = (MagZ[0] + MagZ[1]) / 2;
    #endif

    // Magnet read
    return(SUCESSO);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte MagCal(void)
{
    auto byte  i;
    auto long  Read[3];

    /* configura bias positivo */

        /* Configure slave address (HMC5800) for write */
        I2CMasterSlaveAddrSet(MAG_I2C_BASE , MAGNET_ADDRESS , false);

        /* Endereço do primeiro registrador CRA */
        I2CMasterDataPut(MAG_I2C_BASE, 0x00);
        I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }

        /* configura 8 amostras por leitura, a 15Hz, normal */
        I2CMasterDataPut(MAG_I2C_BASE, MAG_CRA_POS);
        I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }

        /* descarta 100 leituras */
        for(i = 0; i < 20; i++){ MagRead(); Halt(80); }

        /* amostra 100 leituras */
        Read[0] = 0;
        Read[1] = 0;
        Read[2] = 0;

        for(i = 0; i < 20; i++)
        {
            // faz a leitura
            MagRead();

            // acumula
            Read[0] += MagRaw[MAG_X];
            Read[1] += MagRaw[MAG_Y];
            Read[2] += MagRaw[MAG_Z];

            Halt(80);
        }

        MagPos[0] = (short)(Read[0] + 10) / 20;
        MagPos[1] = (short)(Read[1] + 10) / 20;
        MagPos[2] = (short)(Read[2] + 10) / 20;

    /* configura bias negativo */

        /* Configure slave address (HMC5800) for write */
        I2CMasterSlaveAddrSet(MAG_I2C_BASE , MAGNET_ADDRESS , false);

        /* Endereço do primeiro registrador CRA */
        I2CMasterDataPut(MAG_I2C_BASE, 0x00);
        I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }

        /* configura 8 amostras por leitura, a 15Hz, normal */
        I2CMasterDataPut(MAG_I2C_BASE, MAG_CRA_NEG);
        I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }

        /* descarta 100 leituras */
        for(i = 0; i < 20; i++){ MagRead(); Halt(80); }

        /* amostra 100 leituras */
        Read[0] = 0;
        Read[1] = 0;
        Read[2] = 0;

        for(i = 0; i < 20; i++)
        {
            // faz a leitura
            MagRead();

            // acumula
            Read[0] += MagRaw[MAG_X];
            Read[1] += MagRaw[MAG_Y];
            Read[2] += MagRaw[MAG_Z];

            Halt(80);
        }

        MagNeg[0] = (short)(Read[0] + 10) / 20;
        MagNeg[1] = (short)(Read[1] + 10) / 20;
        MagNeg[2] = (short)(Read[2] + 10) / 20;


    /* configura bias normal */

        /* Configure slave address (HMC5800) for write */
        I2CMasterSlaveAddrSet(MAG_I2C_BASE , MAGNET_ADDRESS , false);

        /* Endereço do primeiro registrador CRA */
        I2CMasterDataPut(MAG_I2C_BASE, 0x00);
        I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }

        /* configura 8 amostras por leitura, a 15Hz, normal */
        I2CMasterDataPut(MAG_I2C_BASE, MAG_CRA);
        I2CMasterControl(MAG_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        if(I2CMasterWait1K(MAG_I2C_BASE)) { return(ERRO); }

        /* descarta 100 leituras */
        for(i = 0; i < 100; i++){ MagRead(); Halt(20); }

   /* ok */
   return(SUCESSO);
}

