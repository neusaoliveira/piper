/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  drv/itg3200.c                                                             */
/*                                                                            */
/*  Modulo de leitura e processamento do sensor girometro                     */
/*                                                                            */
/*  2012-10-10 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/



#include "etc/defines.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "lib/i2c.h"
#include "drv/itg3200.h"
#include "app/imu/axis.h"
#include "utl/math.h"
#include "utl/utils.h"
#include "conf.h"

// I2c hardware
#ifndef GYR_I2C_BASE
    #define GYR_I2C_BASE                I2C0_MASTER_BASE
#endif

// gyro address
#define ITG3200_ADDRESS                 0x68 //0xD0

// endian
#ifndef GYR_BIG_ENDIAN
    #define GYR_BIG_ENDIAN              FALSE
#endif

// internal low pass filter (0..6) (Datasheet Pg 24)
#define DFLP_CFG                        6

// filter enabled?
#ifndef GYR_FILTRO
    #define GYR_FILTRO                  0
#endif
#if GYR_FILTRO
    #define GYR_THRESH                  10 // Raw
    #define GYR_WGHT_MAX                85 // %
    #define GYR_WGHT_MIN                10 // %
    #define GYR_WGHT_GAIN               15 // %/step
#endif

short Gyr[3];               // Array that store the 3 ADC filtered data (gyros)
short GyrRaw[3];            // Array that store the 3 ADC raw data
short GyrOfst[3]={0,0,0};   // Array that stores the Offset of the sensors

#if GYR_READ_TEMP
    short GyrTmpRaw;
    short GyrTmp;
#endif

// Axis alignment constant
static int AxisAlign[9]                  = CNF_IMU_AXIS;

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte GyrInit(void)
{
    // load offset
    GyrOfst[0] = 0;
    GyrOfst[1] = 0;
    GyrOfst[2] = 0;

    /* Configure slave address (ITG3200) for write */
    I2CMasterSlaveAddrSet(GYR_I2C_BASE , ITG3200_ADDRESS , false);

    /* Envia HW Reset */
    I2CMasterDataPut(GYR_I2C_BASE, 0x3E);
    I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(GYR_I2C_BASE, 0x80);
    I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }

    /* Configure slave address (ITG3200) for write */
    I2CMasterSlaveAddrSet(GYR_I2C_BASE , ITG3200_ADDRESS , false);

    /* Configura modo de operacao: Fundo de Escala e Filtro PB */
    I2CMasterDataPut(GYR_I2C_BASE, 0x16);
    I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(GYR_I2C_BASE, 0x18 | DFLP_CFG);
    I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }

    /* Configure slave address (ITG3200) for write */
    I2CMasterSlaveAddrSet(GYR_I2C_BASE , ITG3200_ADDRESS , false);

    /* NOTA:                                                        */
    /*  Como recomendado no datasheet, fonte de clock referenciada  */
    /*  em um dos gyros (x-y-z). O eixo com menor vibracao é o eixo */
    /*  longitudinal da acft X, logo, Gyro-Y                        */
    /*                                                              */
    #define EIXO_CLK    (GYR_Y + 1) // Eixo-X
    //#define EIXO_CLK    (GYR_X + 1) // Eixo-Y
    //#define EIXO_CLK    (GYR_Z + 1) // Gyro-Z

    /* Sai do Reset */
    I2CMasterDataPut(GYR_I2C_BASE, 0x3E);
    I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }
    I2CMasterDataPut(GYR_I2C_BASE, 0x00 | EIXO_CLK);
    I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }

    // gyro initialized
    return(SUCESSO);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte GyrRead(void)
{
    auto byte *pData;
    auto byte  i;

    #if GYR_READ_TEMP
        auto byte *pTemp;
    #endif

    #if GYR_FILTRO
        auto   short GyrTmp[3];
        static short GyrWgt[3];
    #endif

    /* points to raw data */
    pData = (byte*)GyrRaw;

    #if GYR_READ_TEMP
        //pTemp = (byte*)&GyrTmpRaw;
        pTemp = (byte*)&GyrTmp;
    #endif

    /* Configure slave address (ITG3200) for write */
    I2CMasterSlaveAddrSet(GYR_I2C_BASE , ITG3200_ADDRESS , false);

    #if GYR_READ_TEMP
        /* Request temp & gyro */
        I2CMasterDataPut(GYR_I2C_BASE, 0x1B);
    #else
        /* Request gyro */
        I2CMasterDataPut(GYR_I2C_BASE, 0x1D);
    #endif

    I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }

    /* Configure slave address (ITG3200) for read */
    I2CMasterSlaveAddrSet(GYR_I2C_BASE , ITG3200_ADDRESS , true);

    /* Read 1 of 6 bytes of data */
    I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }

    #if GYR_READ_TEMP
        /* Read temp MSB */
        *pTemp = (byte)I2CMasterDataGet(GYR_I2C_BASE);
        pTemp++;

        /* Read temp LSB */
        I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }
        *pTemp = (byte)I2CMasterDataGet(GYR_I2C_BASE);
        pTemp++;

        /* Read GYRO_X MSB */
        I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }
    #endif

    /* Read gyro */
    *pData = (byte)I2CMasterDataGet(GYR_I2C_BASE);
    pData++;

    for(i = 0; i < 4; i++)
    {
        /* Read 2~5 of 6 bytes of data */
        I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }
        *pData = (byte)I2CMasterDataGet(GYR_I2C_BASE);
        pData++;
    }

    /* Read 6 of 6 bytes of data */
    I2CMasterControl(GYR_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if(I2CMasterWait1K(GYR_I2C_BASE)) { return(ERRO); }
    *pData = (byte)I2CMasterDataGet(GYR_I2C_BASE);

    #if GYR_BIG_ENDIAN
        Swap16((unsigned short*)&GyrRaw[0]);
        Swap16((unsigned short*)&GyrRaw[1]);
        Swap16((unsigned short*)&GyrRaw[2]);

        #if GYR_READ_TEMP
            Swap16((unsigned short*)&GyrTmpRaw);
        #endif
    #endif


    /* Devemos usar filtro? */
    #if GYR_FILTRO

        // Update variables (Sequence is 1-0-2 for alignment!)
        GyrTmp[0] = AxisAlign[0] * (GyrRaw[GYR_X] - GyrOfst[0]);
        GyrTmp[1] = AxisAlign[1] * (GyrRaw[GYR_Y] - GyrOfst[1]);
        GyrTmp[2] = AxisAlign[2] * (GyrRaw[GYR_Z] - GyrOfst[2]);

        // check variationess
        if(abss(GyrTmp[0] - Gyr[0]) < GYR_THRESH) { GyrWgt[0] += GYR_WGHT_GAIN; }
        else                                      { GyrWgt[0] -= GYR_WGHT_GAIN; }
        if(abss(GyrTmp[1] - Gyr[1]) < GYR_THRESH) { GyrWgt[1] += GYR_WGHT_GAIN; }
        else                                      { GyrWgt[1] -= GYR_WGHT_GAIN; }
        if(abss(GyrTmp[2] - Gyr[2]) < GYR_THRESH) { GyrWgt[2] += GYR_WGHT_GAIN; }
        else                                      { GyrWgt[2] -= GYR_WGHT_GAIN; }

        // limit values
        GyrWgt[0] = lims(GyrWgt[0], GYR_WGHT_MIN, GYR_WGHT_MAX);
        GyrWgt[1] = lims(GyrWgt[1], GYR_WGHT_MIN, GYR_WGHT_MAX);
        GyrWgt[2] = lims(GyrWgt[2], GYR_WGHT_MIN, GYR_WGHT_MAX);

        // Update variables (Sequence is 1-0-2 for alignment!)
        Gyr[0] = (((GyrTmp[0] * (100-GyrWgt[0])) + (Gyr[0] * GyrWgt[0])) + 50) / 100;
        Gyr[1] = (((GyrTmp[1] * (100-GyrWgt[1])) + (Gyr[1] * GyrWgt[1])) + 50) / 100;
        Gyr[2] = (((GyrTmp[2] * (100-GyrWgt[2])) + (Gyr[2] * GyrWgt[2])) + 50) / 100;
    #else
        // Update variables (Sequence is 1-0-2 for alignment!)
        Gyr[0] = AxisAlign[0] * (GyrRaw[GYR_X] - GyrOfst[0]);
        Gyr[1] = AxisAlign[1] * (GyrRaw[GYR_Y] - GyrOfst[1]);
        Gyr[2] = AxisAlign[2] * (GyrRaw[GYR_Z] - GyrOfst[2]);
    #endif

    // Filtro adaptativo -> quanto maior a variação, menor o peso
    // quando menor a variacao, maior o peso (menos atraso)

    // Magnet read
    return(SUCESSO);
}
