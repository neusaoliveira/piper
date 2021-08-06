/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  drv\bmp085.c                                                              */
/*                                                                            */
/*  Driver para leitura do sensor de pressao BMP085                           */
/*                                                                            */
/*  2012-11-29 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#include "etc/defines.h"
#include "etc/system.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "lib/gpio.h"
#include "lib/i2c.h"
#include "drv/bmp085.h"
#include "utl/math.h"
#include "conf.h"

// Accelerometer address
#define BARO_ADDRESS                    0x77 // 0xEE

// Endian mode
#ifndef BAR_BIG_ENDIAN
    #define BAR_BIG_ENDIAN              FALSE
#endif

// Use End oc conversion pin?
#ifndef BAR_USE_EOC
    #define BAR_USE_EOC                 FALSE
#endif

// display debug
#ifndef BAR_VERBOSE
    #define BAR_VERBOSE                 FALSE
#endif

#if BAR_VERBOSE
    #include "lib/utils/uartstdio.h"
#endif

// oversampling level
#define BAR_OVERSAMPLING                3


// I2c hardware
#ifndef BAR_I2C_BASE
    #define BAR_I2C_BASE                I2C0_MASTER_BASE
#endif

// registers from datasheet
//  EEPROM	0xAA to 0xBF
//  START	0xF4 + Val
//  RESULT 	0xF6[H] 0xF7[L]
//
//  0xF4	Val	Time
//  Temp	2E	4.5
//  P S0	34	4.5
//  P S1	74	7.5
//  P S2	B4	13.5
//  P S3	F4	25.5
// https://github.com/adafruit/Adafruit-BMP085-Library/blob/master/Adafruit_BMP085.cpp
// http://video-one.com/video/c15e036e9d56f9c39cf390166bbc0178.html?fid=Teen


// eeprom registers index
#define AC1                             0x00
#define AC2                             0x01
#define AC3                             0x02
#define AC4                             0x03 // uns short
#define AC5                             0x04 // uns short
#define AC6                             0x05 // uns short
#define B1                              0x06
#define B2                              0x07
#define MB                              0x08
#define MC                              0x09
#define MD                              0x0A

// states
#define BAR_STE_IDLE                    0x00
#define BAR_STE_RQ_TEMP                 0x01
#define BAR_STE_AG_TEMP                 0x02
#define BAR_STE_RD_TEMP                 0x03
#define BAR_STE_RQ_PRESS                0x04
#define BAR_STE_AG_PRESS                0x05
#define BAR_STE_RD_PRESS                0x06
#define BAR_STE_COMPUTE                 0x07

// Outtput pressure and temperature
long  BarBaro;
long  BarTemp;
long  BarRawTmp;
long  BarRawBar;
dword BarTout;
byte  Xlsb;
byte  BarState;

// eeprom parameters from sensor
short BarEep[11];

// prototipes
byte  Bmp085ReadByte(byte Addr);
short Bmp085ReadShort(byte Addr);
void  Bmp085WriteByte(byte Addr, byte Data);

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte BarInit(void)
{
    // initial state
    BarState = BAR_STE_IDLE;

    // No data
    BarBaro = -1;
    BarTemp = -1;

    // read data
    BarEep[AC1] = Bmp085ReadShort(0xAA);
    BarEep[AC2] = Bmp085ReadShort(0xAC);
    BarEep[AC3] = Bmp085ReadShort(0xAE);
    BarEep[AC4] = Bmp085ReadShort(0xB0);
    BarEep[AC5] = Bmp085ReadShort(0xB2);
    BarEep[AC6] = Bmp085ReadShort(0xB4);

    BarEep[B1]  = Bmp085ReadShort(0xB6);
    BarEep[B2]  = Bmp085ReadShort(0xB8);

    BarEep[MB]  = Bmp085ReadShort(0xBA);
    BarEep[MC]  = Bmp085ReadShort(0xBC);
    BarEep[MD]  = Bmp085ReadShort(0xBE);

    #if BAR_VERBOSE

        // show eeprom data
        UARTprintf("Baro eeprom parameters:\r\n");
        UARTprintf("  AC1: %7d\r\n", BarEep[0]);
        UARTprintf("  AC2: %7d\r\n", BarEep[1]);
        UARTprintf("  AC3: %7d\r\n", BarEep[2]);
        UARTprintf("  AC4: %7u\r\n", (unsigned short)BarEep[3]);
        UARTprintf("  AC5: %7u\r\n", (unsigned short)BarEep[4]);
        UARTprintf("  AC6: %7u\r\n", (unsigned short)BarEep[5]);

        UARTprintf("  B1 : %7d\r\n", BarEep[6]);
        UARTprintf("  B2 : %7d\r\n", BarEep[7]);

        UARTprintf("  MB : %7d\r\n", BarEep[8]);
        UARTprintf("  MC : %7d\r\n", BarEep[9]);
        UARTprintf("  MD : %7d\r\n", BarEep[10]);
        UARTprintf("\r\n");

    #endif


    // Barometer initialized
    return(SUCESSO);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte BarReadBlock(void)
{
    // mark to make a new readout
    BarState = BAR_STE_RQ_TEMP;

    // ok
    return(SUCESSO);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void BarRefresh(void)
{
    switch(BarState)
    {
        /**********************************************************************/
        /*                                                                    */
        case BAR_STE_IDLE:

            // faz porra nenhuma

        break;

        /**********************************************************************/
        /*                                                                    */
        case BAR_STE_RQ_TEMP:

            // start temperature conversion (write 0x2E to Addr 0xF4)
            Bmp085WriteByte(0xF4, 0x2E);

            // wait for conversion to be done within 20ms timeout
            BarTout = SysSetaTimeout(20);

            // wait for it
            BarState = BAR_STE_AG_TEMP;
        break;

        /**********************************************************************/
        /*                                                                    */
        case BAR_STE_AG_TEMP:

            #if BAR_USE_EOC
                if(GPIOPinRead(BAR_PORT, BAR_PIN_EOC) == 0)
                {
                    // still running, check timeout
                    if(SysVenceuTimeout(BarTout))
                    {
                        // error
                        BarState = BAR_STE_IDLE;
                    }
                }
                else
                {
                    // change to read temp
                    BarState = BAR_STE_RD_TEMP;
                }
            #else
                // timeout is over
                if(SysVenceuTimeout(BarTout))
                {
                    // change to read temp
                    BarState = BAR_STE_RD_TEMP;
                }
            #endif

        break;

        /**********************************************************************/
        /*                                                                    */
        case BAR_STE_RD_TEMP:

            // read 2 byte conversion result
            //RawTmp = Bmp085ReadByte(0xF6); // H
            //RawTmp <<= 8;
            //RawTmp |= Bmp085ReadByte(0xF7); // L
            BarRawTmp = (long)Bmp085ReadShort(0xF6) & 0x0000FFFF;
            if(BarRawTmp == 0x7FFF)
            {
                // say that values are not ok
                BarBaro = -1;
                BarTemp = -1;

                // restart
                BarState = BAR_STE_IDLE;
            }
            else
            {
                // req pressure
                BarState = BAR_STE_RQ_PRESS;
            }
        break;

        /**********************************************************************/
        /*                                                                    */
        case BAR_STE_RQ_PRESS:

            // start temperature conversion (write 0x2E to Addr 0xF4)
            Bmp085WriteByte(0xF4, 0xF4);

            // wait for conversion to be done within 50ms timeout
            BarTout = SysSetaTimeout(50);

            // wait for it
            BarState = BAR_STE_AG_PRESS;
        break;

        /**********************************************************************/
        /*                                                                    */
        case BAR_STE_AG_PRESS:

            #if BAR_USE_EOC
                if(GPIOPinRead(BAR_PORT, BAR_PIN_EOC) == 0)
                {
                    // still running, check timeout
                    if(SysVenceuTimeout(BarTout))
                    {
                        // error
                        BarState = BAR_STE_IDLE;
                    }
                }
                else
                {
                    // change to read pressure
                    BarState = BAR_STE_RD_PRESS;
                }
            #else
                // timeout is over
                if(SysVenceuTimeout(BarTout))
                {
                    // change to read pressure
                    BarState = BAR_STE_RD_PRESS;
                }
            #endif

        break;

        /**********************************************************************/
        /*                                                                    */
        case BAR_STE_RD_PRESS:

            // read 2 byte conversion result
            //RawBar = Bmp085ReadByte(0xF6); // H
            //RawBar <<= 8;
            //RawBar |= Bmp085ReadByte(0xF7); // L
            BarRawBar = (long)Bmp085ReadShort(0xF6) & 0x0000FFFF;
            if(BarRawBar == 0x7FFF)
            {
                // say that values are not ok
                BarBaro = -1;
                BarTemp = -1;

                // abort
                BarState = BAR_STE_IDLE;
                break;
            }

            // read xlsb
            Xlsb = (byte)Bmp085ReadByte(0xF8);
            if(Xlsb == 0xFF)
            {
                // say that values are not ok
                BarBaro = -1;
                BarTemp = -1;

                // abort
                BarState = BAR_STE_IDLE;
                break;
            }

            // compute!
            BarState = BAR_STE_COMPUTE;
        break;

        /**********************************************************************/
        /*                                                                    */
        case BAR_STE_COMPUTE:
        {
            auto long X1, X2, X3;
            auto long B3, B4, B5, B6, B7;
            auto dword dwAC4, Rb;

            #if BAR_VERBOSE
                UARTprintf("BarReadBlock()\r\n{\r\n");
                UARTprintf("  RawTmp: %05d\r\n  RawBar: %05d + %d\r\n", BarRawTmp, BarRawBar, Xlsb);
            #endif

            // calculates full pressure
            BarRawBar = ((BarRawBar << 8) | (long)Xlsb) >> (8 - BAR_OVERSAMPLING);

            //
            // Extended temperature calculations
            //

                // do temperature calculations
                X1 = (((long)BarRawTmp - (dword)BarEep[AC6]) * (dword)BarEep[AC5]) >> 15;
                X2 = ((long)BarEep[MC] << 11) + (X1 + BarEep[MD])/2;     // round up (UP not DOWN!)
                X2 /= (X1 + BarEep[MD]);
                B5 = X1 + X2;

                BarTemp = (B5 + 8) >> 4;
                //Temp /= 10;

                #if BAR_VERBOSE
                    UARTprintf(" >Temp\r\n");
                    UARTprintf("  X1: %5d\r\n", X1);
                    UARTprintf("  X2: %5d\r\n", X2);
                    UARTprintf("  B5: %5d\r\n", B5);
                    UARTprintf("  T : %5d\r\n", BarTemp);
                #endif


            //
            // Extended pressure calculations
            //

                // get dword AC4
                dwAC4 = BarEep[AC4];
                dwAC4 &= 0x0000FFFF;
                Rb = (dword)BarRawBar;

                // do pressure calcs
                B6 = B5 - 4000;
                X1 = ((long)BarEep[B2] * ((B6 * B6)>>12 )) >> 11;
                X2 = ((long)BarEep[AC2] * B6) >> 11;
                X3 = X1 + X2;
                B3 = (dword)((((long)BarEep[AC1]*4 + X3) << BAR_OVERSAMPLING) + 2) / 4;

            #if BAR_VERBOSE
                UARTprintf(" >Baro\r\n");
                UARTprintf("  B6: %5d\r\n", B6);
                UARTprintf("  X1: %5d\r\n", X1);
                UARTprintf("  X2: %5d\r\n", X2);
                UARTprintf("  X3: %5d\r\n", X3);
                UARTprintf("  B3: %5d\r\n", B3);
            #endif


                // Scooby dooby doooooo
                X1 = ((long)BarEep[AC3] * B6) >> 13;
                X2 = ((long)BarEep[B1] * ((B6 * B6) >> 12)) >> 16;
                X3 = ((X1 + X2) + 2) >> 2;
                B4 = (dwAC4 * (unsigned long)(X3 + 32768)) >> 15;
                B7 = (Rb - (dword)B3) * (50000UL >> BAR_OVERSAMPLING );

            #if BAR_VERBOSE
                UARTprintf("  X1: %5d\r\n", X1);
                UARTprintf("  X2: %5d\r\n", X2);
                UARTprintf("  X3: %5d\r\n", X3);
                UARTprintf("  B4: %5d\r\n", B4);
                UARTprintf("  B7: %5d\r\n", B7);
            #endif

                // murc murc murc!
                //if (B7 < 0x80000000) { BarBaro = (B7 * 2) / B4; }
                //else                 { BarBaro = (B7 / B4) * 2; }
                if (B7 & 0xC0000000) { BarBaro = (B7 / B4) * 2; } // <  0
                else                 { BarBaro = (B7 * 2) / B4; } // >= 0

            #if BAR_VERBOSE
                UARTprintf("  B : %5d\r\n", BarBaro);
            #endif


                X1 = (BarBaro >> 8) * (BarBaro >> 8);
                X1 = (X1 * 3038) >> 16;
                X2 = (-7357 * BarBaro) >> 16;

                // ta-da!
                BarBaro = BarBaro + ((X1 + X2 + (long)3791)>>4);

            #if BAR_VERBOSE
                UARTprintf("  X1: %5d\r\n", X1);
                UARTprintf("  X2: %5d\r\n", X2);
                UARTprintf("  B : %5d\r\n", BarBaro);
                UARTprintf("}\r\n");
            #endif

            // back to idle
            BarState = BAR_STE_IDLE;

            break;
        }
    }
}

/******************************************************************************/
/*                                                                            */
/* Converte de altitude barometrica para metros                               */
/*                                                                            */
/******************************************************************************/
long BarAltitude(long Qnh)
{
    float Alt;

    // verifica se temos pressao
    if(BarBaro == -1)
    {
        return(0);
    }

    // relative pressure
    Alt = (float)BarBaro / (float)Qnh;

    // 1  - Rel^1/5.255
    Alt = 1.0F - pow(Alt, 0.19029495718363463368F);

    // 44330 *
    Alt = Alt * 44330.0F;

    return(ftol(Alt));
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte Bmp085ReadByte(byte Addr)
{
    /* wait bus busy */
    //while(I2CMasterBusBusy(BAR_I2C_BASE));

    /* Configure slave address (BMP085) for write */
    I2CMasterSlaveAddrSet(BAR_I2C_BASE , BARO_ADDRESS , false);

    /* Send register Addr */
    I2CMasterDataPut(BAR_I2C_BASE, Addr);
    I2CMasterControl(BAR_I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if(I2CMasterWait1K(BAR_I2C_BASE)) { return(0xFF); }

    /* Configure slave address (BMP085) for read */
    I2CMasterSlaveAddrSet(BAR_I2C_BASE , BARO_ADDRESS , true);

    /* Read 1 byte of data */
    I2CMasterControl(BAR_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    if(I2CMasterWait1K(BAR_I2C_BASE)) { return(0xFF); }
    return((byte)I2CMasterDataGet(BAR_I2C_BASE));
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
short Bmp085ReadShort(byte Addr)
{
    auto short Ret;

    /* wait bus busy */
    //while(I2CMasterBusBusy(BAR_I2C_BASE));

    /* Configure slave address (BMP085) for write */
    I2CMasterSlaveAddrSet(BAR_I2C_BASE , BARO_ADDRESS , false);

    /* Send register Addr */
    I2CMasterDataPut(BAR_I2C_BASE, Addr);
    I2CMasterControl(BAR_I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if(I2CMasterWait1K(BAR_I2C_BASE)) { return(0x7FFF); }

    /* Configure slave address (BMP085) for read */
    I2CMasterSlaveAddrSet(BAR_I2C_BASE , BARO_ADDRESS , true);

    /* Read H byte of data */
    I2CMasterControl(BAR_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if(I2CMasterWait1K(BAR_I2C_BASE)) { return(0x7FFF); }
    Ret = (short)I2CMasterDataGet(BAR_I2C_BASE);
    Ret <<= 8;

    /* Read L byte of data */
    I2CMasterControl(BAR_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if(I2CMasterWait1K(BAR_I2C_BASE)) { return(0x7FFF); }
    Ret |= (short)I2CMasterDataGet(BAR_I2C_BASE) & 0x00FF;

    return(Ret);
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void Bmp085WriteByte(byte Addr, byte Data)
{
    /* wait bus busy */
    //while(I2CMasterBusBusy(BAR_I2C_BASE));

    /* Configure slave address (BMP085) for write */
    I2CMasterSlaveAddrSet(BAR_I2C_BASE , BARO_ADDRESS , false);

    /* Send register Addr */
    I2CMasterDataPut(BAR_I2C_BASE, Addr);
    I2CMasterControl(BAR_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if(I2CMasterWait1K(BAR_I2C_BASE)) { return; }

    /* Write 1 byte of data */
    I2CMasterDataPut(BAR_I2C_BASE, Data);
    I2CMasterControl(BAR_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    I2CMasterWait1K(BAR_I2C_BASE);
}
