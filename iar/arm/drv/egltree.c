/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  drv/etree.c                                                               */
/*                                                                            */
/*  Modulo de leitura e processamento do Eagle Tree Airspeed V3               */
/*                                                                            */
/*  2012-12-11 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#include "etc/defines.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "lib/i2c.h"
#include "drv/egltree.h"
#include "utl/math.h"

#ifndef ETR_I2C_BASE
    #define ETR_I2C_BASE                 I2C0_MASTER_BASE
#endif

// http://www.rcgroups.com/forums/showthread.php?t=1118096

/*

I am pretty sure that the values are differential pressures measured in Pa (Pascal).
A speed of 530 km/h corresponds to around 13800 Pa, than you can calculate the
incompressible speed by using Bernoulli's equation:

v = sqrt(2*pdiff/rho),

where pdiff is the reading (I assume that the initial value of 1570 in your case is an
offset and needs to be subtracted) and rho the air density (1.225 kg/m^3 for standard
atmosphere at ground level). The result is m/s, to get km/h just multiply by 3.6

In this example it will give 508 km/h, which is a little bit different from the
530 above, but they may have some different assumptions about the air density.
To implemented compressibility correction, the equation looks like

v = sqrt(2*pdiff/(rho*(1+0.25*Ma^2)))

where Ma is the Mach number Ma = v/a (a speed of sound).

Of cause, this requires an iterative solution.

KT = m/s * 1.94384
ofset = 1585

*/

/******************************************************************************/
/** Faz a leitura do Eagle Tree e retorna o valor do tipo sensor
 * Caso de erro, o valor 0xFFFF (65535) e' retornado.
 * \return Retorna o valor lido do sensor
 */
/******************************************************************************/
unsigned short EtrRead(byte Addr)
{
    auto unsigned short  Ias;
    auto byte           *p;

    /* Points to data */
    p = (byte*)&Ias;

    /* wait bus busy */
    //while(I2CMasterBusBusy(ETR_I2C_BASE));

    /* Configure slave address (ETREE_IASV3) for write */
    I2CMasterSlaveAddrSet(ETR_I2C_BASE , Addr , false);

    /* Request airspeed */
    I2CMasterDataPut(ETR_I2C_BASE, 0x07);
    I2CMasterControl(ETR_I2C_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    if(I2CMasterWait1K(ETR_I2C_BASE)) { return(0xFFFF); }

    /* Configure slave address (ETREE_IASV3) for read */
    I2CMasterSlaveAddrSet(ETR_I2C_BASE , Addr , true);

    /* Read airspeed integer */
    I2CMasterControl(ETR_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    if(I2CMasterWait1K(ETR_I2C_BASE)) { return(0xFFFF); }
    *p = (byte)I2CMasterDataGet(ETR_I2C_BASE);
    p++;

    /* Read airspeed integer */
    I2CMasterControl(ETR_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    if(I2CMasterWait1K(ETR_I2C_BASE)) { return(0xFFFF); }
    *p = (byte)I2CMasterDataGet(ETR_I2C_BASE);
    p++;

    /* check endian */
    #if IAS_BIG_ENDIAN
        Swap16(&Ias);
    #endif

	// return airspeed
	return(Ias);
}

/******************************************************************************/
/** Faz a conversao do valor raw de leitura para velocidade indicada (kias)
 * Caso de erro, o valor 0xFFFF (65535) e' retornado.
 * \return Retorna o valor lido do sensor
 */
/******************************************************************************/
unsigned short IasKnots(unsigned short hPa)
{
    auto float Spd;

    // se valor invalido, retorna invalido tambem
    if(hPa == 0xFFFF)
    {
        return(0xFFFF);
    }

    // se valor abaixo do minimo, nem processa e retorna 0
    if(hPa < 1600)
    {
        return(0);
    }

    // obtem o valor de pressao excluindo offset
    Spd = (float)(hPa - 1585);

    // v = sqrt(2 * dp / rho)
    //Spd = 2 * Spd / 1.225F;
    Spd = Spd * 1.63265306122449F;

    // raiz
    Spd = sqrt(Spd);

    // change from m/s to knots
    Spd = Spd * 1.94384; // KNOTS
    //Spd = Spd * 2.23694; // MPH

    // verifica se nao estourou
    if(Spd > 65535.0F)
    {
        return(0xFFFF);
    }

    // retorna o valor em knots (unsigned short)
    return(ftos(Spd));
}

/******************************************************************************/
/** Faz a conversao do valor raw de leitura para altitude em pes msl
 * Caso de erro, o valor 0xFFFF (65535) e' retornado.
 * \return Retorna o valor lido do sensor
 */
/******************************************************************************/
unsigned short AltFeet(unsigned short hPa)
{
    // se valor invalido, retorna invalido tambem
    if(hPa == 0xFFFF)
    {
        return(0xFFFF);
    }

    // retorna qq coisa por enquanto
    return(hPa);
}
