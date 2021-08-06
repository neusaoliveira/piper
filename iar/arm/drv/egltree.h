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

#ifndef EAGLETREE_H
#define EAGLETREE_H

#include "etc/defines.h"

// general funcion
unsigned short EtrRead(byte Addr);

// device function
#define IasRead()       EtrRead(0x75)
#define AltRead()       EtrRead(0x76)

// unit conversion
unsigned short IasKnots(unsigned short hPa);
unsigned short AltFeet(unsigned short hPa);

#endif
