/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  etc/system.c                                                              */
/*                                                                            */
/*  Modulo de funcoes basicas do sistema                                      */
/*                                                                            */
/*  2013-07-01 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#ifndef SYSTEM_H
#define SYSTEM_H

#include "etc/defines.h"

// timeout no infinito!
#define TOUT_NUNCA                      0x7FFFFFFF

// prototypes
void  SysTick(long ms);
dword SysNow(void);
dword SysSetaTimeout(dword Timeout);
byte  SysVenceuTimeout(dword Timeout);
void  Halt(dword Timer);


#endif
