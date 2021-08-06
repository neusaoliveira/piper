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

#include "etc/defines.h"
#include "etc/system.h"

// internal timer counter
unsigned long SysTimer = 0L;

/******************************************************************************/
/*                                                                            */
/* SysTick interrupt handler                                                  */
/*                                                                            */
/******************************************************************************/
void SysTick(long ms)
{
    //
    // Increment the tick count.
    //
    SysTimer += ms;
}

/******************************************************************************/
/*                                                                            */
/* Return up time in ms                                                       */
/*                                                                            */
/******************************************************************************/
dword SysNow(void)
{
	return(SysTimer);
}

/******************************************************************************/
/*                                                                            */
/* Sets a timeout value in ms                                                 */
/*                                                                            */
/******************************************************************************/
dword SysSetaTimeout(dword Timeout)
{
	return(SysTimer + Timeout);
}

/******************************************************************************/
/*                                                                            */
/* Check if a timeout has occurred                                            */
/*                                                                            */
/******************************************************************************/
byte SysVenceuTimeout(dword Timeout)
{
	if(Timeout <= SysTimer)
	{
		return(TRUE);
	}
	else
	{
		return(FALSE);
	}
}

/******************************************************************************/
/*                                                                            */
/* Halt system for value in ms                                                */
/*                                                                            */
/******************************************************************************/
void Halt(dword Timer)
{
    Timer = SysSetaTimeout(Timer);
    while(!SysVenceuTimeout(Timer));
}
