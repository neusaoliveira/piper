/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  screensaver.c                                                             */
/*                                                                            */
/*  Modulo de interface para o screen saver                                   */
/*                                                                            */
/*  2013-04-13 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#include "etc/defines.h"

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/

extern unsigned long  ScrTime;

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/

tBoolean ScrIsSaving(void);
void ScrHit(void);
void ScrUpdate(pFncVoid FncAtiva, pFncVoid FncDesativa);
byte ScrRefresh(void);
