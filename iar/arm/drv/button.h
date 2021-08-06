/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  etc\button.h                                                              */
/*                                                                            */
/*  Leitura de botoes switch                                                  */
/*                                                                            */
/*  2013-11-05 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#ifndef BTN_H
#define BTN_H

#include "conf.h"
#include "etc/defines.h"

/******************************************************************************/
/*                                                                            */
/* External constants                                                         */
/*                                                                            */
/******************************************************************************/

/* prototipo de uma função que processa os botoes */
typedef void (*pFncBtnEvent)(byte,byte);

/* máximo de botoes definidos */
#ifndef BTN_MAX_BOTOES
    //#error "button.c: BTN_MAX_BOTOES deve ser definida!"
    //#error "button.c: incluindo #define BTN_XXX de cada botao"
#endif

/******************************************************************************/
/*                                                                            */
/* prototypes                                                                 */
/*                                                                            */
/******************************************************************************/

byte BtnInicia(void);
void BtnTick(dword ms);
byte BtnState(void);
void BtnClear(void);
void BtnConfig(byte Id, long Periph, long Port, long Pin);
void BtnAssign(const pFncBtnEvent* pFuncs);

#endif

