/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  etc\button.c                                                              */
/*                                                                            */
/*  Leitura de botoes switch                                                  */
/*                                                                            */
/*  2013-11-05 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#include "etc/defines.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "lib/sysctl.h"
#include "lib/timer.h"
#include "lib/gpio.h"
#include "drv/button.h"
#include "conf.h"

#ifdef BTN_USE_ISR
    #include "lib/interrupt.h"
#endif


#ifndef CNF_BUTTON
    //#error "button.c: Modulo nao ativo."
#endif

/******************************************************************************/
/*                                                                            */
/* Internal structures                                                        */
/*                                                                            */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/* Internal variables                                                         */
/*                                                                            */
/******************************************************************************/

/* botoes */
byte BtnSwitches     = 0x00;

/* Clocks */
byte BtnSwitchClockA = 0;
byte BtnSwitchClockB = 0;

/* lista de ponteiros para cada botao */
const pFncBtnEvent* pBtnFuncs;

/******************************************************************************/
/*                                                                            */
/* Internal Functions                                                         */
/*                                                                            */
/******************************************************************************/


/******************************************************************************/
/*                                                                            */
/* Startup function                                                           */
/*                                                                            */
/******************************************************************************/
byte BtnInicia(void)
{

    /* initialized successfully */
    return(SUCESSO);
}

/******************************************************************************/
/*                                                                            */
/* Retorna o estado dos botoes                                                */
/*                                                                            */
/******************************************************************************/
byte BtnState(void)
{
    return(BtnSwitches);
}

/******************************************************************************/
/*                                                                            */
/* Retorna o estado dos botoes                                                */
/*                                                                            */
/******************************************************************************/
void BtnClear(void)
{
    BtnSwitches = 0x00;
}

/******************************************************************************/
/*                                                                            */
/* Timer procedure for button pooling                                         */
/*                                                                            */
/******************************************************************************/
void BtnTick(dword ms)
{
    byte   ucData, ucDelta;
    static byte ucSwitch = 0x1F;

    //
    // Read the state of the push buttons.
    //
    ucData = (byte)(GPIOPinRead(GPIO_PORTE_BASE, (GPIO_PIN_0 | GPIO_PIN_1 |
                                                  GPIO_PIN_2 | GPIO_PIN_3)) |
             (byte)(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) << 3));

    #if 0 // 2965
    //
    // Read the state of the push buttons.
    //
    ucData = ((byte)(GPIOPinRead(GPIO_PORTF_BASE, (GPIO_PIN_4 | GPIO_PIN_5 |
                                                   GPIO_PIN_6 | GPIO_PIN_7)) >> 4) |
              (byte)GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_4));
    #endif

    //
    // Determine the switches that are at a different state than the debounced
    // state.
    //
    ucDelta = ucData ^ ucSwitch;

    //
    // Increment the clocks by one.
    //
    BtnSwitchClockA ^= BtnSwitchClockB;
    BtnSwitchClockB = ~BtnSwitchClockB;

    //
    // Reset the clocks corresponding to switches that have not changed state.
    //
    BtnSwitchClockA &= ucDelta;
    BtnSwitchClockB &= ucDelta;

    //
    // Get the new debounced switch state.
    //
    ucSwitch &= BtnSwitchClockA | BtnSwitchClockB;
    ucSwitch |= (~(BtnSwitchClockA | BtnSwitchClockB)) & ucData;

    //
    // Determine the switches that just changed debounced state.
    //
    ucDelta ^= (BtnSwitchClockA | BtnSwitchClockB);

    // Tell which buttons were pressed and are now released
    // were =1, and changed and =0
    // 1=sw press, 0=sw no change
    BtnSwitches = ucDelta & ucSwitch;

    //
    // See if the select button was just pressed.
    //
    //if((ucDelta & BTN_SL) && !(ucSwitch & BTN_SL))
}

void BtnConfig(byte Id, long Periph, long Port, long Pin)
{
}

void BtnAssign(const pFncBtnEvent* pFuncs)
{
}
