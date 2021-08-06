/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  srv\display.h                                                             */
/*                                                                            */
/*  Implementa funcoes de desenho e escrita no display RIT                    */
/*                                                                            */
/*  2012-11-16 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#ifndef RITDISPLAY_H
#define RITDISPLAY_H

#include "etc/defines.h"
#include "inc/hw_types.h"
#include "drv/rit128x96x4.h"

// Macro para calcular a linha do display
#define DspPy(l)     ((l)*8)
#define DspPx(c)     ((c)*6)

//*****************************************************************************
//
// Storage for a local frame buffer.
//
//*****************************************************************************
extern unsigned char DspFrame[6144];

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/* linha */
void DspLine(word x0, word y0, word x1, word y1, byte Cor);

/* Display static image */
void
Display(const unsigned char *pucLogo, unsigned long Px, unsigned long Py,
        unsigned long Tx, unsigned long Ty, tBoolean Inv);

/* Fade in and fade out image with 'delay' display time */
void
DisplayLogo(const unsigned char *pucLogo, unsigned long ulWidth,
            unsigned long ulHeight, unsigned long ulDelay);


#endif
