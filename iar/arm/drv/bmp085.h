/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  drv\bmp085.h                                                              */
/*                                                                            */
/*  Driver para leitura do sensor de pressao BMP085                           */
/*                                                                            */
/*  2012-11-29 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/


#ifndef BMP085_H
#define BMP085_H

#include "etc/defines.h"

#define BAR_ISA_QNH                     101325 // 1013.25 hPa
//#define BAR_ISA_QNH                     101900 // 1019.00 hPa (Patrocínio)

// Outtput pressure and temperature
extern long  BarBaro;
extern long  BarTemp;
extern long  BarRawTmp;
extern long  BarRawBar;
extern short BarEep[11];

// prototypes
byte BarInit(void);
byte BarReadBlock(void);
void BarRefresh(void);
long BarAltitude(long Qnh);

#endif
