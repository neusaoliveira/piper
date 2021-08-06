/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  drv\hmc58xx.h                                                             */
/*                                                                            */
/*  Driver para leitura do magnetometro integrado 'a imu                      */
/*                                                                            */
/*  2012-04-13 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/


#ifndef HMC58XX_H
#define HMC58XX_H

#include "etc/defines.h"

// Outtput Accel
extern short Mag[3];
extern short MagRaw[3];
extern short MagOfst[3];
extern short MagPos[3];
extern short MagNeg[3];

// prototypes
byte MagInit(void);
byte MagRead(void);
byte MagCal(void);

#endif
