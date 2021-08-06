/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  drv\adxl345.h                                                             */
/*                                                                            */
/*  Driver para leitura do accelerometro integrado 'a imu                     */
/*                                                                            */
/*  2012-04-10 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/


#ifndef BMA180_H
#define BMA180_H

#include "conf.h"
#include "etc/defines.h"

#ifndef ACC_GAIN_X
    /* Measured gain to 1g=256 */
    #define ACC_GAIN_X                  0.967817292F
    #define ACC_GAIN_Y                  0.98102442F
    #define ACC_GAIN_Z                  1.01058715F
#endif

// gravity in m/s2
#define ACC_GRAVITY                     9.81F

// Outtput Accel
extern short Acc[3];
extern short AccRaw[3];
extern short AccOfst[3];

// prototypes
byte AccInit(void);
byte AccRead(void);

#endif
