/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  drv/itg3200.h                                                             */
/*                                                                            */
/*  Modulo de leitura e processamento do sensor girometro                     */
/*                                                                            */
/*  2012-10-10 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#ifndef ITG3200_H
#define ITG3200_H

#include "conf.h"
#include "etc/defines.h"

#ifndef GYR_GAIN_X
    // Gyro Gain adapted from the LPR5xx!!
    #define GYR_GAIN_X                  (0.0695652F) // 0.0695652F    (datasheet)
    #define GYR_GAIN_Y                  (0.0695652F) // 0.213F * 0.4F (antigo)
    #define GYR_GAIN_Z                  (0.0695652F) //
#endif

// read temperature
#ifndef GYR_READ_TEMP
    #define GYR_READ_TEMP               FALSE
#endif

// Outputs the offset-free gyro output with axis correction
extern short Gyr[3];

// The raw sensor data
extern short GyrRaw[3];

// Calculated offset
extern short GyrOfst[3];

// Measured temperature
#ifndef GYR_READ_TEMP
    extern short GyrTmp;
#endif

// prototypes
byte GyrInit(void);
byte GyrRead(void);

#endif

