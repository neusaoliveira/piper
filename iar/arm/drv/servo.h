/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  etc\sistema.c                                                             */
/*                                                                            */
/*  Controle do sistema                                                       */
/*                                                                            */
/*  2008-09-26 Mateus Inicial                                                 */
/*  2010-01-28 Mateus Corrigido CMPA de 1000 para 999. Adicionado int CMPB    */
/*  2013-08-31 Mateus Portado para IAR ARM                                    */
/*                                                                            */
/******************************************************************************/

#ifndef SERVO_H
#define SERVO_H

#include "conf.h"
#include "etc/defines.h"

/******************************************************************************/
/*                                                                            */
/* External constants                                                         */
/*                                                                            */
/******************************************************************************/

/* max number of connected servos */
#ifndef SVO_MAX_SERVOS
    #define SVO_MAX_SERVOS              4
#endif

/* Servo values */
#define SERVO_MIN                       1000
#define SERVO_MID                       1500
#define SERVO_MAX                       2000

/* default time value */
#ifndef SVO_DEFAULT_TIME
    #define SVO_DEFAULT_TIME            SERVO_MID
#endif

/******************************************************************************/
/*                                                                            */
/* prototypes                                                                 */
/*                                                                            */
/******************************************************************************/

byte SvoInicia(void);
long SvoDeg2Pwm(float Ang);
void SvoConfig(byte Id, long Pin);
void SvoPosition(byte Id, long Pwm);
void SvoDisable(byte Id);

#endif

