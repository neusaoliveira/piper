/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  frmpeg.h                                                                  */
/*                                                                            */
/*  Modulo de interface do protocolo pegasus                                  */
/*                                                                            */
/*  2013-04-13 Mateus Inicial                                                 */
/*  2017-10-27 Marcelo Henrique                                               */
/*                                                                            */
/******************************************************************************/

#ifndef FRM_PEG_H
#define FRM_PEG_H

#include "etc/defines.h"

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/
typedef struct
{   
    float Acc[3];
    float Mag[3];

    float PthRate;
    float RolRate;
    float YawRate;
    
    float Pth;
    float Rol;
    float Yaw;
        
    // Position[Inertial Frame (NED)]
    float North;
    float East;
    float Alt;
    
    // Aerodynamics Variables
    float Alpha;
    float Beta;
    float VT;
}StPegasusNav;

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/
typedef struct
{
    float Ail;
    float Ele;
    float Rud;
    float Trtl;

    float TrimAil;
    float TrimEle;
    float TrimRud;
    float TrimTrtl;

}StPegasusAct;

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/

/* received pegasus data */
//extern StPegasusNav     RxPegasus;

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/
byte FrmPegInicia(void);
void FrmPegRefresh(void);
void FrmPegAtiva(void);
void FrmPegDesativa(void);

#endif
