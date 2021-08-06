/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  app/dcm.h                                                                 */
/*                                                                            */
/*  Modulo de operacoes para a matriz de cossenos diretores                   */
/*                                                                            */
/*  2012-09-04 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#ifndef DCM_H
#define DCM_H

typedef struct
{
    /* Float values */
    float PthF;
    float RolF;
    float YawF;

    /* integer values */
    int   PthI;
    int   RolI;
    int   YawI;
}StDcmEuler;

// Variavel externa contendo os angulos de euler
extern StDcmEuler DcmEuler;

// controle de tempo
extern unsigned long DcmTimer;

// debug
extern float DcmGyrRads[3];
extern float DcmAccMpss[3];

// prototypes
byte DcmInit(void);
byte DcmRefresh(void);
void DcmAlignment(byte Samples);
void DcmEval(dword Tmr);
void DcmOrtho(void);
void DcmError(void);
void DcmGetEuler(void);
void DcmGetHeading(void);
byte DcmGyroCheck(void);
void DcmPrintMatrix(void);

#endif
