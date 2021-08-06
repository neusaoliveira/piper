/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  srv\pid.h                                                                 */
/*                                                                            */
/*  Modulo que implementa um sistema de controle em malha fechada atraves do  */
/*  mecanismo PID - Proporcional - Integrativo - Derivativo                   */
/*                                                                            */
/*  Alterações: Rotina para a implementação digital do algoritmo de controle  */
/*  P_I_D segue a formulação da Aproximação de Tustin (Transformação Bilinear)*/
/*  (FRANKLIN; POWELL, 2013) 4.4 Introdução ao Controle Digital               */
/*                                                                            */
/*  2009-11-26 Mateus Inicial                                                 */
/*  2016-10-28 Marcelo Henrique Santos                                        */
/*                                                                            */
/******************************************************************************/

#ifndef PID_H
#define PID_H

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Estrutura do PID */
typedef struct
{   
    /* Fatores */
	int Kp;
	int Ki;
	int Kd;
    int Dv;

    /* Parciais */
	int Integrativo;

    /* Dados internos */
	int Error;
	int LastErr;
	int SetPoint;
        
}StPid;

/* Estrutura do PID com ponto flutuante */
typedef struct
{
    /* Fatores */
	float Kp;
	float Ki;
	float Kd;

    /* Parciais */
	float Integrativo;

    /* Dados internos */
	float Error;
	float LastErr;
	float SetPoint;
	
}StPidf;


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

unsigned PidfClear(StPidf *pPid);
float PidfExecute(StPidf *pPid, float Input, float SampleTime);

#endif

