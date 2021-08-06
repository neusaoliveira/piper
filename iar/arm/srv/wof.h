/******************************************************************************/
/*                                                                            */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  Washout Filter Library - Header                                           */
/*  srv\wof.h                                                                 */
/*                                                                            */
/*                                                                            */
/*  Implementação digital: Washout Filter [High Pass Filter (HPF) - 1st Order]*/ 
/*  Método de Aproximação de Tustin (Transformação Bilinear)                  */
/*  (FRANKLIN; POWELL, 2013) 8.3 Projeto Utilizando Equivalentes Discretos    */
/*                                                                            */
/*  2017-10-25 Marcelo Henrique Santos                                        */
/*                                                                            */
/******************************************************************************/

#ifndef WOF_H
#define WOF_H

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
/* Estrutura do PID com ponto flutuante */
typedef struct
{    
    /* Dados internos */
    float Input;
    float LastInput;
    float Output;
    float LastOutput;
	
}StWoFilter;


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void WoFilterInit(StWoFilter *wof, int freqSample, float tau);
void WoFilterExecute(StWoFilter *pPid, float Input);

#endif

