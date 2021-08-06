/******************************************************************************/
/*                                                                            */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  Washout Filter Library                                                    */
/*  srv\wof.c                                                                 */
/*                                                                            */
/*                                                                            */
/*  Implementação digital: Washout Filter [High Pass Filter (HPF) - 1st Order]*/ 
/*  Método de Aproximação de Tustin (Transformação Bilinear)                  */
/*  (FRANKLIN; POWELL, 2013) 8.3 Projeto Utilizando Equivalentes Discretos    */
/*                                                                            */
/*  2017-10-25 Marcelo Henrique Santos                                        */
/*                                                                            */
/******************************************************************************/

#include "srv/wof.h"

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
// Helper variables during coefficient calcutions
static float a, b;

// Difference equation terms 
static float k, j;

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void WoFilterInit(StWoFilter *wof, int freqSample, float tau)
{
    /* Controles internos */
    wof->Input          = 0.0f;    
    wof->Output         = 0.0f;    
    wof->LastInput      = 0.0f;
    wof->LastOutput     = 0.0f;

    /* Discretizar Filtro */    
    a = 1.0;
    b = 2*tau*(float)freqSample;
    
    j = (b - a)/(b + a);
    k = (  b  )/(b + a);
    
    /* Washout Filter criado */
}

/******************************************************************************/
/*                                                                            */
/* Execute PID: single calls only                                             */
/*                                                                            */
/******************************************************************************/
void WoFilterExecute(StWoFilter *wof, float Input)
{    
    /* [Buffer] Ultima : Entrada & Saida */
    wof->LastInput      = wof->Input;
    wof->LastOutput     = wof->Output;
    
    wof->Input          = Input;
    wof->Output         =  j*wof->LastOutput + k*(wof->Input - wof->LastInput);
}
