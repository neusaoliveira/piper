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
/*  PID segue a formulação da Aproximação de Tustin (Transformação Bilinear)  */                                              
/*                                                                            */
/*  2009-11-26 Mateus Inicial                                                 */
/*  2016-10-28 Marcelo Henrique Santos                                        */
/*                                                                            */
/******************************************************************************/

#include "srv/pid.h"

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
unsigned PidfClear(StPidf *pPid)
{
    /* Clear Gains*/
    pPid->Kp           = 0.0f;
    pPid->Ki           = 0.0f;
    pPid->Kd           = 0.0f;
  
    /* Zera as parciais */
    pPid->Integrativo  = 0.0;

    /* Controles internos */
    pPid->Error        = 0;
    pPid->LastErr      = 0;
    pPid->SetPoint     = 0;

    /* Pid criado */
    return(1);
}

/******************************************************************************/
/*                                                                            */
/* Execute PID: single calls only                                             */
/*                                                                            */
/******************************************************************************/
float PidfExecute(StPidf *pPid, float Input, float SampleTime)
{
    //static int Input;
    auto float Output;

    /* salva o ultimo erro */
    pPid->LastErr = pPid->Error;

    /* calcula o erro */
    pPid->Error = pPid->SetPoint - Input;

    /* No output yet */
    Output = 0.0;

    /* Se tem proporcional */
    if(pPid->Kp)
    {
        /* Joga na saida com o fator de ganho */
        Output = (pPid->Error * pPid->Kp);
    }

    /* Se tem integrativo */
    if(pPid->Ki)
    {
        /* calcula o integrativo */
        pPid->Integrativo = pPid->Integrativo + pPid->Ki*(pPid->Error + pPid->LastErr)*0.5*SampleTime;
       
        /* Soma o integrativo a saída*/
        Output = Output + pPid->Integrativo;
    }

    /* se tem derivativo */
    if(pPid->Kd)
    {
        auto float Derivativo;

        /* calcula o derivativo delta_heading / delta_t (t esta em ms) */
        Derivativo = pPid->Error - pPid->LastErr;

        /* coloca o derivativo */
        Output = Output + ((Derivativo * pPid->Kd));
    }    
    return(Output);
}






