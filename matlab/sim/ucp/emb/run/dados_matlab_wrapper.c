
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <string.h>
#include "defines.h"
#include "crc.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 12
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
 
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void dados_matlab_Outputs_wrapper(const real32_T *y,
			uint8_T *Pkt)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/*************************************************************************/
/*                                                                       */
/*  Instituto Tecnologico de Aeronautica                                 */
/*  Divisao de Engenharia Eletronica e Computacao                        */
/*                                                                       */
/*  dados_matlab_wrapper.c                                               */
/*  Converte variáveis Matlab/Simulink para o padrão C                   */
/*                                                                       */
/*  2016-05-10 Marcelo Henrique Santos                                   */
/*                                                                       */
/*************************************************************************/
static byte *p;
static byte *pLrc;
static byte  Lrc;

// first byte
p = Pkt;

// STX
*p = 'P';                                       p++;
*p = 'N';                                       p++;
*((short*)p) = 0x40 | 0x10 | 0x08 | 0x04;       p+=sizeof(short);

// Group 3 : p q r
    *p = '3';                                   p++;
    pLrc = p;

    // RollRate PitchRate YawRate
    *((float*)p) = y[3];                        p+=sizeof(float);
    *((float*)p) = y[4];                        p+=sizeof(float);
    *((float*)p) = y[5];                        p+=sizeof(float);
    *((float*)p) = -999;                        p+=sizeof(float);

    // Group CRC
    Lrc = CrcPegasus(pLrc, 0, p - pLrc);
    *p = Lrc;                                   p++;
        
// Group 4 : phi theta psi
    *p = '4';                                   p++;
    pLrc = p;

    // Roll Pitch Yaw
    *((float*)p) = y[6];                        p+=sizeof(float);
    *((float*)p) = y[7];                        p+=sizeof(float);
    *((float*)p) = y[8];                        p+=sizeof(float);
    *((float*)p) = -999;                        p+=sizeof(float);

    // Group CRC
    Lrc = CrcPegasus(pLrc, 0, p - pLrc);
    *p = Lrc;                                   p++;
    
// Group 5 : Vt alpha beta
    *p = '5';                                   p++;
    pLrc = p;

    // 	North East Down
    *((float*)p) = y[0];                        p+=sizeof(float);
    *((float*)p) = y[1];                        p+=sizeof(float);
    *((float*)p) = y[2];                        p+=sizeof(float);
    *((float*)p) = -999;                        p+=sizeof(float);

    // Group CRC
    Lrc = CrcPegasus(pLrc, 0, p - pLrc);
    *p = Lrc;                                   p++;
    
// Group 7 : NED
    *p = '7';                                   p++;
    pLrc = p;
    
    *((float*)p) =  y[9];                       p+=sizeof(float);
    *((float*)p) = y[10];                       p+=sizeof(float);
    *((float*)p) = y[11];                       p+=sizeof(float);
    *((float*)p) = -999;                        p+=sizeof(float);

    // Group CRC
    Lrc = CrcPegasus(pLrc, 0, p - pLrc);
    *p = Lrc;                                   p++;    
    
// ETX
*p = '!';
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


