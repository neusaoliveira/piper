
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

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 41
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */

/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void dados_uC_Outputs_wrapper(const uint8_T *Pkt,
			real_T *u,
			real_T *WP,
			uint8_T *stop)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */

/*************************************************************************/
/*                                                                       */
/*  Instituto Tecnologico de Aeronautica                                 */
/*  Divisao de Engenharia Eletronica e Computacao                        */
/*                                                                       */
/*  dados_xplane_wrapper.c                                               */
/*  Converte variáveis Matlab/Simulink para o padrão C                   */
/*                                                                       */
/*  2016-05-10 Marcelo Henrique Santos                                   */
/*                                                                       */
/*************************************************************************/
short i, group;
unsigned char *pData;
pData = Pkt;

// check first fields
if(!(strncmp(pData, "PA", 2)))
{
    pData += sizeof(short);
    
    // group packets
    group = ((short*)pData);                            pData += sizeof(short);

    // check all packets
    for(i = 0x01; i <= 0x80; i <<= 1)
    {
        /* check if this packet is present */
        if(group & i)
        {
            /* Id */
            switch(*pData)
            {
                /* Ctrl surfaces... */
                case '1':
                    // skip Id
                    pData++;
                    // copy
                    u[0] = (double)*((float*)pData);	pData += sizeof(float);
                    u[1] = (double)*((float*)pData);  	pData += sizeof(float);
                    u[2] = (double)*((float*)pData); 	pData += sizeof(float);
                    u[3] = (double)*((float*)pData); 	pData += sizeof(float);
                    // skip CRC
                    pData++;
                break;

                /* Guidance references... */
                case '6':
                    // skip Id
                    pData++;
                    // copy
                    WP [0] = (double)*((float*)pData);	pData += sizeof(float);
                    WP [1] = (double)*((float*)pData);	pData += sizeof(float);
                    WP [2] = (double)*((float*)pData);	pData += sizeof(float);
                                                        pData += sizeof(float);
                    // skip CRC
                    pData++;
                break; 
            }
        }
    }

    // EOF
    stop[0] = (*pData == '#') ? 1 : 0;        
}
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


