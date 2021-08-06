/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  srv/crc.h                                                                 */
/*                                                                            */
/*  Modulo de calculo de CRC8                                                 */
/*                                                                            */
/*  2012-05-30 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#ifndef CRC_H
#define CRC_H

#include "defines.h"

// prototypes
byte Crc(byte *pDat, byte Stx, byte Etx, byte **pCrc);
byte CrcL(byte *pDat, byte Len, byte **pCrc);
char CrcPegasus(const unsigned char message[], const unsigned char ucInit, const unsigned char ucFinal);
void CrcFletcher(byte *pDat, byte Len, byte **pCrc, byte *pCrcA, byte *pCrcB);

#endif
