/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  utl\utils.c                                                               */
/*                                                                            */
/*  Fornece funcoes uteis para diversas implementacoes                        */
/*                                                                            */
/*  2009-04-26 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/


#ifndef UTILS_H
#define UTILS_H

#include "etc/defines.h"

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Tipos de dados para conversão */
#define CNV_TIPO_BYTE             0x00
#define CNV_TIPO_SWORD            0x01
#define CNV_TIPO_WORD             0x02
#define CNV_TIPO_DWORD            0x03
#define CNV_TIPO_CHAR             0x04
#define CNV_TIPO_SHORT            0x05
#define CNV_TIPO_INT              0x06
#define CNV_TIPO_LONG             0x07
#define CNV_TIPO_STR              0x08
#define CNV_TIPO_BIT_FLAG         0x09

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Decimal, float and Hexadecimal functions */
dword CnvStrHex2Decimal(byte *Hex, byte Chars);
dword CnvStr2Decimal(char *Texto, byte Inicio, byte Chars);
dword CnvStrFloat2Decimal(char *Texto, byte NumCasas, char Separador);
#define CnvStrHex2Byte(a) (byte)CnvStrHex2Decimal(a,2)
#define CnvStrHex2Word(a) (word)CnvStrHex2Decimal(a,4)
#define CnvStrHex2Dword(a) (dword)CnvStrHex2Decimal(a,8)
#define CnvStrHex2Stream(a) (dword)CnvStrHex2Decimal(a,0xFF)

/* String */
char* StrSeek(char *Str, char Letra); // search for
char* StrSkip(char *Str, char Letra); // skip while

/* Char */
byte IsNum(char c);
byte IsAplha(char c);

/* Geodetic coordinates functions */
long CnvStr2Longitude(char *Texto, char *Hem);
long CnvStr2Latitude(char *Texto, char *Hem);
void GpsToDeg(long Coord, float *pOut);

/* Endian */
void Swap16(unsigned short* p);
void Swap32w(dword* p);
void Swap32b(dword* p);

/* Formata uma string numerica nos parametros */
byte CnvFormata(char *Out, char *Str, char *Fmt);
byte CnvFormataBackward(char *Data, char *Format, byte Fill);
byte CnvFormataTipo(char *Texto, void *Data, byte Tipo, word Tam);
byte CnvObtemDadoTipo(void *Data, char *Texto, byte Tipo);
byte CnvTamanhoTipo(byte Tipo);

/* caracteres */
void ToUpper(char *pStr);
void ToLower(char *pStr);

#endif

