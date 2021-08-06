/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  etc\defines.h                                                             */
/*                                                                            */
/*  Definições gerais do projeto. Deve ser incluido por todos os demais       */
/*  arquivos .c ou .h                                                         */
/*                                                                            */
/*  2008-09-26 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#ifndef DEFINES_H
#define DEFINES_H


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Tipos primarios de dados */
typedef unsigned char    byte;  //  8
typedef unsigned short   sword; // 16
typedef unsigned int     word;  // 32
typedef unsigned long    dword; // 32

/* Handler */
typedef void*            Handler;

/* ponteiro para função simples */
typedef byte(*pFncByte)(void);

/* ponteiro para função simples */
typedef void(*pFncVoid)(void);

/* ponteiro para função simples */
typedef void(*pFncArgB)(byte);

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Retorno basico de função */
#define ERRO             0
#define SUCESSO          1

/* Retorno comparativo de funcao */
#define IGUAL            0
#define MENOR            1
#define MAIOR            2

/* Booleanas */
//#define TRUE             1
//#define FALSE            0

/* Boolean type */
typedef enum { FALSE = 0, TRUE } BOOL;

/* ponteiro invalido */
#ifndef NULL
	#define NULL         0
#endif

/* Função invalida */
#define FNC_NONE         NULL

/* Controle de Fluxo */
#define OUTPUT           0xFF
#define INPUT            0x00

/* Nivel logico dos pinos */
#define LOW	             0
#define HIGH             1

#endif
