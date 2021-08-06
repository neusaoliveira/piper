/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  utl\math.c                                                                */
/*                                                                            */
/*  Fornece funcoes uteis para operacoes matematicas                          */
/*                                                                            */
/*  2009-04-26 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/


#ifndef MATH_H
#define MATH_H

#include "etc/defines.h"
#include <math.h>

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* valor de pi */
#define THE_PI                    3.14159265358979323846f

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Estrutura de um seno e cosseno */
typedef struct
{
	int Sen;
	int Cos;
}StMthSenCos;

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Macros */
#define ToRad(x) (x*0.01745329252F)  // *pi/180
#define ToDeg(x) (x*57.2957795131F)  // *180/pi

/* Matrix ops */
void MthMatrixAdd(void *Mc, void *Ma, void *Mb, byte n);
void MthMatrixMul(void *Mc, void *Ma, void *Mb, byte n);

/* Vector ops */
void MthVectorAdd(void *R, void *Va, void *Vb, byte n);
float MthVectorDot(void *Va, void *Vb, byte n);
void MthVectorCross(void *R, void *Va, void *Vb, byte n);
void MthVectorScale(void *R, void *Va, float c, byte n);



/* Basic */
short abss(short val);
int absi(int val);
long absl(long val);
float absf(float val);

/* type conversion */
short ftos(float val);
int ftoi(float val);
long ftol(float val);

/* limit */
short lims(short val, short min, short max);
int limi(int val, int min, int max);
long liml(long val, long min, long max);
float limf(float val, float min, float max);

/* Trigonometric */
//int MthSen(int Angulo);/
//int MthCos(int Angulo);
//int MthSenPrec(int Angulo);
int MthCart2Geo(int Angulo);
void MthPolarDec2Cartesiano(int X0, int Y0, int *X1, int *Y1, int Angulo, word Raio);
void MthScale(int X0, int Y0, int *X1, int *Y1, int Mult, int Div);
void MthRotate(int X0, int Y0, int *X1, int *Y1, int Angulo);

#endif

