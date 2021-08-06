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

#include "etc/defines.h"
#include "utl/math.h"
//#include "utl/math_table.h"

/******************************************************************************/
/*                                                                            */
/*   Converte angulos cartesianos para angulos geodesicos e vice versa        */
/*                                                                            */
/*              -----..__                                    <-..             */
/*                       ``-.                                    ´,           */
/*               0°          \                         90°         \          */
/*              ___          _\|                       ___          |         */
/*          .-´´   ``-.                            .-´´   ``-.       [        */
/*         /           \                          /           \       |       */
/*        Y             Y                        Y             Y      |       */
/*   270° |             |  90°              180° |             | 0°   |       */
/*        l             !                        l             !              */
/*         \           /                          \           /               */
/*          "-.,___,.-"                            "-.,___,.-"                */
/*                                                                            */
/*              180°                                   270°                   */
/*                                                                            */
/*          Geodesicas                              Cartesianas               */
/*                                                                            */
/******************************************************************************/
int MthCart2Geo(int Angulo)
{
	/* Conversão é simples, inverte o sentido de rotação "Ang = -Ang" */
	Angulo = -Angulo;

	/* A defasagem eh de 90° a mais */
	Angulo = Angulo + 90;

	/* Se o resultado eh negativo, soma 360 */
	if(Angulo < 0)
	{
		Angulo = Angulo + 360;
	}

	/* Angulo geodesico */
	return(Angulo);
}


/*************************************************************************/
/*                                                                       */
/* Converte os valores de polar para cartesiano                          */
/*                                                                       */
/*************************************************************************/
void MthPolarDec2Cartesiano(int X0, int Y0, int *X1, int *Y1, int Angulo, word Raio)
{
	float Sen;
	float Cos;
	float Ang;

	/* Converte para polar */
	Ang = ToRad((float)Angulo);

	/* Obtem os senos */
	Sen = sin(Ang);
	Cos = cos(Ang);

	/* Multiplica ao Raio */
	Sen = Sen * (float)Raio;
	Cos = Cos * (float)Raio;

    /* Calcula o valor do segndo ponto a partir do primeiro */
    *X1 = X0 + ftoi(Cos);
    *Y1 = Y0 + ftoi(Sen);
}

/*************************************************************************/
/*                                                                       */
/* Converte os valores de polar para cartesiano                          */
/*                                                                       */
/*************************************************************************/
void MthScale(int X0, int Y0, int *X1, int *Y1, int Mult, int Div)
{
    // Mult factor
    X0 = X0 * Mult;
    Y0 = Y0 * Mult;

    // step
    Mult = (Div/2);

    // result
    *X1 = (X0 + Mult) / Div;
    *Y1 = (Y0 + Mult) / Div;
}

/*************************************************************************/
/*                                                                       */
/* Converte os valores de polar para cartesiano                          */
/*                                                                       */
/*************************************************************************/
void MthRotate(int X0, int Y0, int *X1, int *Y1, int Angulo)
{
	float Sen;
	float Cos;
	float Ang;
    float x, y;

	/* Converte para polar */
	Ang = ToRad((float)Angulo);

	/* Obtem os senos */
	Sen = sin(Ang);
	Cos = cos(Ang);

    /* X e Y */
    x = (float)X0;
    y = (float)Y0;

    /* Rotaciona */
    *X1 = ftoi((x * Cos) + (y * Sen));
    *Y1 = ftoi((y * Cos) - (x * Sen));
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void MthMatrixAdd(void *Mc, void *Ma, void *Mb, byte n)
{
    auto float *c;
    auto float *a;
    auto float *b;

    // points
    c = (float*)Mc;
    a = (float*)Ma;
    b = (float*)Mb;

    // total items
    n = n * n;

    // add items
    while(n)
    {
        // add
        *c = *a + *b;

        // next
        c++;
        a++;
        b++;
        n--;
    }
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void MthMatrixMul(void *Mc, void *Ma, void *Mb, byte n)
{
    auto float *a;
    auto float *b;
    auto float *c;
    auto byte   ai, aj, ci, nn;

    // tamanho total
    nn = n * n;

    // limpa a matriz c
    c = (float*)Mc;
    for(ci = nn; ci; ci--)
    {
        // limpa e vai pro proximo
        *c = 0.0F;
        c++;
    }

    // primeiro item de a
    a = (float*)Ma;

    // para cada linha de a
    for(aj = n; aj; aj--)
    {
        // primeiro item de b
        b = (float*)Mb;

        // para cada coluna de a
        for(ai = n; ai; ai--)
        {
            // primeira coluna de c
            c = (float*)Mc;

            // para cada coluna de c
            for(ci = n; ci; ci--)
            {
                // produto dos itens
                *c += *a * *b;

                // proxima coluna de b e c
                c++;
                b++;
            }

            // proximo item de a
            a++;
        }

        // proxima linha da matriz c
        Mc = c;
    }
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void MthVectorAdd(void *R, void *Va, void *Vb, byte n)
{
    auto float *r;
    auto float *a;
    auto float *b;

    // points
    r = (float*)R;
    a = (float*)Va;
    b = (float*)Vb;

    // all items
    while(n)
    {
        // add
        *r = *a + *b;

        // next
        r++;
        a++;
        b++;
        n--;
    }
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
float MthVectorDot(void *Va, void *Vb, byte n)
{
    auto float Ret;
    auto float *a;
    auto float *b;

    a = (float*)Va;
    b = (float*)Vb;

    // nothing first
    Ret = 0;

    // dot product
    while(n)
    {
        // dot product
        Ret += (*a) * (*b);

        // next
        a++;
        b++;
        n--;
    }

    // return value
    return(Ret);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void MthVectorCross(void *R, void *Va, void *Vb, byte n)
{
    auto float *r;
    auto float *a;
    auto float *b;

    // pointers
    r = (float*)R;
    a = (float*)Va;
    b = (float*)Vb;

    // operation
    r[0] = (a[1]*b[2]) - (a[2]*b[1]);
    r[1] = (a[2]*b[0]) - (a[0]*b[2]);
    r[2] = (a[0]*b[1]) - (a[1]*b[0]);

    // Parameter n for compatibility in future re-implementation
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void MthVectorScale(void *R, void *Va, float c, byte n)
{
    auto float *r;
    auto float *a;

    // points
    r = (float*)R;
    a = (float*)Va;

    // all items
    while(n)
    {
        // scale
        *r = (*a) * c;

        // next
        r++;
        a++;
        n--;
    }
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
int absi(int val)
{
    if(val < 0)
    {
        val = -val;
    }

    return(val);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
short abss(short val)
{
    if(val < 0)
    {
        val = -val;
    }

    return(val);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
long absl(long val)
{
    if(val < 0)
    {
        val = -val;
    }

    return(val);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
float absf(float val)
{
    if(val < 0)
    {
        val = -val;
    }

    return(val);
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
short lims(short val, short min, short max)
{
    if(val < min) { val = min; }
    if(val > max) { val = max; }
    return(val);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
int limi(int val, int min, int max)
{
    if(val < min) { val = min; }
    if(val > max) { val = max; }
    return(val);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
long liml(long val, long min, long max)
{
    if(val < min) { val = min; }
    if(val > max) { val = max; }
    return(val);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
float limf(float val, float min, float max)
{
    if(val < min) { val = min; }
    if(val > max) { val = max; }
    return(val);
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
int ftoi(float val)
{
    if(val < 0.0F)
    {
        // return nearest negative edge
        return((int)(val - 0.5F));
    }

    // return nearest positive edge
    return((int)(val + 0.5F));
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
short ftos(float val)
{
    if(val < 0.0F)
    {
        // return nearest negative edge
        return((short)(val - 0.5F));
    }

    // return nearest positive edge
    return((short)(val + 0.5F));
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
long ftol(float val)
{
    if(val < 0.0F)
    {
        // return nearest negative edge
        return((long)(val - 0.5F));
    }

    // return nearest positive edge
    return((long)(val + 0.5F));
}







