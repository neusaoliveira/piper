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

#include "etc/defines.h"
#include "utl/utils.h"
#include <string.h>
#include <stdio.h>


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
dword CnvStrFloat2Decimal(char *Texto, byte NumCasas, char Separador)
{
    auto byte Minus;
    auto dword Mnt;
    auto dword Dec;

    // check minus
    if(*Texto == '-') { Minus = TRUE; Texto++; } else { Minus = FALSE; }

    // get mantissa
    Mnt = 0;
    while(*Texto && *Texto != Separador)
    {
        // convert
        Mnt = Mnt * 10;
        Mnt = Mnt + *Texto - '0';

        Texto++;
    }

    // skip separator
    Texto++;

    // get decimal
    Dec = 0;
    while(*Texto)
    {
        // convert
        Dec = Dec * 10;
        Dec = Dec + *Texto - '0';

        Texto++;
    }

    // result
    Mnt = Mnt * NumCasas;
    Mnt = Mnt + Dec;

    if(Minus)
    {
        Mnt = -Mnt;
    }

    return(Mnt);
}

/*************************************************************************/
/*                                                                       */
/* Converte uma string hexadecimal em um numero decimal                  */
/*                                                                       */
/*************************************************************************/
dword CnvStrHex2Decimal(byte *Hex, byte Chars)
{
    dword Data;

    Data = 0;

    /* Se nao for passado o numero de parametros, ir ate o final da string */
    if(!Chars)
    {
        /* Colocamos no maximo possivel */
        Chars = 0xFF;
    }

    while((Chars) && (*Hex != '\0'))
    {
        Data = Data * 16;

        if((*Hex - '0') < 10)
        {
            Data = Data + (*Hex - '0');
        }
        else
        {
            if((*Hex - 'A') < 6)
            {
                Data = Data + (*Hex - 'A') + 10;
            }
            else
            {
                return(0xFFFFFFFFL);
            }
        }

        /* Proximo caracter */
        Hex++;
        Chars--;
    }

    return(Data);
}

/*************************************************************************/
/*                                                                       */
/* Converte uma string em um numero decimal                              */
/*                                                                       */
/*************************************************************************/
dword CnvStr2Decimal(char *Texto, byte Inicio, byte Chars)
{
    dword Data;

    Data = 0L;

    /* Vai ate a posição de inicio */
    while(Inicio)
    {
        Texto++;
        Inicio--;
    }

    /* Se nao for passado o numero de parametros, ir ate o final da string */
    if(!Chars)
    {
        /* Colocamos no maximo possivel */
        Chars = 0xFF;
    }

    while((Chars) && (*Texto != '\0') && (*Texto != '.') && (*Texto != ','))
    {
        Data = Data * 10L;

        if((*Texto - '0') < 10)
        {
            Data = Data + (dword)(*Texto - '0');
        }
        else
        {
            return(0xFFFFFFFFL);
        }

        /* Proximo caracter */
        Texto++;
        Chars--;
    }

    return(Data);
}


/*************************************************************************/
/*                                                                       */
/* Converte a string do GPS para Latitude                                */
/*                                                                       */
/*************************************************************************/
long CnvStr2Latitude(char *Texto, char *Hem)
{
    /* ddmm.mmmm */
    long Angle;

    /* Converte os graus em minutos */
    Angle = 60L * CnvStr2Decimal(Texto, 0, 2);

    /* Primeira parte dos minutos */
    Angle = Angle + CnvStr2Decimal(Texto, 2, 2);

    /* Segunda parte do sminutos */
    Angle = (10000L * Angle) + CnvStr2Decimal(Texto, 5, 4);

    /* verifica o hemisferio */
    switch(*Hem)
    {
        case 'n':
        case 'N':
        {
            /* deu na mesma */
            break;
        }

        case 's':
        case 'S':
        {
            /* inverte o angulo */
            Angle = -Angle;
            break;
        }
        default:
        {
            return(0x7FFFFFFFL);
        }
    }

    return(Angle);
}

/*************************************************************************/
/*                                                                       */
/* Converte a string do GPS para longitude                               */
/*                                                                       */
/*************************************************************************/
long CnvStr2Longitude(char *Texto, char *Hem)
{
    /* dddmm.mmmm */
    long Angle;

    /* Converte os graus em minutos */
    Angle = 60L * CnvStr2Decimal(Texto, 0, 3);

    /* Primeira parte dos minutos */
    Angle = Angle + CnvStr2Decimal(Texto, 3, 2);

    /* Segunda parte do sminutos */
    Angle = (10000L * Angle) + CnvStr2Decimal(Texto, 6, 4);

    /* verifica o hemisferio */
    switch(*Hem)
    {
        case 'e':
        case 'E':
        {
            /* deu na mesma */
            break;
        }

        case 'w':
        case 'W':
        {
            /* inverte o angulo */
            Angle = -Angle;
            break;
        }
        default:
        {
            return(0x7FFFFFFFL);
        }
    }

    return(Angle);
}

/*************************************************************************/
/*                                                                       */
/* Converts MMMM.MMMM to DD.DDDDDD                                       */
/*                                                                       */
/*************************************************************************/
void GpsToDeg(long Coord, float *pOut)
{
    static float Ang;
    static float Dec;
    static float Deg;

    // get angle in float
    Ang = (float)Coord;

    // integer degrees
    Deg = Ang / 600000.0F;

    // decimal degrees
    Dec = (Ang - (Deg * 600000.0F)) / 600000.0F;

    // full coordinate
    Dec = Dec + Deg;

    // outputs bullshit
    *pOut = Dec;
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
char* StrSeek(char *Str, char Letra)
{
    while(*Str != Letra)
    {
        Str = Str + 1;

        if(*Str == '\0')
        {
            return(NULL);
        }
    }

    return(Str);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
char* StrSkip(char *Str, char Letra)
{
    while(*Str == Letra)
    {
        Str = Str + 1;

        if(*Str == '\0')
        {
            return(NULL);
        }
    }

    return(Str);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte IsNum(char c)
{
    if((c >= '0') && (c <= '9'))
    {
        return(TRUE);
    }

    return(FALSE);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte IsAplha(char c)
{
    // lower case
    c |= 0x20;
    if((c >= 'a') && (c <= 'z'))
    {
        return(TRUE);
    }

    return(FALSE);
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void Swap16(unsigned short* p)
{
    auto byte *Data;
    auto byte Tmp;

    /* Points to data */
    Data = (byte*)p;

    /* Swap */
    Tmp = Data[0];
    Data[0] = Data[1];
    Data[1] = Tmp;
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void Swap32w(dword* p)
{
    auto unsigned short *Data;
    auto unsigned short  Tmp;

    /* Points to data */
    Data = (unsigned short*)p;

    /* Swap */
    Tmp = Data[0];
    Data[0] = Data[1];
    Data[1] = Tmp;
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void Swap32b(dword* p)
{
    auto byte *Data;
    auto byte  Tmp;

    /* Points to data */
    Data = (byte*)p;

    /* Swap B0-B3 */
    Tmp = Data[0];
    Data[0] = Data[3];
    Data[3] = Tmp;

    /* Swap B1-B2 */
    Tmp = Data[1];
    Data[1] = Data[2];
    Data[2] = Tmp;
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte CnvFormata(char *Out, char *Str, char *Fmt)
{
    auto byte Cnt;

    Cnt = 0;
    while(*Fmt)
    {
        // check format string
        if((*Fmt >= '0') && (*Fmt <= '9'))
        {
            // there is number to copy?
            if(*Str)
            {
                // copy number
                *Out = *Str;
                Str++;
            }
        }
        else
        {
            // copy format
            *Out = *Fmt;
        }

        // next
        Out++;
        Fmt++;
        Cnt++;
    }

    // close output
    *Out = '\0';

    // return the string length
    return(Cnt);
}


/*************************************************************************/
/*                                                                       */
/* Formata uma string de apenas numeros com o formato indicado           */
/* Str:20070811 Fmt:9999/99/99, ''  Saida: 2007/08/11                    */
/* Str:070811   Fmt:9999/99/99, ' ' Saida:   07/08/11                    */
/* Str:118      Fmt:999.99    , ''  Saida: 1.18                          */
/* Str:118      Fmt:999.99    , '0' Saida: 001.18                        */
/* Str:118      Fmt:999.99    , '_' Saida: __1.18                        */
/*                                                                       */
/*************************************************************************/
byte CnvFormataBackward(char *Data, char *Format, byte Fill)
{
    auto char Texto[30];
    auto char *Aux;
    auto char *pIni;
    auto char *pOut;

    /* Nosso marcador */
    Texto[0] = 0x7F;

    /* Área livre */
    Aux = Texto + 1;

    /* verifica se é negativo */
    if(*Data == '-')
    {
        /* Ignora na conversao */
        Data++;

        /* Indica negativo */
        Texto[0] = 0x7E;
    }

    /* Salva o comeco do formato */
    pIni = Format;
    pOut = Data;

    /* aponta para o final das strings */
    while(*Format) Format++;
    while(*Data) Data++;
    Format--;
    Data--;

    /* Percorre ate chegar no inicio dos formato */
    while(Format >= pIni)
    {
        /* Vejamos o formato */
        if(*Format == '9')
        {
            /* Se ainda tem string de dados */
            if(Data < pOut)
            {
                /* Tem preenchimento extra? */
                if(Fill)
                {
                    *Aux = Fill;
                    Aux++;
                }
            }
            else
            {
                /* Copia do texto */
                *Aux = *Data;
                Aux++;

                /* Proximo caracter do dado */
                Data--;
            }
        }
        else
        {
            /* copia o que vier */
            *Aux = *Format;
            Aux++;
        }

        /* Proximo de saida e formato */
        Format--;
    }

    /* Termina a stirng */
    *Aux = '\0';

    /* Se sobraram dados, deu zica */
    if(Data >= pOut)
    {
        return(1);
    }

    /* Se o valor era negativo */
    if(Texto[0] == 0x7E)
    {
        pOut--;
        *pOut = '-';
        pOut++;

        /* volta o marcador */
        Texto[0] = 0x7F;
    }

    /* Copia a string para o comeco */
    Aux--;
    while(*Aux != 0x7F)
    {
        *pOut = *Aux;
        pOut++;
        Aux--;
    }
    *pOut = '\0';

    /* Se chegamos aqui deu certo */
    return(0);
}

/******************************************************************************/
/*                                                                            */
/* Formata a string de acordo com o tipo de dado repassado                    */
/*                                                                            */
/******************************************************************************/
byte CnvFormataTipo(char *Texto, void *Data, byte Tipo, word Tam)
{
    /* Tipos de dados para conversão */
    switch(Tipo)
    {
        case CNV_TIPO_BYTE:
        {
            return(sprintf(Texto, "%u", *((byte*)Data)));
        }

        case CNV_TIPO_WORD:
        {
            return(sprintf(Texto, "%u", *((word*)Data)));
        }

        case CNV_TIPO_SWORD:
        {
            return(sprintf(Texto, "%u", *((unsigned short*)Data)));
        }

        case CNV_TIPO_DWORD:
        {
            return(sprintf(Texto, "%lu", *((dword*)Data)));
        }

        case CNV_TIPO_CHAR:
        {
            return(sprintf(Texto, "%c", *((char*)Data)));
        }

        case CNV_TIPO_SHORT:
        {
            return(sprintf(Texto, "%d", *((short*)Data)));
        }

        case CNV_TIPO_INT:
        {
            return(sprintf(Texto, "%d", *((int*)Data)));
        }

        case CNV_TIPO_LONG:
        {
            return(sprintf(Texto, "%ld", *((long*)Data)));
        }

        case CNV_TIPO_STR:
        {
            /* Apenas copia */
            //strcpy(Texto, (char*)Data);
            return(sprintf(Texto, "%s", (char*)Data));
        }

        case CNV_TIPO_BIT_FLAG:
        {
            auto dword Valor;
            auto dword Mask;
            auto byte  Size;
            auto byte  Total;

            Size = ((Tam >> 6) & 0x03) + 1; // 1..4 chars
            Tam &= 0x3F; // 0..63 bits

            // quantos bits temos
            Mask = (1<<(Tam-1));

            // copia 32 de qualquer forma
            memcpy(&Valor, Data, sizeof(dword));

            // Tamanho total
            Total = Tam;

            // nesse caso Texto contem todos os caracteres
            while(Tam)
            {
                // se bit zerado
                if((Valor & Mask) == 0)
                {
                    // limpa string
                    memset(Texto, '-', Size);
                }

                Texto += Size;
                Mask >>= 1;
                Tam--;
            }

            // retorna o tamanho armazenado em 'Data'
            return(Total);
        }

        default:
        {
            /* Indica erro */
            memset(Texto, '*', Tam - 1);
            Texto[Tam - 1] = '\0';
            return(Tam);
        }
    }

    /* Apenas para garantia */
    //return(0);
}

/******************************************************************************/
/*                                                                            */
/* Formata a string de acordo com o tipo de dado repassado                    */
/*                                                                            */
/******************************************************************************/
byte CnvObtemDadoTipo(void *Data, char *Texto, byte Tipo)
{
    auto byte Num;

    /* Tipos de dados para conversão */
    switch(Tipo)
    {
        case CNV_TIPO_BYTE:
        {
            auto int Tmp;

            //sprintf(Texto, "%u", *((byte*)Data));
            Num = sscanf(Texto, "%d", &Tmp);
            *((byte*)Data) = (byte)Tmp;
            break;
        }

        case CNV_TIPO_SWORD:
        {
            //sprintf(Texto, "%u", *((word*)Data));
            Num = sscanf(Texto, "%hu", (unsigned short*)Data);
            break;
        }

        case CNV_TIPO_WORD:
        {
            //sprintf(Texto, "%u", *((word*)Data));
            Num = sscanf(Texto, "%lu", (word*)Data);
            break;
        }

        case CNV_TIPO_DWORD:
        {
            //sprintf(Texto, "%lu", *((dword*)Data));
            Num = sscanf(Texto, "%lu", (dword*)Data);
            break;
        }

        case CNV_TIPO_CHAR:
        {
            //sprintf(Texto, "%c", *((char*)Data));
            Num = sscanf(Texto, "%c", (char*)Data);
            break;
        }

        case CNV_TIPO_SHORT:
        {
            //sprintf(Texto, "%d", *((int*)Data));
            Num = sscanf(Texto, "%hd", (short*)Data);
            break;
        }

        case CNV_TIPO_INT:
        {
            //sprintf(Texto, "%d", *((int*)Data));
            Num = sscanf(Texto, "%d", (int*)Data);
            break;
        }

        case CNV_TIPO_LONG:
        {
            //sprintf(Texto, "%ld", *((long*)Data));
            Num = sscanf(Texto, "%ld", (long*)Data);
            break;
        }

        case CNV_TIPO_STR:
        {
            /* Apenas copia */
            Num = (byte)sprintf((char*)Data, "%s", Texto);

            /* Conta '\0' pois faz parte da string */
            Num++;
            break;
        }

        default:
        {
            /* Indica erro */
            return(0);
        }
    }

    /* retorna quantos bytes copiou */
    return(Num);
}

/******************************************************************************/
/*                                                                            */
/* Formata a string de acordo com o tipo de dado repassado                    */
/*                                                                            */
/******************************************************************************/
byte CnvTamanhoTipo(byte Tipo)
{
    /* Tipos de dados para conversão */
    switch(Tipo)
    {
        case CNV_TIPO_BYTE:  return(sizeof(byte));
        case CNV_TIPO_SWORD: return(sizeof(unsigned short));
        case CNV_TIPO_WORD:  return(sizeof(word));
        case CNV_TIPO_DWORD: return(sizeof(dword));
        case CNV_TIPO_CHAR:  return(sizeof(char));
        case CNV_TIPO_SHORT: return(sizeof(short));
        case CNV_TIPO_INT:   return(sizeof(int));
        case CNV_TIPO_LONG:  return(sizeof(long));
        case CNV_TIPO_BIT_FLAG: return(sizeof(dword));
    }

    /* Indica erro */
    return(0);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void ToUpper(char *pStr)
{
    while(*pStr)
    {
        if((*pStr >= 'a') && (*pStr <= 'z'))
        {
            *pStr &= ~0x20;
        }

        pStr++;
    }
}

/******************************************************************************/
/*                                                                            */
/* Formata a string de acordo com o tipo de dado repassado                    */
/*                                                                            */
/******************************************************************************/
void ToLower(char *pStr)
{
    while(*pStr)
    {
        if((*pStr >= 'A') && (*pStr <= 'Z'))
        {
            *pStr |= 0x20;
        }

        pStr++;
    }
}


