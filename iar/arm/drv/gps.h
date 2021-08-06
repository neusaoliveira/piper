/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  drv\gps.h                                                                 */
/*                                                                            */
/*  Driver para receber os dados do GPS pelo protocolo NMEA                   */
/*                                                                            */
/*  2008-10-22 Mateus Inicial                                                 */
/*  2012-10-17 Mateus Adaptaro IAR ARM                                        */
/*                                                                            */
/******************************************************************************/

#ifndef GPS_H
#define GPS_H

#include "etc/defines.h"
#include "conf.h"

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* tipo de função a ser chamada para evento */
typedef void(*pGpsFnc)(byte);

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

#ifndef GPS_SATELLITE_MONITOR
	#define GPS_SATELLITE_MONITOR       FALSE
#endif

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Constantes de posição */
#define GPS_POS_NORTH_NONE              0x0L
#define GPS_POS_EAST_NONE               0x0L
#define GPS_POS_ALT_NONE                0x0L

/* Maximo de canais de satelite */
#define GPS_MAX_SATELITES               12

/* Indica satelite invalido no channel */
#define GPS_SAT_NONE                    0xFF

/* Estados do GPS */
#define GPS_INICIALIZA                  0x00
#define GPS_AUSENTE                     0x01
#define GPS_INVALIDO                    0x02
#define GPS_OPERANDO                    0x03

/* Eventos */
#define GPS_EVT_NOVA_POS                0x01
#define GPS_EVT_NOVA_MENSAGEM           0x02
#define GPS_EVT_MUDOU_STATUS            0x04
#define GPS_EVT_SATELITES               0x08

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

#if GPS_SATELLITE_MONITOR
typedef struct
{
	byte Id;
	byte Elev;
	byte Azim; /* Azimuth /2 */
	byte Pot;
}StGpsSat;
#endif

/* estrutura de dados do GPS */
typedef struct
{
	dword Utc;

	byte  Status;
	byte  Hdop;
	byte  SatUsed;
	byte  SatView;

	char  Hora[14];       /* hh:mm:ss.sss           */
	char  Data[10];        /* dd/mm/aa               */

	long  North;
	long  East;
	long  Alt;

	short Knots;          /* Velocidade x 100       */
	short Hdg;            /* Direção em graus x 100 */

	#if GPS_SATELLITE_MONITOR
		StGpsSat Sat[GPS_MAX_SATELITES];
	#endif
}StGpsData;

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Dados do GPS */
extern StGpsData GpsPos;

/* Timeout for system timer */
extern dword     GpsTimeout;

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Prototipos */
byte GpsInicia(void);
void GpsRefresh(void);
byte GpsRegistraFncProcessa(pGpsFnc FncProcessa);
void GpsRequest(byte Evt);
void GpsUartHandler(byte Data);
const char *GpsStringStatus(byte Status);
void GpsSimulaMsg(char Msg);

#endif

