/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  drv\gps.c                                                                 */
/*                                                                            */
/*  Driver para receber os dados do GPS pelo protocolo NMEA                   */
/*                                                                            */
/*  2008-10-22 Mateus Inicial                                                 */
/*  2012-10-17 Mateus Adaptaro IAR ARM                                        */
/*                                                                            */
/******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "etc/defines.h"
#include "etc/system.h"
#include "drv/gps.h"
#include "utl/utils.h"

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/

#ifndef GPS_VERBOSE
	#define GPS_VERBOSE FALSE
#endif

#ifndef GPS_ECHO
	#define GPS_ECHO    FALSE
#endif

#if GPS_VERBOSE
	char GpsHeader[7];
#endif

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/

/* Ponteiro para função de decodificação */
typedef byte(*pFncDecod)(char*);

/*************************************************************************/
/*                                                                       */
/* TK1315TA                                                              */
/*                                                                       */
/* $GPGGA,004401.995,2312.1131,S,04553.6039,W,0,00,0.0,742.50,M,,,,*3A   */
/* $GPGSA,A,1,,,,,,,,,,,,,,,*1E                                          */
/* $GPRMC,004401.995,V,2312.1131,S,04553.6039,W,,,141112,,,N*70          */
/* $GPVTG,,,,,,,,,N*30                                                   */
/* $GPGSV,1,1,00,,,,*79                                                  */
/*                                                                       */
/*************************************************************************/


/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/

/* Maximo de satelites que vem na mensagem */
#define GPS_GSV_MAX_SATELLITE_MSG     4

/* Funções de trigger */
#ifndef GPS_MAX_FNC_PROCESSA
	#define GPS_MAX_FNC_PROCESSA      5
#endif

/* Timeout do GPS (ms) */
#ifndef GPS_TIMEOUT
    #define GPS_TIMEOUT               5000
#endif

/* Use double buffer? */
#ifndef GPS_DBL_BUFFER
    #define GPS_DBL_BUFFER            FALSE
#endif

/* Buffer size */
#ifndef GPS_TAM_BUFFER
    #define GPS_TAM_BUFFER            120
#endif

/* Flags */
#define GPS_FLG_RX_BUF2               0x01 // 0=buf1 1=buf2
#define GPS_FLG_BUFF_1_RDY            0x02
#define GPS_FLG_BUFF_2_RDY            0x04
#define GPS_FLG_DATA_READY            0x08

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/


/* Estrutura para processamento das mensagens */
typedef struct
{
	const char Header[8];  /* Identificador do cabeçalho    */
	pFncDecod FncDecod;    /* Função de processamento       */
}StGpsProcessa;

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/


/* Variavel de dados do GPS */
StGpsData       GpsPos;

/* Controle de timeout */
dword           GpsTimeout;

/* Controle de triggerlist pra processar */
static byte     GpsFncTotal;
static pGpsFnc  GpsFncProcessa[GPS_MAX_FNC_PROCESSA];

/* Buffer de recepção */
static char     GpsBuff[GPS_TAM_BUFFER];

#if GPS_DBL_BUFFER
    /* Segundo buffer de recepcao */
    static char GpsBuff2[GPS_TAM_BUFFER];
    #warning "GPS using double buffer!"
#endif

/* Controle de processamento */
static byte     GpsFlag;

/* Controle de updates */
static byte     GpsUltStatus;
static long     GpsUltNorth;
static long     GpsUltEast;
static long     GpsUltAlt;

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/

/* Calcula Cheksum */
byte GpsNmeaCalculaChecksum(char *Msg, char **PosCheck);
//void GpsBuscaItemMensagem(char **GpsData, char *PosValor);
char *GpsBuscaItemMensagem(char *GpsData, char *PosValor);
void GpsConfMensagem(byte Msg, byte Tempo);
void GpsMarcaGpsInvalido(void);
void GpsMarcaHoraInvalida(void);
byte GpsDecodifica(char *pGpsData);

/* Funções de decodificação */
byte GpsObtemNmeaGGA(char *GpsData);
byte GpsObtemNmeaGSA(char *GpsData);
byte GpsObtemNmeaGSV(char *GpsData);
byte GpsObtemNmeaRMC(char *GpsData);

/* Dispara trigger */
void GpsDisparaTrigger(byte Evento);

/* Calcula Hdop */
#define GpsCalculaHdop(Texto)      (byte)CnvStrFloat2Decimal(Texto, 1, '.')

/*************************************************************************/
/*                                                                       */
/*  Constante com os tipos de mensagens e funções de decodificação       */
/*                                                                       */
/*************************************************************************/
const StGpsProcessa GpsProcessa[] =
{
	/* Mensagem RMC */
	{ "$GPRMC\0" ,     GpsObtemNmeaRMC    },

	/* Mensagem GGA */
	{ "$GPGGA\0" ,     GpsObtemNmeaGGA    },

	/* MEnsagem GSA */
	{ "$GPGSA\0" ,     GpsObtemNmeaGSA    },

	/* Mensagem GSV */
	{ "$GPGSV\0" ,     GpsObtemNmeaGSV    },

	/* Fim */
	{ "\0" ,           FNC_NONE           }
};


/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/
byte GpsInicia(void)
{
	#if GPS_SATELLITE_MONITOR
		auto byte      SatId;
		auto StGpsSat *pSat;
	#endif

    /* Posição */
    GpsMarcaGpsInvalido();

    /* Hora */
    GpsMarcaHoraInvalida();

	/* Hdop */
	GpsPos.Hdop      = 0;

	/* Reseta timeout */
	GpsTimeout       = SysSetaTimeout(GPS_TIMEOUT);

	/* Marca sem nenhuma função registrada */
	GpsFncTotal      = 0;

    /* Nao estamos recebendo nada */
    GpsFlag          = 0x00;

    /* Limpa buffer */
    memset(GpsBuff, 0xFF, GPS_TAM_BUFFER);
    #if GPS_DBL_BUFFER
        memset(GpsBuff2, 0xFF, GPS_TAM_BUFFER);
    #endif

    /* Inicializa array de satellites */
    #if GPS_SATELLITE_MONITOR
    	pSat = &GpsPos.Sat[0];

    	for(SatId = 0; SatId < GPS_MAX_SATELITES; SatId++)
    	{
    		/* Marca como infvalido */
    		pSat->Id = GPS_SAT_NONE;

    		/* Proximo */
    		pSat++;
    	}
    #endif

    // iniciado!
    return(SUCESSO);
}

/*************************************************************************/
/*                                                                       */
/* Request an event from GPS                                             */
/*                                                                       */
/*************************************************************************/
void GpsRequest(byte Evt)
{
    // send events
    GpsDisparaTrigger(Evt);
}

/*************************************************************************/
/*                                                                       */
/*  Faz o refresh do status do GPS                                       */
/*                                                                       */
/*************************************************************************/
void GpsRefresh(void)
{
    // check if timeout expires
    if(SysVenceuTimeout(GpsTimeout) && (GpsPos.Status != GPS_AUSENTE))
    {
        // GPS ausente
        GpsMarcaGpsInvalido();

        // trigger
        GpsDisparaTrigger(GPS_EVT_MUDOU_STATUS);
    }

    // no data and no event ready at first
    GpsFlag &= ~GPS_FLG_DATA_READY;

    // check buffer 1 pending data
    if(GpsFlag & GPS_FLG_BUFF_1_RDY)
    {
        // decode message
        if(GpsDecodifica(GpsBuff) == SUCESSO)
        {
            // data ready!
            GpsFlag |= GPS_FLG_DATA_READY;
        }

        // clear buffer 1 flag
        GpsFlag &= ~GPS_FLG_BUFF_1_RDY;
    }

    #if GPS_DBL_BUFFER
        // check buffer 2 pending data
        if(GpsFlag & GPS_FLG_BUFF_2_RDY)
        {
            // decode message
            if(GpsDecodifica(GpsBuff2) == SUCESSO)
            {
                // data ready!
                GpsFlag |= GPS_FLG_DATA_READY;
            }

            // clear buffer 2 flag
            GpsFlag &= ~GPS_FLG_BUFF_2_RDY;
        }
    #endif

    // if no data is ready
    if((GpsFlag & GPS_FLG_DATA_READY) == 0)
    {
        // abort here
        return;
    }

    // Clear timeout
    GpsTimeout = SysSetaTimeout(GPS_TIMEOUT);

    /* Verifica se mudou de status */
    if(GpsUltStatus != GpsPos.Status)
    {
        /* Salva */
        GpsUltStatus = GpsPos.Status;

        /* Dispara trigger */
	    GpsDisparaTrigger(GPS_EVT_MUDOU_STATUS);

        /* aborta demais triggers */
        return;
    }

	/* Verifica se mudou de latitude */
	if(GpsUltNorth != GpsPos.North)
	{
		/* Salva  */
		GpsUltNorth = GpsPos.North;
		GpsUltEast  = GpsPos.East;
		GpsUltAlt   = GpsPos.Alt;

	    /* Dispara trigger */
	    GpsDisparaTrigger(GPS_EVT_NOVA_POS);

        /* aborta demais triggers */
        return;
	}

	/* Verifica se mudou de longitude */
	if(GpsUltEast != GpsPos.East)
	{
		/* Salva */
		GpsUltNorth = GpsPos.North;
		GpsUltEast  = GpsPos.East;
		GpsUltAlt   = GpsPos.Alt;

	    /* Dispara trigger */
	    GpsDisparaTrigger(GPS_EVT_NOVA_POS);

        /* aborta demais triggers */
        return;
	}

	/* Verifica se mudou de altitude */
	if(GpsUltAlt != GpsPos.Alt)
	{
		/* Salva */
		GpsUltNorth = GpsPos.North;
		GpsUltEast  = GpsPos.East;
		GpsUltAlt   = GpsPos.Alt;

	    /* Dispara trigger */
	    GpsDisparaTrigger(GPS_EVT_NOVA_POS);

        /* aborta demais triggers */
        return;
	}
}

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/
byte GpsDecodifica(char *pGpsData)
{
    static byte ChkSum;
    static char *pChkSum;

    /* Calcula o checksum */
    ChkSum = GpsNmeaCalculaChecksum(pGpsData, &pChkSum);

    /* Verifica se sao iguais */
    if(ChkSum != CnvStrHex2Byte((byte*)pChkSum))
    {
        // aborta
        return(ERRO);
    }

    // verifica o buffer
    if(strncmp(pGpsData, "$GPGGA", 6) == 0)
    {
        auto byte Ret;

        /* Primeiro decodifica a mensagem */
        Ret = GpsObtemNmeaGGA(pGpsData);

        /* Dispara trigger com nova mensagem */
        GpsDisparaTrigger(GPS_EVT_NOVA_MENSAGEM);

        // Retorna o que deu
        return(Ret);
    }

    // verifica o buffer
    if(strncmp(pGpsData, "$GPRMC", 6) == 0)
    {
        // decodifica RMC
        return(GpsObtemNmeaRMC(pGpsData));
    }

    // verifica o buffer
    if(strncmp(pGpsData, "$GPGSA", 6) == 0)
    {
        // decodifica RMC
        //return(GpsObtemNmeaGSA(pGpsData));
    }

    // verifica o buffer
    if(strncmp(pGpsData, "$GPGSV", 6) == 0)
    {
        // decodifica RMC
        return(GpsObtemNmeaGSV(pGpsData));
    }

	// nao deveria chegar aqui, mas se chegou da erro
    // Erro se recebeu alguma mensagem diferente das 4
	return(ERRO);
}


/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*  NMEA GGA DATA                                                        */
/*                                                                       */
/*  Message ID       $GPGGA          GGA protocol header                 */
/*  UTC Time         161229.487      hhmmss.sss                          */
/*  Latitude         3723.2475       ddmm.mmmm                           */
/*  N/S Indicator    N               N=north or S=south                  */
/*  Longitude        12158.3416      dddmm.mmmm                          */
/*  E/W Indicator    W               E=east or W=west                    */
/*  Position Fix     1               0=Invalid, 1=SPS, 2=Diff SPS, 3=PPS */
/*  Satellites Used  07              Range 0 to 12                       */
/*  HDOP             1.0             Horizontal Dilution of Precision    */
/*  MSL Altitude     9.0             meters                              */
/*  Units            M               meters                              */
/*  Geoid Separation                 meters                              */
/*  Units            M               meters                              */
/*  Age of Diff.                     second Null when DGPS not used      */
/*  Diff.Station ID  0000                                                */
/*  Checksum         *18                                                 */
/*  <CR><LF>                         End of message termination          */
/*************************************************************************/
byte GpsObtemNmeaGGA(char *GpsData)
{
	char Texto[16];
	char Hem[3];

	/* Busca hora */
	GpsData = GpsBuscaItemMensagem(GpsData, Texto);
    CnvFormata(GpsPos.Hora, Texto, "99:99:99");

	/* LATITUDE */

        /* Busca a latitude */
        GpsData = GpsBuscaItemMensagem(GpsData, Texto);

        /* Busca o hemisferio da latitude */
        GpsData = GpsBuscaItemMensagem(GpsData, Hem);

        /* Converte para latitude */
        GpsUltNorth      = GpsPos.North;
        GpsPos.North     = CnvStr2Latitude(Texto, Hem);

    /* LONGITUDE */

        /* Busca a longitude */
        GpsData = GpsBuscaItemMensagem(GpsData, Texto);

        /* Busca o hemisferio da longitude */
        GpsData = GpsBuscaItemMensagem(GpsData, Hem);

        /* Converte para longitude */
        GpsUltEast      = GpsPos.East;
        GpsPos.East     = CnvStr2Longitude(Texto, Hem);

	/* Position Fix */
	GpsData = GpsBuscaItemMensagem(GpsData, Texto);

	/* verifica se é valido */
	if(*Texto == '0')
	{
        /* Status invalido */
       	GpsPos.Status = GPS_INVALIDO;
    }
    else
    {
        /* Status operacional */
       	GpsPos.Status = GPS_OPERANDO;
    }

	/* Satellites Used  */
	GpsData = GpsBuscaItemMensagem(GpsData, Texto);
    GpsPos.SatUsed = (byte)CnvStr2Decimal(Texto, 0, 0);

	/* HDOP */
	GpsData = GpsBuscaItemMensagem(GpsData, Texto);
    GpsPos.Hdop = GpsCalculaHdop(Texto);

	/* ALTITUDE */

    	/* MSL Altitude */
    	GpsData = GpsBuscaItemMensagem(GpsData, Texto);

    	/* MSL Altitude Units */
    	GpsData = GpsBuscaItemMensagem(GpsData, Hem);

    	/* Converte o valor */
    	GpsUltAlt     = GpsPos.Alt;
    	GpsPos.Alt    = (int)CnvStr2Decimal(Texto, 0, 0);

	#if GPS_VERBOSE
		sprintf(Debug, "    GGA.NORTH :%10ld\r\n", GpsPos.North );   SerPuts(Debug);
		sprintf(Debug, "    GGA.EAST  :%10ld\r\n", GpsPos.East  );   SerPuts(Debug);
		sprintf(Debug, "    GGA.ALT   :%d\r\n"   , GpsPos.Alt   );   SerPuts(Debug);
		sprintf(Debug, "    GGA.TIME  :%s\r\n"   , GpsPos.Hora  );   SerPuts(Debug);
	#endif

    /* Se chegamos aqui esta tudo certo */
    return(SUCESSO);
}


/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/* "$GPRMC,hhmmss.sss,[A/V],ddmm.mmmm,[N/S],dddmm.mmmm,[E/W],99.9,       */
/*  999.99,ddmmyy,99.9,*<ChkSum><CR><LF>"                                */
/*                                                                       */
/*  NMEA RMC DATA                                                        */
/*                                                                       */
/*  Message ID       $GPRMC          RMC protocol header                 */
/*  UTC Time         161229.487      hhmmss.sss                          */
/*  Status           A               A=data valid or V=data not valid    */
/*  Latitude         3723.2475       ddmm.mmmm                           */
/*  Hemisphere       N               N=north or S=south                  */
/*  Longitude        12158.3416      dddmm.mmmm                          */
/*  hemisphere       W               E=east or W=west                    */
/*  Ground Speed     0.13            Knots                               */
/*  Ground Course    309.62          Degrees True                        */
/*  Date             120598          ddmmyy                              */
/*  Magnetic Var     ? degrees       E=east or W=west                    */
/*  Checksum         *10                                                 */
/*  <CR><LF>                         End of message termination          */
/*************************************************************************/
byte GpsObtemNmeaRMC(char *GpsData)
{
	char Texto[16];
	char Hem[3];

	/* Busca hora */
	GpsData = GpsBuscaItemMensagem(GpsData, Texto);

	/* Busca estado valido / invalido */
	GpsData = GpsBuscaItemMensagem(GpsData, Texto);

	/* Verifica se o texto é valido */
	if(*Texto != 'A')
	{
        /* Status invalido */
       	GpsPos.Status = GPS_INVALIDO;
    }
    else
    {
        /* Status operacional */
       	GpsPos.Status = GPS_OPERANDO;
    }

	/* LATITUDE */

        /* Busca a latitude */
        GpsData = GpsBuscaItemMensagem(GpsData, Texto);

        /* Busca o hemisferio da latitude */
        GpsData = GpsBuscaItemMensagem(GpsData, Hem);

        /* Converte para latitude */
        GpsUltNorth      = GpsPos.North;
        GpsPos.North     = CnvStr2Latitude(Texto, Hem);

    /* LONGITUDE */

        /* Busca a longitude */
        GpsData = GpsBuscaItemMensagem(GpsData, Texto);

        /* Busca o hemisferio da longitude */
        GpsData = GpsBuscaItemMensagem(GpsData, Hem);

        /* Converte para longitude */
        GpsUltEast      = GpsPos.East;
        GpsPos.East     = CnvStr2Longitude(Texto, Hem);

	/* KNOTS */

        /* Busca a velocidade */
        GpsData = GpsBuscaItemMensagem(GpsData, Texto);
        GpsPos.Knots = (word)CnvStr2Decimal(Texto, 0, 0);

    /* HEADING */

        /* Busca o heading */
        GpsData = GpsBuscaItemMensagem(GpsData, Texto);
        GpsPos.Hdg = (word)CnvStr2Decimal(Texto, 0, 0);

	/* DATA */

        /* Busca a data */
    	GpsData = GpsBuscaItemMensagem(GpsData, Texto);
        CnvFormata(GpsPos.Data, Texto, "99/99/99");

	#if GPS_VERBOSE
		sprintf(Debug, "    RMC.NORTH :%10ld\r\n", GpsPos.North    );   SerPuts(Debug);
		sprintf(Debug, "    RMC.EAST  :%10ld\r\n", GpsPos.East     );   SerPuts(Debug);
		sprintf(Debug, "    RMC.TIME  :%s\r\n"   , GpsPos.Hora     );   SerPuts(Debug);
		sprintf(Debug, "    RMC.DATE  :%s\r\n"   , GpsPos.Data     );   SerPuts(Debug);
		sprintf(Debug, "    RMC.HDG   :%d\r\n"   , GpsPos.Rmc.Hdg  );   SerPuts(Debug);
		sprintf(Debug, "    RMC.KNOTS :%d\r\n"   , GpsPos.Rmc.Knots);   SerPuts(Debug);
	#endif

	/* Terminamos com êxito */
	return(SUCESSO);
}


/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/* $GPGSA,[A/M],[1/2/3],[aa],[bb],[cc],,,,,,,,,,[PDOP],[HDOP],[VDOP]*33  */
/*                                                                       */
/*  NMEA GSA DATA                                                        */
/*                                                                       */
/*  Message ID       $GPGSA          GSA protocol header                 */
/*  Mode1            A               A = Automatico, M = Manual          */
/*  Mode2            3               1 ERR, 2 Oper 2D, 3 Oper 3D         */
/*  Satellite        07              Sv on Channel 1                     */
/*  Satellite        02              Sv on Channel 2                     */
/*  ...                                                                  */
/*  Satellite                        Sv on Channel 12                    */
/*  PDOP             1.8             Position dilution of Precision      */
/*  HDOP             1.0             Horizontal dilution of Precision    */
/*  VDOP             1.5             Vertical dilution of Precision      */
/*  Checksum         *33                                                 */
/*  <CR><LF>                         End of message termination          */
/*************************************************************************/
byte GpsObtemNmeaGSA(char *GpsData)
{
	#if GPS_SATELLITE_MONITOR
		auto byte      SatIdx;
		auto byte      Sat;
		auto StGpsSat *pSat;
	#endif

	auto char Texto[16];

	/* Busca estado valido / invalido */
	GpsData = GpsBuscaItemMensagem(GpsData, Texto);

	/* Verifica o modo do GPS */
	//if(*Texto != 'A')
	//{
    //
	//}

	/* Busca modo de operação */
	GpsData = GpsBuscaItemMensagem(GpsData, Texto);

	/* Verifica se o texto é valido */
	if(*Texto == '1')
	{
		/* Muda estado para Invalido */
		GpsPos.Status = GPS_INVALIDO;
	}
	else
	{
		/* Muda estado para Invalido */
		GpsPos.Status = GPS_OPERANDO;
	}

	#if GPS_SATELLITE_MONITOR
	/* Primeiro satelite */
	pSat = GpsPos.Sat;

	/* Preenche os IDs dos satelites nos canais */
	for(SatIdx = 0; SatIdx < GPS_MAX_SATELITES; SatIdx++)
	{
		/* Obtem o ID */
		GpsData = GpsBuscaItemMensagem(GpsData, Texto);
		Sat = (byte)CnvStr2Decimal(Texto, 0, 2);

		/* Verifica se tem satelite */
		if(Sat)
		{
			/* Marca com o satelite visado */
			pSat->Id = Sat;
		}
		else
		{
			/* channel nao utilizado */
			pSat->Id  = GPS_SAT_NONE;
			pSat->Pot = 0;
		}

		/* Proximo */
		pSat++;
	}
	#endif

    /* Se chegamos aqui esta tudo certo */
    return(SUCESSO);
}

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/* $GPGSV,2,1,07,07,79,048,42,02,51,062,43,26,36,256,42,27,27,138,42*71  */
/* $GPGSV,2,2,07,09,23,313,42,04,19,159,41,15,12,041,42*41               */
/*                                                                       */
/*  NMEA GSV DATA                                                        */
/*                                                                       */
/*  Message ID       $GPGSV          GSV protocol header                 */
/*  #Messages        2               Range 1 to 3                        */
/*  Message#         1               Range 1 to 3                        */
/*  Sat in View      07                                                  */
/*  Sat ID           07              Channel 1(Range 1 to 32)            */
/*  Elevation        79  degrees     Channel 1(Maximum90)                */
/*  Azimuth          048 degrees     Channel 1(True, Range 0 to 359)     */
/*  SNR(C/No)        42 dBHz         0 to 99. 0 when not tracking        */
/*  ...                                                                  */
/*  Sat ID           27              Channel 4 (Range 1 to 32)           */
/*  Elevation        27  Degrees     Channel 4 (Maximum90)               */
/*  Azimuth          138 Degrees     Channel 4 (True, Range 0 to 359)    */
/*  SNR(C/No)        42 dBHz         0 to 99. 0 when not tracking        */
/*  Checksum         *71                                                 */
/*  <CR><LF>                         End of message termination          */
/*************************************************************************/
byte GpsObtemNmeaGSV(char *GpsData)
{
	#if GPS_SATELLITE_MONITOR

        auto char MsgNumber;
        auto char MsgTotal;

        auto byte Sat;
        auto byte IdxChn;
        auto byte MsgSat;

        auto StGpsSat  Satelite;
        //auto StGpsSat *pSat;

        auto char Texto[16];

        /* Numero de mensagens */
        GpsData  = GpsBuscaItemMensagem(GpsData, Texto);
        MsgTotal = (byte)CnvStr2Decimal(Texto, 0, 1);

        /* Mensagem numero # */
        GpsData   = GpsBuscaItemMensagem(GpsData, Texto);
        MsgNumber = (byte)CnvStr2Decimal(Texto, 0, 1);

        /* Satelites visados */
        GpsData = GpsBuscaItemMensagem(GpsData, Texto);
        GpsPos.SatView = (byte)CnvStr2Decimal(Texto, 0, 2);

        /* Obtem o numero de satelites da mensagem */
        if(MsgNumber == MsgTotal)
        {
            /* Restante */
            MsgSat = GpsPos.SatView - ((MsgNumber - 1) * GPS_GSV_MAX_SATELLITE_MSG);
        }
        else
        {
            /* Nao é a ultima, tem 4 satelites */
            MsgSat = GPS_GSV_MAX_SATELLITE_MSG;
        }

		/* Primeiro satelite */
		Sat = 0;

		/* varre todos os satelites da mensagem */
		while((Sat < GPS_GSV_MAX_SATELLITE_MSG) && (MsgSat))
		{
			/* Obtem ID */
			GpsData = GpsBuscaItemMensagem(GpsData, Texto);
			Satelite.Id = (byte)CnvStr2Decimal(Texto, 0, 2);

			/* Obtem Elevacao */
			GpsData = GpsBuscaItemMensagem(GpsData, Texto);
			Satelite.Elev = (byte)CnvStr2Decimal(Texto, 0, 2);

			/* Obtem Azimuth */
			GpsData = GpsBuscaItemMensagem(GpsData, Texto);
			Satelite.Azim = (byte)(CnvStr2Decimal(Texto, 0, 3) / 2);

			/* Obtem Potencia */
			GpsData = GpsBuscaItemMensagem(GpsData, Texto);
			Satelite.Pot = (byte)CnvStr2Decimal(Texto, 0, 2);

            #if 0
    			pSat = &GpsPos.Sat[0];

    			for(IdxChn = 0; IdxChn < GPS_MAX_SATELITES; IdxChn++)
    			{
    				/* Verifica se o satelite encontrado esta nos canais */
    				if(pSat->Id == Satelite.Id)
    				{
    					/* Atualiza os dados */
    					pSat->Elev = Satelite.Elev;
    					pSat->Azim = Satelite.Azim;
    					pSat->Pot  = Satelite.Pot;

    					break;
    				}

    				/* Proximo */
    				pSat++;
    			}
			#else
			    /* Calcula o index do satelite */
			    IdxChn = ((MsgNumber - 1) * GPS_GSV_MAX_SATELLITE_MSG) + Sat;
			    if(IdxChn < GPS_MAX_SATELITES)
			    {
			        // copy entire satellite data
			        memcpy(&GpsPos.Sat[IdxChn], &Satelite, sizeof(StGpsSat));
			    }
            #endif

			#if GPS_VERBOSE
				sprintf(Debug, "    GSV.SAT   :%02d ELEV: %02d AZIM: %03d POT:%02d\r\n", Satelite.Id, Satelite.Elev, 2*Satelite.Azim, Satelite.Pot );
				SerPuts(Debug);
			#endif

			/* Proximo */
			Sat++;

			/* Decrementa 1 satelite lido */
			MsgSat--;
		}

        /*  Se esta é a ultima mensagem, dispara trigger de novos satelites */
        GpsDisparaTrigger(GPS_EVT_SATELITES);
	#endif

	#if GPS_VERBOSE
		sprintf(Debug, "    GSV.TOTAL :%d\r\n", GpsPos.SatUsed );   SerPuts(Debug);
	#endif

    /* Se chegamos aqui esta tudo certo */
    return(SUCESSO);
}

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/
const char *GpsStringStatus(byte Status)
{
    switch(Status)
    {
        /* Gps nao esta presente */
        case GPS_AUSENTE:
            return("AUSENTE");

        /* Gps sem solução */
        case GPS_INVALIDO:
            return("INVALIDO");

        /* Gps valido */
        case GPS_OPERANDO:
            return("OK");
    }

    /* dafuk? */
    return("//////");
}

/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/
char *GpsBuscaItemMensagem(char *GpsData, char *PosValor)
{
	/* Busca a virgula */
	while(*GpsData != ',')
	{
    	/* Se terminou a mensagem, sai */
    	if((*GpsData == '\0') || (*GpsData == '*'))
    	{
            *PosValor = '\0';
            return(GpsData);
        }

        /* proximo */
		GpsData++;
	}

    /* Pula a virgula */
	GpsData++;

	/* Copia ate encontrar ',' ou '*' */
	while((*GpsData != ',') && (*GpsData != '*'))
	{
	    /* copia */
		*PosValor = *GpsData;

        /* Proximo */
		GpsData++;
		PosValor++;
	}

	/* Termina a string de dados */
	*PosValor = '\0';
	return(GpsData);
}

#if 0
/*************************************************************************/
/*                                                                       */
/*                                                                       */
/*                                                                       */
/*************************************************************************/
void GpsBuscaItemMensagem(char **GpsData, char *PosValor)
{
	/* Busca o começo */
	while((**GpsData != ',') && ((**GpsData != '\0') || (**GpsData != '*')))
	{
		*GpsData = *GpsData + 1;
	}

	/* Se terminou a mensagem, sai */
	if((**GpsData == '\0') || (**GpsData == '*'))
	{
        *PosValor = '\0';
        return;
    }

    /* Pula a virgula */
	*GpsData = *GpsData + 1;

	/* Copia ate encontrar ',' */
	while((**GpsData != ',') && ((**GpsData != '\0') || (**GpsData != '*')))
	{
		*PosValor = **GpsData;

		*GpsData = *GpsData + 1;
		PosValor++;
	}

	/* Termina a string de dados */
	*PosValor = '\0';
}
#endif

/*************************************************************************/
/*                                                                       */
/*  Calcula o valor de checksum da msg do gps                            */
/*                                                                       */
/*************************************************************************/
byte GpsNmeaCalculaChecksum(char *Msg, char **PosCheck)
{
	byte Lrc;
	Lrc = 0;

	/* Procura inicio atraves do '$' */
	while(*Msg != '$')
	{
		Msg ++;
	}
	Msg ++;

	/* Calcula o CheckSum ate o '*' */
	while(*Msg != '*')
	{
		Lrc = Lrc ^ *Msg;
		Msg ++;
	}

    if(PosCheck)
    {
    	/* Posição pra inserir o valor do checksum */
    	*PosCheck = Msg + 1;
    }

	return(Lrc);
}

/*************************************************************************/
/*                                                                       */
/*  Marca que nossa posição é invalida                                   */
/*                                                                       */
/*************************************************************************/
void GpsMarcaGpsInvalido(void)
{
    /* Posição */
    GpsPos.North     = GPS_POS_NORTH_NONE;
    GpsPos.East      = GPS_POS_EAST_NONE;
    GpsPos.Alt       = GPS_POS_ALT_NONE;
    GpsUltNorth      = GpsPos.North;
    GpsUltEast       = GpsPos.East;
    GpsUltAlt        = GpsPos.Alt;

    /* Status */
    GpsPos.Status    = GPS_AUSENTE;
    GpsUltStatus     = GpsPos.Status;

    /* Satelites usados */
    GpsPos.SatUsed   = 0;
    GpsPos.SatView   = 0;
}

/*************************************************************************/
/*                                                                       */
/*  Marca a hora do GPS como invalida                                    */
/*                                                                       */
/*************************************************************************/
void GpsMarcaHoraInvalida(void)
{
	/* Copia string indicando data hora invalida */
	strcpy(GpsPos.Data, "--/--/--");
	strcpy(GpsPos.Hora, "--:--:--");
}

/*************************************************************************/
/*                                                                       */
/*  Configura mensagem do GPS                                            */
/*                                                                       */
/*************************************************************************/
void GpsConfMensagem(byte Msg, byte Tempo)
{
	auto byte Rate;
	auto byte Query;
	auto char Conf[50];
	auto char *pChkSum;


	/* Seta os parametros de requisição */
	if(Tempo)
	{
		/* Taxa de repetição */
		Rate  = Tempo;
		Query = FALSE;
	}
	else
	{
		/* Desativa */
		Rate  = 0;
		Query = TRUE;
	}


	/* Formata a mensagem */
	sprintf(Conf, "$PSRF103,%02d,%02d,%02d,1*", Msg, Query, Rate);

	/* Calcula ChkSum */
	Rate = GpsNmeaCalculaChecksum(Conf, &pChkSum);

	/* Termina a mensagem */
	sprintf(pChkSum, "%02X%c%c", Rate, 0x0D, 0x0A);

	/* Envia */
	//SerPuts(Conf);
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte GpsRegistraFncProcessa(pGpsFnc FncProcessa)
{
	/* Verifica se cabe */
	if(GpsFncTotal >= GPS_MAX_FNC_PROCESSA)
	{
		return(ERRO);
	}

	/* Adiciona */
	GpsFncProcessa[GpsFncTotal] = FncProcessa;

	/* Conta mais uma */
	GpsFncTotal++;

    /* Envia evento de status pra atualizar */
    FncProcessa(GPS_EVT_MUDOU_STATUS);

	return(SUCESSO);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void GpsDisparaTrigger(byte Evento)
{
	auto byte    i;
	auto pGpsFnc pFnc;

	/* Verifica se cabe */
	for(i = 0; i < GpsFncTotal; i++)
	{
		/* Função */
		pFnc = GpsFncProcessa[i];

		/* Chama função */
		if(pFnc)
		{
			pFnc(Evento);
		}
	}
}


/*************************************************************************/
/*                                                                       */
/* Handle for the UART hardware                                          */
/*                                                                       */
/*************************************************************************/
void GpsUartHandler(byte Data)
{
    static short Len = 0;
    static char *p   = NULL;

    #if GPS_DBL_BUFFER
        // check receiving buffer #2
        if(GpsFlag & GPS_FLG_RX_BUF2)
        {
            // buffer 2 is being used
            // check if empty
            if(GpsFlag & GPS_FLG_BUFF_2_RDY)
            {
                // abort!
                return;
            }
        }
        else
        {
            // buffer 1 is being used
            // check if empty
            if(GpsFlag & GPS_FLG_BUFF_1_RDY)
            {
                // abort!
                return;
            }
        }
    #else
        // check receiving single buffer
        if(GpsFlag & GPS_FLG_BUFF_1_RDY)
        {
            // abort!
            return;
        }
    #endif

    //
    // check if we are receiving
    //
    if(!p)
    {
        // STX?
        if(Data == '$')
        {
            #if GPS_DBL_BUFFER
                // check which buffer we are using now
                if(GpsFlag & GPS_FLG_RX_BUF2)
                {
                    // start receiving buffer 1
                    GpsBuff2[0] = '$';
                    p = GpsBuff2 + 1;
                }
                else
                {
                    // start receiving buffer 1
                    GpsBuff[0] = '$';
                    p = GpsBuff + 1;
                }
            #else
                // start receiving buffer 1
                GpsBuff[0] = '$';
                p = GpsBuff + 1;
            #endif

            // for '$' and last '\0'
            Len = 2;
        }
    }
    else
    {
        // check ETX
        if((Data == '\r') || (Data == '\n') || (Len >= GPS_TAM_BUFFER))
        {
            // terminate buffer, wait for STX again
            *p = '\0';
            p  = NULL;

            #if GPS_DBL_BUFFER
                // check which buffer we were using
                if(GpsFlag & GPS_FLG_RX_BUF2)
                {
                    // set flags for buffer 2
                    GpsFlag |= GPS_FLG_BUFF_2_RDY;

                    // check if buffer 1 is free
                    if((GpsFlag & GPS_FLG_BUFF_1_RDY) == 0)
                    {
                        // change to buffer 1 now...
                        GpsFlag &= ~GPS_FLG_RX_BUF2;
                    }
                }
                else
                {
                    // set flags for buffer 1
                    GpsFlag |= GPS_FLG_BUFF_1_RDY;

                    // check if buffer 2 is free
                    if((GpsFlag & GPS_FLG_BUFF_2_RDY) == 0)
                    {
                        // change to buffer 2 now...
                        GpsFlag |= GPS_FLG_RX_BUF2;
                    }
                }
            #else
                // set flags for buffer 1
                GpsFlag |= GPS_FLG_BUFF_1_RDY;
            #endif
        }
        else
        {
            // copy into buffer
            *p = Data;
            Len++;
            p++;
        }
    }
}


/*************************************************************************/
/*                                                                       */
/* Simula mensagens de GPS                                               */
/*                                                                       */
/*************************************************************************/
const char GpsSmGga[] = "$GPGGA,125813.410,2113.9445,S,04459.7725,W,1,6,1.7,860,M,15,M,,*";
const char GpsSmRmc[] = "$GPRMC,,,,,,,,,,,,,*";
const char GpsSmGsa[] = "$GPGSA,A,3,05,12,07,04,17,08,,,,,,,1.8,1.0,1.5*";
const char GpsSmGsv1[]= "$GPGSV,2,1,6,05,10,036,0,12,45,072,81,07,12,189,0,04,52,112,80*";
const char GpsSmGsv2[]= "$GPGSV,2,2,6,17,19,280,0,08,87,358,97,,,,,,,,*";

// por enquanto só 2 GSV com 6 sat
//const char GpsSmGsa[] = "$GPGSA,A,3,05,12,07,04,17,08,03,11,10,06,,,1.8,1.0,1.5*";
//const char GpsSmGsv2[]= "$GPGSV,3,2,10,17,19,121,0,08,87,358,97,03,81,266,95,11,22,307,0*";
//const char GpsSmGsv3[]= "$GPGSV,3,3,10,10,66,080,76,06,72,218,77,,,,,,,,*";

void GpsSimulaMsg(char Msg)
{
    auto       char   Crc;
    auto       char   Fim[5];
    auto const char  *pTx;
    auto const char  *pMsg;

    switch(Msg)
    {
        // GGA
        default:
        case 'a':
        case 'A': pMsg = GpsSmGga;  break;

        // RMC
        case 'r':
        case 'R': pMsg = GpsSmRmc;  break;

        // GSA
        case 's':
        case 'S': pMsg = GpsSmGsa;  break;

        // GSV (todas)
        case 'v':
        case 'V': pMsg = GpsSmGsv1;  break;
    }

    // o que vamos transmitir
    pTx = pMsg;
    while(*pTx) { GpsUartHandler(*pTx); pTx++; }

    // calcula o CRC
    Crc = GpsNmeaCalculaChecksum((char*)pMsg, NULL);
    sprintf(Fim, "%02X", Crc);
    GpsUartHandler(Fim[0]);
    GpsUartHandler(Fim[1]);
    GpsUartHandler('\r');
    GpsUartHandler('\n');

    // a mensagem era a GSV?
    if(pMsg == GpsSmGsv1)
    {
        // manda a segunda!
        pMsg = GpsSmGsv2;
        pTx = pMsg;
        while(*pTx) { GpsUartHandler(*pTx); pTx++; }

        // calcula o CRC
        Crc = GpsNmeaCalculaChecksum((char*)pMsg, NULL);
        sprintf(Fim, "%02X", Crc);
        GpsUartHandler(Fim[0]);
        GpsUartHandler(Fim[1]);
        GpsUartHandler('\r');
        GpsUartHandler('\n');
    }
}

