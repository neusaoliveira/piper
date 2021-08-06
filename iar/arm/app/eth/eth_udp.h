/******************************************************************************/
/*                                                                            */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  app\eth\eth_udp.h                                                         */
/*                                                                            */
/*                                                                            */
/*  Funções básicas para utilização do protocolo UDP                          */
/*                                                                            */
/*  2015-08-06 Marcelo Henrique Santos                                        */
/*                                                                            */
/******************************************************************************/

#ifndef ETH_UDP_H
#define ETH_UDP_H

#define ETH_UDP_RX_MAX_PACKETS          64
#define ETH_UDP_TX_PACKET_MAX_SIZE      256

#include "lwip/udp.h"
#include "etc/defines.h"


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
typedef struct
{	
    struct pbuf *p;
    struct ip_addr remoteIP;
    unsigned short remotePort;
    struct ip_addr destIP;
	
}Stpacket;

typedef struct 
{
    Stpacket packets[ETH_UDP_RX_MAX_PACKETS];    

    struct udp_pcb *pcb;
    struct pbuf *p;
    unsigned short port;
    
    /* IP and port filled in when receiving a packet */
    struct ip_addr remoteIP;
    unsigned short remotePort;
    struct ip_addr destIP;
    
    /* pbuf, pcb, IP and port used when acting as a client */
    struct pbuf *sendTop;
    struct udp_pcb *sendToPcb;
    struct ip_addr sendToIP;	
    unsigned short sendToPort;
	
} StEthUdp;


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
extern StEthUdp udp;


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
unsigned char UdpBegin( unsigned short port );
unsigned char UdpParsePacket();
unsigned char UdpRead( unsigned char *buffer, unsigned short size );
unsigned char UdpWrite( struct ip_addr ip, unsigned short port,
                       unsigned char *buffer, unsigned short size );

#endif
