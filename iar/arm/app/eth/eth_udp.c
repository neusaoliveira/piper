/******************************************************************************/
/*                                                                            */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  app\eth\eth_udp.c                                                         */
/*                                                                            */
/*                                                                            */
/*  Funções básicas para utilização do protocolo UDP                          */
/*                                                                            */
/*  2015-08-06 Marcelo Henrique Santos                                        */
/*                                                                            */
/******************************************************************************/

#include "app/eth/eth_udp.h"
#include "inc/hw_types.h"

#include <string.h>

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
StEthUdp udp;

static unsigned char front;
static unsigned char rear;
static unsigned char count;

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void receive( void *arg, struct udp_pcb *pcb, struct pbuf *p,
			  struct ip_addr *addr, unsigned short port );
			  

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
unsigned char UdpBegin( unsigned short port )
{
    udp.port = port;
    udp.pcb = udp_new();
    udp_bind(udp.pcb, IP_ADDR_ANY, udp.port);	

    udp_recv(udp.pcb, receive, false);
    return true;
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
unsigned char UdpParsePacket()
{
    /* Discard the current packet */
    if(udp.p) 
    {
        pbuf_free(udp.p);
        udp.p = NULL;
        udp.remotePort = 0;
        //udp.remoteIP = IPAddress(IPADDR_NONE);
        //udp.destIP = IPAddress(IPADDR_NONE);
    }

    /* No more packets in the queue */
    if(!count) 
    {
        return 0;
    }

    /* Take the next packet from the front of the queue */
    udp.p = udp.packets[front].p;
    udp.remoteIP = udp.packets[front].remoteIP;
    udp.remotePort = udp.packets[front].remotePort;
    udp.destIP = udp.packets[front].destIP;

    count--;

    /* Advance the front of the queue */
    front++;

    /* Wrap around if end of queue has been reached */
    if(front == ETH_UDP_RX_MAX_PACKETS)
        front = 0;

    /* Return the total len of the queue */
    return udp.p->tot_len;	
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
unsigned char UdpRead( unsigned char *buffer, unsigned short size )
{
    unsigned char *p = (unsigned char *)udp.p->payload;    
    memcpy(buffer, p, udp.p->len);

    if( udp.p->next) 
    {        
        struct pbuf *p;
        p = udp.p->next;
        /* Increase ref count on p->next
         * 1->2->1->etc */
        pbuf_ref(udp.p->next);
        /* Free p which decreases ref count of the chain
         * and frees up to p->next in this case
         * ...->1->1->etc */
        pbuf_free(udp.p);
        udp.p = NULL;
        udp.p = p;
        
    } else 
    {       
        pbuf_free(udp.p);
        udp.p = NULL;
    }

    return true;	
	
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void receive( void *arg, struct udp_pcb *pcb, struct pbuf *p,
			   struct ip_addr *addr, unsigned short port )
{
	/* No more space in the receive queue */
    if(count >= ETH_UDP_RX_MAX_PACKETS) 
    {
        pbuf_free(p);
        return;
    }

    /* Increase the number of packets in the queue
     * that are waiting for processing */
    count++;
	
    /* Add pacekt to the rear of the queue */
    udp.packets[rear].p = p;
	
    /* Record the IP address and port the pacekt was received from */
    //udp->packets[udp->rear].remoteIP = addr;
    udp.packets[rear].remotePort = port;
    //udp->packets[udp->rear].destIP = addr;

    /* Advance the rear of the queue */
    rear++;

    /* Wrap around the end of the array was reached */
    if(rear == ETH_UDP_RX_MAX_PACKETS)
        rear = 0;
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
unsigned char UdpWrite( struct ip_addr ip, unsigned short port,
                        unsigned char *buffer, unsigned short size )   
{       
    
    udp.sendToIP = ip;
    udp.sendToPort = port;
    udp.sendTop = pbuf_alloc(PBUF_TRANSPORT, ETH_UDP_TX_PACKET_MAX_SIZE, PBUF_POOL);
    
    if(udp.sendTop == NULL)
        return false;
    
    unsigned short avail = udp.sendTop->tot_len;

    /* If there is no more space available
     * then return immediately */
    if(avail == 0)
        return false;

    /* If size to send is larger than is available,
     * then only send up to the space available */
    if(size > avail)
        size = avail;

    /* Copy buffer into the pbuf */
    pbuf_take(udp.sendTop, buffer, size);	

    /* Shrink the pbuf to the actual size that was written to it */
    pbuf_realloc(udp.sendTop, size);

    /* Send the buffer to the remote host */
    err_t err = udp_sendto(udp.pcb, udp.sendTop, &udp.sendToIP, udp.sendToPort);

    /* udp_sendto is blocking and the pbuf is
     * no longer needed so free it */
    pbuf_free(udp.sendTop);

    if(err != ERR_OK)
        return false;

    return true;	
	
}