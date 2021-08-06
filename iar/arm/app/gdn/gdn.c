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

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "app/gdn/gdn.h"

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
struct fifoWP
{  
    struct StWaypoint data[WAYPOINTS_MAX_SIZE]; 
    volatile unsigned int size;   
    volatile unsigned int head;
    volatile unsigned int tail;
};

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
char empty(WP *wp);
char  full(WP *wp);


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
WP* WaypointBegin()
{
    WP *wp = (WP*) malloc(sizeof(struct fifoWP));
    if(wp != NULL)
    {
        wp->head = 0;
        wp->tail = 0;
        wp->size = 0;
    }	
    return wp;
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
char WaypointAvailable(WP *wp)
{
    if (wp == NULL) return (false);    
    return (!(wp->size-1)) ? false : true;
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void WaypointClear(WP *wp)
{
    free(wp);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
char WaypointRemove(WP *wp)
{
    if (wp == NULL || empty(wp)) return (false);
    
    wp->head = (wp->head + 1) % WAYPOINTS_MAX_SIZE;
    wp->size --;
    return(true);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
char WaypointInsert(WP *wp, struct StWaypoint waypoint)
{
    if (wp == NULL) return (false);
    if (full(wp))   return (false);
    
    wp->data[wp->tail] = waypoint;
    wp->tail = (wp->tail + 1) % WAYPOINTS_MAX_SIZE;
    wp->size ++;
    
    return(true);
    
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
char getWaypoint(WP *wp, struct StWaypoint *waypoint)
{
    if (wp == NULL || empty(wp)) return (false);
    
    *waypoint = wp->data[wp->head];
    return(true);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
char getNextWaypoint(WP *wp, struct StWaypoint *waypoint)
{
    if (wp == NULL || empty(wp)) return (false);
    
    *waypoint = wp->data[wp->head+1];
    return(true);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
char empty(WP *wp)
{
    if(wp == NULL) return(-1);
    
    if(!wp->size)
        return(true);
    else
        return(false);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
char full(WP *wp)
{
    if(wp == NULL) return(-1);
    
    if(wp->size == WAYPOINTS_MAX_SIZE)
        return(true);
    else
        return(false);
}