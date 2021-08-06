/******************************************************************************/
/*                                                                            */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  app\gdnc\gdn.h                                                            */
/*                                                                            */
/*                                                                            */
/*  Funções básicas para utilização do protocolo UDP                          */
/*                                                                            */
/*  2015-08-06 Marcelo Henrique Santos                                        */
/*                                                                            */
/******************************************************************************/

#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "etc/defines.h"
#define WAYPOINTS_MAX_SIZE      0x10

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
struct StWaypoint
{
    float x;
    float y;
    float z;
    
    float vt;
};

/******************************************************************************/
/*                                                                            */
/*   FIFO Buffer                                                              */
/*                                                                            */
/******************************************************************************/
typedef struct fifoWP WP;

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
WP*  WaypointBegin();
void WaypointClear(WP *wp);
char WaypointRemove(WP *wp);
char getWaypoint(WP *wp, struct StWaypoint *waypoint);
char getNextWaypoint(WP *wp, struct StWaypoint *waypoint);
char WaypointInsert(WP *wp, struct StWaypoint waypoint);
char WaypointAvailable(WP *wp);

#endif