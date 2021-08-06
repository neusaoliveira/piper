//*****************************************************************************
//
// globals.h - Shared configuration and global variables.
//
// Copyright (c) 2006-2011 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 7243 of the EK-LM3S2965 Firmware Package.
//
//*****************************************************************************

#ifndef __GLOBALS_H__
#define __GLOBALS_H__

//*****************************************************************************
//
// A set of flags used to track the state of the application.
//
//*****************************************************************************
extern unsigned long g_ulFlags;
#define FLAG_CLOCK_TICK         0           // A timer interrupt has occurred
#define FLAG_CONTROLE           1           // Compute Controller AutoPilot
#define FLAG_MATLAB_START       2           // Received data from NAV
//#define FLAG_ECS_UART0          3           // Also send ECS data via UART0
//#define FLAG_PEGASUS            4           // Received pegasus packet

//*****************************************************************************
//
// The speed of the processor.
//
//*****************************************************************************
extern unsigned long g_ulSystemClock;

//*****************************************************************************
//
// The debounced state of the five push buttons.  The bit positions correspond
// to 1=sw pressed. 0=sw unchanged:
//
//     0 - Up
//     1 - Down
//     2 - Left
//     3 - Right
//     4 - Select
//
//*****************************************************************************
extern unsigned char g_ucSwitches;
#if 0
    #ifdef RIT_UPSIDE
        #define BTN_UP    0x02
        #define BTN_DN    0x01
        #define BTN_LF    0x08
        #define BTN_RT    0x04
    #else
        #define BTN_UP    0x01
        #define BTN_DN    0x02
        #define BTN_LF    0x04
        #define BTN_RT    0x08
    #endif
    #define BTN_SL        0x10
#endif

//*****************************************************************************
//
// Delay function
//
//*****************************************************************************
void Delay(unsigned long ulCount);

#endif // __GLOBALS_H__
