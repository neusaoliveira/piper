//*****************************************************************************
//
// cmdline.c - Functions to help with processing command lines.
//
// Copyright (c) 2007-2012 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 9453 of the Stellaris Firmware Development Package.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup cmdline_api
//! @{
//
//*****************************************************************************

#include "conf.h"
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "lib/uart.h"
#include "lib/utils/cmdline.h"
#include "lib/utils/uartstdio.h"

//*****************************************************************************
//
// Defines the maximum number of arguments that can be parsed.
//
//*****************************************************************************
#ifndef CMDLINE_MAX_ARGS
    #define CMDLINE_MAX_ARGS            8
#endif

//*****************************************************************************
//
// Defines the size of the buffer that holds the command line.
//
//*****************************************************************************
#ifndef CMD_BUF_SIZE
    #define CMD_BUF_SIZE                64
#endif

//*****************************************************************************
//
// Flags
//
//*****************************************************************************
#define CMD_FLG_BUFF_READY              0x01
#define CMD_FLG_CTRL_CHAR               0x02
#define CMD_FLG_DISCARD                 0x04


//*****************************************************************************
//
// The buffer that holds the command line.
//
//*****************************************************************************
static char  g_cCmdBuf[CMD_BUF_SIZE];
static char  g_cCmdLst[CMD_BUF_SIZE];
static char  g_cCmdFlags = 0;

// user defined functions
extern void CmdShowPrompt(void);
extern void CmdShowStatus(int nStatus);

//*****************************************************************************
//
//! Process a command line string into arguments and execute the command.
//!
//! \param pcCmdLine points to a string that contains a command line that was
//! obtained by an application by some means.
//!
//! This function will take the supplied command line string and break it up
//! into individual arguments.  The first argument is treated as a command and
//! is searched for in the command table.  If the command is found, then the
//! command function is called and all of the command line arguments are passed
//! in the normal argc, argv form.
//!
//! The command table is contained in an array named <tt>g_sCmdTable</tt> which
//! must be provided by the application.
//!
//! \return Returns \b CMDLINE_BAD_CMD if the command is not found,
//! \b CMDLINE_TOO_MANY_ARGS if there are more arguments than can be parsed.
//! Otherwise it returns the code that was returned by the command function.
//
//*****************************************************************************
int
CmdLineProcess(char *pcCmdLine)
{
    static char *argv[CMDLINE_MAX_ARGS + 1];
    char *pcChar;
    int argc;
    int bFindArg = 1;
    int bOpenString = 0;
    tCmdLineEntry *pCmdEntry;

    //
    // Initialize the argument counter, and point to the beginning of the
    // command line string.
    //
    argc = 0;
    pcChar = pcCmdLine;

    //
    // Advance through the command line until a zero character is found.
    //
    while(*pcChar)
    {
        //
        // Check what do we have here
        //
        switch(*pcChar)
        {
            //
            // There is an open \", the argument is a string!
            //
            case '\"':

                //
                // If its marked as open string
                //
                if(bOpenString)
                {
                    //
                    // Ignore last \" character
                    //
                    *pcChar = '\0';

                    //
                    // Close string
                    //
                    bOpenString = 0;

                    //
                    // We found an arg
                    //
                    bFindArg = 1;
                }
                else
                {
                    //
                    // Mark to find match \"
                    //
                    bOpenString = 1;
                }
            break;

            //
            // If there is a space.
            //
            case ' ':

                //
                // If open string is marked, ignore spaces
                //
                if(!bOpenString)
                {
                    //
                    // Replace it with a zero, and set the flag
                    // to search for the next argument.
                    //
                    *pcChar = 0;
                    bFindArg = 1;
                }

            break;

            //
            // Otherwise it is not a space, so it must be a character that is part
            // of an argument.
            //
            default:
                //
                // If bFindArg is set, then that means we are looking for the start
                // of the next argument.
                //
                if(bFindArg)
                {
                    //
                    // As long as the maximum number of arguments has not been
                    // reached, then save the pointer to the start of this new arg
                    // in the argv array, and increment the count of args, argc.
                    //
                    if(argc < CMDLINE_MAX_ARGS)
                    {
                        argv[argc] = pcChar;
                        argc++;
                        bFindArg = 0;
                    }

                    //
                    // The maximum number of arguments has been reached so return
                    // the error.
                    //
                    else
                    {
                        return(CMDLINE_TOO_MANY_ARGS);
                    }
                }
            break;
        }

        //
        // Advance to the next character in the command line.
        //
        pcChar++;
    }

    //
    // did we get an open string?
    //
    if(bOpenString)
    {
        //
        // Command line ends before closing \"
        //
        return(CMDLINE_INVALID_ARG);
    }

    //
    // If one or more arguments was found, then process the command.
    //
    if(argc)
    {
        //
        // Start at the beginning of the command table, to look for a matching
        // command.
        //
        pCmdEntry = &g_sCmdTable[0];

        //
        // Search through the command table until a null command string is
        // found, which marks the end of the table.
        //
        while(pCmdEntry->pcCmd)
        {
            //
            // If this command entry command string matches argv[0], then call
            // the function for this command, passing the command line
            // arguments.
            //
            if(!strcmp(argv[0], pCmdEntry->pcCmd))
            {
                return(pCmdEntry->pfnCmd(argc, argv));
            }

            //
            // Not found, so advance to the next entry.
            //
            pCmdEntry++;
        }
    }

    //
    // Fall through to here means that no matching command was found, so return
    // an error.
    //
    return(CMDLINE_BAD_CMD);
}



//*****************************************************************************
//
// Refresh cmd line task
//
//*****************************************************************************
void CmdRefresh(void)
{
    int nStatus;

    //
    // Check if serial data is ready
    //
    if((g_cCmdFlags & CMD_FLG_BUFF_READY) == 0)
    {
        // abort
        return;
    }

    //
    // copy last cmd
    //
    strcpy(g_cCmdLst, g_cCmdBuf);

    //
    // Pass the line from the user to the command processor.
    // It will be parsed and valid commands executed.
    //
    nStatus = CmdLineProcess(g_cCmdBuf);

    // clear buffer
    memset(g_cCmdBuf, 0x00, CMD_BUF_SIZE);

    // show status
    CmdShowStatus(nStatus);

    // mark as processed
    g_cCmdFlags &= ~CMD_FLG_BUFF_READY;

    // show command line
    CmdShowPrompt();
}



//*****************************************************************************
//
// This is the uart handler for CMD line data
//
//*****************************************************************************
void Uart0Handler(void)
{
    static unsigned long ulStatus;

    //
    // Get the interrrupt status.
    //
    ulStatus = UARTIntStatus(UART0_BASE, 1);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {
        static char  Data;
        static char *p = g_cCmdBuf;

        //
        // Read the next character from the UART0 and write it back to the UART1.
        //
        Data = UARTCharGetNonBlocking(UART0_BASE);

        //
        // check if discard was requested
        //
        if(g_cCmdFlags & CMD_FLG_DISCARD)
        {
            // stop discarding
            g_cCmdFlags &= ~CMD_FLG_DISCARD;

            // abort this one
            return;
        }

        //
        // check if buffer is ready for cmd process
        //
        if(g_cCmdFlags & CMD_FLG_BUFF_READY)
        {
            // abort receiving
            return;
        }

        //
        // If we are expecting a control char
        //
        if(g_cCmdFlags & CMD_FLG_CTRL_CHAR)
        {
            auto char *l;

            // done
            g_cCmdFlags &= ~CMD_FLG_CTRL_CHAR;

            // lets see what we got
            switch(Data)
            {
                // UP arrow
                case 'A':

                    // last buffer
                    l = g_cCmdLst;

                    // insert if available
                    while((p < (g_cCmdBuf + CMD_BUF_SIZE - 1)) && *l)
                    {
                        // insert in current position
                        *p = *l;

                        // echo
                        UARTCharPut(UART0_BASE, *l);

                        // next
                        p++;
                        l++;
                    }

                    // ensure line termination
                    *p = '\0';

                break;

                // LEFT
                case 'D':

                    // move cursor back
                    if(p > g_cCmdBuf)
                    {
                        // go back
                        p--;

                        // go back
                        UARTCharPut(UART0_BASE, '\b');
                    }

                break;

                // RIGHT
                case 'C':

                    // move cursor forward
                    if(*p)
                    {
                        // re-print the character
                        UARTCharPut(UART0_BASE, *p);

                        // go forward
                        p++;
                    }

                break;
            }

            /* Forget everything */
            return;
        }

        //
        // check data
        switch(Data)
        {
            // bkspc code
            case '\b': // default backspace escape
            case 0x7F: // PUTTY backspace

                // remove char in buffer
                if(p > g_cCmdBuf)
                {
                    // go back
                    p--;

                    // ensure line termination
                    *p = '\0';

                    // go back, erase char, go back again
                    UARTCharPut(UART0_BASE, '\b');
                    UARTCharPut(UART0_BASE, ' ');
                    UARTCharPut(UART0_BASE, '\b');
                }

            break;

            // control char
            case 0x1B:

                // discard next one
                g_cCmdFlags |= CMD_FLG_DISCARD;

                // expect control char
                g_cCmdFlags |= CMD_FLG_CTRL_CHAR;

            break;

            // carriage return
            case '\r':
            case '\n':

                // mark as buff ready
                g_cCmdFlags |= CMD_FLG_BUFF_READY;

                // ensure line termination
                // made with memset();
                //*p = '\0';

                // back to begining
                p = g_cCmdBuf;

                // new line
                UARTCharPut(UART0_BASE, '\r');
                UARTCharPut(UART0_BASE, '\n');

            break;

            // any char
            default:

                // check valid ascii range
                if((Data >= ' ') && (Data <= '~'))
                {
                    // insert if available
                    if(p < (g_cCmdBuf + CMD_BUF_SIZE - 1))
                    {
                        // insert in current position
                        *p = Data;

                        // next
                        p++;

                        // echo
                        UARTCharPut(UART0_BASE, Data);
                    }
                }

            break;
        }

    }
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
