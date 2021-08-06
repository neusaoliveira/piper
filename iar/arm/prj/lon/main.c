#include "globals.h"
#include "app/eth/eth_udp.h"
#include "etc/system.h"
#include "drv/rit128x96x4.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "lib/ethernet.h"
#include "lib/flash.h"
#include "lib/gpio.h"
#include "lib/interrupt.h"
#include "lib/sysctl.h"
#include "lib/systick.h"
#include "lib/utils/lwiplib.h"
#include "lib/utils/uartstdio.h"
#include "srv/display.h"
#include "srv/pid.h"
#include "utl/math.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0


/******************************************************************************/
/*                                                                            */
/* Prototypes                                                                 */
/*                                                                            */
/******************************************************************************/
void SetupHardware(void);
void ShowStartup(const char *pStr);

/******************************************************************************/
/*                                                                            */
/* Variáveis                                                                  */
/*                                                                            */
/******************************************************************************/

/* Flags gerais */
unsigned long g_ulFlags;

/* Clock do sistema */
unsigned long g_ulSystemClock;

/* Timers */
static unsigned long  g_ucControllerCount = 0;

/* Computer IP Address*/
static struct ip_addr Pc_ip;

// Data IO
static unsigned char rxbuf[64];  // buffer for incoming UDP packet (from Matlab)
static unsigned char txbuf[64];  // buffer for outgoing UDP packet (to Matlab)

typedef struct
{
    /* Dados de Navegação do Modelo Longitudinal */
	float Vt;
	float alpha;
        float q;
        float theta;
        float altitude; 
	
}StDataLongitudinal;
static StDataLongitudinal dataLongitudinal;
static float vt_ref;
static float theta_ref;
static float h_ref;


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif



//*****************************************************************************
//
// The interrupt handler for the SysTick interrupt. (1000 HZ)
//
//*****************************************************************************
void
SysTickIntHandler(void)
{    
    //
    // System tick, 1ms
    //
    SysTick(1);
  
  
    //
    // Call the lwIP timer handler.
    //
    lwIPTimer(1);
    
    //
    // Indicate that a timer interrupt has occurred.
    //
    HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) = 1;
    
    //
    // Increment sensor counter
    //
    g_ucControllerCount++;

    //
    // See if 1 second has passed since the last sensor update.
    //
    if(g_ucControllerCount >= 50)
    {
        //
        // Mark to read sensors
        //
        HWREGBITW(&g_ulFlags, FLAG_CONTROLE) = 1;

        //
        // Restart counter
        //
        g_ucControllerCount = 0;
    }
    
}

//*****************************************************************************
//
// This example demonstrates the use of the Ethernet Controller.
//
//*****************************************************************************

int
main(void)
{
    StPidf pidTheta;
    StPidf pidH;
    StPidf pidVT;
    static unsigned long now, timeChange, lastTime; 
    static float aileron, elevator, rudder, throttle;
    float Kq, r, r_;
    
    Kq = 0.0877;
    vt_ref = theta_ref = aileron = elevator = rudder = throttle = 0.0f;    
    now = timeChange = lastTime = 0;
    
    //
    // Setup hardware
    //
    SetupHardware();
    
    //
    // Initialize the OSRAM OLED display.
    //
    RIT128x96x4Init(3500000);      
    
    
    //
    // Initialize UDP
    //
    UdpBegin(45004);    
    IP4_ADDR(&Pc_ip,192,168,1,107);
    
    //
    // Initialize PIDs
        // Theta
        PidfClear(&pidTheta);
        pidTheta.Kp = -0.45319;
        pidTheta.Ki = -0.66623;
        // Altitude
        PidfClear(&pidH);
        pidH.Kp = 0.0172410;
        pidH.Ki = 0.0059742;    
        //Aerodynamic Velocity    
        PidfClear(&pidVT);
        pidVT.Kp = 0.030269;
        pidVT.Ki = 0.022003;
        
    ShowStartup("Longitudinal Controller");
    
    while(1)
    {               
        if(UdpParsePacket())
        {
            HWREGBITW(&g_ulFlags, FLAG_MATLAB_START) = 1;
          
            // extract flight parameters of interest from Matlab UDP packet
            UdpRead(rxbuf, ETH_UDP_TX_PACKET_MAX_SIZE);
            
            now = SysNow();
            if(lastTime)
                timeChange = (now - lastTime);
            else
                timeChange = 10;
            
            memcpy(&dataLongitudinal.Vt,        &rxbuf[0],  4);
            memcpy(&dataLongitudinal.alpha,     &rxbuf[4],  4);
            memcpy(&dataLongitudinal.q,         &rxbuf[8],  4);
            memcpy(&dataLongitudinal.theta,     &rxbuf[12], 4);
            memcpy(&dataLongitudinal.altitude,  &rxbuf[16], 4);
            memcpy(&vt_ref,                     &rxbuf[20], 4);
            memcpy(&h_ref,                      &rxbuf[24], 4);          
            
            /* Longitudinal */
            pidVT.SetPoint = vt_ref;
            throttle = PidfExecute(&pidVT, dataLongitudinal.Vt, (timeChange*0.001f));  
            
            if( absf(h_ref-dataLongitudinal.altitude) > 5.0f )
            {
                if ( h_ref-dataLongitudinal.altitude > 0)
                    theta_ref =  ToRad(5);
                else
                    theta_ref = -ToRad(5);
                           
                pidTheta.SetPoint = theta_ref + 0.0511f;
                r = PidfExecute(&pidTheta, dataLongitudinal.theta, (timeChange*0.001f));
                elevator = r - Kq*dataLongitudinal.q;
                
            }else{
                
                pidH.SetPoint = h_ref;
                r_ = PidfExecute(&pidH, dataLongitudinal.altitude, (timeChange*0.001F));   
                
                pidTheta.SetPoint = r_;
                r = PidfExecute(&pidTheta, dataLongitudinal.theta, (timeChange*0.001F));
                elevator = r - Kq*dataLongitudinal.q;
            }           
            
            // Preparando pacote de envio
            memcpy(&txbuf[0],   &throttle, 4);
            memcpy(&txbuf[4],   &elevator, 4);
            memcpy(&txbuf[8],   &aileron,  4);            
            memcpy(&txbuf[12],  &rudder,   4);  
            
            UdpWrite(Pc_ip, 45000, txbuf, 16); 
            lastTime = now;
        }
    }
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void SetupHardware(void)
{
    unsigned long ulUser0, ulUser1;
    unsigned char pucMACArray[8];
	
    //
    // Processos Basics
    //
        
        //
        // Set the clocking to run at 50MHz from the PLL.
        //
        SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_8MHZ);

        //
        // Get the system clock speed.
        //
        g_ulSystemClock = SysCtlClockGet();

    //
    // All used peripherals
    //

        //
        // Enable the peripherals used by the application.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    //
    // Some GPIOs
    //

        //
        // Configure the 4 directional buttons
        //
        GPIOPinTypeGPIOInput(GPIO_PORTE_BASE,
                             GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
        GPIOPadConfigSet(GPIO_PORTE_BASE,
                         GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

        //
        // Configure the select button
        //
        GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA,
                         GPIO_PIN_TYPE_STD_WPU);
        
        //
        // Turn LED on
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);    
    

    //
    // Ethernet
    //

        //
        // Enable and Reset the Ethernet Controller.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);
        SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);
        
        //
        // Enable Port F for Ethernet LEDs.
        //  LED0        Bit 3   Output
        //  LED1        Bit 2   Output
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
        
        //
        // Configure the hardware MAC address for Ethernet Controller filtering of
        // incoming packets.
        //
        // For the LM3S6965 Evaluation Kit, the MAC address will be stored in the
        // non-volatile USER0 and USER1 registers.  These registers can be read
        // using the FlashUserGet function, as illustrated below.
        //
        FlashUserGet(&ulUser0, &ulUser1);
        if((ulUser0 == 0xffffffff) || (ulUser1 == 0xffffffff))
        {
                //
                // We should never get here.  This is an error if the MAC address has
                // not been programmed into the device.  Exit the program.
                //
                RIT128x96x4Enable(1000000);
                RIT128x96x4StringDraw("MAC Address", 0, 16, 15);
                RIT128x96x4StringDraw("Not Programmed!", 0, 24, 15);
                while(1)
                {
                }
        }

        //
        // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
        // address needed to program the hardware registers, then program the MAC
        // address into the Ethernet Controller registers.
        //
        pucMACArray[0] = ((ulUser0 >>  0) & 0xff);
        pucMACArray[1] = ((ulUser0 >>  8) & 0xff);
        pucMACArray[2] = ((ulUser0 >> 16) & 0xff);
        pucMACArray[3] = ((ulUser1 >>  0) & 0xff);
        pucMACArray[4] = ((ulUser1 >>  8) & 0xff);
        pucMACArray[5] = ((ulUser1 >> 16) & 0xff);
                
        
        //
        // Initialze the lwIP library, using DHCP.
        //
        lwIPInit( pucMACArray, inet_addr("192.168.1.101"), inet_addr("255.255.255.0"), 
                        inet_addr("192.168.1.1"), IPADDR_USE_STATIC);

    
    //
    // Clock tick
    //

        //
        // Configure SysTick to periodically interrupt.
        //
        SysTickPeriodSet(g_ulSystemClock / 1000);
        SysTickIntEnable();
        SysTickEnable();    
   

    //
    // Enable all interrupts
    //

        //
        // Set the interrupt priorities.  We set the SysTick interrupt to a higher
        // priority than the Ethernet interrupt to ensure that the file system
        // tick is processed if SysTick occurs while the Ethernet handler is being
        // processed.  This is very likely since all the TCP/IP and HTTP work is
        // done in the context of the Ethernet interrupt.
        //
        IntPriorityGroupingSet(4);
        IntPrioritySet(INT_ETH, ETHERNET_INT_PRIORITY);
        IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);
        
        //
        // Enable processor interrupts.
        //
        IntMasterEnable();

    //
    // EOF
    //
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void ShowStartup(const char *pStr)
{
    static byte Line = 0;

    /* show */
    RIT128x96x4StringDraw(pStr, 0, Line, 0x0F);

    /* Next line */
    Line += 8;

    /* check */
    if(Line >= 96)
    {
        /* reset... */
        Line = 0;
    }
}