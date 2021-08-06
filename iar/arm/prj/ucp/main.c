#include "formpeg.h"
#include "globals.h"
#include "app/eth/eth_udp.h"
#include "app/gdn/gdn.h"
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
#include "lib/uart.h"
#include "lib/utils/lwiplib.h"
#include "lib/utils/uartstdio.h"
#include "srv/display.h"
#include "srv/pid.h"
#include "srv/wof.h"
#include "utl/crc.h"
#include "utl/math.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0
#define PEG_PRT_TAM_BUFFER      0x80

/******************************************************************************/
/*                                                                            */
/* Prototypes                                                                 */
/*                                                                            */
/******************************************************************************/
void ComputeController(int timeSample);
float LineOfSight(void);
byte PegPrtRecvPkt(byte *pData);
void PegPrtSendPkt(void);
void SetupController(void);
void SetupGuidance(void);
void SetupHardware(void);
void SwitchWaypoints(void);

/******************************************************************************/
/*                                                                            */
/* Variáveis                                                                  */
/*                                                                            */
/******************************************************************************/

/* Flags gerais */
unsigned long g_ulFlags;

/* Clock do sistema */
unsigned long g_ulSystemClock;

/* Computer IP Address */
static struct ip_addr Pc_IP;

/* Data IO */
static byte PegPrtBufferRx[PEG_PRT_TAM_BUFFER];  // buffer for incoming UDP packet (from Matlab)
static byte PegPrtBufferTx[PEG_PRT_TAM_BUFFER];  // buffer for outgoing UDP packet (to Matlab)
static StPegasusAct TxPegasus;
static StPegasusNav RxPegasus;

/* Control */
static float Kp, Kq, Kr, Kpsi;
static StPidf pidPhi, pidTheta, pidH, pidVT;
static StWoFilter wofYaw;

/* Guidance */
typedef struct
{
    float vtR;
    float hR;
    float psiR;	
}GuidanceRef;
static GuidanceRef ref;
static WP *waypoints;
static bool stop;
static int lambda;
static int phases;

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
    
}

//*****************************************************************************
//
// Delay for a multiple of the system tick clock rate. (header at globals)
//
//*****************************************************************************
void
Delay(unsigned long ulCount)
{
    //
    // Loop while there are more clock ticks to wait for.
    //
    while(ulCount--)
    {
        //
        // Wait until a SysTick interrupt has occurred.
        //
        while(!HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK))
        {
        }

        //
        // Clear the SysTick interrupt flag.
        //
        HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) = 0;
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
    static unsigned long now, timeChange, lastTime; 
    now = timeChange = lastTime = 0x00;   
        
    /* Setup */
    SetupHardware();
    SetupController();
    SetupGuidance();
    
    /* Send a welcome message to the UART.*/
    UARTprintf("-- Welcome [] --\r\n");
    
    /* Initialize UDP */   
    UdpBegin(45004);    
    IP4_ADDR(&Pc_IP,192,168,1,107);   
    
    while(1)
    {               
        if(UdpParsePacket())
        {            
            now = SysNow();
            if(lastTime)
                timeChange = (now - lastTime);
            else
                timeChange = 0x0A;
            
            /* Extract flight parameters of interest from Matlab UDP packet */
            UdpRead(PegPrtBufferRx, ETH_UDP_TX_PACKET_MAX_SIZE);
            PegPrtRecvPkt(PegPrtBufferRx); 
            
            SwitchWaypoints();
            ComputeController(timeChange);
            
            /* Pack control surfaces of the UDP Matlab packet */
            PegPrtSendPkt();            
            lastTime = now;
        }
    }
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void SetupController(void)
{   
    auto float tau;
    
    /* Lateral-Directional */
        tau             = 1.0f;
        
        Kp              = 0.1190f;
        Kr              = 0.1050f; 
        Kpsi            = 0.2716f;
        
        // Initialize PIDs    
        PidfClear(&pidPhi);
        pidPhi.Kp       = -0.090681f;
        pidPhi.Ki       = -0.019958f;
        
        // Initialize Washout Filter 
        WoFilterInit(&wofYaw, 0x64, tau);
        
    /* Longitudinal */    
        Kq              = 0.0877f;
        
        // Initialize PIDs        
        PidfClear(&pidTheta);
        pidTheta.Kp     = -0.45319;
        pidTheta.Ki     = -0.66623;        

        PidfClear(&pidH);
        pidH.Kp         = 0.0172410;
        pidH.Ki         = 0.0059742;  
    
        PidfClear(&pidVT);
        pidVT.Kp        = 0.030269;
        pidVT.Ki        = 0.022003;
    
    /* Control Surfaces */    
        TxPegasus.Ail   =  0.0000f; 
        TxPegasus.Ele   = -0.0703f;
        TxPegasus.Rud   =  0.0000f;
        TxPegasus.Trtl  =  0.0416f; 
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void SetupGuidance(void)
{ 
    auto struct StWaypoint wpn; 
    auto char i;
    auto float R, c1, c2, s1, s2;
    
    R = 1000.0f;
    c1 = R*cos(2*THE_PI/5);
    c2 = R*cos(THE_PI/5);
    s1 = R*sin(2*THE_PI/5);
    s2 = R*sin(4*THE_PI/5);
  
    auto float wp[7][4] = {{0  ,  0  , 400 , 20},
                           {R  ,  0  , 400 , 20},
                           {-c2, s2  , 420 , 20},
                           {c1 , -s1 , 380 , 20},
                           {c1 ,  s1 , 420 , 20},
                           {-c2, -s2 , 380 , 20},
                           {R  ,  0  , 400 , 20}};

    /*
    auto float wp[6][4] = {{ 0   ,  0  , 400, 20},
                           {800  , 800 , 400, 20},
                           {-800 , 800 , 425, 20},
                           {800  ,-800 , 375, 20},
                           {-800 ,-800 , 350, 20},
                           {800  , 800 , 400, 20}};
  
    */
    waypoints = WaypointBegin();     
    for(i=0; i<7; i++)
    {   
        wpn.x  = wp[i][0];
        wpn.y  = wp[i][1];
        wpn.z  = wp[i][2];
        wpn.vt = wp[i][3];
             
        WaypointInsert(waypoints, wpn);        
    }       
    lambda = 10;
    phases = 1;
    stop = false;    
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
        SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

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
    // UART0 for output
    //

            //
            // Configure pin for UART
            //
            GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);


        #if 0
            //
            // Configure the first UART for 115,200, 8-N-1 operation.
            //
            UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                 UART_CONFIG_PAR_NONE));
            UARTEnable(UART0_BASE);
        #else
            //
            // Initialize the UART as a console for text I/O.
            //
            UARTStdioInit(0);

            // enable interrupts
            //IntEnable(INT_UART0);
            //UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
        #endif

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
byte PegPrtRecvPkt(byte *pData)
{
    auto short i;
    auto short Pkt;

    // check first fields
    if(*pData != 'P') { return(ERRO); } pData++;
    if(*pData != 'N') { return(ERRO); } pData++;

    // packets
    Pkt = *((short*)pData);             pData += sizeof(short);

    // check all 4 packets
    for(i = 0x01; i <= 0x80; i <<= 1)
    {
        /* check if this packet is present */
        if(Pkt & i)
        {
            /* Id */
            switch(*pData)
            {
                /* ax ay az */
                case '2':
                    // skip Id
                    pData++;
                    // copy
                    RxPegasus.Acc[0]    = *((float*)pData); pData += sizeof(float);
                    RxPegasus.Acc[1]    = *((float*)pData); pData += sizeof(float);
                    RxPegasus.Acc[2]    = *((float*)pData); pData += sizeof(float);
                                                            pData += sizeof(float);
                    // skip CRC
                    pData++;
                break;

                /* RollRate PitchRate YawRate */
                case '3':
                    // skip Id
                    pData++;
                    // copy
                    RxPegasus.RolRate   = *((float*)pData); pData += sizeof(float);
                    RxPegasus.PthRate   = *((float*)pData); pData += sizeof(float);
                    RxPegasus.YawRate   = *((float*)pData); pData += sizeof(float);
                                                            pData += sizeof(float);
                    // skip CRC
                    pData++;
                break;

                /* Roll Pitch Yaw */
                case '4':
                    // skip Id
                    pData++;
                    // copy
                    RxPegasus.Rol       = *((float*)pData); pData += sizeof(float);
                    RxPegasus.Pth       = *((float*)pData); pData += sizeof(float);
                    RxPegasus.Yaw       = *((float*)pData); pData += sizeof(float);
                                                            pData += sizeof(float);
                    // skip CRC
                    pData++;
                break;

                /* North East Down */
                case '7':
                    // skip Id
                    pData++;
                    // copy
                    RxPegasus.North     = *((float*)pData); pData += sizeof(float);
                    RxPegasus.East      = *((float*)pData); pData += sizeof(float);
                    RxPegasus.Alt       = *((float*)pData); pData += sizeof(float);
                                                            pData += sizeof(float);
                    // skip CRC
                    pData++;
                break;
                
                /* Alpha Beta VT */
                case '5':
                    // skip Id
                    pData++;
                    // copy
                    RxPegasus.VT        = *((float*)pData); pData += sizeof(float);
                    RxPegasus.Alpha     = *((float*)pData); pData += sizeof(float);
                    RxPegasus.Beta      = *((float*)pData); pData += sizeof(float);                    
                                                            pData += sizeof(float);
                    // skip CRC
                    pData++;
                break;               

                /*   hoje ? sim : não */
                default:
                    // ignore alllll
                    pData++;
                    pData+= 4 * sizeof(float);
                    pData++;
                    return(ERRO);
            }
        }
    }

    // EOF
    if(*pData != '!') {return(ERRO);}

    // received a packet
    //HWREGBITW(&g_ulFlags, FLAG_PEGASUS) = 1;

    // ok ok ratinho
    return(SUCESSO);
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void PegPrtSendPkt(void)
{    
    static byte *p;
    static byte *pLrc;
    static byte  Lrc;

    // fisrt byte
    p = PegPrtBufferTx;

    // STX
    *p = 'P';                                   p++;
    *p = 'A';                                   p++;    
    *((short*)p) = 0x01;                        p += sizeof(short);
    
    // Group 1 : dT de da dr
        *p              = '1';                  p++;
        pLrc            = p;
        
        *((float*)p)    = TxPegasus.Trtl;       p+=sizeof(float);
        *((float*)p)    = TxPegasus.Ele;        p+=sizeof(float);
        *((float*)p)    = TxPegasus.Ail;        p+=sizeof(float);        
        *((float*)p)    = TxPegasus.Rud;        p+=sizeof(float); 
        // Group CRC
        Lrc = CrcPegasus(pLrc, 0, p - pLrc);
        *p              = Lrc;                  p++;
        
    // Group 6 : VT_wp H_wp \psi_wp
        *p              = '6';                  p++;
        pLrc            = p;
        
        *((float*)p)    = ref.vtR;              p+=sizeof(float);
        *((float*)p)    = ref.hR;               p+=sizeof(float);
        *((float*)p)    = LineOfSight();        p+=sizeof(float);        
        *((float*)p)    = -999;                 p+=sizeof(float); 
        // Group CRC
        Lrc = CrcPegasus(pLrc, 0, p - pLrc);
        *p              = Lrc;                  p++; 

    // ETX
    *p = stop ? '#' : '!';                  p++;

    // Uses Lrc as counter for the 0..41 bytes to send
    Lrc = (byte)(p - PegPrtBufferTx);    
    UdpWrite(Pc_IP, 45000, PegPrtBufferTx, Lrc);    

}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void SwitchWaypoints(void)
{
    static struct StWaypoint WPn; 
    static struct StWaypoint WPnext; 
    
    static float aux, m, er, Nr, Er, d, dx, dy;
    static float ds, eMax;
    aux = m = er = Nr = Er = d = dx = dy = 0.0f;
    ds = eMax = 0.0f;
      
    getWaypoint(waypoints, &WPn);
    getNextWaypoint(waypoints, &WPnext);     
    
    if(!WaypointAvailable(waypoints))
    {                
        stop = true;
        UARTprintf("Simulacao encerrada!\n");
        /*set_param('PegasusAutopilot', 'SimulationCommand', 'stop'); */ 
    }else{    
        switch (phases){
        case 1:
            ref.psiR = LineOfSight();
            ref.vtR = WPn.vt;
            ref.hR = WPn.z;
            
            if (ToDeg(absf(RxPegasus.Yaw - ref.psiR)) <= 5)
                phases++;
        break;

        case 2:
            if ((WPnext.x -  WPn.x))
            {
                m   = (WPnext.y -  WPn.y) / (WPnext.x -  WPn.x);
                aux = absf(RxPegasus.East - m*RxPegasus.North - (WPn.y - m*WPn.x));
                er  = aux/sqrt((m*m)+1);    
                Nr  = RxPegasus.North - er*RxPegasus.Yaw;
                Er  = RxPegasus.East - er*RxPegasus.Yaw;      

                dx = (WPnext.x - Nr);
                dy = (WPnext.y - Er);        
                d   = sqrt( dx*dx + dy*dy );

                dx = (WPnext.x - WPn.x);
                dy = (WPnext.y - WPn.y);
                ds  = sqrt( dx*dx + dy*dy );        
                eMax = (d*((70+40)*0.5))/ds;        

                if (er > eMax) 
                    ref.psiR = LineOfSight();
            }else{               
                dx = (WPnext.x - RxPegasus.North);
                dy = (WPnext.y - RxPegasus.East);        
                d   = sqrt( dx*dx + dy*dy );
                ref.psiR = LineOfSight();
            }            
            ref.vtR = WPnext.vt;
            ref.hR = WPnext.z;
            
            if (d<=lambda)
            {
                UARTprintf("Alcancou waypoint: [d]\r\n");             
                while(!WaypointRemove(waypoints));
                phases = 1;
            }
        break;
        }
    }
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void ComputeController(int timeSample)
{
    static float r, r_;
    static float thetaRef;
    
    /* Latero-Directional */
        r_ = (LineOfSight() - RxPegasus.Yaw) * Kpsi;    
        pidPhi.SetPoint = r_;
        r = PidfExecute(&pidPhi, RxPegasus.Rol, (timeSample*0.001f));
        TxPegasus.Ail = r - Kp*RxPegasus.RolRate;
          
        WoFilterExecute(&wofYaw, RxPegasus.YawRate);
        r = Kr*wofYaw.Output;
        TxPegasus.Rud = 0.0f - r;          
    
    /* Longitudinal */
        pidVT.SetPoint = ref.vtR; 
        TxPegasus.Trtl = PidfExecute(&pidVT, RxPegasus.VT, (timeSample*0.001f));
        TxPegasus.Trtl = limf(TxPegasus.Trtl, -0.0416f, 1-0.0416f);
          
        if( absf(ref.hR - RxPegasus.Alt) > 5.0f )
        {
            if ( ref.hR - RxPegasus.Alt > 0)
                thetaRef =  ToRad(5);
            else
                thetaRef = -ToRad(5);
                       
            pidTheta.SetPoint = thetaRef + 0.0511f;
            r = PidfExecute(&pidTheta, RxPegasus.Pth, (timeSample*0.001f));
            TxPegasus.Ele = r - Kq*RxPegasus.PthRate;      
            
        }else{
            
            pidH.SetPoint = ref.hR;
            r_ = PidfExecute(&pidH, RxPegasus.Alt, (timeSample*0.001F));   
            
            pidTheta.SetPoint = r_;
            r = PidfExecute(&pidTheta, RxPegasus.Pth, (timeSample*0.001F));
            TxPegasus.Ele = r - Kq*RxPegasus.PthRate;    
        } 
            
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
float LineOfSight()
{    
    static struct StWaypoint WPnext;    
    static float dx, dy, psiR, LoS, aux, m;
    
    getNextWaypoint(waypoints, &WPnext);   
    dx = (WPnext.x - RxPegasus.North);
    dy = (WPnext.y - RxPegasus.East );    
    LoS = atan2(dy, dx);
    
    if((absf(LoS - RxPegasus.Yaw)) > THE_PI)
    {
        m = (int)(RxPegasus.Yaw/(2*THE_PI));
        psiR = LoS + (2*THE_PI)*m;
        
        if((absf(psiR - RxPegasus.Yaw)) > THE_PI)
        {
            aux = psiR - (2*THE_PI);
            if ((absf(aux - RxPegasus.Yaw)) < THE_PI)
                LoS = aux;
            else
            {
                aux = psiR + (2*THE_PI);
                if ((absf(aux - RxPegasus.Yaw)) < THE_PI)
                    LoS = aux;
            }
        }else
        {
            LoS = psiR;    
        }
    }
    
    return (LoS);
}