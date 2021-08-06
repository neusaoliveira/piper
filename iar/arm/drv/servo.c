/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  dsp\servo.c                                                               */
/*                                                                            */
/*  Gerenciamento e controle de servomotores                                  */
/*                                                                            */
/*  2010-03-11 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/


#include "etc/defines.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "lib/interrupt.h"
#include "lib/sysctl.h"
#include "lib/timer.h"
#include "lib/gpio.h"
#include "drv/servo.h"
#include "utl/math.h"
#include "conf.h"

#ifndef CNF_SERVO
    #error "servo.c: Modulo nao ativo."
#endif

#ifndef SVO_TIMER
    #error "servo.c: Timer SVO_TIMER nao definido!"
#endif

#ifndef SVO_GPIO
    #error "servo.c: Gpio SVO_GPIO nao definido!"
#endif

#if SVO_TIMER == 0
    #define SVO_TMRBASE                 TIMER0_BASE
    #define SVO_TMRPERIPH               SYSCTL_PERIPH_TIMER0
    #define SVO_TMRINT                  INT_TIMER0A
#endif
#if SVO_TIMER == 1
    #define SVO_TMRBASE                 TIMER1_BASE
    #define SVO_TMRPERIPH               SYSCTL_PERIPH_TIMER1
    #define SVO_TMRINT                  INT_TIMER1A
#endif
#if SVO_TIMER == 2
    #define SVO_TMRBASE                 TIMER2_BASE
    #define SVO_TMRPERIPH               SYSCTL_PERIPH_TIMER2
    #define SVO_TMRINT                  INT_TIMER2A
#endif

#if SVO_GPIO == 'B'
    #define SVO_GPIOPERIPH              SYSCTL_PERIPH_GPIOB
    #define SVO_GPIOBASE                GPIO_PORTB_BASE
#endif

/******************************************************************************/
/*                                                                            */
/* Internal structures                                                        */
/*                                                                            */
/******************************************************************************/

/* servo struct */
typedef struct
{
    /* value of pulse */
    long Pwm;

    /* pin connected */
    long Pin;
}StServo;

/******************************************************************************/
/*                                                                            */
/* Internal variables                                                         */
/*                                                                            */
/******************************************************************************/

/* array of avaliable servos */
StServo  Servos[SVO_MAX_SERVOS];

/* current servo pointer */
StServo *pServo;

/* current servo index */
byte     iServo;


/******************************************************************************/
/*                                                                            */
/* Internal Functions                                                         */
/*                                                                            */
/******************************************************************************/

void SvoTimerIntHandler(void);

/******************************************************************************/
/*                                                                            */
/* Startup function                                                           */
/*                                                                            */
/******************************************************************************/
byte SvoInicia(void)
{
    //
    // Setup internal structures
    //

        /* first servo */
        pServo = Servos;

        /* disables all servos */
        for(iServo = 0; iServo < SVO_MAX_SERVOS; iServo++)
        {
            /* disable */
            pServo->Pwm = 0;

            /* next servo */
            pServo++;
        }

        /* fisrt servo */
        pServo = Servos;
        iServo = SVO_MAX_SERVOS - 1;

    //
    // Configure timer
    //

        // peripheral
        SysCtlPeripheralEnable(SVO_TMRPERIPH);
        TimerConfigure(SVO_TMRBASE, TIMER_CFG_32_BIT_PER);

        // Initial and Match values (1 second)
        TimerLoadSet(SVO_TMRBASE, TIMER_A, F_CPU);

        // enable timer
        TimerEnable(SVO_TMRBASE, TIMER_A);

        // config interrupts
        TimerIntEnable(SVO_TMRBASE, TIMER_TIMA_TIMEOUT);
        IntRegister(SVO_TMRINT, SvoTimerIntHandler);
        IntEnable(SVO_TMRINT);

    //
    // Configure GPIOS
    //

        // gpio
        SysCtlPeripheralEnable(SVO_GPIOPERIPH);

        // each pin is configured independly

    /* initialized successfully */
    return(SUCESSO);
}

/******************************************************************************/
/*                                                                            */
/* Convert degrees to PWM -> converte de -90 a 90 para 1000 a 2000            */
/*                                                                            */
/******************************************************************************/
long SvoDeg2Pwm(float Ang)
{
    /* displacement (0..180) */
    Ang = Ang + 90.0F;

    /* scale (0..1000) */
    Ang = Ang * ((float)(SERVO_MAX - SERVO_MIN)) * 0.0055555555F;

    /* offset (1000..2000) */
    return(ftol(Ang) + SERVO_MIN);
}

/******************************************************************************/
/*                                                                            */
/* config the servo                                                           */
/*                                                                            */
/******************************************************************************/
void SvoConfig(byte Id, long Pin)
{
    /* check for integrity */
    if(Id >= SVO_MAX_SERVOS)
    {
        return;
    }

    /* Access the servo and store the pin connect */
    Servos[Id].Pin = Pin;

    /* clear pin and set output */
    GPIOPinTypeGPIOOutput(SVO_GPIOBASE, Pin);
    GPIOPinWrite(SVO_GPIOBASE, Pin, 0);
}


/******************************************************************************/
/*                                                                            */
/* Set the servo position                                                     */
/*                                                                            */
/******************************************************************************/
void SvoPosition(byte Id, long Pwm)
{
    /* check for integrity */
    if(Id >= SVO_MAX_SERVOS)
    {
        return;
    }

    /* minimum value */
    if(Pwm < SERVO_MIN)
    {
        /* limits to minimum value */
        Pwm = SERVO_MIN;
    }

    /* maximum value */
    if(Pwm > SERVO_MAX)
    {
        /* limits to maximum value */
        Pwm = SERVO_MAX;
    }

    /* converts from us to timer value */
    Pwm = Pwm * (F_CPU / 1000000L);

    /* Access the servo and store the position */
    Servos[Id].Pwm = Pwm;
}

/******************************************************************************/
/*                                                                            */
/* Disable the servo                                                          */
/*                                                                            */
/******************************************************************************/
void SvoDisable(byte Id)
{
    /* check for integrity */
    if(Id >= SVO_MAX_SERVOS)
    {
        return;
    }

    /* Access the servo and store the position */
    Servos[Id].Pwm = 0;

    /* Clear pin */
    GPIOPinWrite(SVO_GPIOBASE, Servos[Id].Pin, 0);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void SvoTimerIntHandler(void)
{
    static dword Pwm;

    //
    // Clear the timer interrupt.
    //
    TimerIntClear(SVO_TMRBASE, TIMER_TIMA_TIMEOUT);

    /* default 100us interval */
    Pwm = 100L * (F_CPU / 1000000L);

    /* there is servo to set? */
    if(!iServo)
    {
        /* first one */
        pServo = Servos;
        iServo = SVO_MAX_SERVOS - 1;

        /* Servo is active? */
        if(pServo->Pwm)
        {
            /* set first one timer */
            Pwm = pServo->Pwm;

            /* Set pin */
            GPIOPinWrite(SVO_GPIOBASE, pServo->Pin, pServo->Pin);
        }
    }
    else
    {
        /* Current Servo is active? */
        if(pServo->Pwm)
        {
            /* clear last servo */
            GPIOPinWrite(SVO_GPIOBASE, pServo->Pin, 0);
        }

        /* Next! */
        pServo++;
        iServo--;

        /* is it valid? */
        if(iServo)
        {
            /* Servo is active? */
            if(pServo->Pwm)
            {
                /* set timer */
                Pwm = pServo->Pwm;

                /* Set pin */
                GPIOPinWrite(SVO_GPIOBASE, pServo->Pin, pServo->Pin);
            }
        }
        else
        {
            /* time-out value of 20ms */
            Pwm = 20000L * (F_CPU / 1000000L);
        }
    }

    //
    // Reload Timer
    //
    TimerLoadSet(SVO_TMRBASE, TIMER_A, Pwm);
}

