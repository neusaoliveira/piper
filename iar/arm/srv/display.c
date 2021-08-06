/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Department of Defense & Airspace                                          */
/*                                                                            */
/*  srv\display.c                                                             */
/*                                                                            */
/*  Implementa funcoes de desenho e escrita no display RIT                    */
/*                                                                            */
/*  2012-11-16 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#include "etc/defines.h"
#include "drv/rit128x96x4.h"
#include "srv/display.h"
#include "globals.h"

//*****************************************************************************
//
// Storage for a local frame buffer.
//
//*****************************************************************************
unsigned char DspFrame[6144];


#if 1
word Variar(int Dx ,int Dy)
{
    if(Dx < 0) Dx = -Dx;
    if(Dy < 0) Dy = -Dy;

    return(Dx > Dy);
}

/******************************************************************************/
/*                                                                            */
/* Traça uma linha genérica na tela                                           */
/*                                                                            */
/******************************************************************************/
void DspLine(word x0, word y0, word x1, word y1, byte Cor)
{
    byte      Pixel;
    int       Dx;
    int       Dy;
    int       Coef;
    short int Incr;

    /* Calcula primeiro para testar */
    Dx  = x1 - x0;
    Dy  = y1 - y0;

    if(Variar(Dx ,Dy))
    {
        /* Mapeamos por X */
        if(x1 < x0)
        {
            auto word Tmp;
            Tmp = x0; x0 = x1; x1 = Tmp;
            Tmp = y0; y0 = y1; y1 = Tmp;
        }
        Dx  = x1 - x0;

        if(y1 < y0)
        {
            Incr = -1;
            Dy  = y0 - y1;
        }
        else
        {
            Incr = 1;
            Dy  = y1 - y0;
        }

        Coef  = Dy;

        /* Plota os pixeis */
        while(x0 <= x1)
        {
            Coef += Dy;

            /* Plota */
            //FncPlot(x0 ,y0);
            if(x0 & 0x01)
            {
                // este pixel
                Pixel |= Cor;

                // plota
                RIT128x96x4ImageDraw(&Pixel, x0 & 0xFFFE, y0, 2, 1);
            }
            else
            {
                // primeiro pixel
                Pixel = Cor << 4;
            }

            if(Coef >= Dx)
            {
                Coef -= Dx;
                y0 += Incr;
            }

            /* Avança em X */
            x0++;
        }
    }
    else
    {
        /* Mapeamos por Y */
        if(y1 < y0)
        {
            auto word Tmp;
            Tmp = x0; x0 = x1; x1 = Tmp;
            Tmp = y0; y0 = y1; y1 = Tmp;
        }
        Dy  = y1 - y0;

        if(x1 < x0)
        {
            Incr = -1;
            Dx  = x0 - x1;
        }
        else
        {
            Incr = 1;
            Dx  = x1 - x0;
        }

        Coef  = Dx;

        /* Plota os Pixeis */
        while (y0 <= y1)
        {
            Coef += Dx;

            /* Plota */
            //FncPlot(x0 ,y0);
            if(x0 & 0x01)
            {
                // este pixel
                Pixel |= Cor;
            }
            else
            {
                // salva pro proximo
                Pixel = Cor << 4;
            }

            // plota
            RIT128x96x4ImageDraw(&Pixel, x0 & 0xFFFE, y0, 2, 1);

            if (Coef >= Dy)
            {
                Coef -= Dy;
                x0 += Incr;
            }

            /* Avança em Y */
            y0++;
        }
    }
}
#endif

//*****************************************************************************
//
// Displays a logo for a specified amount of time.
//
//*****************************************************************************
void
Display(const unsigned char *pucLogo, unsigned long Px, unsigned long Py,
        unsigned long Tx, unsigned long Ty, tBoolean Inv)
{
    auto unsigned long Sz;
    auto unsigned char *p;
    auto unsigned char t;

    // check for invert
    if(Inv)
    {
        // points to local frame
        p = DspFrame;

        // size in bytes (2 px/byte)
        Sz = Tx * Ty / 2;

        // loops
        while(Sz)
        {
            // invert
            t = *pucLogo;
            *p = ~t;

            // next
            p++;
            pucLogo++;
            Sz--;
        }

        //
        // Display the local frame buffer on the display.
        //
        RIT128x96x4ImageDraw(DspFrame, Px, Py, Tx, Ty);
    }
    else
    {
        //
        // Display the local frame buffer on the display.
        //
        RIT128x96x4ImageDraw(pucLogo, Px, Py, Tx, Ty);
    }
}



//*****************************************************************************
//
// Displays a logo for a specified amount of time.
//
//*****************************************************************************
void
DisplayLogo(const unsigned char *pucLogo, unsigned long ulWidth,
            unsigned long ulHeight, unsigned long ulDelay)
{
    unsigned char *pucDest, ucHigh, ucLow;
    unsigned long ulLoop1, ulLoop2;
    const unsigned char *pucSrc;
    long lIdx;

    //
    // Loop through thirty two intensity levels to fade the logo in from black.
    //
    for(lIdx = 1; lIdx <= 32; lIdx++)
    {
        //
        // Clear the local frame buffer.
        //
        for(ulLoop1 = 0; ulLoop1 < sizeof(DspFrame); ulLoop1 += 4)
        {
            *(unsigned long *)(DspFrame + ulLoop1) = 0;
        }

        //
        // Get a pointer to the beginning of the logo.
        //
        pucSrc = pucLogo;

        //
        // Get a point to the upper left corner of the frame buffer where the
        // logo will be placed.
        //
        pucDest = (DspFrame + (((96 - ulHeight) / 2) * 64) +
                   ((128 - ulWidth) / 4));

        //
        // Copy the logo into the frame buffer, scaling the intensity.  Loop
        // over the rows.
        //
        for(ulLoop1 = 0; ulLoop1 < ulHeight; ulLoop1++)
        {
            //
            // Loop over the columns.
            //
            for(ulLoop2 = 0; ulLoop2 < (ulWidth / 2); ulLoop2++)
            {
                //
                // Get the two nibbles of the next byte from the source.
                //
                ucHigh = pucSrc[ulLoop2] >> 4;
                ucLow = pucSrc[ulLoop2] & 15;

                //
                // Scale the intensity of the two nibbles.
                //
                ucHigh = ((unsigned long)ucHigh * lIdx) / 32;
                ucLow = ((unsigned long)ucLow * lIdx) / 32;

                //
                // Write the two nibbles to the frame buffer.
                //
                pucDest[ulLoop2] = (ucHigh << 4) | ucLow;
            }

            //
            // Increment to the next row of the source and destination.
            //
            pucSrc += (ulWidth / 2);
            pucDest += 64;
        }

        //
        // Wait until an update has been requested.
        //
        while(HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) == 0)
        {
        }

        //
        // Clear the update request flag.
        //
        HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) = 0;

        //
        // Display the local frame buffer on the display.
        //
        RIT128x96x4ImageDraw(DspFrame, 0, 0, 128, 96);
    }

    //
    // Delay for the specified time while the logo is displayed.
    //
    //Delay(ulDelay);

    //
    // Loop through the thirty two intensity levels to face the logo back to
    // black.
    //
    for(lIdx = 31; lIdx >= 0; lIdx--)
    {
        //
        // Clear the local frame buffer.
        //
        for(ulLoop1 = 0; ulLoop1 < sizeof(DspFrame); ulLoop1 += 4)
        {
            *(unsigned long *)(DspFrame + ulLoop1) = 0;
        }

        //
        // Get a pointer to the beginning of the logo.
        //
        pucSrc = pucLogo;

        //
        // Get a point to the upper left corner of the frame buffer where the
        // logo will be placed.
        //
        pucDest = (DspFrame + (((96 - ulHeight) / 2) * 64) +
                   ((128 - ulWidth) / 4));

        //
        // Copy the logo into the frame buffer, scaling the intensity.  Loop
        // over the rows.
        //
        for(ulLoop1 = 0; ulLoop1 < ulHeight; ulLoop1++)
        {
            //
            // Loop over the columns.
            //
            for(ulLoop2 = 0; ulLoop2 < (ulWidth / 2); ulLoop2++)
            {
                //
                // Get the two nibbles of the next byte from the source.
                //
                ucHigh = pucSrc[ulLoop2] >> 4;
                ucLow = pucSrc[ulLoop2] & 15;

                //
                // Scale the intensity of the two nibbles.
                //
                ucHigh = ((unsigned long)ucHigh * lIdx) / 32;
                ucLow = ((unsigned long)ucLow * lIdx) / 32;

                //
                // Write the two nibbles to the frame buffer.
                //
                pucDest[ulLoop2] = (ucHigh << 4) | ucLow;
            }

            //
            // Increment to the next row of the source and destination.
            //
            pucSrc += (ulWidth / 2);
            pucDest += 64;
        }

        //
        // Wait until an update has been requested.
        //
        while(HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) == 0)
        {
        }

        //
        // Clear the update request flag.
        //
        HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) = 0;

        //
        // Display the local frame buffer on the display.
        //
        RIT128x96x4ImageDraw(DspFrame, 0, 0, 128, 96);
    }
}


