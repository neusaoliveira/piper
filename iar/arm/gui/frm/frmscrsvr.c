/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  screensaver.c                                                             */
/*                                                                            */
/*  Modulo de interface para o screen saver                                   */
/*                                                                            */
/*  2013-04-13 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

#include "conf.h"
#include "etc/defines.h"
#include "etc/system.h"
#include "inc/hw_types.h"
#include "drv/rit128x96x4.h"
#include "srv/display.h"
#include "gui/frm/frmscrsvr.h"
#include <string.h>
#include "resources.h"


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* TImeout for screensaver (default 5 minutos) */
#ifndef SCR_DELAY
    #define SCR_DELAY                   5
#endif


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

/* Funcoes para ativar e desativar o form atual */
/* Cada form 'Ativa' deve atualiza-las          */
pFncVoid ScrFncAtiva;
pFncVoid ScrFncDesativa;

/* Screensaver timeout */
unsigned long   ScrTime  = SCR_DELAY * 60 * 1000;

/* Indicates we are in full screen */
static tBoolean OnScreen = FALSE;


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
tBoolean ScrIsSaving(void)
{
    /* retorna se estamos ou nao em screen saver */
    return(OnScreen);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void ScrHit(void)
{
    /* volta o timer padrao */
    ScrTime = SysSetaTimeout(SCR_DELAY * 60 * 1000);

    /* Se nao estamos em screensaver, nao redesenha a tela */
    if(!OnScreen)
    {
        return;
    }

    /* sai do screensaver */
    OnScreen = FALSE;

    #if CNF_LOGGER
        RIT128x96x4Enable(3500000);
    #endif

    // apaga o display
    RIT128x96x4Clear();

    #if CNF_LOGGER
        RIT128x96x4Disable();
    #endif

    /* Aiva form anterior */
    if(ScrFncAtiva)
    {
        ScrFncAtiva();
    }
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void ScrUpdate(pFncVoid FncAtiva, pFncVoid FncDesativa)
{
    /* Salva as funcoes pra sair e voltar */
    ScrFncAtiva    = FncAtiva;
    ScrFncDesativa = FncDesativa;
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte ScrRefresh(void)
{
    const short Px[] = {  0, 84, 36, 74, 18, 32,  4, 54, 74,  8 };
    const short Py[] = { 30, 10,  8, 60, 34, 26, 20,  2, 18, 44 };
    static byte i    = 0;

    /* Timer expirou? */
    if(!SysVenceuTimeout(ScrTime))
    {
        // ainda nao, aborta e indica a situacao do screensaver
        return(OnScreen);
    }

    /* Se ainda nao estamos no screen saver */
    if(!OnScreen)
    {
        /* DEsativa o form atual */
        if(ScrFncDesativa)
        {
            ScrFncDesativa();
        }
    }

    /* Entramos no screen saver */
    OnScreen = TRUE;

    /* os proximos timers serao de 15 seg */
    ScrTime = SysSetaTimeout(15 * 1000);

    #if CNF_LOGGER
        RIT128x96x4Enable(3500000);
    #endif

    /* Apaga a posicao atual */
    memset(DspFrame, 0x00, 38*38/2);
    Display(DspFrame, Px[i], Py[i], 38, 38, 0x00);

    /* proxima posicao */
    i = (i + 1) & 0x07;

    /* desenha bitmap */
    Display(BmpSkull, Px[i], Py[i], 38, 38, 0x00);

    #if CNF_LOGGER
        RIT128x96x4Disable();
    #endif

    /* estamos em scrsvr */
    return(TRUE);
}
