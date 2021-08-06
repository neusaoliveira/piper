/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  app/dcm.c                                                                 */
/*                                                                            */
/*  Modulo de operacoes para a matriz de cossenos diretores                   */
/*                                                                            */
/*  2012-09-04 Mateus Inicial                                                 */
/*                                                                            */
/******************************************************************************/

/* @@ NOTE: All variables static for performance, otherwise indicated */

#include "conf.h"
#include "etc/defines.h"
#include "etc/system.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "utl/math.h"
#include "app/imu/axis.h"
#include "app/imu/dcm.h"

#ifdef CNF_ITG3200
    #include "drv/itg3200.h"
#endif

#ifdef CNF_LPR5XX
    #include "drv/lpr5xx.h"
#endif

#ifdef CNF_ADXL345
    #include "drv/adxl345.h"
#endif

#ifdef CNF_BMA180
    #include "drv/bma180.h"
#endif

#ifdef CNF_HMC58XX
    #include "drv/hmc58xx.h"
#endif

#include <stdio.h>
#include <string.h>

#include "globals.h"

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

// internal angles in radians
static float Pitch;
static float Roll;
static float Yaw;
static float Hdg;

// Variavel externa contendo os angulos de euler
StDcmEuler   DcmEuler;

// timer to calc
unsigned long DcmTimer                  = 0L;

// Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible
static float DcmDt                      = 0.02F;

// Gyro rad/sec with gain and offset
float DcmGyrRads[3]                     = {0,0,0};

// Accel m/sec2 with gain and offset
float DcmAccMpss[3]                     = {0,0,0};

// Armazena o vetor com as medidas do girômetro corrigida pelo vetor de ajuste
static float DcmCorrectGyro[3]          = {0,0,0};

// Vetor de ajuste (parte proporcional) utilizado para correção do erro de medida acumulado do girômetro
static float DcmLoopGainP[3]            = {0,0,0};

// Vetor de ajuste (parte integral) utilizado para correção do erro de medida acumulado do girômetro
static float DcmLoopGainI[3]            = {0,0,0};


// Matriz de rotação assumida inicialmente
static float DcmMtxRot[3][3]            = {{ 0,0,0 },{ 0,0,0 },{ 0,0,0 }};

// Matriz auxiliar usada para determinação da Matriz de rotação
static float DcmMtxUpdate[3][3]         = {{0,0,0},{0,0,0},{0,0,0}};

// Vetor de erro obtido a partir do acelerômetro
static float DcmErrAccel[3]             = {0,0,0};

// Vetor de erro obtido a partir do magnetômetro
static float DcmErrMagnet[3]            = {0,0,0};

// Axis alignment constant
static int AxisAlign[9]                  = CNF_IMU_AXIS;

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/

// magnetometer delay
#ifndef MAG_DELAY
    #define MAG_DELAY                   5
#endif

#if 0 // antigos
    // CLosed loop PI gains
    #define Kp_ROLLPITCH                0.02F      // Define valor de Kp para correção de pitch e roll
    #define Ki_ROLLPITCH                0.00002F   // Define valor de Ki para coreeção de pitch e roll
    #define Kp_YAW                      1.2F       // Define valor de Kp para a correção de yaw
    #define Ki_YAW                      0.00002F   // Define valor de Ki para a correção de yaw
#else
    // CLosed loop PI gains
    #define Kp_ROLLPITCH                0.005F      // Define valor de Kp para correção de pitch e roll
    #define Ki_ROLLPITCH                0.000005F   // Define valor de Ki para coreeção de pitch e roll
    #define Kp_YAW                      1.2F       // Define valor de Kp para a correção de yaw
    #define Ki_YAW                      0.00002F   // Define valor de Ki para a correção de yaw
#endif

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte DcmInit(void)
{
    // default dt used in PI loop
    DcmDt       = 0.02F;

    // timer is 200ms
    DcmTimer    = 20;

    // inicializa manualmente o primeiro vetor
    DcmGyrRads[0] = 0.0f;
    DcmGyrRads[1] = 0.0f;
    DcmGyrRads[2] = 0.0f;

    // copia para o restante
    memcpy(DcmAccMpss     , DcmGyrRads, 3 * sizeof(float));
    memcpy(DcmCorrectGyro , DcmGyrRads, 3 * sizeof(float));
    memcpy(DcmLoopGainP   , DcmGyrRads, 3 * sizeof(float));
    memcpy(DcmLoopGainI   , DcmGyrRads, 3 * sizeof(float));
    memcpy(DcmMtxRot[0]   , DcmGyrRads, 3 * sizeof(float));
    memcpy(DcmMtxRot[1]   , DcmGyrRads, 3 * sizeof(float));
    memcpy(DcmMtxRot[2]   , DcmGyrRads, 3 * sizeof(float));
    memcpy(DcmMtxUpdate[0], DcmGyrRads, 3 * sizeof(float));
    memcpy(DcmMtxUpdate[1], DcmGyrRads, 3 * sizeof(float));
    memcpy(DcmMtxUpdate[2], DcmGyrRads, 3 * sizeof(float));
    memcpy(DcmErrAccel    , DcmGyrRads, 3 * sizeof(float));
    memcpy(DcmErrMagnet   , DcmGyrRads, 3 * sizeof(float));

    // do initial alignment with 100 steps
    DcmAlignment(100);

    // done!
    return(SUCESSO);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte DcmRefresh(void)
{
    static byte MagDelay = MAG_DELAY;

    // its our time to work?
    if(DcmTimer>0)
    {
        // not yet
        return(FALSE);
    }

    // next delay is 50ms
    DcmTimer = TIMER_OUTPUT;

    // read gyro
    GyrRead();

    // read accelerometer
    AccRead();

    // read magnetometer, if so
    if(!MagDelay)
    {
        // Read magnetometer
        MagRead();

        // compute heading
        DcmGetHeading();

        // reload delay
        MagDelay = MAG_DELAY;
    }
    else
    {
        // wait another loop
        MagDelay--;
    }

    // Evaluate DCM at 50ms step
    DcmEval(TIMER_OUTPUT);

    // Renormalize DCM
    DcmOrtho();

    // Compute sensor errors
    DcmError();

    // done with new data
    return(TRUE);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void DcmAlignmentOld(byte Samples)
{
    // i as int for math compatibility
    auto short i;

    auto long tAccOfst[3];
    auto long tGyrOfst[3];

    // number of samples
    i = (short)Samples;

    // clear
    memset(tAccOfst, 0, 3 * sizeof(long));
    memset(tGyrOfst, 0, 3 * sizeof(long));

    // Read and DISCARD "samples"
    while(i)
    {
        // Process Gyros, accelerometers and magnetometer
        // for initial, unreliable readouts
        GyrRead();
        AccRead();
        MagRead();

        // Delay for 10 ms (2x Tick interrupt)
        HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) = 0;
        while(!HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK))
        {
        }

        // Delay for 10 ms (2x Tick interrupt)
        HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) = 0;
        while(!HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK))
        {
        }

        // next sample loop
        i--;
    }

    // number of samples
    i = (short)Samples;

    // Read and accumulate "samples"
    while(i)
    {
        // Process Gyros and accelerometers
        GyrRead();
        AccRead();

        // Accumulate values for gyro
        tGyrOfst[0] += (long)GyrRaw[GYR_X];
        tGyrOfst[1] += (long)GyrRaw[GYR_Y];
        tGyrOfst[2] += (long)GyrRaw[GYR_Z];

        // Accumulate values for accel
        // Sequence is 1-0-2 for alignment!
        tAccOfst[0] += (long)AccRaw[ACC_X];
        tAccOfst[1] += (long)AccRaw[ACC_Y];
        tAccOfst[2] += (long)AccRaw[ACC_Z];

        // Delay for 10 ms (2x Tick interrupt)
        HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) = 0;
        while(!HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK))
        {
        }

        // Delay for 10 ms (2x Tick interrupt)
        HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) = 0;
        while(!HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK))
        {
        }

        // next sample loop
        i--;
    }

    // half of samples.
    // used for mean-degree average
    i = Samples / 2;

    // Divide by mean of "samples" items
    GyrOfst[0] = (short)((tGyrOfst[0] + i) / (long)Samples);
    GyrOfst[1] = (short)((tGyrOfst[1] + i) / (long)Samples);
    GyrOfst[2] = (short)((tGyrOfst[2] + i) / (long)Samples);

    AccOfst[0] = (short)((tAccOfst[0] + i) / (long)Samples);
    AccOfst[1] = (short)((tAccOfst[1] + i) / (long)Samples);
    AccOfst[2] = (short)((tAccOfst[2] + i) / (long)Samples);

    // Accel Y without gravity (value by datasheet!)
    AccOfst[2] -= ACC_GRAVITY * AxisAlign[5];

    // matriz de rotacao "Alinhada"
    // Aqiu entraria o alinhammento inicial com accel (Pth, Rol) e mag (Yaw)!
    DcmMtxRot[0][0] = 1.0f;
    DcmMtxRot[1][1] = 1.0f;
    DcmMtxRot[2][2] = 1.0f;
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void DcmAlignment(byte Samples)
{
    // i as int for math compatibility
    auto short i;

    auto long AccData[3];
    auto long tGyrOfst[3];

    auto float Pitch;
    auto float Roll;
    auto float Yaw;
    auto float cp, cr, cy, sp, sr, sy;

    // double of number of samples
    i = (short)Samples * 2;

    // clear
    memset(AccData,  0, 3 * sizeof(long));
    memset(tGyrOfst, 0, 3 * sizeof(long));

    // Read and DISCARD "samples"
    while(i)
    {
        // Process Gyros, accelerometers and magnetometer
        // for initial, unreliable readouts
        GyrRead();
        AccRead();
        MagRead();

        // Delay for 20 ms
        Halt(20);

        // next sample loop
        i--;
    }

    // number of samples
    i = (short)Samples;

    // Read and accumulate "samples"
    while(i)
    {
        // Process Gyros and accelerometers
        GyrRead();
        AccRead();
        MagRead();

        // Accumulate values for gyro
        tGyrOfst[0] += (long)GyrRaw[GYR_X];
        tGyrOfst[1] += (long)GyrRaw[GYR_Y];
        tGyrOfst[2] += (long)GyrRaw[GYR_Z];

        // Accumulate values for accel
        // Sequence is 1-0-2 for alignment!
        AccData[0] += (long)AccRaw[ACC_X];
        AccData[1] += (long)AccRaw[ACC_Y];
        AccData[2] += (long)AccRaw[ACC_Z];

        // Delay for 20 ms
        Halt(20);

        // next sample loop
        i--;
    }

    // half of samples.
    // used for mean-degree average
    i = Samples / 2;

    // Divide by mean of "samples" items
    GyrOfst[0] = (short)((tGyrOfst[0] + i) / (long)Samples);
    GyrOfst[1] = (short)((tGyrOfst[1] + i) / (long)Samples);
    GyrOfst[2] = (short)((tGyrOfst[2] + i) / (long)Samples);

    /* Etapa 2 - Acelerometro */

        /* Obtem a média de todas as leituras */
        AccData[0] = (short)((AccData[0] + i) / (long)Samples);
        AccData[1] = (short)((AccData[1] + i) / (long)Samples);
        AccData[2] = (short)((AccData[2] + i) / (long)Samples);

        /* Elimina o bias das leituras e alinha com os eixos */
        AccData[0] = AxisAlign[3] * (AccData[0] - AccOfst[0]);
        AccData[1] = AxisAlign[4] * (AccData[1] - AccOfst[1]);
        AccData[2] = AxisAlign[5] * (AccData[2] - AccOfst[2]);

        /* Pitch */
        Pitch = sqrt(AccData[1]*AccData[1] + AccData[2]*AccData[2]);
        Pitch = atan2(-AccData[0], Pitch);
        cp = cos(Pitch);
        sp = sin(Pitch);

        /* Roll -> atan2(x, sqrt(y*y + z*z) ) -> Farrel */
        //Roll  = atan2(AccData[0], AccData[2]) / cp;
        Roll = atan2(AccData[1], AccData[2]);
        cr = cos(Roll);
        sr = sin(Roll);

    /* Etapa 3 - Magnetometro */

        // get magnetic components in X and Y (aproveita cy, sy)
        cy = (Mag[0] * cp) + (Mag[1] * sr * sp) + (Mag[2] * cr * sp);
        sy = (Mag[1] * cr) - (Mag[2] * sr);

        // get heading
        Yaw = atan2(-sy, cy);

        // calcula cy sy
        cy = cos(Yaw);
        sy = sin(Yaw);

        // @*@ Sera que antes cy e sy ja nao sao seno e cosseno?
        // Talvez sy esteja invertido, pois atan2(-sy, )

    /* Etapa 4 - Inicialização */


        /* obtém a DCM a partir dos angulos */
        DcmMtxRot[0][0] = cp * cy;
        DcmMtxRot[0][1] = sr * cp * cy  -  cr * sy;
        DcmMtxRot[0][2] = cr * sp * cy  +  sr * sy;

        DcmMtxRot[1][0] = cp * sy;
        DcmMtxRot[1][1] = sr * sp * sy  +  cr * cy;
        DcmMtxRot[1][2] = cr * sp * sy  -  sr * cy;

        DcmMtxRot[2][0] = -sp;
        DcmMtxRot[2][1] = sr * cp;
        DcmMtxRot[2][2] = cr * cp;
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void DcmEval(dword Tmr)
{
    static float  TempVec[3];
    static float  TempMtx[3][3];

    // update timer from ms to seconds
    DcmDt = (float)Tmr * 0.001F;

    // Get gyro rad from deg with sensibility
    DcmGyrRads[0] = ToRad((float)Gyr[0]) * GYR_GAIN_X;
    DcmGyrRads[1] = ToRad((float)Gyr[1]) * GYR_GAIN_Y;
    DcmGyrRads[2] = ToRad((float)Gyr[2]) * GYR_GAIN_Z;

    // Soma os canais Proporcional e Integral da malha
    MthVectorAdd(TempVec, DcmLoopGainP, DcmLoopGainI, 3);

    // Aplica a realimentação no vetor velocidade angular
    MthVectorAdd(DcmCorrectGyro, DcmGyrRads, TempVec, 3);

    //Construção da matriz beseada na aproximação adotada para a obtenção da matriz de rotação
    //Multiplica a constante de tempo sobre a leitura corrigida.
    // Calcula \omega_b * \Delta t
    DcmMtxUpdate[0][0] =  0;
    DcmMtxUpdate[0][1] = -DcmDt * DcmCorrectGyro[2];
    DcmMtxUpdate[0][2] =  DcmDt * DcmCorrectGyro[1];
    DcmMtxUpdate[1][0] =  DcmDt * DcmCorrectGyro[2];
    DcmMtxUpdate[1][1] =  0;
    DcmMtxUpdate[1][2] = -DcmDt * DcmCorrectGyro[0];
    DcmMtxUpdate[2][0] = -DcmDt * DcmCorrectGyro[1];
    DcmMtxUpdate[2][1] =  DcmDt * DcmCorrectGyro[0];
    DcmMtxUpdate[2][2] =  0;

    // Multiplica a matriz aproximada (DcmMtxUpdate) pela matriz de rotação atual (matriz tipo identidade)
    //
    MthMatrixMul(TempMtx, DcmMtxRot, DcmMtxUpdate, 3);

    // Efetua a somatória da matirz temporaria com o valor atual da matriz de rotação, gerando um nova matriz de rotação.
    MthMatrixAdd(DcmMtxRot, DcmMtxRot, TempMtx, 3);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void DcmOrtho(void)
{
    static float Err;
    static float TmpX[3];
    static float TmpY[3];
    static float TmpZ[3];
    static float Cte;

    // Calcula o erro de ortogonalidade entre X e Y (eq:ahrs_ortho_dot)
    Err = -(MthVectorDot(&DcmMtxRot[0][0], &DcmMtxRot[1][0], 3) * 0.5F);

    // Aplica o erro nos vetores X e Y da matriz (eq:ahrs_ortho_xy)
    MthVectorScale(TmpX, &DcmMtxRot[1][0], Err, 3);
    MthVectorScale(TmpY, &DcmMtxRot[0][0], Err, 3);

    // Adiciona o vetor da matriz aos vetores temporarios (eq:ahrs_ortho_xy)
    MthVectorAdd(TmpX, TmpX, &DcmMtxRot[0][0], 3);
    MthVectorAdd(TmpY, TmpY, &DcmMtxRot[1][0], 3);

    // Obtem o vetor ortogonal a X e Y (eq:ahrs_ortho_z)
    MthVectorCross(TmpZ, TmpX, TmpY, 3);

    // Renormaliza para X (eq:ahrs_ortho_tay)
    Cte = 0.5F * (3.0F - MthVectorDot(TmpX, TmpX, 3));
    MthVectorScale(&DcmMtxRot[0][0], TmpX, Cte, 3);

    // Renormaliza para Y (eq:ahrs_ortho_tay)
    Cte = 0.5F * (3.0F - MthVectorDot(TmpY, TmpY, 3));
    MthVectorScale(&DcmMtxRot[1][0], TmpY, Cte, 3);

    // Renormaliza para Z (eq:ahrs_ortho_tay)
    Cte = 0.5F * (3.0F - MthVectorDot(TmpZ, TmpZ, 3));
    MthVectorScale(&DcmMtxRot[2][0], TmpZ, Cte, 3);
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void DcmError(void)
{
    static float  MagFator;

    static float  AccelMagnitude;
    static float  AccelWeight;

    // must be static
    static float TempVec_P[3];
    static float TempVec_I[3];

    /**************************************************************************/
    /* Correção pelo acelerômetro                                             */

        // convert integer accel to float and apply gain
        DcmAccMpss[0] = (float)Acc[0] * ACC_GAIN_X;
        DcmAccMpss[1] = (float)Acc[1] * ACC_GAIN_Y;
        DcmAccMpss[2] = (float)Acc[2] * ACC_GAIN_Z;

        // Acelerometer magnitude in G's
        AccelMagnitude = (DcmAccMpss[0]*DcmAccMpss[0]) +
                         (DcmAccMpss[1]*DcmAccMpss[1]) +
                         (DcmAccMpss[2]*DcmAccMpss[2]);
        AccelMagnitude = sqrt(AccelMagnitude);
        AccelMagnitude = AccelMagnitude * (1.0F/ACC_GRAVITY);
        //AccelMagnitude = AccelMagnitude * 0.0040322580645F;

        // Aqui removemos a aceleração centripeta da curva

        // do something without comments. Bad programmer....
        // Restrict AccelWeight between 0 and 1, from 0.5g to 1.5g like __/\__
        AccelWeight = 1.0F - (2.0F * absf(1.0F - AccelMagnitude));  // ramp
        AccelWeight = limf(AccelWeight, 0.0F, 1.0F);                // limit

        // Calcula o vetor de erro com auxilio do acelerometro
        //  Efetua o produto vetorial entre o vetor Z da matriz de rotação normalizada e o vetor que
        // contem as leituras das aceleraçoes efetuadas no eixo Z.
        MthVectorCross(DcmErrAccel, DcmAccMpss, &DcmMtxRot[2][0], 3);

        // Após, <DcmErrAccel> indica a intensidade e direção do desalinhamento
        // entre o vetor g e o eixo z_n da matriz.
        // <AccelWeight> indica o quanto da correção vamos considerar.

    /**************************************************************************/
    /* Correção pelo magnetometro                                             */

        // Calcula o indice de erro de orientação.
        MagFator = (DcmMtxRot[0][0]*sin(Hdg)) - (DcmMtxRot[1][0]*cos(Hdg));

        // Calcula o vetor de erro a partir do magnetômetro. Em função do indice calculado e do vetor Z da matriz de rotação.
        MthVectorScale(DcmErrMagnet, &DcmMtxRot[2][0], MagFator, 3);

        // Apos, <DcmErrMagnet> indica a

    /**************************************************************************/
    /* Aplica os ganhos da malha fechada para computar os canais P e I        */

        // Calcula o vetor de ajuste (parte proporcional) em função do ganho Kp
        // definido e do vetor de erro DcmErrAccel.
        MthVectorScale(DcmLoopGainP, DcmErrAccel, Kp_ROLLPITCH * AccelWeight, 3);

        // Calcula o vetor de ajuste (parte integral) em função do ganho Ki
        // definido e do vetor de erro DcmErrAccel.
        //MthVectorScale(TempVec_I, DcmErrAccel, Ki_ROLLPITCH * AccelWeight * DcmDt, 3);
        MthVectorScale(TempVec_I, DcmErrAccel, Ki_ROLLPITCH * AccelWeight, 3); // Spark sem Dt!!

        // Efetua a soma entre o TempVec_I atual e o DcmLoopGainI anterior.
        MthVectorAdd(DcmLoopGainI, DcmLoopGainI, TempVec_I, 3);

        // Calcula o vetor de ajuste (parte proporcional) em função do ganho Kp definido e do vetor de erro DcmErrMagnet.
        MthVectorScale(TempVec_P, DcmErrMagnet, Kp_YAW, 3);

        // Efetua a soma entre o TempVec_P atual e o DcmLoopGainP anterior.
        MthVectorAdd(DcmLoopGainP, DcmLoopGainP, TempVec_P, 3);

        // Calcula o vetor de ajuste (parte integral) em função do ganho Ki definido e do vetor de erro DcmErrMagnet.
        //MthVectorScale(TempVec_I, DcmErrMagnet, Ki_YAW * DcmDt, 3);
        MthVectorScale(TempVec_I, DcmErrMagnet, Ki_YAW, 3); // Spark sem Dt!

        // Efetua a soma entre o TempVec_I atual e o DcmLoopGainI anterior.
        MthVectorAdd(DcmLoopGainI, DcmLoopGainI, TempVec_I, 3);
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void DcmGetEuler(void)
{
    // get rad values
    //Pitch =  MthArcsin(DcmMtxRot[2][0]);
    Pitch =  asin(DcmMtxRot[2][0]);
    Pitch = -Pitch;
    Roll  =  atan2(DcmMtxRot[2][1], DcmMtxRot[2][2]);
    Yaw   =  atan2(DcmMtxRot[1][0], DcmMtxRot[0][0]);

    // float degrees
    DcmEuler.PthF = ToDeg(Pitch);
    DcmEuler.RolF = ToDeg(Roll);
    DcmEuler.YawF = ToDeg(Yaw);

    // check angle
    //if(DcmEuler.YawF < 0)
    //{
    //    // adds 360 degrees
    //    DcmEuler.YawF += 360.0F;
    //}

    // integer degrees
    DcmEuler.PthI = ftoi(DcmEuler.PthF*10.0F);
    DcmEuler.RolI = ftoi(DcmEuler.RolF*10.0F);
    DcmEuler.YawI = ftoi(DcmEuler.YawF*10.0F);
}

/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void DcmGetHeading(void)
{
    static float mag_x;
    static float mag_y;
    static float cp, cr, sp, sr;

    // pre calculate sin and cos values
    cr = cos(Roll);
    sr = sin(Roll);
    cp = cos(Pitch);
    sp = sin(Pitch);

    // get magnetic components in X and Y
    mag_x = (Mag[0] * cp) + (Mag[1] * sr * sp) + (Mag[2] * cr * sp);
    mag_y = (Mag[1] * cr) - (Mag[2] * sr);


    // get heading
    Hdg = atan2(-mag_y,mag_x);
    //Hdg = Yaw;
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
byte DcmGyroCheck(void)
{
    static byte GyroSat = 0;

    // Check if gyro is saturated
    if((absf(DcmGyrRads[0]) >= ToRad(300.0F))||(absf(DcmGyrRads[1]) >= ToRad(300.0F))||(absf(DcmGyrRads[2]) >= ToRad(300.0F)))
    {
        // Check limit
        if(GyroSat < 50) { GyroSat += 10; }
    }
    else
    {
        // wait a bit to tell its not saturated...
        if(GyroSat) { GyroSat--; }
    }

    // return how much it is saturated
    return(GyroSat);
}


/******************************************************************************/
/*                                                                            */
/*                                                                            */
/*                                                                            */
/******************************************************************************/
void DcmPrintMatrix(void)
{
    // full matrix
    printf("DCM {%1.2f, %1.2f, %1.2f; %1.2f, %1.2f, %1.2f; %1.2f, %1.2f, %1.2f}\r",
                      DcmMtxRot[0][0], DcmMtxRot[0][1], DcmMtxRot[0][2],
                      DcmMtxRot[1][0], DcmMtxRot[1][1], DcmMtxRot[1][2],
                      DcmMtxRot[2][0], DcmMtxRot[2][1], DcmMtxRot[2][2]);

}




