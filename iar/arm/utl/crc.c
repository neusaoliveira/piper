/******************************************************************************/
/*                                                                            */
/*  MAF                                                                       */
/*  Instituto Tecnologico de Aeronautica                                      */
/*  Divisao de Engenharia Eletronica e Computacao                             */
/*                                                                            */
/*  srv/crc.h                                                                 */
/*                                                                            */
/*  Modulo de calculo de CRC8                                                 */
/*                                                                            */
/*  2012-05-30 Mateus Inicial                                                 */
/*  2012-11-24 Mateus Adicionado CrcL e CrcPegasus                            */
/*                                                                            */
/******************************************************************************/

#include "etc/defines.h"
#include "utl/crc.h"

#define INITIAL_REMAINDER	0xFF
#define FINAL_XOR_VALUE		0x00

/* CRC table needed for CrcPegasus() */
static const unsigned char  pucCRCTable[256] =
{
    0,33,66,99,132,165,198,231,41,8,107,74,173,140,239,206,
    82,115,16,49,214,247,148,181,123,90,57,24,255,222,189,
    156, 164, 133, 230, 199, 32, 1, 98, 67, 141, 172, 207,
    238, 9, 40, 75, 106, 246, 215, 180, 149, 114, 83, 48,
    17, 223, 254, 157, 188, 91, 122, 25, 56, 105, 72, 43,
    10, 237, 204, 175, 142, 64, 97, 2, 35, 196, 229, 134,
    167, 59, 26, 121, 88, 191, 158, 253, 220, 18, 51, 80,
    113, 150, 183, 212, 245, 205, 236, 143, 174, 73, 104,
    11, 42, 228, 197, 166, 135, 96, 65, 34, 3, 159, 190,
    221, 252, 27, 58, 89, 120, 182, 151, 244, 213, 50, 19,
    112, 81, 210, 243, 144, 177, 86, 119, 20, 53, 251, 218,
    185, 152, 127, 94, 61, 28, 128, 161, 194, 227, 4, 37, 70,
    103, 169, 136, 235, 202, 45, 12, 111, 78, 118, 87, 52, 21,
    242, 211, 176, 145, 95, 126, 29, 60, 219, 250, 153, 184,
    36, 5, 102, 71, 160, 129, 226, 195, 13, 44, 79, 110, 137,
    168, 203, 234, 187, 154, 249, 216, 63, 30, 125, 92, 146,
    179, 208, 241, 22, 55, 84, 117, 233, 200, 171, 138, 109,
    76, 47, 14, 192, 225, 130, 163, 68, 101, 6, 39, 31, 62,
    93, 124, 155, 186, 217, 248, 54, 23, 116, 85, 178, 147,
    240, 209, 77, 108, 15, 46, 201, 232, 139, 170, 100, 69,
    38, 7, 224, 193, 162, 131
};

/* CRC for pegasus, as it... */
char CrcPegasus(const unsigned char message[], const unsigned char ucInit, const unsigned char ucFinal)
{
    char  ucRemainder = INITIAL_REMAINDER;
    unsigned char  ucData;
    unsigned char  ucByte;

    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (ucByte = ucInit; ucByte < ucFinal; ucByte++)
    {
        ucData = message[ucByte] ^ ucRemainder;
  	ucRemainder = pucCRCTable[ucData] ^ (ucRemainder << 8);

    }

    /*
     * The final remainder is the CRC.
     */
    return ((ucRemainder) ^ FINAL_XOR_VALUE);

}   /* crcFast() */


/******************************************************************************/
/**  Implementa a operacao de CRC para uso nas mensagens de GPS e Telemetria.
 *
 *   O calculo de CRC e' feito atraves da operacao de ou-exclusivo (XOR) sobre
 *   todos os bytes do pacote/string em questao.
 *
 *   \param pDat Ponteiro para os dados/string a ser calculado o CRC
 *   \param Stx Caracter ou valor contido na stream pDat utilizado para marcar
 *          o inicio da mensagem (Start of Transmition)
 *   \param Etx Caracter ou valor contido na stream pDat utilizado para marcar
 *          o final da mensagem (End of Transmition). Deve vir apos o Stx.
 *   \param pCrc Deve ser fornecido o ponteiro de uma variavel do tipo byte* e
 *          apos a execucao do calculo, esta variavel aponta para o final da
 *          mensagem, onde o valor do CRC e' geralmente inserido nesta. Se NULL
 *          nada e' repassado.
 *   \return Retorna o CRC de 8 bits calculado sobre a mensagem.
 */
/******************************************************************************/
byte Crc(byte *pDat, byte Stx, byte Etx, byte **pCrc)
{
	auto byte Lrc;

	// wait for stx
	while(*pDat != Stx) { pDat++; }

	// initialize lrc
	Lrc = 0;

	// calculates
	while(*pDat && *pDat != Etx)
	{
		// calculates CRC
		Lrc ^= *pDat;

		// next
		pDat++;
	}

	// if crc pointer valid
	if(pCrc)
	{
		// saves pointer after Etx
		*pCrc = pDat + 1;
	}

	// return calculated lrc
	return(Lrc);
}

/******************************************************************************/
/**  Implementa a operacao de CRC para uso em pacotes de dados binarios.
 *
 *   O calculo de CRC e' feito atraves da operacao de ou-exclusivo (XOR) sobre
 *   todos os bytes do pacote/string em questao.
 *
 *   \param pDat Ponteiro para os dados/string a ser calculado o CRC
 *   \param Len Deve conter o tamanho dos dados a ser calculado o CRC
 *   \param pCrc Deve ser fornecido o ponteiro de uma variavel do tipo byte* e
 *          apos a execucao do calculo, esta variavel aponta para o final da
 *          mensagem, onde o valor do CRC e' geralmente inserido nesta. Se NULL
 *          nada e' repassado.
 *   \return Retorna o CRC de 8 bits calculado sobre a mensagem.
 */
/******************************************************************************/
byte CrcL(byte *pDat, byte Len, byte **pCrc)
{
	auto byte Lrc;

	// initialize lrc
	Lrc = 0;

	// calculates for size
	while(Len)
	{
		// calculates CRC
		Lrc ^= *pDat;

		// next
		pDat++;
		Len--;
	}

	// if crc pointer valid
	if(pCrc)
	{
		// saves pointer after Len
		*pCrc = pDat;
	}

	// return calculated lrc
	return(Lrc);
}


/******************************************************************************/
/**  Implementa a operacao de CRC com algoritmo Fletcher
 *
 *   O calculo de CRC e' feito atraves da operacao de adição de todos os bytes
 *   do pacote/string em questao.
 *
 *   \param pDat Ponteiro para os dados/string a ser calculado o CRC
 *   \param Len Deve conter o tamanho dos dados a ser calculado o CRC
 *   \param pCrc Deve ser fornecido o ponteiro de uma variavel do tipo byte* e
 *          apos a execucao do calculo, esta variavel aponta para o final da
 *          mensagem, onde o valor do CRC e' geralmente inserido nesta. Se NULL
 *          nada e' repassado.
 *   \param pCrcA Deve ser fornecido um ponteiro tipo byte para o valor CRC A
 *   \param pCrcB Deve ser fornecido um ponteiro tipo byte para o valor CRC B
 *   \return Retorna nada
 */
/******************************************************************************/
void CrcFletcher(byte *pDat, byte Len, byte **pCrc, byte *pCrcA, byte *pCrcB)
{
	auto byte LrcA;
	auto byte LrcB;

	// initialize lrc
	LrcA = 0;
	LrcB = 0;

	// calculates for size
	while(Len)
	{
		// calculates CRC
		LrcA += *pDat;

        // CRC B
        LrcB += LrcA;

		// next
		pDat++;
		Len--;
	}

	// if crc pointer valid
	if(pCrc)
	{
		// saves pointer after Len
		*pCrc = pDat;
	}

	// if crc a pointer valid
	if(pCrcA)
	{
	    // copy crc valur
	    *pCrcA = LrcA;
	}

	// if crc a pointer valid
	if(pCrcB)
	{
	    // copy crc valur
	    *pCrcB = LrcB;
	}
}

