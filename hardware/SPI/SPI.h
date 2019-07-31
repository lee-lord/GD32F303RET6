#ifndef _SPI_H_
#define _SPI_H_
#include "sysType.h"
#define SET_TLE1_NSS_HIGH          PORTn_BITx_OUT(GPIOB,12)=1
//PORT_PIN_SET(GPIOB,GPIO_PIN_12)
#define SET_TLE1_NSS_LOW           PORTn_BITx_OUT(GPIOB,12)=0
//PORT_PIN_CLR(GPIOB,GPIO_PIN_12)

#define SET_TLE2_NSS_HIGH          PORTn_BITx_OUT(GPIOC,9)=1
//PORT_PIN_SET(GPIOC,GPIO_PIN_9)
#define SET_TLE2_NSS_LOW           PORTn_BITx_OUT(GPIOC,9)=0
//PORT_PIN_CLR(GPIOC,GPIO_PIN_9)

void SPIX_3wires_initial(U32 SPIX,U32 baud,U8 BitWidh);
void SPI1_Init(void);

U16 ReadWrite2Byte(U16 cmd);
U16 transfer16Bits(U16 *cmd,U16 *data,U16 Len);
U16 ReadWrite2ByteCMD(U16 cmd,U16 data);
#endif
