#ifndef _SPI_H_
#define _SPI_H_
#include "sysType.h"
#define SET_TLE1_NSS_HIGH          PORT_PIN_SET(GPIOB,GPIO_PIN_12);
#define SET_TLE1_NSS_LOW           PORT_PIN_CLR(GPIOB,GPIO_PIN_12);

#define SET_TLE2_NSS_HIGH          PORT_PIN_SET(GPIOC,GPIO_PIN_8);
#define SET_TLE2_NSS_LOW           PORT_PIN_CLR(GPIOC,GPIO_PIN_8);

void SPIX_3wires_initial(U32 SPIX,U32 baud,U8 BitWidh);
void SPI2_Init(void);

#endif
