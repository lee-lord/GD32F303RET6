#ifndef _SYSTYPE_H_
#define _SYSTYPE_H_
typedef signed char   S8;
typedef unsigned char U8;
typedef signed short int S16;
typedef unsigned short int U16;
typedef signed int   S32;
typedef unsigned int U32;
typedef unsigned long long U64;
typedef long long S64;

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h> 
#include "gd32f30x.h"

#define CYCLES_PER_MICROSECOND 120
#define MAX_OVERFLOW    ((1 << 16) - 1)

#define TOGGLE_GPIO(port,pin) GPIO_OCTL(port) ^=(pin)

#define BITBAND(addr,bitnum) ((addr & 0xF0000000) +0x2000000+((addr & 0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr,bitnum) MEM_ADDR(BITBAND(addr, bitnum))


#define PORTn_BITx_OUT(PORT,index) BIT_ADDR(GPIO_OCTL(PORT),index)
#define PORTn_BITx_IN(PORT,index)  BIT_ADDR(GPIO_ISTAT(PORT),index)


#define PORT_PIN_SET(PORT,pin)   GPIO_BOP(PORT)=pin
#define PORT_PIN_CLR(PORT,pin)   GPIO_BC(PORT)=pin
#endif




