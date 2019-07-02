#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "gd32f30x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/5/27
//版本：V1.3
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
////////////////////////////////////////////////////////////////////////////////// 
//extern u8 USART_RX_BUF[64];     //接收缓冲,最大63个字节.末字节为换行符 
//extern u8 USART_RX_STA;         //接收状态标记	


#define TxBufLen  1024
typedef struct {
u16 Txfile_head;
u16 Txfile_tail;
u16 Rxfile_head;
u16 Rxfile_tail;
u8 *RxBuff;//[TxBufLen];
u8 *TxBuff;//[TxBufLen];	
u8 busy;
u8 state;	
}File_Buffer;
void uart1_init(u32 bound);
void uart2_init(u32 bound);

__inline u8 Com1SendDatas(const char *pStr ,u16 len);
__inline u8 Com2SendDatas(const char *pStr ,u16 len);

__inline u16 USART1Read(void);
__inline u16 USART2Read(void);
__inline u16 USART3Read(void);
__inline u16 USART1Clear(void);
__inline u16 USART2Clear(void);
__inline u16 USART3Clear(void);
__inline u16 USART1available(void);
__inline u16 USART2available(void);
__inline u16 USART3available(void);
__inline u8 USART1Writes(const char * Pbuf,u8 Len);
__inline u8 USART1WriteBytes(const char * Pbuf,u8 len);
__inline u8 USART2Writes(const char * Pbuf,u8 Len);
__inline u8 USART3Writes(const char * Pbuf,u8 Len);
__inline u8 USART1Write(const char ch);
__inline u8 USART2Write(const char ch);
__inline u8 USART3Write(const char ch);

#endif
