#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "gd32f30x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/5/27
//�汾��V1.3
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
////////////////////////////////////////////////////////////////////////////////// 
//extern u8 USART_RX_BUF[64];     //���ջ���,���63���ֽ�.ĩ�ֽ�Ϊ���з� 
//extern u8 USART_RX_STA;         //����״̬���	


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
