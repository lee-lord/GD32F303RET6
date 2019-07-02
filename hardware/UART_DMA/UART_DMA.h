#ifndef _UART_DMA_H_
#define _UART_DMA_H_
#include "sysType.h"
#define TxBufLen  1024
typedef struct {
U16 Txfile_head;
U16 Txfile_tail;
U16 Rxfile_head;
U16 Rxfile_tail;
U8 *RxBuff;//[TxBufLen];
U8 *TxBuff;//[TxBufLen];	
U8 busy;
U8 state;	
}File_Buffer;


void usartA_config(void);
void UartA_DmaInitial(void);

void usartB_config(void);
void UartB_DmaInitial(void);

void usartC_config(void);
void UartC_DmaInitial(void);




U8 UartA_read(void);
U8 UartB_read(void);
U8 UartC_read(void);
	
U16 UartA_Available(void);
U16 UartB_Available(void);
U16 UartC_Available(void);
	
__inline void UartA_write(U8 * data,U16 len);
__inline void UartB_write(U8 * data,U16 len);	
__inline void UartC_write(U8 * data,U16 len);

#endif