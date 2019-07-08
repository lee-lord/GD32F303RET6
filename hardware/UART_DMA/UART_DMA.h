#ifndef _UART_DMA_H_
#define _UART_DMA_H_
#include "sysType.h"
#define TxBufLen  1024

#define USART0_DATA_ADDRESS    ((uint32_t)0x40013804)
#define USART1_DATA_ADDRESS    ((uint32_t)0x40004404)
#define USART2_DATA_ADDRESS    ((uint32_t)0x40004804)

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
/*
   if(dev==USART1)
        {dev->dmadev=DMA1;
       // nvic_irq_set_priority(dev->irq_num,13);
        }
    else if(dev==USART2)   
         {dev->dmadev=DMA1;//nvic_irq_set_priority(dev->irq_num,13);//set usart IRQ priority than SYSTICK IRQ's 14 by lee
                  }
    else if(dev==USART3)
         {dev->dmadev=DMA1;//nvic_irq_set_priority(dev->irq_num,15);//set usart IRQ priority than SYSTICK IRQ's 14 by lee
          // nvic_irq_set_priority(dev->irq_num,13);
          ch1--tx 
          ch2--rx
         }
    else if(dev==UART4)
        {dev->dmadev=DMA2;//nvic_irq_set_priority(dev->irq_num,13);//set usart IRQ priority than SYSTICK IRQ's 14 by lee
       // nvic_irq_set_priority(dev->irq_num,13);
        }
*/

void usartAC_config(U32 UartX,U32 baudrate);
void UartA_DmaInitial(U32 UartX);
void UartC_DmaInitial(U32 UartX);

void UartB_DmaInitial(void);





U8 UartA_read(void);
U8 UartB_read(void);
U8 UartC_read(void);
	
U16 UartA_Available(void);
U16 UartB_Available(void);
U16 UartC_Available(void);
	
 void UartA_write(U8 * data,U16 len);
 void UartB_write(U8 * data,U16 len);	
 void UartC_write(U8 * data,U16 len);

 
 
void UCprintf(const U8 *fmt, ...);
void UAprintf(const U8 *fmt, ...);
#endif