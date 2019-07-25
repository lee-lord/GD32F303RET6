#ifndef _UART_DMA_H_
#define _UART_DMA_H_
#include "gd32f30x.h"
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


extern volatile File_Buffer Uart1DMAbuf;
extern volatile File_Buffer Uart2DMAbuf;

 
extern volatile dma_parameter_struct DMA_CH1_Tx_InitStruc;
extern volatile dma_parameter_struct DMA_CH2_Tx_InitStruc;

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

// void UartB_DmaInitial(void);

__inline U8 UartA_read(void);
__inline U8 UartB_read(void);
__inline U8 UartC_read(void);
	
__inline U16 UartA_Available(void);
__inline U16 UartB_Available(void);
__inline U16 UartC_Available(void);
	
__inline  void UartA_write(U8 * data,U16 len);
__inline  void UartB_write(U8 * data,U16 len);	
__inline  void UartC_write(U8 * data,U16 len);






 
 
void UCprintf(const U8 *fmt, ...);
void UAprintf(const U8 *fmt, ...);
#endif

