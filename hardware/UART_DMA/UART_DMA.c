#include "gd32f30x.h"
#include "sysType.h"
#include "UART_DMA.h"
volatile File_Buffer Uart1DMAbuf;
volatile File_Buffer Uart2DMAbuf;

 
volatile dma_parameter_struct DMA_CH1_Tx_InitStruc;
volatile dma_parameter_struct DMA_CH2_Tx_InitStruc;

U8 Com1RX[TxBufLen];
U8 Com1TX[TxBufLen];
U8 Com2RX[TxBufLen];
U8 Com2TX[TxBufLen];

void DMAbufInit(void)
{
   Uart1DMAbuf.Txfile_head=0;
	 Uart1DMAbuf.Txfile_tail=0;
   Uart1DMAbuf.Rxfile_head=0;
	 Uart1DMAbuf.Rxfile_tail=0;	
	Uart1DMAbuf.RxBuff=Com1RX;
	Uart1DMAbuf.TxBuff=Com1TX;
	 Uart1DMAbuf.busy=0;
	
   Uart2DMAbuf.Txfile_head=0;
	 Uart2DMAbuf.Txfile_tail=0;
   Uart2DMAbuf.Rxfile_head=0;
	 Uart2DMAbuf.Rxfile_tail=0;
   Uart2DMAbuf.RxBuff=Com2RX;
	 Uart2DMAbuf.TxBuff=Com2TX;	
	 Uart2DMAbuf.busy=0;

}

/*
void DMA_Config_Tx(DMA_Channel_TypeDef* DMA_CHx,DMA_InitTypeDef *DMAstruc,U32 cpar,U32 cmar,U16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	
    DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值
	DMAstruc->DMA_PeripheralBaseAddr = cpar;  //DMA外设ADC基地址
	DMAstruc->DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMAstruc->DMA_DIR = DMA_DIR_PeripheralDST;  //外设作为数据传输的目的地
	DMAstruc->DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMAstruc->DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMAstruc->DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMAstruc->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMAstruc->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; ///工作在正常缓存模式
	DMAstruc->DMA_Priority = DMA_Priority_Medium; //DMA通道 /数据宽度为8位
	DMAstruc->DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;//x拥有中优先级 
	DMAstruc->DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, DMAstruc);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
	  	
}

void DMA_Config_RX(DMA_Channel_TypeDef* DMA_CHx,DMA_InitTypeDef *DMAstruc,U32 cpar,U32 cmar,U16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	
    DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值

	DMAstruc->DMA_PeripheralBaseAddr = cpar;  //DMA外设ADC基地址
	DMAstruc->DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMAstruc->DMA_DIR = DMA_DIR_PeripheralSRC;  //外设作为数据传输的目的地
	DMAstruc->DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMAstruc->DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMAstruc->DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMAstruc->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMAstruc->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMAstruc->DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 

	DMAstruc->DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;  //工作在正常缓存模式
    DMAstruc->DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, DMAstruc);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
	  	
} 
*/

void UartA_DmaInitial(void)//U32 DMA_X,dma_channel_enum channelRx,dma_channel_enum channelTx,)
{
    dma_parameter_struct dma_init_struct;
    /* enable DMA0 */
    rcu_periph_clock_enable(RCU_DMA0);	//使能DMA传输
    /* initialize USART */
    usartA_config();
    //////////////initial TX DMA 
    /* deinitialize DMA channel3(USART0 tx) */
    dma_deinit(DMA0, DMA_CH3);   //将DMA的通道1寄存器重设为缺省值
    DMA_CH1_Tx_InitStruc.direction = DMA_MEMORY_TO_PERIPHERAL;//外设作为数据传输的目的地
    //DMAstruc->DMA_DIR = DMA_DIR_PeripheralDST;  
    DMA_CH1_Tx_InitStruc.memory_addr = (uint32_t)Com1TX;//DMA内存基地址
    //DMAstruc->DMA_MemoryBaseAddr = cmar;  
    DMA_CH1_Tx_InitStruc.memory_inc = DMA_MEMORY_INCREASE_ENABLE;  //内存地址寄存器递增
    //DMAstruc->DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_CH1_Tx_InitStruc.memory_width = DMA_PERIPHERAL_WIDTH_8BIT;//DMA通道 /数据宽度为8位
    //DMAstruc->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
    DMA_CH1_Tx_InitStruc.number = TxBufLen;//ARRAYNUM(txbuffer);//DMA通道的DMA缓存的大小
    //DMAstruc->DMA_BufferSize = cndtr;  
    DMA_CH1_Tx_InitStruc.periph_addr = USART0;//DMA外设ADC基地址
    //DMAstruc->DMA_PeripheralBaseAddr = cpar;  
    DMA_CH1_Tx_InitStruc.periph_inc = DMA_PERIPH_INCREASE_DISABLE;  //外设地址寄存器不变
    //DMAstruc->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_CH1_Tx_InitStruc.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;	  //数据宽度为8位
    //DMAstruc->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_CH1_Tx_InitStruc.priority = DMA_PRIORITY_ULTRA_HIGH;
    //DMAstruc->DMA_Priority = DMA_Priority_Medium; 
    dma_init(DMA0, DMA_CH4, &DMA_CH1_Tx_InitStruc);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH3);   
    //DMAstruc->DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;//x拥有中优先级
    dma_memory_to_memory_disable(DMA0, DMA_CH3); //DMA通道x没有设置为内存到内存传输
    ///DMAstruc->DMA_M2M = DMA_M2M_Disable;  
    /* enable DMA channel3 */
    dma_channel_enable(DMA0, DMA_CH3);
    /* USART DMA enable for transmission and reception */
    usart_dma_transmit_config(USART0, USART_DENT_ENABLE); 	
	
    /////here we intial the interupt of DMA 
   // DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
    dma_interrupt_enable(DMA0,DMA_CH3,DMA_INT_FTF);

/////////initial DMA RX DMA mode
	dma_deinit(DMA0, DMA_CH4);   //将DMA的通道1寄存器重设为缺省值
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;//外设作为数据传输的目的地
    //DMAstruc->DMA_DIR = DMA_DIR_PeripheralDST;  
    dma_init_struct.memory_addr = (uint32_t)Com1RX;//DMA内存基地址
    //DMAstruc->DMA_MemoryBaseAddr = cmar;  
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;  //内存地址寄存器递增
    //DMAstruc->DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_init_struct.memory_width = DMA_PERIPHERAL_WIDTH_8BIT;//DMA通道 /数据宽度为8位
    //DMAstruc->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
    dma_init_struct.number = TxBufLen;//ARRAYNUM(txbuffer);//DMA通道的DMA缓存的大小

    //DMAstruc->DMA_BufferSize = cndtr;  
    dma_init_struct.periph_addr = USART0;//DMA外设ADC基地址
    //DMAstruc->DMA_PeripheralBaseAddr = cpar;  
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;  //外设地址寄存器不变
    //DMAstruc->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;	  //数据宽度为8位
    //DMAstruc->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    //DMAstruc->DMA_Priority = DMA_Priority_Medium; 
    dma_init(DMA0, DMA_CH4, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_enable(DMA0, DMA_CH4);   
    //DMAstruc->DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Circular;//x拥有中优先级
    dma_memory_to_memory_disable(DMA0, DMA_CH4); //DMA通道x没有设置为内存到内存传输
    ///DMAstruc->DMA_M2M = DMA_M2M_Disable;  
    /* enable DMA channel3 */
    dma_channel_enable(DMA0, DMA_CH4);

    usart_dma_receive_config(USART0, USART_DENR_ENABLE);
  /////here we intial the interupt of DMA 
}

void usartA_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_AF);
    /* configure USART Tx as alternate function push-pull */
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9|GPIO_PIN_8);
    gpio_init(GPIOA,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_10);
    /* configure USART synchronous mode */
    usart_synchronous_clock_enable(USART0);
    usart_synchronous_clock_config(USART0, USART_CLEN_EN, USART_CPH_2CK, USART_CPL_HIGH);
    
    usart_baudrate_set(USART0, 115200);
    /* configure USART transmitter */
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    /* configure USART receiver */
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    /* enable USART */
    usart_enable(USART0);
}

void DMA_SendEn(uint32_t DMA_X,dma_channel_enum channelx,dma_parameter_struct *DMA_Tx_InitStruc,U32 dataLen,U32 dataAdd)
{ 
	dma_channel_disable(DMA_X, channelx );  //关闭USART1 TX DMA1 所指示的通道   4   
    DMA_Tx_InitStruc->number     = 	 dataLen;
	DMA_Tx_InitStruc->memory_addr = dataAdd;  //DMA内存基地址
	dma_init(DMA_X, channelx, DMA_Tx_InitStruc);
  //DMA_ITConfig(DMA_CHx,DMA_IT_TC,ENABLE);
 	dma_channel_enable(DMA_X, channelx);  //使能USART1 TX DMA1 所指示的通道 
}	

void DMA0_Channel3_IRQHandler(void)
{
U32 SramAdd=0;
U32 tempLen=0;
if(dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF)){     
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_G);	 
		SramAdd = (U32)(&Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_head]);
			///正常情况的发送，天冲在前，发送在后。
			 if(Uart1DMAbuf.Txfile_head<Uart1DMAbuf.Txfile_tail)
			 {
				 tempLen = Uart1DMAbuf.Txfile_tail -Uart1DMAbuf.Txfile_head;
				   Uart1DMAbuf.Txfile_head =Uart1DMAbuf.Txfile_tail;
			 }// 队列颠倒，填充在后，发送在前，先把队列的尾部发送完成。发送头归零。等下一次发送。
			 else if(Uart1DMAbuf.Txfile_head>Uart1DMAbuf.Txfile_tail)
			 {  //发送到buf的尾端
				 // SramAdd = (U32)(&Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_head]);
				  tempLen = TxBufLen -Uart1DMAbuf.Txfile_head;
        	Uart1DMAbuf.Txfile_head =0; //LED1=!LED1;
			 }else if(Uart1DMAbuf.Txfile_head==Uart1DMAbuf.Txfile_tail)
			 {
				 Uart1DMAbuf.busy=0;// LED0=!LED0;
				  return;
       }
//			if(Uart1DMAbuf.Txfile_head==TxBufLen-1)
//				LED0=!LED0;
		if(tempLen>0)
		{	 
		Uart1DMAbuf.busy=1;	 
		DMA_SendEn(DMA0, DMA_CH3,&DMA_CH1_Tx_InitStruc,tempLen,SramAdd);
			//LED1=!LED1;
		}
		else
		Uart1DMAbuf.busy=0;
	}
}


void DMA0_Channel6_IRQHandler(void)
{
	U32 SramAdd=0;
	U32 tempLen=0;
if(dma_interrupt_flag_get(DMA0, DMA_CH6, DMA_INT_FLAG_FTF)){     
        dma_interrupt_flag_clear(DMA0, DMA_CH6, DMA_INT_FLAG_G);
		///正常情况的发送，天冲在前，发送在后。
		SramAdd = (U32)(&Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_head]);	
			 if(Uart2DMAbuf.Txfile_head<Uart2DMAbuf.Txfile_tail)
			 {
				 tempLen = Uart2DMAbuf.Txfile_tail -Uart2DMAbuf.Txfile_head;
				   Uart2DMAbuf.Txfile_head =Uart2DMAbuf.Txfile_tail;
				// LED0=!LED0;
			 }// 队列颠倒，填充在后，发送在前，先把队列的尾部发送完成。发送头归零。等下一次发送。
			 else if(Uart2DMAbuf.Txfile_head>Uart2DMAbuf.Txfile_tail)
			 {  //发送到buf的尾端
	//			  SramAdd = (U32)(&Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_head]);
				  tempLen = TxBufLen -Uart2DMAbuf.Txfile_head;
        	Uart2DMAbuf.Txfile_head =0;
				// LED1=!LED1;
			 }
	   //if(tempLen==1)  LED1=!LED1;
		if(tempLen)
		{ 
			Uart2DMAbuf.busy=1;
		  DMA_SendEn(DMA0, DMA_CH6,&DMA_CH1_Tx_InitStruc,tempLen,SramAdd);
		}
		else
		Uart2DMAbuf.busy=0;
	}
   // printf("\n\nRx half ISR\n"); 	
}



__inline U8 UartA_read(void)
{
  U8 tmp=0;
	tmp = Uart1DMAbuf.RxBuff[Uart1DMAbuf.Rxfile_head++];
	if(Uart1DMAbuf.Rxfile_head>(TxBufLen-1))
	  Uart1DMAbuf.Rxfile_head=0;
	return tmp;
}

__inline U8 UartB_read(void)
{
  U8 tmp=0;
	tmp = Uart2DMAbuf.RxBuff[Uart2DMAbuf.Rxfile_head++];
	if(Uart2DMAbuf.Rxfile_head>(TxBufLen-1))
	  Uart2DMAbuf.Rxfile_head=0;
	return tmp;
}


U8 UartC_read(void)
{
	
}



__inline U16 UartA_Available(void)
{
	 Uart1DMAbuf.Rxfile_tail=TxBufLen-dma_transfer_number_get(DMA0, DMA_CH4);//DMA_GetCurrDataCounter(DMA1_Channel5);	
  return (Uart1DMAbuf.Rxfile_tail>=Uart1DMAbuf.Rxfile_head)? (Uart1DMAbuf.Rxfile_tail-Uart1DMAbuf.Rxfile_head ):(TxBufLen+Uart1DMAbuf.Rxfile_tail-Uart1DMAbuf.Rxfile_head);
}


__inline U16 UartB_Available(void)
{
 Uart2DMAbuf.Rxfile_tail=TxBufLen-dma_transfer_number_get(DMA0, DMA_CH5);//DMA_GetCurrDataCounter(DMA1_Channel6);	
  return (Uart2DMAbuf.Rxfile_tail>=Uart2DMAbuf.Rxfile_head)? (Uart2DMAbuf.Rxfile_tail-Uart2DMAbuf.Rxfile_head ):(TxBufLen+Uart2DMAbuf.Rxfile_tail-Uart2DMAbuf.Rxfile_head);
}


U16 UartC_Available(void)
{
	
}


////////由中断配合完成一次发送。
////////DMA////////////TAIL//// ---set current data len
///////////////DMA/////////TAIL ---wait for DMA cpmplete
///////TAIL/////////DMA//////// ---set current data len ,let it go 
///////TAIL/////////////////DMA///
__inline void UartA_write(U8 *pStr ,U16 len)
{
   U16 i=0;
	 U32 tempLen=0;
	 U32 SramAdd=0;
	   for(i=0;i<len;i++)
         {
			 Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_tail++]=*(pStr+i);
			 if(Uart1DMAbuf.Txfile_tail>(TxBufLen-1)){ Uart1DMAbuf.Txfile_tail=0; 
                                             //LED0=!LED0;
  											 }
		 }

		if(Uart1DMAbuf.busy==0) // is idle
		{
			//printf("\n\nBusy=0 send:\n");
				SramAdd = (U32)(&Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_head]);
			///正常情况的发送，天冲在前，发送在后。
			 if(Uart1DMAbuf.Txfile_head<Uart1DMAbuf.Txfile_tail)
			 {
				 tempLen = Uart1DMAbuf.Txfile_tail -Uart1DMAbuf.Txfile_head;
				 Uart1DMAbuf.Txfile_head =Uart1DMAbuf.Txfile_tail;
			 }// 队列颠倒，填充在后，发送在前，先把队列的尾部发送完成。发送头归零。等下一次发送。
			 else  if(Uart1DMAbuf.Txfile_head>Uart1DMAbuf.Txfile_tail)
			 {  //发送到buf的尾端
				 // SramAdd = (U32)(&Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_head]);
				  tempLen = TxBufLen -Uart1DMAbuf.Txfile_head;
				 // tempLen = TxBufLen -Uart1DMAbuf.Txfile_head; 
        	Uart1DMAbuf.Txfile_head =0;
				 //LED1=!LED1;
			 }
			 if(tempLen)
			 {
			 	Uart1DMAbuf.busy=1;
				 DMA_SendEn(DMA0, DMA_CH3,&DMA_CH1_Tx_InitStruc,tempLen,SramAdd);
			 }else
			   Uart1DMAbuf.busy=0;
		}
}


////////DMA////////////TAIL//// ---set current data len
///////////////DMA/////////TAIL ---wait for DMA cpmplete
///////TAIL/////////DMA//////// ---set current data len ,let it go 
///////TAIL/////////////////DMA///
__inline void UartB_write(U8 *pStr ,U16 len)
{
   U16 i=0;
	static U8 overlap=0;
	 U32 tempLen=0;
	 U32 SramAdd=0;
	  for(i=0;i<len;i++)
         {
			 Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_tail++]=*(pStr+i);
			 if(Uart2DMAbuf.Txfile_tail>(TxBufLen-1)){ Uart2DMAbuf.Txfile_tail=0;}
		 }
		 
		if(Uart2DMAbuf.busy==0) // is idle
		{
				SramAdd = (U32)(&Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_head]);
			///正常情况的发送，天冲在前，发送在后。
			 if(Uart2DMAbuf.Txfile_head<Uart2DMAbuf.Txfile_tail)
			 {
				 tempLen = Uart2DMAbuf.Txfile_tail -Uart2DMAbuf.Txfile_head;
				 Uart2DMAbuf.Txfile_head =Uart2DMAbuf.Txfile_tail;
				 //LED1=!LED1;
			 }// 队列颠倒，填充在后，发送在前，先把队列的尾部发送完成。发送头归零。等下一次发送。
			 else if(Uart2DMAbuf.Txfile_head>Uart2DMAbuf.Txfile_tail)
			 {  //发送到buf的尾端
				 // SramAdd = (U32)(&Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_head]);
				  tempLen = TxBufLen -Uart2DMAbuf.Txfile_head;
        	Uart2DMAbuf.Txfile_head =0;	 
			 }
			 if(tempLen)
			 {
			 Uart2DMAbuf.busy=1;	
			//DMA_SendEn(DMA1_Channel7,&DMA_CH2_Tx_InitStruc,tempLen,SramAdd);
			DMA_SendEn(DMA0, DMA_CH6,&DMA_CH1_Tx_InitStruc,tempLen,SramAdd);
			 }
			 else
				Uart2DMAbuf.busy=0; 
		}
}

void UartC_write(U8 * data,U16 len)
{
	
}

