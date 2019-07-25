#include "gd32f30x.h"
#include "sysType.h"
#include "UART_DMA.h"
#include "MyMath.h"
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


void UartC_DmaInitial(U32 UartX)//U32 DMA_X,dma_channel_enum channelRx,dma_channel_enum channelTx,)
{
    dma_parameter_struct dma_init_struct;
    DMAbufInit();
    /* initialize USART */
    usartAC_config(USART2,256000);
    /* enable DMA0 */
    rcu_periph_clock_enable(RCU_DMA0);	//使能DMA传输
/////////initial DMA RX DMA mode
    dma_deinit(DMA0, DMA_CH2);   //将DMA的通道1寄存器重设为缺省值
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;//外设作为数据传输的目的地
    //DMAstruc->DMA_DIR = DMA_DIR_PeripheralDST;  
    dma_init_struct.memory_addr = (uint32_t)Com2RX;//DMA内存基地址
    //DMAstruc->DMA_MemoryBaseAddr = cmar;  
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;  //内存地址寄存器递增
    //DMAstruc->DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;//DMA通道 /数据宽度为8位
    //DMAstruc->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
    dma_init_struct.number = TxBufLen;//ARRAYNUM(txbuffer);//DMA通道的DMA缓存的大小
    //DMAstruc->DMA_BufferSize = cndtr;  
    dma_init_struct.periph_addr = USART2_DATA_ADDRESS;//USART_DATA(USART2);//DMA外设ADC基地址
    //DMAstruc->DMA_PeripheralBaseAddr = cpar;  
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;  //外设地址寄存器不变
    //DMAstruc->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;     //数据宽度为8位
    //DMAstruc->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    //DMAstruc->DMA_Priority = DMA_Priority_Medium; 
    dma_init(DMA0, DMA_CH2, &dma_init_struct);

    /* configure DMA mode */
    dma_circulation_enable(DMA0, DMA_CH2);   
    //DMAstruc->DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Circular;//x拥有中优先级
    dma_memory_to_memory_disable(DMA0, DMA_CH2); //DMA通道x没有设置为内存到内存传输
    ///DMAstruc->DMA_M2M = DMA_M2M_Disable;  
    /* enable DMA channel3 */

             ////////////dma_channel_enable(DMA0, DMA_CH2);
    dma_channel_enable(DMA0, DMA_CH2);
    //DMA_Cmd(DMA_CHx, ENABLE);  //使能USART1 TX DMA1 所指示的通道 
      /////here we intial the interupt of DMA 
     usart_dma_receive_config(USART2, USART_DENR_ENABLE);



    //////////////initial TX DMA 
    /* deinitialize DMA channel3(USART0 tx) */
    dma_deinit(DMA0, DMA_CH1);   //将DMA的通道1寄存器重设为缺省值
    DMA_CH2_Tx_InitStruc.direction = DMA_MEMORY_TO_PERIPHERAL;//外设作为数据传输的目的地
    //DMAstruc->DMA_DIR = DMA_DIR_PeripheralDST;  
    DMA_CH2_Tx_InitStruc.memory_addr = (uint32_t)Com2TX;//DMA内存基地址
    //DMAstruc->DMA_MemoryBaseAddr = cmar;  
    DMA_CH2_Tx_InitStruc.memory_inc = DMA_MEMORY_INCREASE_ENABLE;  //内存地址寄存器递增
    //DMAstruc->DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_CH2_Tx_InitStruc.memory_width = DMA_MEMORY_WIDTH_8BIT;//DMA通道 /数据宽度为8位
    //DMAstruc->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
    DMA_CH2_Tx_InitStruc.number = TxBufLen;//ARRAYNUM(txbuffer);//DMA通道的DMA缓存的大小
    //DMAstruc->DMA_BufferSize = cndtr;  
    DMA_CH2_Tx_InitStruc.periph_addr = USART2_DATA_ADDRESS;//USART_DATA(USART2);//DMA外设ADC基地址
    //DMAstruc->DMA_PeripheralBaseAddr = cpar;  
    DMA_CH2_Tx_InitStruc.periph_inc = DMA_PERIPH_INCREASE_DISABLE;  //外设地址寄存器不变
    //DMAstruc->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_CH2_Tx_InitStruc.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;	  //数据宽度为8位
    //DMAstruc->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_CH2_Tx_InitStruc.priority = DMA_PRIORITY_ULTRA_HIGH;
    //DMAstruc->DMA_Priority = DMA_Priority_Medium; 
    dma_init(DMA0, DMA_CH1, (dma_parameter_struct *)&DMA_CH2_Tx_InitStruc);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH1);   
    //DMAstruc->DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;//x拥有中优先级
    dma_memory_to_memory_disable(DMA0, DMA_CH1); //DMA通道x没有设置为内存到内存传输
    ///DMAstruc->DMA_M2M = DMA_M2M_Disable;  
    /* enable DMA channel1 */
    dma_channel_enable(DMA0, DMA_CH1);

    /////here we intial the interupt of DMA 
   // DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
    dma_interrupt_enable(DMA0,DMA_CH1,DMA_INT_FTF);

    /* USART DMA enable for transmission and reception */
    usart_dma_transmit_config(USART2, USART_DENT_ENABLE);   

    // nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    // nvic_irq_enable(DMA0_Channel1_IRQn, 1, 4);
}


void UartA_DmaInitial(U32 UartX)//U32 DMA_X,dma_channel_enum channelRx,dma_channel_enum channelTx,)
{
    dma_parameter_struct dma_init_struct;
    DMAbufInit();
    /* enable DMA0 */
    rcu_periph_clock_enable(RCU_DMA0);	//使能DMA传输
    /* initialize USART */
    usartAC_config(USART0,256000);
    //////////////initial TX DMA 
    /* deinitialize DMA channel3(USART0 tx) */
    dma_deinit(DMA0, DMA_CH3);   //将DMA的通道1寄存器重设为缺省值
    DMA_CH1_Tx_InitStruc.direction = DMA_MEMORY_TO_PERIPHERAL;//外设作为数据传输的目的地
    //DMAstruc->DMA_DIR = DMA_DIR_PeripheralDST;  
    DMA_CH1_Tx_InitStruc.memory_addr = (uint32_t)Com1TX;//DMA内存基地址
    //DMAstruc->DMA_MemoryBaseAddr = cmar;  
    DMA_CH1_Tx_InitStruc.memory_inc = DMA_MEMORY_INCREASE_ENABLE;  //内存地址寄存器递增
    //DMAstruc->DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_CH1_Tx_InitStruc.memory_width = DMA_MEMORY_WIDTH_8BIT;//DMA通道 /数据宽度为8位
    //DMAstruc->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
    DMA_CH1_Tx_InitStruc.number = TxBufLen;//ARRAYNUM(txbuffer);//DMA通道的DMA缓存的大小
    //DMAstruc->DMA_BufferSize = cndtr;  
    DMA_CH1_Tx_InitStruc.periph_addr = USART0_DATA_ADDRESS;//USART_DATA(USART0);//DMA外设ADC基地址
    //DMAstruc->DMA_PeripheralBaseAddr = cpar;  
    DMA_CH1_Tx_InitStruc.periph_inc = DMA_PERIPH_INCREASE_DISABLE;  //外设地址寄存器不变
    //DMAstruc->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_CH1_Tx_InitStruc.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;	  //数据宽度为8位
    //DMAstruc->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_CH1_Tx_InitStruc.priority = DMA_PRIORITY_ULTRA_HIGH;
    //DMAstruc->DMA_Priority = DMA_Priority_Medium; 
    dma_init(DMA0, DMA_CH3, (dma_parameter_struct *)&DMA_CH1_Tx_InitStruc);
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
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;//DMA通道 /数据宽度为8位
    //DMAstruc->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
    dma_init_struct.number = TxBufLen;//ARRAYNUM(txbuffer);//DMA通道的DMA缓存的大小

    //DMAstruc->DMA_BufferSize = cndtr;  
    dma_init_struct.periph_addr = USART0_DATA_ADDRESS;//USART_DATA(USART0);//DMA外设ADC基地址
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
    // nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    // nvic_irq_enable(DMA0_Channel3_IRQn, 1, 3);
}


void usartAC_config(U32 UartX,U32 baudrate)
{
	if(UartX==USART0)
    {
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_AF);
    /* configure USART Tx as alternate function push-pull */
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9|GPIO_PIN_8);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_10);//GPIO_MODE_IPU
    }
    else if(UartX==USART2)
    {
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_USART2);
    rcu_periph_clock_enable(RCU_AF);
    /* configure USART Tx as alternate function push-pull */
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10);//
    gpio_init(GPIOB,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_11);//GPIO_MODE_IN_FLOATING
    }

    /* configure USART synchronous mode */
    //usart_synchronous_clock_enable(UartX);
    //usart_synchronous_clock_config(UartX, USART_CLEN_EN, USART_CPH_2CK, USART_CPL_HIGH);
    
    usart_baudrate_set(UartX,baudrate);//115200
    usart_parity_config(UartX,USART_PM_NONE);//No parity  
    usart_word_length_set(UartX,USART_WL_8BIT);// 8 bit
    usart_stop_bit_set(UartX,USART_STB_1BIT);// 1 stop bit
    /* configure USART transmitter */
    usart_transmit_config(UartX, USART_TRANSMIT_ENABLE);
    /* configure USART receiver */
    usart_receive_config(UartX, USART_RECEIVE_ENABLE);
    /* enable USART */
    usart_enable(UartX);
}




__inline  void DMA_SendEn(uint32_t DMA_X,dma_channel_enum channelx,dma_parameter_struct *DMA_Tx_InitStruc,U32 dataLen,U32 dataAdd)
{ 
  dma_channel_disable(DMA_X, channelx );  //关闭USART1 TX DMA1 所指示的通道   4   
    DMA_Tx_InitStruc->number     =   dataLen;
  DMA_Tx_InitStruc->memory_addr = dataAdd;  //DMA内存基地址
  dma_init(DMA_X, channelx, DMA_Tx_InitStruc);
  //DMA_ITConfig(DMA_CHx,DMA_IT_TC,ENABLE);
  dma_channel_enable(DMA_X, channelx);  //使能USART1 TX DMA1 所指示的通道 
} 

/////CH3-Tx CH4-Rx for Uarta
  void DMA0_Channel3_IRQHandler(void)
{
U32 SramAdd=0;
U32 tempLen=0;
if(dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF)){     
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_FTF);   
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
//      if(Uart1DMAbuf.Txfile_head==TxBufLen-1)
//        LED0=!LED0;
    if(tempLen>0)
    {  
    Uart1DMAbuf.busy=1;  
    DMA_SendEn(DMA0, DMA_CH3,(dma_parameter_struct *)&DMA_CH1_Tx_InitStruc,tempLen,SramAdd);
      //LED1=!LED1;
    }
    else
    Uart1DMAbuf.busy=0;
  }
}

///CH1--Tx Ch2--Rx for UartC
  void DMA0_Channel1_IRQHandler(void)
{
  U32 SramAdd=0;
  U32 tempLen=0;
if(dma_interrupt_flag_get(DMA0, DMA_CH1, DMA_INT_FLAG_FTF)){     
        dma_interrupt_flag_clear(DMA0, DMA_CH1, DMA_INT_FLAG_FTF);
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
  //        SramAdd = (U32)(&Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_head]);
          tempLen = TxBufLen -Uart2DMAbuf.Txfile_head;
                 Uart2DMAbuf.Txfile_head =0;
        // LED1=!LED1;
       }
     //if(tempLen==1)  LED1=!LED1;
    if(tempLen)
    { 
      Uart2DMAbuf.busy=1;
      DMA_SendEn(DMA0, DMA_CH1,(dma_parameter_struct *)&DMA_CH2_Tx_InitStruc,tempLen,SramAdd);
    }
    else
    Uart2DMAbuf.busy=0;
  }
   // printf("\n\nRx half ISR\n");  
}



  U8 UartA_read(void)
{
  U8 tmp=0;
  tmp = Uart1DMAbuf.RxBuff[Uart1DMAbuf.Rxfile_head++];
  if(Uart1DMAbuf.Rxfile_head>(TxBufLen-1))
    Uart1DMAbuf.Rxfile_head=0;
  return tmp;
}

   U8 UartB_read(void)
{
  return 0;
}


  U8 UartC_read(void)
{
  U8 tmp=0;
    tmp = Uart2DMAbuf.RxBuff[Uart2DMAbuf.Rxfile_head++];
    if(Uart2DMAbuf.Rxfile_head>(TxBufLen-1))
      Uart2DMAbuf.Rxfile_head=0;
    return tmp;
}



   U16 UartA_Available(void)
{
   Uart1DMAbuf.Rxfile_tail=TxBufLen-dma_transfer_number_get(DMA0, DMA_CH4);//DMA_GetCurrDataCounter(DMA1_Channel5); 
  return (Uart1DMAbuf.Rxfile_tail>=Uart1DMAbuf.Rxfile_head)? (Uart1DMAbuf.Rxfile_tail-Uart1DMAbuf.Rxfile_head ):(TxBufLen+Uart1DMAbuf.Rxfile_tail-Uart1DMAbuf.Rxfile_head);
}


  U16 UartB_Available(void)
{
 return 0;

}


   U16 UartC_Available(void)
{
   Uart2DMAbuf.Rxfile_tail=TxBufLen-dma_transfer_number_get(DMA0, DMA_CH2);//DMA_GetCurrDataCounter(DMA1_Channel6); 
  return (Uart2DMAbuf.Rxfile_tail>=Uart2DMAbuf.Rxfile_head)? (Uart2DMAbuf.Rxfile_tail-Uart2DMAbuf.Rxfile_head ):(TxBufLen+Uart2DMAbuf.Rxfile_tail-Uart2DMAbuf.Rxfile_head);
}


////////由中断配合完成一次发送。
////////DMA////////////TAIL//// ---set current data len
///////////////DMA/////////TAIL ---wait for DMA cpmplete
///////TAIL/////////DMA//////// ---set current data len ,let it go 
///////TAIL/////////////////DMA///
   void UartA_write(U8 *pStr ,U16 len)
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
         DMA_SendEn(DMA0, DMA_CH3,(dma_parameter_struct *)&DMA_CH1_Tx_InitStruc,tempLen,SramAdd);
       }else
         Uart1DMAbuf.busy=0;
    }
}

  void UartB_write(U8 * pStr,U16 len)
{
    
}


////////DMA////////////TAIL//// ---set current data len
///////////////DMA/////////TAIL ---wait for DMA cpmplete
///////TAIL/////////DMA//////// ---set current data len ,let it go 
///////TAIL/////////////////DMA///
   void UartC_write(U8 *pStr ,U16 len)
{
   U16 i=0;
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
         DMA_SendEn(DMA0, DMA_CH1,(dma_parameter_struct *)&DMA_CH2_Tx_InitStruc,tempLen,SramAdd);
       }
       else
        Uart2DMAbuf.busy=0; 
    }
}








/* '__ftoa_engine' return next flags (in buf[0]):   */
#define FTOA_MINUS  1
#define FTOA_ZERO   2
#define FTOA_INF    4
#define FTOA_NAN    8
#define FTOA_CARRY  16  /* Carry was to master position.    */

#define FL_ZFILL    0x01
#define FL_PLUS     0x02
#define FL_SPACE    0x04
#define FL_LPAD     0x08
#define FL_ALT      0x10
#define FL_WIDTH    0x20
#define FL_PREC     0x40
#define FL_LONG     0x80

#define FL_PGMSTRING    FL_LONG
#define FL_NEGATIVE FL_LONG

#define FL_ALTUPP   FL_PLUS
#define FL_ALTHEX   FL_SPACE

#define FL_FLTUPP   FL_ALT
#define FL_FLTEXP   FL_PREC
#define FL_FLTFIX   FL_LONG



/* Next flags are to use with `base'. Unused fields are reserved.   */
#define XTOA_PREFIX 0x0100  /* put prefix for octal or hex  */
#define XTOA_UPPER  0x0200  /* use upper case letters   */
/*
 * 2^b ~= f * r * 10^e
 * where
 * i = b div 8
 * r = 2^(b mod 8)
 * f = factorTable[i]
 * e = exponentTable[i]
 */
U16 con_strnlen(const char *s,S16 maxlen)
{
    const char *p;

    for (p = s; *p != '\0' && ((p - s) < maxlen); ++p)
        ;

    return (U16)(p - s);
}

static const int8_t exponentTable[32]  = {
    -36, -33, -31, -29, -26, -24, -21, -19,
    -17, -14, -12, -9,  -7, -4, -2,  0,
    3, 5, 8, 10,    12, 15,  17, 20,
    22, 24, 27, 29,  32, 34, 36, 39
};

static const uint32_t factorTable[32]  = {
    2295887404UL,
    587747175UL,
    1504632769UL,
    3851859889UL,
    986076132UL,
    2524354897UL,
    646234854UL,
    1654361225UL,
    4235164736UL,
    1084202172UL,
    2775557562UL,
    710542736UL,
    1818989404UL,
    465661287UL,
    1192092896UL,
    3051757813UL,
    781250000UL,
    2000000000UL,
    512000000UL,
    1310720000UL,
    3355443200UL,
    858993459UL,
    2199023256UL,
    562949953UL,
    1441151881UL,
    3689348815UL,
    944473297UL,
    2417851639UL,
    618970020UL,
    1584563250UL,
    4056481921UL,
    1038459372UL
};

int16_t ftoa_engine(float val, char *buf, uint8_t precision, uint8_t maxDecimals) 
{
    uint8_t flags;

    // Bit reinterpretation hacks. This will ONLY work on little endian machines.
    uint8_t *valbits = (uint8_t*)&val;
    union {
        float v;
        uint32_t u;
    } x;
    x.v = val;
    uint32_t frac = x.u & 0x007fffffUL;

    if (precision>7) precision=7;

    // Read the sign, shift the exponent in place and delete it from frac.
    if (valbits[3] & (1<<7)) flags = FTOA_MINUS; else flags = 0;
    uint8_t exp = valbits[3]<<1;
    if(valbits[2] & (1<<7)) exp++;  // TODO possible but in case of subnormal

    // Test for easy cases, zero and NaN
    if(exp==0 && frac==0) {
            uint8_t i;
        buf[0] = flags | FTOA_ZERO;
        for(i=0; i<=precision; i++) {
            buf[i+1] = '0';
        }
        return 0;
    }

    if(exp == 0xff) {
        if(frac == 0) flags |= FTOA_INF; else flags |= FTOA_NAN;
    }

    // The implicit leading 1 is made explicit, except if value subnormal.
    if (exp != 0) {frac |= (1UL<<23);}

    uint8_t idx = exp>>3;
    int8_t exp10 = (exponentTable[idx]);

    // We COULD try making the multiplication in situ, where we make
    // frac and a 64 bit int overlap in memory and select/weigh the
    // upper 32 bits that way. For starters, this is less risky:
    int64_t prod = (int64_t)frac * (int64_t)(factorTable[idx]);

    // The expConvFactorTable are factor are correct iff the lower 3 exponent
    // bits are 1 (=7). Else we need to compensate by divding frac.
    // If the lower 3 bits are 7 we are right.
    // If the lower 3 bits are 6 we right-shift once
    // ..
    // If the lower 3 bits are 0 we right-shift 7x
    prod >>= (15-(exp & 7));

    // Now convert to decimal.
    uint8_t hadNonzeroDigit = 0; // a flag
    uint8_t outputIdx = 0;
    int64_t decimal = 100000000000000ull;

    do {
        char digit = '0';
        while(1) {// find the first nonzero digit or any of the next digits.
            while ((prod -= decimal) >= 0)
                digit++;
            // Now we got too low. Fix it by adding again, once.
            // it might appear more efficient to check before subtract, or
            // to save and restore last nonnegative value - but in fact
            // they take as long time and more space.
            prod += decimal;
            decimal /= 10;

            // If already found a leading nonzero digit, accept zeros.
            if (hadNonzeroDigit) break;

            // Else, don't return results with a leading zero! Instead
            // skip those and decrement exp10 accordingly.
            if(digit == '0') {
                exp10--;
                continue;
            }

            hadNonzeroDigit = 1;

            // Compute how many digits N to output.
            if(maxDecimals != 0) {                      // If limiting decimals...
                int8_t beforeDP = exp10+1;              // Digits before point
                if (beforeDP < 1) beforeDP = 1;         // Numbers < 1 should also output at least 1 digit.
                /*
                 * Below a simpler version of this:
                int8_t afterDP = outputNum - beforeDP;
                if (afterDP > maxDecimals-1)
                    afterDP = maxDecimals-1;
                outputNum = beforeDP + afterDP;
                */
                maxDecimals = maxDecimals+beforeDP-1;
                if (precision > maxDecimals)
                    precision = maxDecimals;

            } else {
                precision++;                            // Output one more digit than the param value.
            }

            break;
        }

        // Now have a digit.
        outputIdx++;
        if(digit < '0' + 10) // normal case.
            buf[outputIdx] = digit;
        else {
            // Abnormal case, write 9s and bail.
            // We might as well abuse hadNonzeroDigit as counter, it will not be used again.
            for(hadNonzeroDigit=outputIdx; hadNonzeroDigit>0; hadNonzeroDigit--)
                buf[hadNonzeroDigit] = '9';
            goto roundup; // this is ugly but it _is_ code derived from assembler :)
        }
    } while (outputIdx<precision);

    // Rounding:
    decimal *= 10;

    if (prod - (decimal >> 1) >= 0) {

    roundup:
        // Increment digit, cascade
        while(outputIdx != 0) {
            if(++buf[outputIdx] == '0' + 10) {
                if(outputIdx == 1) {
                    buf[outputIdx] = '1';
                    exp10++;
                    flags |= FTOA_CARRY;
                    break;
                } else
                    buf[outputIdx--] = '0'; // and the loop continues, carrying to next digit.
            }
            else break;
        }
    }

    buf[0] = flags;
    return exp10;
}


typedef struct {
    unsigned int index;
    unsigned char *buf;
} BUFFERCS;
//add by lee 2015-10-28 to fill all the char into a buf ,and then ,send total once.
void writeOneByte2Buffer(BUFFERCS *pts,unsigned char Chr)
{
  if(pts->index>255) return;
  pts->buf[pts->index]=Chr;
  pts->index++;

}

char * ultoa_invert (uint32_t val, char *s, uint8_t base) {
    if (base == 8) {
        do {
            *s = '0' + (val & 0x7);
            val >>= 3;
        } while(val);
        return s;
    }

    if (base == 16) {
        do {
            uint8_t digit = '0' + (val & 0xf);
#if XTOA_UPPER == 0
            if (digit > '0' + 9)
                digit += ('a' - '0' - 10);
#else
            if (digit > '0' + 9)
                digit += ('A' - '0' - 10);
#endif
            *s++ = digit;
            val >>= 4;
        } while(val);
        return s;
    }

    // Every base which in not hex and not oct is considered decimal.

    // 33 bits would have been enough.
    uint64_t xval = val;
    do {
        uint8_t saved = xval;
        xval &= ~1;
        xval += 2;
        xval += xval >> 1;      // *1.5
        xval += xval >> 4;      // *1.0625
        xval += xval >> 8;      // *1.00390625
        xval += xval >> 16;     // *1.000015259
        xval += xval >> 32;     // it all amounts to *1.6
        xval >>= 4;             // /16 ... so *1.6/16 is /10, fraction truncated.
        *s++ = '0' + saved - 10 * (uint8_t)xval;
    } while (xval);
    return s;
}

void print_vprintf (U8 UartX,const U8 *fmt, va_list ap)
{
        unsigned char c;        /* holds a char from the format string */
        unsigned char flags;
        unsigned char width;
        unsigned char prec;
        unsigned char buf[13];
        unsigned char _buffer[256];
        BUFFERCS vComBuf;
        vComBuf.buf=_buffer;
        vComBuf.index=0;
        for (;;) {
                /*
                 * Process non-format characters
                 */
                for (;;) {
                        c = *(fmt++);// (in_progmem, 1, fmt);
                        if (!c) {
                            if(UartX==0)
                               UartA_write(vComBuf.buf,vComBuf.index);
                            else if(UartX==1)
                               UartB_write(vComBuf.buf,vComBuf.index);
                            else if(UartX==2)
                                UartC_write(vComBuf.buf,vComBuf.index);
                            return;
                             } 
                        if (c == '%') {
                                c = *(fmt++);// (in_progmem, 1, fmt);
                                if (c != '%') break;
                        }
                        /* emit cr before lf to make most terminals happy */
                        if (c == '\n')
                               writeOneByte2Buffer(&vComBuf,'\r');//  s->write('\r');
                        writeOneByte2Buffer(&vComBuf,c);// s->write(c);
                }

                flags = 0;
                width = 0;
                prec = 0;
                
                /*
                 * Process format adjustment characters, precision, width.
                 */
                do {
                        if (flags < FL_WIDTH) {
                                switch (c) {
                                case '0':
                                        flags |= FL_ZFILL;
                                        continue;
                                case '+':
                                        flags |= FL_PLUS;
                                        /* FALLTHROUGH */
                                case ' ':
                                        flags |= FL_SPACE;
                                        continue;
                                case '-':
                                        flags |= FL_LPAD;
                                        continue;
                                case '#':
                                        flags |= FL_ALT;
                                        continue;
                                }
                        }

                        if (flags < FL_LONG) {
                                if (c >= '0' && c <= '9') {
                                        c -= '0';
                                        if (flags & FL_PREC) {
                                                prec = 10*prec + c;
                                                continue;
                                        }
                                        width = 10*width + c;
                                        flags |= FL_WIDTH;
                                        continue;
                                }
                                if (c == '.') {
                                        if (flags & FL_PREC)
                                               {
                                                if(UartX==0)
                                                   UartA_write(vComBuf.buf,vComBuf.index);
                                                else if(UartX==1)
                                                   UartB_write(vComBuf.buf,vComBuf.index);
                                                else if(UartX==2)
                                                    UartC_write(vComBuf.buf,vComBuf.index);                                                
                                                    return;} 
                                        flags |= FL_PREC;
                                        continue;
                                }
                                if (c == 'l') {
                                        flags |= FL_LONG;
                                        continue;
                                }
                                if (c == 'h')
                                        continue;
                        }
            
                        break;
                } while ( (c = *(fmt++))!= 0);// (in_progmem, 1, fmt)) != 0);

                /*
                 * Handle floating-point formats E, F, G, e, f, g.
                 */
                if (c >= 'E' && c <= 'G') {
                        flags |= FL_FLTUPP;
                        c += 'e' - 'E';
                        goto flt_oper;

                } else if (c >= 'e' && c <= 'g') {

                        int exp;                /* exponent of master decimal digit     */
                        int n;
                        unsigned char vtype;    /* result of float value parse  */
                        unsigned char sign;     /* sign character (or 0)        */
                        unsigned char ndigs;
                        float value;
                        flags &= ~FL_FLTUPP;

                flt_oper:
                        value = va_arg(ap,double);
                        if (!(flags & FL_PREC))
                                prec = 6;
                        flags &= ~(FL_FLTEXP | FL_FLTFIX);
                        if (c == 'e') {
                                flags |= FL_FLTEXP;
                        } else if (c == 'f') {
                                flags |= FL_FLTFIX;
                        } else if (prec > 0)
                                prec -= 1;
                        if ((flags & FL_FLTFIX) && fabs(value) > 9999999) {
                                flags = (flags & ~FL_FLTFIX) | FL_FLTEXP;
                        }

                        if (flags & FL_FLTFIX) {
                                vtype = 7;              /* 'prec' arg for 'ftoa_engine' */
                                ndigs = prec < 60 ? prec + 1 : 60;
                        } else {
                                if (prec > 10) prec = 10;
                                vtype = prec;
                                ndigs = 0;
                        }
                        memset(buf, 0, sizeof(buf));
                        exp = ftoa_engine(value, (char *)buf, vtype, ndigs);
                        vtype = buf[0];
    
                        sign = 0;
                        if ((vtype & FTOA_MINUS) && !(vtype & FTOA_NAN))
                                sign = '-';
                        else if (flags & FL_PLUS)
                                sign = '+';
                        else if (flags & FL_SPACE)
                                sign = ' ';

                        if (vtype & (FTOA_NAN | FTOA_INF)) {
                                ndigs = sign ? 4 : 3;
                                if (width > ndigs) {
                                        width -= ndigs;
                                        if (!(flags & FL_LPAD)) {
                                                do {
                                                       writeOneByte2Buffer(&vComBuf,' ');//s->write(' ');
                                                } while (--width);
                                        }
                                } else {
                                        width = 0;
                                }
                                if (sign)
                                       writeOneByte2Buffer(&vComBuf,sign);//  s->write(sign);
                                const U8 *p = "inf";
                                if (vtype & FTOA_NAN)
                                        p = "nan";
                                while ( (ndigs = (*p)) != 0) {
                                        if (flags & FL_FLTUPP)
                                                ndigs += 'I' - 'i';
                                         writeOneByte2Buffer(&vComBuf,ndigs);//s->write(ndigs);
                                        p++;
                                }
                                goto tail;
                        }

                        /* Output format adjustment, number of decimal digits in buf[] */
                        if (flags & FL_FLTFIX) {
                                ndigs += exp;
                                if ((vtype & FTOA_CARRY) && buf[1] == '1')
                                        ndigs -= 1;
                                if ((signed char)ndigs < 1)
                                        ndigs = 1;
                                else if (ndigs > 8)
                                        ndigs = 8;
                        } else if (!(flags & FL_FLTEXP)) {              /* 'g(G)' format */
                                if (exp <= prec && exp >= -4)
                                        flags |= FL_FLTFIX;
                                while (prec && buf[1+prec] == '0')
                                        prec--;
                                if (flags & FL_FLTFIX) {
                                        ndigs = prec + 1;               /* number of digits in buf */
                                        prec = prec > exp
                                                ? prec - exp : 0;       /* fractional part length  */
                                }
                        }
    
                        /* Conversion result length, width := free space length */
                        if (flags & FL_FLTFIX)
                                n = (exp>0 ? exp+1 : 1);
                        else
                                n = 5;          /* 1e+00 */
                        if (sign) n += 1;
                        if (prec) n += prec + 1;
                        width = width > n ? width - n : 0;
    
                        /* Output before first digit    */
                        if (!(flags & (FL_LPAD | FL_ZFILL))) {
                                while (width) {
                                         writeOneByte2Buffer(&vComBuf,' ');//s->write(' ');
                                        width--;
                                }
                        }
                        if (sign)  writeOneByte2Buffer(&vComBuf,sign);// //s->write(sign);
                        if (!(flags & FL_LPAD)) {
                                while (width) {
                                        writeOneByte2Buffer(&vComBuf,'0');// s->write('0');
                                        width--;
                                }
                        }
    
                        if (flags & FL_FLTFIX) {                /* 'f' format           */

                                n = exp > 0 ? exp : 0;          /* exponent of left digit */
                                do {
                                        if (n == -1)
                                                writeOneByte2Buffer(&vComBuf,'.');// s->write('.');
                                        flags = (n <= exp && n > exp - ndigs)
                                                ? buf[exp - n + 1] : '0';
                                        if (--n < -prec || flags == 0)
                                                break;
                                         writeOneByte2Buffer(&vComBuf,flags);//s->write(flags);
                                } while (1);
                                if (n == exp
                                    && (buf[1] > '5'
                                        || (buf[1] == '5' && !(vtype & FTOA_CARRY))) )
                                        {
                                                flags = '1';
                                        }
                                if (flags)  writeOneByte2Buffer(&vComBuf,flags);//s->write(flags);
        
                        } else {                                /* 'e(E)' format        */

                                /* mantissa     */
                                if (buf[1] != '1')
                                        vtype &= ~FTOA_CARRY;
                                 writeOneByte2Buffer(&vComBuf,buf[1]);//s->write(buf[1]);
                                if (prec) {
                                         writeOneByte2Buffer(&vComBuf,'.');//s->write('.');
                                        sign = 2;
                                        do {
                                                 writeOneByte2Buffer(&vComBuf,buf[sign++]);//s->write(buf[sign++]);
                                        } while (--prec);
                                }

                                /* exponent     */
                                 writeOneByte2Buffer(&vComBuf,flags & FL_FLTUPP ? 'E' : 'e');//s->write(flags & FL_FLTUPP ? 'E' : 'e');
                                ndigs = '+';
                                if (exp < 0 || (exp == 0 && (vtype & FTOA_CARRY) != 0)) {
                                        exp = -exp;
                                        ndigs = '-';
                                }
                                 writeOneByte2Buffer(&vComBuf,ndigs);//s->write(ndigs);
                                for (ndigs = '0'; exp >= 10; exp -= 10)
                                        ndigs += 1;
                                 writeOneByte2Buffer(&vComBuf,ndigs);//s->write(ndigs);
                                 writeOneByte2Buffer(&vComBuf,'0'+exp);//s->write('0' + exp);
                        }

                        goto tail;
                }

                /*
                 * Handle string formats c, s, S.
                 */
                {
                        const char * pnt;
                        size_t size;

                        switch (c) {

                        case 'c':
                                buf[0] = va_arg (ap, int);
                                pnt = (char *)buf;
                                size = 1;
                                goto no_pgmstring;

                        case 's':
                                pnt = va_arg (ap, char *);
                                size = con_strnlen(pnt, (flags & FL_PREC) ? prec : ~0);
                        no_pgmstring:
                                flags &= ~FL_PGMSTRING;
                                goto str_lpad;

                        case 'S':
                        // pgmstring: // not yet used
                                pnt = va_arg (ap, char *);
                                size = con_strnlen(pnt, (flags & FL_PREC) ? prec : ~0);
                                flags |= FL_PGMSTRING;

                        str_lpad:
                                if (!(flags & FL_LPAD)) {
                                        while (size < width) {
                                                writeOneByte2Buffer(&vComBuf,' ');// s->write(' ');
                                                width--;
                                        }
                                }
                                while (size) {
                                        writeOneByte2Buffer(&vComBuf,*(fmt++));// s->write(*(fmt++);//(flags,FL_PGMSTRING,pnt) (flags, FL_PGMSTRING, pnt));
                                        if (width) width -= 1;
                                        size -= 1;
                                }
                                goto tail;
                        }
                }

                /*
                 * Handle integer formats variations for d/i, u, o, p, x, X.
                 */
                if (c == 'd' || c == 'i') {
                        long x = (flags & FL_LONG) ? va_arg(ap,long) : va_arg(ap,int);
                        flags &= ~(FL_NEGATIVE | FL_ALT);
                        if (x < 0) {
                                x = -x;
                                flags |= FL_NEGATIVE;
                        }
                        c = ultoa_invert (x, (char *)buf, 10) - (char *)buf;

                } else {
                        int base;

                        if (c == 'u') {
                                flags &= ~FL_ALT;
                                base = 10;
                                goto ultoa;
                        }

                        flags &= ~(FL_PLUS | FL_SPACE);

                        switch (c) {
                        case 'o':
                                base = 8;
                                goto ultoa;
                        case 'p':
                                flags |= FL_ALT;
                                /* no break */
                        case 'x':
                                if (flags & FL_ALT)
                                        flags |= FL_ALTHEX;
                                base = 16;
                                goto ultoa;
                        case 'X':
                                if (flags & FL_ALT)
                                        flags |= (FL_ALTHEX | FL_ALTUPP);
                                base = 16 | XTOA_UPPER;
                        ultoa:
                                c = ultoa_invert ((flags & FL_LONG)
                                                    ? va_arg(ap, unsigned long)
                                                    : va_arg(ap, unsigned int),
                                                    (char *)buf, base)  -  (char *)buf;
                                flags &= ~FL_NEGATIVE;
                                break;

                        default:
                               {
                                if(UartX==0)
                                    UartA_write(vComBuf.buf,vComBuf.index);
                                else if(UartX==1)
                                     UartB_write(vComBuf.buf,vComBuf.index);
                                else if(UartX==2)
                                        UartC_write(vComBuf.buf,vComBuf.index);//s->write(vComBuf.buf,vComBuf.index);
                               return;
                               } //return;
                        }
                }

                /*
                 * Format integers.
                 */
                {
                        unsigned char len;

                        len = c;
                        if (flags & FL_PREC) {
                                flags &= ~FL_ZFILL;
                                if (len < prec) {
                                        len = prec;
                                        if ((flags & FL_ALT) && !(flags & FL_ALTHEX))
                                                flags &= ~FL_ALT;
                                }
                        }
                        if (flags & FL_ALT) {
                                if (buf[c-1] == '0') {
                                        flags &= ~(FL_ALT | FL_ALTHEX | FL_ALTUPP);
                                } else {
                                        len += 1;
                                        if (flags & FL_ALTHEX)
                                                len += 1;
                                }
                        } else if (flags & (FL_NEGATIVE | FL_PLUS | FL_SPACE)) {
                                len += 1;
                        }

                        if (!(flags & FL_LPAD)) {
                                if (flags & FL_ZFILL) {
                                        prec = c;
                                        if (len < width) {
                                                prec += width - len;
                                                len = width;
                                        }
                                }
                                while (len < width) {
                                         writeOneByte2Buffer(&vComBuf,' ');//s->write(' ');
                                        len++;
                                }
                        }
        
                        width =  (len < width) ? width - len : 0;

                        if (flags & FL_ALT) {
                                 writeOneByte2Buffer(&vComBuf,'0');//s->write('0');
                                if (flags & FL_ALTHEX)
                                 writeOneByte2Buffer(&vComBuf,flags & FL_ALTUPP ? 'X' : 'x');//  s->write(flags & FL_ALTUPP ? 'X' : 'x');
                        } else if (flags & (FL_NEGATIVE | FL_PLUS | FL_SPACE)) {
                                unsigned char z = ' ';
                                if (flags & FL_PLUS) z = '+';
                                if (flags & FL_NEGATIVE) z = '-';
                                 writeOneByte2Buffer(&vComBuf,z);//s->write(z);
                        }
                
                        while (prec > c) {
                                writeOneByte2Buffer(&vComBuf,'0');// s->write('0');
                                prec--;
                        }
        
                        do {
                                writeOneByte2Buffer(&vComBuf,buf[--c]);// s->write(buf[--c]);
                        } while (c);
                }
        
        tail:
                /* Tail is possible.    */
                while (width) {
                         writeOneByte2Buffer(&vComBuf,' ');//s->write(' ');
                        width--;
                }
        } /* for (;;) */
	
//    if(UartX==0)
//       UartA_write(vComBuf.buf,vComBuf.index);
//    else if(UartX==1)
//       UartB_write(vComBuf.buf,vComBuf.index);
//    else if(UartX==2)
//        UartC_write(vComBuf.buf,vComBuf.index);
}


void UCprintf(const U8 *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    print_vprintf (2,fmt, arg_list);
    va_end(arg_list);
}

void UAprintf(const U8 *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    print_vprintf (0,fmt, arg_list);
    va_end(arg_list);
}







