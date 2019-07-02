#include "main.h"
#include "Uart_Dma.h"
#include "usart.h"
#include "led.h"
////////////////////////////////////////////////////////////////////////////////// 	  
////////串口的Tx配置为一般模式发送，RX配置为循环接收

//u16 DMA1_MEM_LEN;//保存DMA每次数据传送的长度 	    
//DMA1的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_CHx:DMA通道CHx
//cpar:外设地址
//cmar:存储器地址
//cndtr:数据传输量  

DMA_InitTypeDef DMA_CH1_Rx_InitStruc;
DMA_InitTypeDef DMA_CH1_Tx_InitStruc;
DMA_InitTypeDef DMA_CH2_Rx_InitStruc;
DMA_InitTypeDef DMA_CH2_Tx_InitStruc;

extern DMA_InitTypeDef DMA_CH1_Tx_InitStruc;
extern DMA_InitTypeDef DMA_CH2_Tx_InitStruc;
extern volatile File_Buffer Uart1DMAbuf;
extern volatile File_Buffer Uart2DMAbuf;

void DMA_Config_Tx(DMA_Channel_TypeDef* DMA_CHx,DMA_InitTypeDef *DMAstruc,u32 cpar,u32 cmar,u16 cndtr)
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

void DMA_Config_RX(DMA_Channel_TypeDef* DMA_CHx,DMA_InitTypeDef *DMAstruc,u32 cpar,u32 cmar,u16 cndtr)
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
	DMAstruc->DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;  //工作在正常缓存模式
	DMAstruc->DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
	DMAstruc->DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, DMAstruc);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
	  	
} 
//开启一次DMA传输
void DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,DMA_InitTypeDef *DMAstruc,u32 ITflag,u32 Len)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //关闭USART1 TX DMA1 所指示的通道      
    DMAstruc->DMA_BufferSize = 	 Len;
	DMA_Init(DMA_CHx, DMAstruc);
	//DMA_ITConfig(DMA_CHx,ITflag,ENABLE);
 	DMA_Cmd(DMA_CHx, ENABLE);  //使能USART1 TX DMA1 所指示的通道 
}

//TX 开启一次DMA传输
void DMA_SendEn(DMA_Channel_TypeDef*DMA_CHx,DMA_InitTypeDef *DMAstruc,u32 dataLen,u32 dataAdd)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //关闭USART1 TX DMA1 所指示的通道   4   
    DMAstruc->DMA_BufferSize     = 	 dataLen;
	DMAstruc->DMA_MemoryBaseAddr = dataAdd;  //DMA内存基地址
	DMA_Init(DMA_CHx, DMAstruc);
  //DMA_ITConfig(DMA_CHx,DMA_IT_TC,ENABLE);
 	DMA_Cmd(DMA_CHx, ENABLE);  //使能USART1 TX DMA1 所指示的通道 
}	


 void DMA1_Channel4_IRQHandler(void)
{
u32 SramAdd=0;
u32 tempLen=0;
if(DMA_GetITStatus(DMA1_IT_TC4))
	{
		DMA_ClearITPendingBit(DMA1_IT_TC4);		 
		SramAdd = (u32)(&Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_head]);
			///正常情况的发送，天冲在前，发送在后。
			 if(Uart1DMAbuf.Txfile_head<Uart1DMAbuf.Txfile_tail)
			 {
				 tempLen = Uart1DMAbuf.Txfile_tail -Uart1DMAbuf.Txfile_head;
				   Uart1DMAbuf.Txfile_head =Uart1DMAbuf.Txfile_tail;
			 }// 队列颠倒，填充在后，发送在前，先把队列的尾部发送完成。发送头归零。等下一次发送。
			 else if(Uart1DMAbuf.Txfile_head>Uart1DMAbuf.Txfile_tail)
			 {  //发送到buf的尾端
				 // SramAdd = (u32)(&Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_head]);
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
		DMA_SendEn(DMA1_Channel4,&DMA_CH1_Tx_InitStruc,tempLen,SramAdd);
			//LED1=!LED1;
		}
		else
		Uart1DMAbuf.busy=0;
	}
}

void DMA1_Channel7_IRQHandler(void)
{
	u32 SramAdd=0;
	u32 tempLen=0;
   //DMA_ClearFlag(DMA1_IT_TC5);
	if(DMA_GetITStatus(DMA1_IT_TC7))
	{
		DMA_ClearITPendingBit(DMA1_IT_TC7);	
		///正常情况的发送，天冲在前，发送在后。
		SramAdd = (u32)(&Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_head]);	
			 if(Uart2DMAbuf.Txfile_head<Uart2DMAbuf.Txfile_tail)
			 {
				 tempLen = Uart2DMAbuf.Txfile_tail -Uart2DMAbuf.Txfile_head;
				   Uart2DMAbuf.Txfile_head =Uart2DMAbuf.Txfile_tail;
				// LED0=!LED0;
			 }// 队列颠倒，填充在后，发送在前，先把队列的尾部发送完成。发送头归零。等下一次发送。
			 else if(Uart2DMAbuf.Txfile_head>Uart2DMAbuf.Txfile_tail)
			 {  //发送到buf的尾端
	//			  SramAdd = (u32)(&Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_head]);
				  tempLen = TxBufLen -Uart2DMAbuf.Txfile_head;
        	Uart2DMAbuf.Txfile_head =0;
				// LED1=!LED1;
			 }
	   //if(tempLen==1)  LED1=!LED1;
		if(tempLen)
		{ 
			Uart2DMAbuf.busy=1;
		  DMA_SendEn(DMA1_Channel7,&DMA_CH2_Tx_InitStruc,tempLen,SramAdd);
		}
		else
		Uart2DMAbuf.busy=0;
	}
   // printf("\n\nRx half ISR\n"); 	
}

void UartDMA_Init(void)
{
/////////////Uart1 Tx Rx DMA init
	  DMA_Config_RX(DMA1_Channel5,&DMA_CH1_Rx_InitStruc,(u32)&USART1->DR,(u32)Uart1DMAbuf.RxBuff,TxBufLen);
	 //MYDMA_Config_RX(DMA1_Channel5,(u32)&USART1->DR,(u32)DMAbuf.Buff,1024);		
	  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);           //使能串口1的DMA接收
	  DMA_Enable(DMA1_Channel5,&DMA_CH1_Rx_InitStruc,DMA_IT_TC,TxBufLen);
	 //	MYDMA_Enable(DMA1_Channel5,DMA_IT_TC,1024);//开始一次DMA接收
		DMA_Config_Tx(DMA1_Channel4,&DMA_CH1_Tx_InitStruc,(u32)&USART1->DR,(u32)Uart1DMAbuf.TxBuff,TxBufLen);
   // MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)Uart1DMAbuf.TxBuff,1024);//DMA1通道4,外设为串口1,存储器为SendBuff,长度5200.
	  DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
	
	  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);          //使能串口1的DMA发送   

////////////Uart2 Tx Rx DMA init
//RX  DMA1_Channel6
//Tx   DMA1_Channel7
	  DMA_Config_RX(DMA1_Channel6,&DMA_CH2_Rx_InitStruc,(u32)&USART2->DR,(u32)Uart2DMAbuf.RxBuff,TxBufLen);
	  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);           //使能串口2的DMA接收
	  DMA_Enable(DMA1_Channel6,&DMA_CH2_Rx_InitStruc,DMA_IT_TC,TxBufLen);
		DMA_Config_Tx(DMA1_Channel7,&DMA_CH2_Tx_InitStruc,(u32)&USART2->DR,(u32)Uart2DMAbuf.TxBuff,TxBufLen);//DMA1通道4,外设为串口1,存储器为SendBuff,长度5200.
		DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);
	   USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);          //使能串口2的DMA发送   
////////////Uart3 Tx Rx DMA init
}



















