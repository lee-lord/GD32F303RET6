#include "main.h"
#include "Uart_Dma.h"
#include "usart.h"
#include "led.h"
////////////////////////////////////////////////////////////////////////////////// 	  
////////���ڵ�Tx����Ϊһ��ģʽ���ͣ�RX����Ϊѭ������

//u16 DMA1_MEM_LEN;//����DMAÿ�����ݴ��͵ĳ��� 	    
//DMA1�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_CHx:DMAͨ��CHx
//cpar:�����ַ
//cmar:�洢����ַ
//cndtr:���ݴ�����  

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
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
	
    DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMAstruc->DMA_PeripheralBaseAddr = cpar;  //DMA����ADC����ַ
	DMAstruc->DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMAstruc->DMA_DIR = DMA_DIR_PeripheralDST;  //������Ϊ���ݴ����Ŀ�ĵ�
	DMAstruc->DMA_BufferSize = cndtr;  //DMAͨ����DMA����Ĵ�С
	DMAstruc->DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMAstruc->DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMAstruc->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMAstruc->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; ///��������������ģʽ
	DMAstruc->DMA_Priority = DMA_Priority_Medium; //DMAͨ�� /���ݿ��Ϊ8λ
	DMAstruc->DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;//xӵ�������ȼ� 
	DMAstruc->DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, DMAstruc);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
	  	
}

void DMA_Config_RX(DMA_Channel_TypeDef* DMA_CHx,DMA_InitTypeDef *DMAstruc,u32 cpar,u32 cmar,u16 cndtr)
{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����
	
  DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ

	DMAstruc->DMA_PeripheralBaseAddr = cpar;  //DMA����ADC����ַ
	DMAstruc->DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMAstruc->DMA_DIR = DMA_DIR_PeripheralSRC;  //������Ϊ���ݴ����Ŀ�ĵ�
	DMAstruc->DMA_BufferSize = cndtr;  //DMAͨ����DMA����Ĵ�С
	DMAstruc->DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMAstruc->DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMAstruc->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMAstruc->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMAstruc->DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;  //��������������ģʽ
	DMAstruc->DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
	DMAstruc->DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, DMAstruc);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
	  	
} 
//����һ��DMA����
void DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,DMA_InitTypeDef *DMAstruc,u32 ITflag,u32 Len)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //�ر�USART1 TX DMA1 ��ָʾ��ͨ��      
    DMAstruc->DMA_BufferSize = 	 Len;
	DMA_Init(DMA_CHx, DMAstruc);
	//DMA_ITConfig(DMA_CHx,ITflag,ENABLE);
 	DMA_Cmd(DMA_CHx, ENABLE);  //ʹ��USART1 TX DMA1 ��ָʾ��ͨ�� 
}

//TX ����һ��DMA����
void DMA_SendEn(DMA_Channel_TypeDef*DMA_CHx,DMA_InitTypeDef *DMAstruc,u32 dataLen,u32 dataAdd)
{ 
	DMA_Cmd(DMA_CHx, DISABLE );  //�ر�USART1 TX DMA1 ��ָʾ��ͨ��   4   
    DMAstruc->DMA_BufferSize     = 	 dataLen;
	DMAstruc->DMA_MemoryBaseAddr = dataAdd;  //DMA�ڴ����ַ
	DMA_Init(DMA_CHx, DMAstruc);
  //DMA_ITConfig(DMA_CHx,DMA_IT_TC,ENABLE);
 	DMA_Cmd(DMA_CHx, ENABLE);  //ʹ��USART1 TX DMA1 ��ָʾ��ͨ�� 
}	


 void DMA1_Channel4_IRQHandler(void)
{
u32 SramAdd=0;
u32 tempLen=0;
if(DMA_GetITStatus(DMA1_IT_TC4))
	{
		DMA_ClearITPendingBit(DMA1_IT_TC4);		 
		SramAdd = (u32)(&Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_head]);
			///��������ķ��ͣ������ǰ�������ں�
			 if(Uart1DMAbuf.Txfile_head<Uart1DMAbuf.Txfile_tail)
			 {
				 tempLen = Uart1DMAbuf.Txfile_tail -Uart1DMAbuf.Txfile_head;
				   Uart1DMAbuf.Txfile_head =Uart1DMAbuf.Txfile_tail;
			 }// ���еߵ�������ں󣬷�����ǰ���ȰѶ��е�β��������ɡ�����ͷ���㡣����һ�η��͡�
			 else if(Uart1DMAbuf.Txfile_head>Uart1DMAbuf.Txfile_tail)
			 {  //���͵�buf��β��
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
		///��������ķ��ͣ������ǰ�������ں�
		SramAdd = (u32)(&Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_head]);	
			 if(Uart2DMAbuf.Txfile_head<Uart2DMAbuf.Txfile_tail)
			 {
				 tempLen = Uart2DMAbuf.Txfile_tail -Uart2DMAbuf.Txfile_head;
				   Uart2DMAbuf.Txfile_head =Uart2DMAbuf.Txfile_tail;
				// LED0=!LED0;
			 }// ���еߵ�������ں󣬷�����ǰ���ȰѶ��е�β��������ɡ�����ͷ���㡣����һ�η��͡�
			 else if(Uart2DMAbuf.Txfile_head>Uart2DMAbuf.Txfile_tail)
			 {  //���͵�buf��β��
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
	  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);           //ʹ�ܴ���1��DMA����
	  DMA_Enable(DMA1_Channel5,&DMA_CH1_Rx_InitStruc,DMA_IT_TC,TxBufLen);
	 //	MYDMA_Enable(DMA1_Channel5,DMA_IT_TC,1024);//��ʼһ��DMA����
		DMA_Config_Tx(DMA1_Channel4,&DMA_CH1_Tx_InitStruc,(u32)&USART1->DR,(u32)Uart1DMAbuf.TxBuff,TxBufLen);
   // MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)Uart1DMAbuf.TxBuff,1024);//DMA1ͨ��4,����Ϊ����1,�洢��ΪSendBuff,����5200.
	  DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
	
	  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);          //ʹ�ܴ���1��DMA����   

////////////Uart2 Tx Rx DMA init
//RX  DMA1_Channel6
//Tx   DMA1_Channel7
	  DMA_Config_RX(DMA1_Channel6,&DMA_CH2_Rx_InitStruc,(u32)&USART2->DR,(u32)Uart2DMAbuf.RxBuff,TxBufLen);
	  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);           //ʹ�ܴ���2��DMA����
	  DMA_Enable(DMA1_Channel6,&DMA_CH2_Rx_InitStruc,DMA_IT_TC,TxBufLen);
		DMA_Config_Tx(DMA1_Channel7,&DMA_CH2_Tx_InitStruc,(u32)&USART2->DR,(u32)Uart2DMAbuf.TxBuff,TxBufLen);//DMA1ͨ��4,����Ϊ����1,�洢��ΪSendBuff,����5200.
		DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);
	   USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);          //ʹ�ܴ���2��DMA����   
////////////Uart3 Tx Rx DMA init
}



















