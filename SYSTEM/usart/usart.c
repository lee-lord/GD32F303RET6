#include "main.h"
//#include "led.h"
#include "usart.h"
#include "Uart_Dma.h"
#include "delay.h"


//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/


extern DMA_InitTypeDef DMA_CH1_Tx_InitStruc;
extern DMA_InitTypeDef DMA_CH2_Tx_InitStruc;
volatile File_Buffer Uart1DMAbuf;
volatile File_Buffer Uart2DMAbuf;
u8 Com1RX[TxBufLen];
u8 Com1TX[TxBufLen];
u8 Com2RX[TxBufLen];
u8 Com2TX[TxBufLen];
u16 USART1Read(void);
u16 USART2Read(void);
u16 USART3Read(void);
u16 USART1Clear(void);
u16 USART2Clear(void);
u16 USART3Clear(void);
u16 USART1available(void);
u16 USART2available(void);
u16 USART3available(void);
u8 USART1Writes(const char * Pbuf,u8 Len);
u8 USART2Writes(const char * Pbuf,u8 Len);
u8 USART3Writes(const char * Pbuf,u8 Len);
u8 USART1Write(const char ch);
u8 USART2Write(const char ch);
u8 USART3Write(const char ch);

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


__inline u16 Com1Avalible(void)
{
	     Uart1DMAbuf.Rxfile_tail=TxBufLen-DMA_GetCurrDataCounter(DMA1_Channel5);	
  return (Uart1DMAbuf.Rxfile_tail>=Uart1DMAbuf.Rxfile_head)? (Uart1DMAbuf.Rxfile_tail-Uart1DMAbuf.Rxfile_head ):(TxBufLen+Uart1DMAbuf.Rxfile_tail-Uart1DMAbuf.Rxfile_head);
}

__inline u16 Com2Avalible(void)
{
	     Uart2DMAbuf.Rxfile_tail=TxBufLen-DMA_GetCurrDataCounter(DMA1_Channel6);	
  return (Uart2DMAbuf.Rxfile_tail>=Uart2DMAbuf.Rxfile_head)? (Uart2DMAbuf.Rxfile_tail-Uart2DMAbuf.Rxfile_head ):(TxBufLen+Uart2DMAbuf.Rxfile_tail-Uart2DMAbuf.Rxfile_head);
}

__inline u8 Com1readByte(void)
{
  u8 tmp=0;
	tmp = Uart1DMAbuf.RxBuff[Uart1DMAbuf.Rxfile_head++];
	if(Uart1DMAbuf.Rxfile_head>(TxBufLen-1))
	  Uart1DMAbuf.Rxfile_head=0;
	return tmp;
}

__inline u8 Com2readByte(void)
{
  u8 tmp=0;
	tmp = Uart2DMAbuf.RxBuff[Uart2DMAbuf.Rxfile_head++];
	if(Uart2DMAbuf.Rxfile_head>(TxBufLen-1))
	  Uart2DMAbuf.Rxfile_head=0;
	return tmp;
}

u8 Com1WriteTxBuf(void)
{
	u16 len=0;
	u16 i=0;
		if(DMA_GetITStatus(DMA1_IT_TC4))
		{
			DMA_ClearITPendingBit(DMA1_IT_TC4);
			Uart1DMAbuf.busy=0;	
		}	
		if(Uart1DMAbuf.busy==1) return 0;
			len = Com1Avalible();
		if(len ==0) return 0;	
	  for(i=0;i<len;i++)
     {
			 Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_tail++]=Com1readByte();
			 if(Uart1DMAbuf.Txfile_tail>(TxBufLen-1)) Uart1DMAbuf.Txfile_tail=0;
		 }
		return len;
}
////////由中断配合完成一次发送。
////////DMA////////////TAIL//// ---set current data len
///////////////DMA/////////TAIL ---wait for DMA cpmplete
///////TAIL/////////DMA//////// ---set current data len ,let it go 
///////TAIL/////////////////DMA///
__inline u8 Com1SendDatas(const char *pStr ,u16 len)
{
   u16 i=0;
	 u32 tempLen=0;
	 u32 SramAdd=0;
	  for(i=0;i<len;i++)
     {
			 Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_tail++]=*(pStr+i);
			 if(Uart1DMAbuf.Txfile_tail>(TxBufLen-1)){ Uart1DMAbuf.Txfile_tail=0; 
                                             //LED0=!LED0;
				                                       }
		 }
// check the DMA status and send the last data
//		if(DMA_GetITStatus(DMA1_IT_TC4))
//		{
//			DMA_ClearITPendingBit(DMA1_IT_TC4);
//			Uart1DMAbuf.busy=0;	
//		}		
		if(Uart1DMAbuf.busy==0) // is idle
		{
			//printf("\n\nBusy=0 send:\n");
				SramAdd = (u32)(&Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_head]);
			///正常情况的发送，天冲在前，发送在后。
			 if(Uart1DMAbuf.Txfile_head<Uart1DMAbuf.Txfile_tail)
			 {
				 tempLen = Uart1DMAbuf.Txfile_tail -Uart1DMAbuf.Txfile_head;
				 Uart1DMAbuf.Txfile_head =Uart1DMAbuf.Txfile_tail;
			 }// 队列颠倒，填充在后，发送在前，先把队列的尾部发送完成。发送头归零。等下一次发送。
			 else  if(Uart1DMAbuf.Txfile_head>Uart1DMAbuf.Txfile_tail)
			 {  //发送到buf的尾端
				 // SramAdd = (u32)(&Uart1DMAbuf.TxBuff[Uart1DMAbuf.Txfile_head]);
				  tempLen = TxBufLen -Uart1DMAbuf.Txfile_head;
				 // tempLen = TxBufLen -Uart1DMAbuf.Txfile_head; 
        	Uart1DMAbuf.Txfile_head =0;
				 //LED1=!LED1;
			 }
			 if(tempLen)
			 {
			 	Uart1DMAbuf.busy=1;
				 DMA_SendEn(DMA1_Channel4,&DMA_CH1_Tx_InitStruc,tempLen,SramAdd);
			 }else
			   Uart1DMAbuf.busy=0;
		}
    return 0;
}


////////DMA////////////TAIL//// ---set current data len
///////////////DMA/////////TAIL ---wait for DMA cpmplete
///////TAIL/////////DMA//////// ---set current data len ,let it go 
///////TAIL/////////////////DMA///
__inline u8 Com2SendDatas(const char *pStr ,u16 len)
{
   u16 i=0;
	static u8 overlap=0;
	 u32 tempLen=0;
	 u32 SramAdd=0;
	  for(i=0;i<len;i++)
     {
			 Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_tail++]=*(pStr+i);
			 if(Uart2DMAbuf.Txfile_tail>(TxBufLen-1)){ Uart2DMAbuf.Txfile_tail=0;}
		 }
		if(Uart2DMAbuf.busy==0) // is idle
		{
				SramAdd = (u32)(&Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_head]);
			///正常情况的发送，天冲在前，发送在后。
			 if(Uart2DMAbuf.Txfile_head<Uart2DMAbuf.Txfile_tail)
			 {
				 tempLen = Uart2DMAbuf.Txfile_tail -Uart2DMAbuf.Txfile_head;
				 Uart2DMAbuf.Txfile_head =Uart2DMAbuf.Txfile_tail;
				 //LED1=!LED1;
			 }// 队列颠倒，填充在后，发送在前，先把队列的尾部发送完成。发送头归零。等下一次发送。
			 else if(Uart2DMAbuf.Txfile_head>Uart2DMAbuf.Txfile_tail)
			 {  //发送到buf的尾端
				 // SramAdd = (u32)(&Uart2DMAbuf.TxBuff[Uart2DMAbuf.Txfile_head]);
				  tempLen = TxBufLen -Uart2DMAbuf.Txfile_head;
        	Uart2DMAbuf.Txfile_head =0;	 
			 }
			 if(tempLen)
			 {
			 Uart2DMAbuf.busy=1;	
			DMA_SendEn(DMA1_Channel7,&DMA_CH2_Tx_InitStruc,tempLen,SramAdd);
			 }
			 else
				Uart2DMAbuf.busy=0; 
		}
    return 0;
}

void uart1_init(u32 bound){
    //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE); 
	
     //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

   //Usart1 NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
  
	    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
	
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
   // USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
    USART_Cmd(USART1, ENABLE);                    //使能串口 
		DMAbufInit();

}
void uart2_init(u32 bound){
    //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
     //USART1_TX   PA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //USART1_RX	  PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

   //Usart1 NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;		//

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
  
		    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);
   // USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
    USART_Cmd(USART2, ENABLE);                    //使能串口 

}
void uart3_init(u32 bound){
    //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
     //USART1_TX   PA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //USART1_RX	  PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

   //Usart1 NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;		//

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
  
		    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART3, &USART_InitStructure);
   // USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
    USART_Cmd(USART3, ENABLE);                    //使能串口 

}
void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		  		 
     } 
} 

__inline u16 USART1Read(void)
{
	u16 temp=0;
	temp=Com1readByte( );
	return  temp;
}

__inline u16 USART2Read(void)
{
	u16 temp=0;
	temp=Com2readByte( );
	return  temp;
}

__inline u16 USART3Read(void)
{
	u16 temp=0;
	return  temp;
}

__inline u16 USART1Clear(void)
{
	u16 temp=0;
	 Uart1DMAbuf.Txfile_head=0;
	 Uart1DMAbuf.Txfile_tail=0;
   Uart1DMAbuf.Rxfile_head=0;
	 Uart1DMAbuf.Rxfile_tail=0;	
	Uart1DMAbuf.busy=0;
	return  temp;
}

__inline u16 USART2Clear(void)
{
	u16 temp=0;
//	  Uart2DMAbuf.Txfile_head=0;
//	 Uart2DMAbuf.Txfile_tail=0;
//   Uart2DMAbuf.Rxfile_head=0;
//	 Uart2DMAbuf.Rxfile_tail=0;	
//	Uart2DMAbuf.busy=0;
	return  temp;
}

__inline u16 USART3Clear(void)
{
	u16 temp=0;
	return  temp;
}


__inline u16 USART1available(void)
{
	 Uart1DMAbuf.Rxfile_tail=TxBufLen-DMA_GetCurrDataCounter(DMA1_Channel5);	
  return (Uart1DMAbuf.Rxfile_tail>=Uart1DMAbuf.Rxfile_head)? (Uart1DMAbuf.Rxfile_tail-Uart1DMAbuf.Rxfile_head ):(TxBufLen+Uart1DMAbuf.Rxfile_tail-Uart1DMAbuf.Rxfile_head);
}


__inline u16 USART2available(void)
{
 Uart2DMAbuf.Rxfile_tail=TxBufLen-DMA_GetCurrDataCounter(DMA1_Channel6);	
  return (Uart2DMAbuf.Rxfile_tail>=Uart2DMAbuf.Rxfile_head)? (Uart2DMAbuf.Rxfile_tail-Uart2DMAbuf.Rxfile_head ):(TxBufLen+Uart2DMAbuf.Rxfile_tail-Uart2DMAbuf.Rxfile_head);
}


__inline u16 USART3available(void)
{
	u16 temp=0;
	return  temp;
}


__inline u8 USART1Writes(const char * Pbuf,u8 len)
{
	Com1SendDatas(Pbuf ,len);
    return 0;
}

__inline u8 USART1WriteBytes(const char * Pbuf,u8 len)
{
	u8 i=0;
	 char pByte=0;
	for(i=0;i<len;i++)
	{
		pByte=Pbuf[i];
		Com1SendDatas((const char *)&pByte ,1);
		delay_ms(1);
	}
    return 0;
}

__inline u8 USART2Writes(const char * Pbuf,u8 Len)
{
	u16 temp=0;
	Com2SendDatas(Pbuf ,Len);
	return  temp;
}
__inline u8 USART3Writes(const char * Pbuf,u8 Len)
{
	u16 temp=0;
	return  temp;
}
__inline u8 USART1Write(const char ch)
{
		u16 temp=0;
	Com1SendDatas((const char)&ch ,1);
	return  temp;
}
__inline u8 USART2Write(char ch)
{
  	u16 temp=0;
	Com2SendDatas(&ch ,1);
	return  temp;
}
__inline u8 USART3Write(char ch)
{
  	u16 temp=0;
	return  temp;
}

