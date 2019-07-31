/******
this file of SPI will do a three wires SPI with 16bits

*******/ 
#include "gd32f30x.h"
#include "sysType.h"
#include "SPI.h"


void SPI1_Init(void)
{   
	rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_SPI1);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_15);
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_14);//GPIO_MODE_IPU
    /* PB12 as NSS */
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    /*PC8 as the second Nss*/
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    spi_parameter_struct spi_init_struct;

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_16BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;                                         //
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_8;//7.5Mhz
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI1, &spi_init_struct);
    
    /* SPI enable */
    spi_enable(SPI1);

}

void SPIX_3wires_initial(U32 SPIX,U32 baud,U8 BitWidh)
{


}

__inline U16 SPI3_ReadWriteByte(U16 TxData)
{		
	while ((SPI_STAT(SPI1) & SPI_FLAG_TBE)==RESET);//SPI_I2S_GetFlagStatus(SPI1, SPI_FLAG_TBE) == RESET)//(SPI1->SR & SPI_I2S_FLAG) //检查指定的SPI标志位设置与否:发送缓存空标志位
  
	SPI_DATA(SPI1)=TxData;//SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
	while ((SPI_STAT(SPI1) & SPI_FLAG_RBNE)==RESET);//(SPI_I2S_GetFlagStatus(SPI1, SPI_FLAG_RBNE) == RESET); //检查指定的SPI标志位设置与否:接受缓存非空标志位

	return SPI_DATA(SPI1);//SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据					    
}

__inline U16 ReadWrite2Byte(U16 cmd) 
{
    while ((SPI_STAT(SPI1) & SPI_FLAG_TBE)==RESET);//SPI_I2S_GetFlagStatus(SPI1, SPI_FLAG_TBE) == RESET)//(SPI1->SR & SPI_I2S_FLAG) //检查指定的SPI标志位设置与否:发送缓存空标志位
  
	SPI_DATA(SPI1)=cmd;//SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
	while ((SPI_STAT(SPI1) & SPI_FLAG_RBNE)==RESET);//(SPI_I2S_GetFlagStatus(SPI1, SPI_FLAG_RBNE) == RESET); //检查指定的SPI标志位设置与否:接受缓存非空标志位

	return SPI_DATA(SPI1);//SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据	
}

__inline U16 transfer16Bits(U16 *cmd,U16 *data,U16 Len) 
{
    U16 i=0;
    while(!(SPI_STAT(SPI1) & SPI_FLAG_TBE));
       while(i<Len)
        {
        SPI_DATA(SPI1) = cmd[i];
        //while(SPI_STAT(SPI1) & SPI_SR_BSY);  //add by lee
        while (!(SPI_STAT(SPI1) & SPI_FLAG_RBNE));//spi_is_rx_nonempty(this->spi_d)
        data[i] = SPI_DATA(SPI1);//spi_rx_reg(this->spi_d);
        i++;
        }
    return 1;
}

__inline U16 ReadWrite2ByteCMD(U16 cmd,U16 data) 
{
    U16 rxData = 0;
    //first write 2Bytes cmd
    while(!(SPI_STAT(SPI1) & SPI_FLAG_TBE));// while (!spi_is_tx_empty(this->spi_d));
      SPI_DATA(SPI1)=cmd;

    while (!(SPI_STAT(SPI1) & SPI_FLAG_RBNE));// while (!spi_is_rx_nonempty(this->spi_d));
    //read the len 2Bytes data
      rxData = SPI_DATA(SPI1);
  
    while(!(SPI_STAT(SPI1) & SPI_FLAG_TBE));// while (!spi_is_tx_empty(this->spi_d));
      SPI_DATA(SPI1)=data;
    while (!(SPI_STAT(SPI1) & SPI_FLAG_RBNE));// while (!spi_is_rx_nonempty(this->spi_d));
    //read the len 2Bytes data
      rxData = SPI_DATA(SPI1);
      return rxData;
}

