#include "gd32f30x.h"
#include "sysType.h"
#include "hardware.h"
#include "systick.h"
#include "SPI.h"
#include "TLE5012B.h"

U16 CRC_table[8];

void TLE5012B_initialise(void)
{
   U16 resault=0,resault2=0,mode2data;
   UAprintf("TLE5012B_initialise\r\n");
 /////////initial SPI2 
    SPI1_Init();
 ////////////Initial Tle5012B_A
    delay_1ms(5);
    resault = readWords(READ_ACSTAT);
    UAprintf("ACSTAT=%x \r\n",resault);//0x58ee
    resault2 = writebytes(WRITE_ACSTAT,0x58ef);
    delay_1ms(50);
    resault = readWords(READ_ACSTAT);
    UAprintf("ACSTAT=%x \r\n",resault);//0x58ee
    // if(resault&0x0008) 
    // {
    //     resault &=0xfff7;//AS_FUSE disable CRC check.
    // }
      if((resault&0x0080)==0x0080) 
      {
        resault &=0xff7f; 
      }
      if((resault&0x0040)==0x0040) 
      {
        resault &=0xffbf;
      }
      if((resault&0x0020)==0x0020) 
      {
        resault &=0xffdf;
      }

     resault2 = writebytes(WRITE_ACSTAT,resault);

     resault = readWords(READ_STATUS);
     UAprintf("STAT=%x\r\n",resault);
     resault = readWords(READ_ACSTAT);
     UAprintf("STAT=%x,SfWrd=%x \r\n",resault,resault2);

     resault2 = readWords(ANGEL_RANGE);
     resault = writebytes(WRITE_MOD2_VALUE,MOD2_VALUE);//|0x0001|0x0008
     mode2data = readWords(ANGEL_RANGE);
     UAprintf("SfWrd=%x ,old=%x,new=%x\r\n",resault,resault2,mode2data);


     readBlockCRC();
 ////////////Initial Tle5012B_B 
	 

	resault = readWordsB(READ_ACSTAT);
    UAprintf("res2=%x \r\n",resault);
    if((resault&0x0080)==0x0080) 
      {
        resault &=0xff7f;
      }
      if((resault&0x0040)==0x0040) 
      {
        resault &=0xffbf;
      }
      if((resault&0x0020)==0x0020) 
      {
        resault &=0xffdf;
      }
     resault2 = writebytesB(WRITE_ACSTAT,resault);
     
     resault = readWordsB(READ_STATUS);
     UAprintf("STAT2=%x\r\n",resault);
     resault = readWordsB(READ_ACSTAT);
     UAprintf("STAT2=%x,SfWrd2=%x \r\n",resault,resault2);

     resault2 =  readWordsB(ANGEL_RANGE);
     resault = writebytesB(WRITE_MOD2_VALUE,MOD2_VALUE);
     mode2data = readWordsB(ANGEL_RANGE);
     UAprintf("SfWrd2=%x ,old2=%x,new2=%x\r\n",resault,resault2,mode2data);
}


U16 readBlockCRC()
{
    U16 tx[10],rx[10];
    memset(tx,0xff,20);///set all 0xff
    tx[0]=READ_BLOCK_CRC;
////////////////new function
     SET_TLE1_NSS_LOW;	
     transfer16Bits(tx,rx,10);
     SET_TLE1_NSS_HIGH;
   for(uint8_t i=0;i<8;i++)
     {
        CRC_table[i]=rx[i+1];
        UAprintf("CRC_table[%d]=%x \r\n",i,CRC_table[i]);
     }
   return 0;
}


U16 readWords(U16 add)
{
    U16 angledata=0,len=add&0x000f;
     SET_TLE1_NSS_LOW;
	 ReadWrite2Byte(add);
    angledata = (ReadWrite2Byte(0xffff));// & 0x7FFF ) << 1;
    ReadWrite2Byte(0xffff);/////////////////////////////
    SET_TLE1_NSS_HIGH;
    return angledata;
} 

float readAnglesDegrees(U16 fullHalf)
{////The angle value is a 15 bit signed integer
    ////value<4000 ->+180   value<4000 -180
    S16 angledata=0;
    U16 tx[3],rx[3];
    float resault=0.0f;
 ////////////////new function
    SET_TLE1_NSS_LOW;
   
    tx[0]=0x8020;//READ_ANGLE_VALUE;
    tx[1]=0xffff;
    tx[2]=0xffff;
     transfer16Bits(tx,rx,2);
    SET_TLE1_NSS_HIGH;
     angledata = rx[1]&0x7fff;
    ///// get the angle in degrees . 
    if(!fullHalf&&((angledata&0x4000)==0x4000)) {angledata= angledata-32768;}
     resault = (float)angledata*360.0f/32768.0f;

   // if((resault+offsetA)>179.99f)  resault = resault+offsetA -360.0f;
   // else if((resault+offsetA)<-180.0f) resault = resault+offsetA +360.0f;

    return resault;   
} 

U16 readAngleAndSpeed(U16 fullHalf,float *angles,float *speeds)
{
    S16 angledata;
    U16 tx[4],rx[4];
    S16 data=0;
    SET_TLE1_NSS_LOW;
    tx[0]=0x8020;//READ_ANGLE_VALUE+1;
    tx[1]=0xffff;
    tx[2]=0xffff;
    tx[3]=0xffff;

     
     transfer16Bits(tx,rx,3);
     SET_TLE1_NSS_HIGH;  
     
    ///// get the angle in degrees . 
    angledata = rx[1]&0x7fff;
    if(!fullHalf&&((angledata&0x4000)==0x4000)) *angles= (angledata-32768)*360.0f/32768.0f;
    else
       *angles= (angledata)*360.0f/32768.0f;

    angledata = rx[2]&0x7fff;
    if(((angledata&0x4000)==0x4000)) angledata= (angledata-32768);
     data  = (S16)angledata;
     *speeds=  ((128.0 / 32768.0) * ((double)data)) /0.0000854;//   (((double)2.0) * 42.7 * 0.000001);
    return angledata;
} 

U16 writebytes(U16 cmd,U16 data)
{
	U16 resault=0;    
    SET_TLE1_NSS_LOW;
    delay_us(2);
    ReadWrite2ByteCMD(cmd,data);
    delay_us(200);
    resault = (ReadWrite2Byte(0xffff));// & 0x7FFF ) << 1;
    SET_TLE1_NSS_HIGH;
    
	return resault;
}



/********************************************************************************/
/////////////////////////////////////////sensor2 operation function

U16 readWordsB(U16 add)
{
    U16 angledata=0,len=add&0x000f;
     SET_TLE2_NSS_LOW;
     ReadWrite2Byte(add);
    angledata = (ReadWrite2Byte(0xffff));// & 0x7FFF ) << 1;
    ReadWrite2Byte(0xffff);/////////////////////////////
    SET_TLE2_NSS_HIGH;
    return angledata;
} 

float readAnglesDegreesB(U16 fullHalf)
{////The angle value is a 15 bit signed integer
    ////value<4000 ->+180   value<4000 -180
    S16 angledata=0;
    U16 tx[3],rx[3];
    float resault=0.0f;
 ////////////////new function
     SET_TLE2_NSS_LOW;
   
    tx[0]=0x8020;//READ_ANGLE_VALUE;
    tx[1]=0xffff;
    tx[2]=0xffff;
     transfer16Bits(tx,rx,2);
     SET_TLE2_NSS_HIGH;
     angledata = rx[1]&0x7fff;
    ///// get the angle in degrees . 
    if(!fullHalf&&((angledata&0x4000)==0x4000)) {angledata= angledata-32768;}
    resault = (float)angledata*360.0f/32768.0f;
     
    // if((resault+offsetB)>179.99f)  resault = resault+offsetB -360.0f;
    // else if((resault+offsetB)<-180.0f) resault = resault+offsetB +360.0f;
    return resault;   
} 

U16 readAngleAndSpeedB(U16 fullHalf,float *angles,float *speeds)
{
    S16 angledata;
    U16 tx[4],rx[4];
    S16 data=0;
    SET_TLE2_NSS_LOW;
  
    tx[0]=0x8020;//READ_ANGLE_VALUE+1;
    tx[1]=0xffff;
    tx[2]=0xffff;
    tx[3]=0xffff;

          transfer16Bits(tx,rx,3);
     SET_TLE2_NSS_HIGH; 
     
    ///// get the angle in degrees . 
    angledata = rx[1]&0x7fff;
    if(!fullHalf&&((angledata&0x4000)==0x4000)) *angles= (angledata-32768)*360.0f/32768.0f;
    else
       *angles= (angledata)*360.0f/32768.0f;

    angledata = rx[2]&0x7fff;
    if(((angledata&0x4000)==0x4000)) angledata= (angledata-32768);
     data  = (S16)angledata;
     *speeds=  ((128.0 / 32768.0) * ((double)data)) /0.0000854;//   (((double)2.0) * 42.7 * 0.000001);
    return angledata;
} 

U16 writebytesB(U16 cmd,U16 data)
{
    U16 resault=0;    
    SET_TLE2_NSS_LOW;
    delay_us(2);
    ReadWrite2ByteCMD(cmd,data);
    delay_us(200);
    resault = (ReadWrite2Byte(0xffff));// & 0x7FFF ) << 1;
    SET_TLE2_NSS_HIGH;
    
    return resault;
}
