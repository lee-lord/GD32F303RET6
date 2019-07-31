#ifndef _TLE_5012B_H_
#define _TLE_5012B_H_
#define TLE_STAT        0
#define TLE_ACSTAT      1
#define TLE_AVAL        2
#define TLE_ASPD        3
#define TLE_AREV        4
#define TLE_FSYNC       5
#define TLE_MOD_1       6
#define TLE_SIL         7
#define TLE_MOD_2       8
#define TLE_MOD_3       9
#define TLE_XOFFX       0x0a
#define TLE_YOFFX       0x0b
#define TLE_SYNCH       0x0c
#define TLE_IFAB        0x0d
#define TLE_MOD_4       0x0e
#define TLE_TCO_Y       0x0f
#define TLE_ADC_X       0x10
#define TLE_ADC_Y       0x11
#define TLE_D_MAG       0x14
#define TLE_T_RAW       0x15
#define TLE_IIF_CNT     0x20
#define TLE_T25O        0x30



#define READ_STATUS             0x8001          //8000
#define READ_ANGLE_VALUE        0x8021          //8020
#define READ_SPEED_VALUE        0x8031          //8030
#define READ_INTMODE_1          0xd061
#define ANGEL_RANGE             0xd081       
#define READ_ADCX_VALUE         0xd101          //0x10
#define READ_ADCY_VALUE         0xd111          //0x11
#define READ_ACSTAT             0x8011
#define WRITE_ACSTAT            0x0011

#define WRITE_MOD1_VALUE        0x5061                          //0_1010_0_000110_0001
#define MOD1_VALUE  0x0001
 
#define WRITE_MOD2_VALUE        0x5081                          //0_1010_0_001000_0001
#define MOD2_VALUE  0x0800            
//Write Data: 0_00010000000_1_0_00
//Set ANG_Range 080H, ANG_DIR: 1B, PREDICT: 0B, AUTOCAL: 01B
#define READ_MODE3_VALUE        0x8091
#define WRITE_MOD3_VALUE        0x5091                          //0_1010_0_001001_0001
#define MOD3_VALUE  0x0000
 
#define WRITE_MOD4_VALUE        0x50E1                          //0_1010_0_001110_0001
#define MOD4_VALUE  0x0098              //9bit 512
 
#define WRITE_IFAB_VALUE        0x50B1
#define IFAB_VALUE 0x000D

#define READ_X_OFFSET           0x80A1      //read  x offset 
#define READ_Y_OFFSET           0x80B1      //read  Y offset
#define READ_BLOCK_CRC              0x8088
/* Functionality mode
--------------------- 
作者：keith_cheung 
来源：CSDN 
原文：https://blog.csdn.net/keith_cheung/article/details/71883155 
版权声明：本文为博主原创文章，转载请附上博文链接！  
 */


#define CRC_POLYNOMIAL              0x1D
#define CRC_SEED                    0xFF



void TLE5012B_initialise(void);
////////////////A sensor
U16 readWords(U16 add);
float readAnglesDegrees(U16 fullHalf);
U16 readAngleAndSpeed(U16 fullHalf,float *angles,float *speeds);
U16 writebytes(U16 cmd,U16 data);

U16 readBlockCRC(void);

//////////////////B sensor
U16 readWordsB(U16 add);
float readAnglesDegreesB(U16 fullHalf);
U16 readAngleAndSpeedB(U16 fullHalf,float *angles,float *speeds);
U16 writebytesB(U16 cmd,U16 data);


#endif
