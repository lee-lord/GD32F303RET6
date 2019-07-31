#ifndef _FLASH_PARA_H_
#define _FLASH_PARA_H_

 typedef __packed struct  {
  float angleP;
  float omegaI;
  float omegaP;
  float omegaD;
  float Tq_P;
  float Tq_I;
  float Tq_D;
  float Elec_offset;
  float MachineOffset;
  U16  direction;
  U8  doTest;
  U8  poles;
  } PARAMETER_G;

 typedef __packed struct  {
  PARAMETER_G   MotorA;
  PARAMETER_G   MotorB;
  U16 caliFlagbits;
  U32 magic;//for intial parameter table 
} EEPROM_Para_G;

extern EEPROM_Para_G Tle5012b_para;

void parametersLoad(void);
void parametersSave(void);
#endif
