#include "gd32f30x.h"
#include "sysType.h"
#include "flash_para.h"
//////page size =2K for HD
#define FMC_PAGE_SIZE           ((uint16_t)0x800U)
#define FMC_WRITE_START_ADDR    ((uint32_t)0x0807F800U)
#define FMC_WRITE_END_ADDR      ((uint32_t)0x08080000U)


/* calculate the number of page to be programmed/erased */
uint32_t PageNum = (FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR) / FMC_PAGE_SIZE;
/* calculate the number of page to be programmed/erased */
uint32_t WordNum = ((FMC_WRITE_END_ADDR - FMC_WRITE_START_ADDR) >> 2);


EEPROM_Para_G Tle5012b_para;
 
/*!
    \brief      erase fmc pages from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_erase_pages(void)
{
    uint32_t EraseCounter;

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR );
    
    /* erase the flash pages */
    for(EraseCounter = 0; EraseCounter < PageNum; EraseCounter++){
        fmc_page_erase(FMC_WRITE_START_ADDR + (FMC_PAGE_SIZE * EraseCounter));
        fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR );
    }

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

/*!
    \brief      program fmc word by word from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_program_word(U32 * dataBuf)
{
	U32 i=0;
	U32 address=0;
    /* unlock the flash program/erase controller */
    fmc_unlock();

    address = FMC_WRITE_START_ADDR;

    /* program flash */
    while(address < FMC_WRITE_END_ADDR){
        fmc_word_program(address, dataBuf[i]);
        address += 4;
        i++;
        fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR );
    }

    /* lock the main FMC after the program operation */
    fmc_lock();
}

void fmc_program_Bytes(U8 * dataBuf,U32 len,U32 addrOffset)
{
	U32 i=0,dataindex=0;
	U32 cuntWords=0,leftBytes=0;
	U32 *dataPtr;
	U8 LeftBytesTab[4];
	U32 address=0;
    /* unlock the flash program/erase controller */
    fmc_unlock();
    cuntWords = len/4;
    leftBytes = len%4;
    
     address = FMC_WRITE_START_ADDR+addrOffset;//set the start address of flash.
   

    dataPtr = (U32 *)dataBuf;
    /* program flash cuntWords */
    while(i < cuntWords){
        fmc_word_program(address, dataPtr[i]);
        address += 4;
        i++;
        fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR );
    }
    dataindex =i*4;
   ////////////programe left Bytes<4
    if(leftBytes)
	{
		for(i=0;i<4;i++)
		{
			if(i<leftBytes){
				LeftBytesTab[i] = dataBuf[i+dataindex];//*((volatile U8 *)(address+i));
			}else{
				LeftBytesTab[i] = 0xFF;
			}
		}
       fmc_word_program(address, *((volatile U32 *)LeftBytesTab));
       fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR );
    }
    /* lock the main FMC after the program operation */
    fmc_lock();
}



//read all data from flash eeprom 
void parametersLoad(void)
{
   U16 i,length;
   U32 beginAddr=FMC_WRITE_START_ADDR;
   length = sizeof(Tle5012b_para);
   volatile U8 * readBuf= (volatile U8 *)(&Tle5012b_para);
   	for(i=0;i<length;i+=1)
	{
		readBuf[i]=((volatile U8*)beginAddr)[i]; 
	}


}

/// save all data to flash eeprom
void parametersSave(void)
{
   fmc_program_Bytes((U8 *)(&Tle5012b_para),sizeof(Tle5012b_para),0);
}




