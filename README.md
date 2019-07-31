#GD32F303 samples
# GD32F303RET6
# GD32F303RET6
# 120Mhz (TIMER0 TIMER7 TIMER8 TIMER9 TIER10)
# 60Mhz  (TIMER1 TIMER2 TIMER3 TIMER4  TIMER5 TIMER6   TIMER11 TIMER12 TIMER13)
# test the git status and git add * git commit -m "delete output" git push origin master.
# 可以直接按照原来的寄存器的架构，进行代码移植，只需比对两种寄存器架构的名称就可以找到正确的函数和移植方法。
# pinMap 
# UART1 UART3 SPI3 (Timer2 Timer3)->PWMout Timer5->ISR  Timer4->PWMin LED1 LED2 
# timer4 test finish ok. but I have found a bug in it ,and shoting it .
# UART DMA test finish ok
# SYStem_tick test ok.
# the MACRO of GPIOtogle test finish OK.
# the flash_as_eeprom test finish OK.
# SPI  finish 
# TLE5012 finish 
# PWMin  function test ok
# pwmOut function 
# FOC    port finish, need test it.
# adc_dma finish and test ok.