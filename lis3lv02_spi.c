#include "lis3lv02_spi.h"
#include "spi.h"
#include "util/delay.h"

#ifndef BT
#define BT(A) (1<<(A))
#endif






void lis3l_init()
{
	// set CS to output and and idle it (high)
    sbi(LIS3L02_CS_DDR,  LIS3L02_CS_BIT);
	cbi(LIS3L02_CS_PORT, LIS3L02_CS_BIT);

	// set data ready pin (DRY) to input and activate pullup
	cbi(LIS3L02_DRY_DDR,  LIS3L02_DRY_BIT);
	sbi(LIS3L02_DRY_PORT, LIS3L02_DRY_BIT);

}


u08 lis3l_ReadRegister(u08 addr)
{
	lis3l_select();
	spiTransferByte(addr|BT(7));
	spiTransferByte(0);
	lis3l_deselect();
	return inb(SPDR);
}

void lis3l_WriteRegister(u08 addr, u08 data)
{
	lis3l_select();
	spiTransferByte(addr&(~BT(7)));
	spiTransferByte(data);
	lis3l_deselect();
	return;
}


s16 lis3l_GetAccel(u08 chxyz)
{
	s16 value;
	
	// todo: this can probably be read in one go with burst mode
	value  = lis3l_ReadRegister(LIS3L02_REG_OUTXL + (chxyz<<1));
//	_delay_us(1);
	value |= lis3l_ReadRegister(LIS3L02_REG_OUTXH + (chxyz<<1))<<8;

	return value;
}


u08 lis3l_Reset(void)
{

	// turn on device and enable X,Y,Z, set samplerate to 160Hz
	lis3l_WriteRegister(LIS3L02_REG_CTRLREG1,
		LIS3L02_CTRLREG1_XEN |
		LIS3L02_CTRLREG1_YEN |
		LIS3L02_CTRLREG1_ZEN |
		LIS3L02_CTRLREG1_DF0 |
		LIS3L02_CTRLREG1_PD0);
	_delay_us(1);

	// scale and justification options, 2g scale
	lis3l_WriteRegister(LIS3L02_REG_CTRLREG2,
		LIS3L02_CTRLREG2_BOOT | 
		LIS3L02_CTRLREG2_DAS |
		LIS3L02_CTRLREG2_BDU |
//		LIS3L02_CTRLREG2_FS |
		LIS3L02_CTRLREG2_DRDY );

	_delay_ms(10);
	u08 id = lis3l_ReadRegister(LIS3L02_REG_WHO_AM_I);
	if(id == 0x3a)
		return 1;
	else
		return 0;
}