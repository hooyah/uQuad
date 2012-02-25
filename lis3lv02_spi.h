#ifndef LIS3LV02_SPI_H
#define LIS3LV02_SPI_H

#include "global.h"



// use these defines in your global.h
//#define LIS3LV02_CS_DDR  DDRB
//#define LIS3LV02_CS_PORT PORTB
//#define LIS3LV02_CS_BIT  4
//#define LIS3L02_DRY_PORT PORTB
//#define LIS3L02_DRY_PIN	 PINB
//#define LIS3L02_DRY_DDR	 DDRB
//#define LIS3L02_DRY_BIT	 2



///// functions

void lis3l_init();

u08  lis3l_ReadRegister(u08 addr);

void lis3l_WriteRegister(u08 addr, u08 data);

s16  lis3l_GetAccel(u08 chxyz);

u08	 lis3l_Reset(void);


inline BOOL lis3l02_DataReady()
{
	return (LIS3L02_DRY_PIN & BT(LIS3L02_DRY_BIT));
}


inline void lis3l_select()
{
	cbi(LIS3L02_CS_PORT, LIS3L02_CS_BIT);
}


inline void lis3l_deselect()
{
	sbi(LIS3L02_CS_PORT, LIS3L02_CS_BIT);
}


// register address defines

#define LIS3L02_REG_WHO_AM_I	0x0f	///< LIS3L02 dummy
#define LIS3L02_REG_OFFSETX		0x16	///< LIS3L02 X-axis digital offset trim
#define LIS3L02_REG_OFFSETY		0x17	///< LIS3L02 Y-axis digital offset trim
#define LIS3L02_REG_OFFSETZ		0x18	///< LIS3L02 Z-axis digital offset trim
#define LIS3L02_REG_GAINX		0x19	///< LIS3L02 X-axis digital gain trim
#define LIS3L02_REG_GAINY		0x1A	///< LIS3L02 Y-axis digital gain trim
#define LIS3L02_REG_GAINZ		0x1B	///< LIS3L02 Z-axis digital gain trim
#define LIS3L02_REG_CTRLREG1	0x20	///< LIS3L02 interface/operation control
#define LIS3L02_REG_CTRLREG2	0x21	///< LIS3L02 interface/operation control
#define LIS3L02_REG_WAKEUPCFG	0x23	///< LIS3L02 interrupt/wakeup config
#define LIS3L02_REG_WAKEUPSRC	0x24	///< LIS3L02 interrupt/wakeup source indicator
#define LIS3L02_REG_WAKEUPACK	0x25	///< LIS3L02 wakeup source clear
#define LIS3L02_REG_STATUS		0x27	///< LIS3L02 Accelerometer Status
#define LIS3L02_REG_OUTXL		0x28	///< LIS3L02 Accelerometer X Output Low-byte
#define LIS3L02_REG_OUTXH		0x29	///< LIS3L02 Accelerometer X Output High-byte
#define LIS3L02_REG_OUTYL		0x2A	///< LIS3L02 Accelerometer Y Output Low-byte
#define LIS3L02_REG_OUTYH		0x2B	///< LIS3L02 Accelerometer Y Output High-byte
#define LIS3L02_REG_OUTZL		0x2C	///< LIS3L02 Accelerometer Z Output Low-byte
#define LIS3L02_REG_OUTZH		0x2D	///< LIS3L02 Accelerometer Z Output High-byte
#define LIS3L02_REG_THSL		0x2E	///< LIS3L02 Accelerometer Threshold Low-byte
#define LIS3L02_REG_THSH		0x2F	///< LIS3L02 Accelerometer Threshold High-byte
#define LIS3L02_REG_MULTIREAD	0x80	///< LIS3L02 Mutliple Read Bit

// LIS3L02 control register 1 bit defines
#define LIS3L02_CTRLREG1_XEN	0x01	///< LIS3L02 CtrlReg1 X-axis Enable
#define LIS3L02_CTRLREG1_YEN	0x02	///< LIS3L02 CtrlReg1 Y-axis Enable
#define LIS3L02_CTRLREG1_ZEN	0x04	///< LIS3L02 CtrlReg1 Z-axis Enable
#define LIS3L02_CTRLREG1_ST		0x08	///< LIS3L02 CtrlReg1 Self-Test Enable
#define LIS3L02_CTRLREG1_DF0	0x10	///< LIS3L02 CtrlReg1 Decimation Factor 0
#define LIS3L02_CTRLREG1_DF1	0x20	///< LIS3L02 CtrlReg1 Decimation Factor 0
#define LIS3L02_CTRLREG1_PD0	0x40	///< LIS3L02 CtrlReg1 Power-down Control 0
#define LIS3L02_CTRLREG1_PD1	0x80	///< LIS3L02 CtrlReg1 Power-down Control 1

// LIS3L02 control register 2 bit defines
#define LIS3L02_CTRLREG2_DAS	0x01	///< LIS3L02 CtrlReg2 Data Alignment Selection
#define LIS3L02_CTRLREG2_SIM	0x02	///< LIS3L02 CtrlReg2 SPI Mode Select
#define LIS3L02_CTRLREG2_DRDY	0x04	///< LIS3L02 CtrlReg2 Enable Data-Ready generation
#define LIS3L02_CTRLREG2_IEN	0x08	///< LIS3L02 CtrlReg2 Interrupt Enable
#define LIS3L02_CTRLREG2_BOOT	0x10	///< LIS3L02 CtrlReg2 Reboot from memory
#define LIS3L02_CTRLREG2_BDU	0x40	///< LIS3L02 CtrlReg2 Block Data Update
#define LIS3L02_CTRLREG2_FS		0x80	///< LIS3L02 CtrlReg2 Full-scale Select (0=2g, 1=6g)

// LIS3L02 WAKEUPCFG register bit defines
#define LIS3L02_WAKEUPCFG_MXL	0x01	///< LIS3L02 WAKEUPCFG Mask X Low Interrupt
#define LIS3L02_WAKEUPCFG_MXH	0x02	///< LIS3L02 WAKEUPCFG Mask X High Interrupt
#define LIS3L02_WAKEUPCFG_MYL	0x04	///< LIS3L02 WAKEUPCFG Mask Y Low Interrupt
#define LIS3L02_WAKEUPCFG_MYH	0x08	///< LIS3L02 WAKEUPCFG Mask Y High Interrupt
#define LIS3L02_WAKEUPCFG_MZL	0x10	///< LIS3L02 WAKEUPCFG Mask Z Low Interrupt
#define LIS3L02_WAKEUPCFG_MZH	0x20	///< LIS3L02 WAKEUPCFG Mask Z High Interrupt
#define LIS3L02_WAKEUPCFG_LIR	0x40	///< LIS3L02 WAKEUPCFG Latch Intr Request

// LIS3L02 WAKEUPSRC register bit defines
#define LIS3L02_WAKEUPSRC_XL	0x01	///< LIS3L02 WAKEUPSRC X Low Interrupt
#define LIS3L02_WAKEUPSRC_XH	0x02	///< LIS3L02 WAKEUPSRC X High Interrupt
#define LIS3L02_WAKEUPSRC_YL	0x04	///< LIS3L02 WAKEUPSRC Y Low Interrupt
#define LIS3L02_WAKEUPSRC_YH	0x08	///< LIS3L02 WAKEUPSRC Y High Interrupt
#define LIS3L02_WAKEUPSRC_ZL	0x10	///< LIS3L02 WAKEUPSRC Z Low Interrupt
#define LIS3L02_WAKEUPSRC_ZH	0x20	///< LIS3L02 WAKEUPSRC Z High Interrupt
#define LIS3L02_WAKEUPSRC_IA	0x40	///< LIS3L02 WAKEUPSRC Interrupt Active

// LIS3L02 WAKEUPSRC register bit defines
#define LIS3L02_STATUS_XDA		0x01	///< LIS3L02 STATUS X New Data Available
#define LIS3L02_STATUS_YDA		0x02	///< LIS3L02 STATUS Y New Data Available
#define LIS3L02_STATUS_ZDA		0x04	///< LIS3L02 STATUS Z New Data Available
#define LIS3L02_STATUS_ZYXDA	0x08	///< LIS3L02 STATUS XYZ New Data Available
#define LIS3L02_STATUS_XOR		0x10	///< LIS3L02 STATUS X-axis Data Overrun
#define LIS3L02_STATUS_YOR		0x20	///< LIS3L02 STATUS Y-axis Data Overrun
#define LIS3L02_STATUS_ZOR		0x40	///< LIS3L02 STATUS Z-axis Data Overrun
#define LIS3L02_STATUS_ZYXOR	0x80	///< LIS3L02 STATUS XYZ-axis Data Overrun






#endif
