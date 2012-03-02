//
// This code is distributed under the GNU General Public License
// which can be found at http://www.gnu.org/licenses/gpl.txt
// It is meant for information purposes only and is in no way guaranteed
// to be accurate, bug free or even working.
// Use it at your own risk.
// 

#ifndef ITG3200_H
#define ITG3200_H

#include "global.h"



// prototypes

u08 itg3200_init();
u08 itg3200_read_register(u08 reg, u08 *value, u08 numBytes);
u08 itg3200_write_register(u08 reg, u08 *value, u08 numBytes);

inline BOOL itg3200_dataReady()
{	return (ITG3200_INT_PIN & BT(ITG3200_INT_BIT)); }



// device bus address

#define ITG3200_DEV_ADDR 104


// registers

#define ITG3200_REG_WHO_AM_I 	0x00
#define ITG3200_REG_SMPLRT 		0x15
#define ITG3200_REG_DLPF_FS 	0x16
#define ITG3200_REG_INT_CFG 	0x17
#define ITG3200_REG_INT_STATUS 	0x1A
#define ITG3200_REG_TEMP_OUT_H 	0x1B
#define ITG3200_REG_TEMP_OUT_L 	0x1C
#define ITG3200_REG_GYRO_XOUT_H	0x1D
#define ITG3200_REG_GYRO_XOUT_L	0x1E
#define ITG3200_REG_GYRO_YOUT_H	0x1F
#define ITG3200_REG_GYRO_YOUT_L	0x20
#define ITG3200_REG_GYRO_ZOUT_H	0x21
#define ITG3200_REG_GYRO_ZOUT_L	0x22
#define ITG3200_REG_PWR_MGM		0x3E

















#endif
