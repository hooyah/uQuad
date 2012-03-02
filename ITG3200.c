//
// This code is distributed under the GNU General Public License
// which can be found at http://www.gnu.org/licenses/gpl.txt
// It is meant for information purposes only and is in no way guaranteed
// to be accurate, bug free or even working.
// Use it at your own risk.
// 

#include "ITG3200.h"
#include "i2c.h"
#include "util/delay.h"


u08 itg3200_init()
{
u08 byte;

	// sampling rate to 100Hz
	byte = 9;	
	itg3200_write_register(ITG3200_REG_SMPLRT, &byte, 1);
	_delay_us(20);

	// range, only full range supported, low pass 98hz
	byte = BV(3)|BV(4) | 2;	
	itg3200_write_register(ITG3200_REG_DLPF_FS, &byte, 1);
	_delay_us(20);

	// set internal clk source to X axis
	byte = BV(0);	
	itg3200_write_register(ITG3200_REG_PWR_MGM, &byte, 1);
	_delay_us(20);

	// set interrupt pin to: active hight, latch until read, data_ready
	byte = BV(5)|BV(4)|BV(0);
	itg3200_write_register(ITG3200_REG_INT_CFG, &byte, 1);

	return 0;
}



u08 itg3200_read_register(u08 reg, u08 *value, u08 numBytes)
{

	// start
	TWCR = BV(TWINT) | BV(TWSTA) | BV(TWEN);
	while(!(TWCR & BV(TWINT)));
	if( (TWSR & 0xF8) != TW_START )
		return 1;

	// SDA_W
	TWDR = (ITG3200_DEV_ADDR << 1)  & 0xFE;
	TWCR = BV(TWINT) | BV(TWEN);
	while(!(TWCR & BV(TWINT)));
	if( (TWSR & 0xF8) != TW_MT_SLA_ACK)
		return 2;

	// REG
	TWDR = reg;
	TWCR = BV(TWINT) | BV(TWEN);
	while(!(TWCR & BV(TWINT)));
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK)
		return 3;

	//START
	TWCR = BV(TWINT) | BV(TWSTA) | BV(TWEN);
	while(!(TWCR & BV(TWINT)));
	if( (TWSR & 0xF8) != TW_REP_START )
		return 4;

	// SDA_R
	TWDR = (ITG3200_DEV_ADDR << 1)  | 0x01;
	TWCR = BV(TWINT) | BV(TWEN);
	while(!(TWCR & BV(TWINT)));
	if( (TWSR & 0xF8) != TW_MR_SLA_ACK)
		return (TWSR & 0xF8);
	
	do {
		// receive + N/ACK
		--numBytes;
		i2cReceiveByte( numBytes > 0 );
		i2cWaitForComplete();
		*value = i2cGetReceivedByte();
		++value;

	}while(numBytes > 0);

	i2cSendStop();
	return 0;


}


u08 itg3200_write_register(u08 reg, u08 *value, u08 numBytes)
{


	// start
	TWCR = BV(TWINT) | BV(TWSTA) | BV(TWEN);
	while(!(TWCR & BV(TWINT)));
	if( (TWSR & 0xF8) != TW_START )
		return 1;

	// SDA_W
	TWDR = (ITG3200_DEV_ADDR << 1)  & 0xFE;
	TWCR = BV(TWINT) | BV(TWEN);
	while(!(TWCR & BV(TWINT)));
	if( (TWSR & 0xF8) != TW_MT_SLA_ACK)
		return 2;

	// REG
	TWDR = reg;
	TWCR = BV(TWINT) | BV(TWEN);
	while(!(TWCR & BV(TWINT)));
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK)
		return 3;

	
	do {

		// SDA_W
		TWDR = *value;
		TWCR = BV(TWINT) | BV(TWEN);
		while(!(TWCR & BV(TWINT)));
		if( (TWSR & 0xF8) != TW_MT_DATA_ACK)
			return 4;

		--numBytes;
		++value;

	}while(numBytes > 0);

	i2cSendStop();
	return 0;

}
