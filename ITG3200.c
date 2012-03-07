//
// This code is distributed under the GNU General Public License
// which can be found at http://www.gnu.org/licenses/gpl.txt
// It is meant for information purposes only and is in no way guaranteed
// to be accurate, bug free or even working.
// Use it at your own risk.
// 

#include "ITG3200.h"
#include "util/delay.h"


#ifdef ITG3200_USE_INTERRUPTS
// interrupt driven state
volatile static u08  ITG3200_state = 10;
volatile static u08  ITG3200_numBytes = 0;
volatile static u08* ITG3200_data = 0;
volatile static u08  ITG3200_regAddr = 0;
#else
#define TWIE 0
#endif


#define TWCR_CMD_MASK		0x0F
#define TWSR_STATUS_MASK	0xF8

#define TW_START					0x08
#define TW_REP_START				0x10
#define TW_MT_SLA_ACK				0x18
#define TW_MT_DATA_ACK				0x28
#define TW_MR_SLA_ACK				0x40
#define TW_MR_DATA_ACK				0x50





void itg3200_i2cSetBitrate(u16 bitrateKHz)
{
	u08 bitrate_div;
	// set i2c bitrate
	// SCL freq = F_CPU/(16+2*TWBR))
	#ifdef TWPS0
		// for processors with additional bitrate division (mega128)
		// SCL freq = F_CPU/(16+2*TWBR*4^TWPS)
		// set TWPS to zero
		cbi(TWSR, TWPS0);
		cbi(TWSR, TWPS1);
	#endif
	// calculate bitrate division	
	bitrate_div = ((F_CPU/1000l)/bitrateKHz);
	if(bitrate_div >= 16)
		bitrate_div = (bitrate_div-16)/2;
	outb(TWBR, bitrate_div);
}


// functions
void itg3200_i2cInit(u16 bitrateKHz)
{
	// set i2c bit rate 
	itg3200_i2cSetBitrate(bitrateKHz);
	// enable TWI (two-wire interface)
	sbi(TWCR, TWEN);
	// enable TWI interrupt and slave address ACK
#ifdef ITG3200_USE_INTERRUPTS
	sbi(TWCR, TWIE);
#endif
	sbi(TWCR, TWEA);
	// don't enable interrupts, do this externally when ready
	//sei();
}




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


inline void itg3200_WaitForComplete()
{
	while(!(TWCR & BV(TWINT)));
}


inline void itg3200_ReceiveByte(u08 ackFlag)
{
	// begin receive over i2c
	if( ackFlag )
	{
		// ackFlag = TRUE: ACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWEA));
	}
	else
	{
		// ackFlag = FALSE: NACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT));
	}
}




u08 itg3200_read_register(u08 reg, u08 *value, u08 numBytes)
{
//1.
	// start
	TWCR =  BV(TWSTA) | BV(TWEN);
	itg3200_WaitForComplete();
	if( (TWSR & 0xF8) != TW_START )
		return 1;
//2.
	// SDA_W
	TWDR = (ITG3200_DEV_ADDR << 1)  & 0xFE;
	TWCR = BV(TWINT) | BV(TWEN);
	itg3200_WaitForComplete();
	if( (TWSR & 0xF8) != TW_MT_SLA_ACK)
		return 2;
//3.
	// REG
	TWDR = reg;
	TWCR = BV(TWINT) | BV(TWEN);
	itg3200_WaitForComplete();
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK)
		return 3;
//4.
	//START
	TWCR = BV(TWINT) | BV(TWSTA) | BV(TWEN);
	itg3200_WaitForComplete();
	if( (TWSR & 0xF8) != TW_REP_START )
		return 4;
//5.
	// SDA_R
	TWDR = (ITG3200_DEV_ADDR << 1)  | 0x01;
	TWCR = BV(TWINT) | BV(TWEN);
	itg3200_WaitForComplete();
	if( (TWSR & 0xF8) != TW_MR_SLA_ACK)
		return (TWSR & 0xF8);
//6.	
	do {
		// receive + N/ACK
		--numBytes;
		itg3200_ReceiveByte( numBytes > 0 );
		itg3200_WaitForComplete();
		*value = inb(TWDR);
		++value;

	}while(numBytes > 0);

	// send stop
	TWCR = BV(TWINT)|BV(TWEA)|BV(TWSTO)|BV(TWEN);
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

	// send stop
	outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWEA)|BV(TWSTO));
	return 0;

}



#ifdef ITG3200_USE_INTERRUPTS

inline void itg3200_SendByte(u08 data)
{
	// save data to the TWDR
	outb(TWDR, data);
	// begin send
	TWCR = BV(TWINT) | BV(TWIE) | BV(TWEN);
}

BOOL itg3200_isBusy()
{
	return ITG3200_state != 10;
}

void itg3200_read_register_async(u08 reg, u08 *value, u08 numBytes)
{
	while(ITG3200_state != 10); // wait until free
	ITG3200_data = value;
	ITG3200_regAddr = reg;
	ITG3200_numBytes = numBytes;
	// start
	ITG3200_state = 1;
	TWCR = BV(TWINT) | BV(TWSTA) | BV(TWEN) | BV(TWIE);
}



SIGNAL(SIG_2WIRE_SERIAL)
{
	switch(ITG3200_state)
	{
		case 1:
			if( (TWSR & 0xF8) != TW_START ) goto EO_TX;
			ITG3200_state++;
			// SDA_W
			TWDR = (ITG3200_DEV_ADDR << 1)  & 0xFE;
			TWCR = BV(TWINT) | BV(TWEN) | BV(TWIE);
			return;
		case 2:
			if( (TWSR & 0xF8) != TW_MT_SLA_ACK) goto EO_TX;	
			ITG3200_state++;
			// REG
			TWDR = ITG3200_regAddr;
			TWCR = BV(TWINT) | BV(TWEN) | BV(TWIE);
			return;
		case 3:
			if( (TWSR & 0xF8) != TW_MT_DATA_ACK) goto EO_TX;
			ITG3200_state++;
			//START
			TWCR = BV(TWINT) | BV(TWSTA) | BV(TWEN) | BV(TWIE);
			return;
		case 4:
			if( (TWSR & 0xF8) != TW_REP_START) goto EO_TX;
			ITG3200_state++;
			// SDA_R
			TWDR = (ITG3200_DEV_ADDR << 1)  | 0x01;
			TWCR = BV(TWINT) | BV(TWEN) | BV(TWIE);
			return;
		case 5:
			if( (TWSR & 0xF8) != TW_MR_SLA_ACK) goto EO_TX;
			ITG3200_state++;
			--ITG3200_numBytes;
			// receive + N/ACK
			itg3200_ReceiveByte( ITG3200_numBytes > 0 );
			return;
		case 6:		
			*ITG3200_data = inb(TWDR);
			if(ITG3200_numBytes == 0) goto EO_TX; // end of transmission
			++ITG3200_data;
			--ITG3200_numBytes;
			// receive + N/ACK
			itg3200_ReceiveByte( ITG3200_numBytes > 0 );
			return;
		case 10:	// nothing to do
		default:
			return;
	}
	
EO_TX:
	// send stop
	TWCR = BV(TWINT)|BV(TWEA)|BV(TWSTO)|BV(TWIE)|BV(TWEN);
	ITG3200_state = 10;
	return;
}
#endif
