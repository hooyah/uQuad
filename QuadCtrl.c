#include "global.h"

#include "util/delay.h"

#include "timer.h"
#include "uart.h"
#include "rprintf.h"
#include "spi.h"
#include "i2c.h"
#include "a2d.h"

#include "lis3lv02_spi.h"
#include "ITG3200.h"

#include "ctrl.h"


static u08 Thrust = 0;
static s08 Yaw = 0;
static s08 Pitch = 0;
static s08 Roll = 0;


s16 gyro_0x = 0;
s16 gyro_0y = 0;
s16 gyro_0z = 0;
s16 accel_0x = 0;
s16 accel_0y = 0;
s16 accel_0z = 0;

float PID_Kp = 5.0f;
float PID_Ki = 0.0f;
float PID_Kd = 2.0f;

float PID_Kp_yaw = 0.2f;
float PID_Ki_yaw = 0.0f;
float PID_Kd_yaw = 0.0f;

float Gain = 1.0; // joystick gain


// access routines
void mySpiInit()
{
    // setup SPI I/O pins
    sbi(PORTB, 7);  // set SCK hi
    sbi(DDRB, 7);   // set SCK as output
    cbi(DDRB,  6);   // set MISO as input
    sbi(PORTB, 6);   // set MISO pullup
    sbi(DDRB, 5);   // set MOSI as output

	// setup SPI interface :
	// master mode
	sbi(SPCR, MSTR);
	// clock = f/4
	cbi(SPCR, SPR0);
	cbi(SPCR, SPR1);
	// clock = f/16
//	cbi(SPCR, SPR0);
//	sbi(SPCR, SPR1);
	// select clock phase positive-going in middle of data
	sbi(SPCR, CPOL);
	sbi(SPCR, CPHA);
	// Data order MSB first
	cbi(SPCR,DORD);
	// enable SPI
	cbi(SPCR, SPIE);
	sbi(SPCR, SPE);
			
	// clear status
	inb(SPSR);

}










void init_peripherals()
{

	// LED
	DDRC |= (1<<2);

	// accel pins
	lis3l_init();



	// UART
	uartInit();
	uartSetBaudRate(19200);
	uartSetFrameFormat(8, 0, 1);
	rprintfInit(uartAddToTxBuffer);
	cbi(DDRB, 1);		// XBee CTS on PB1 


	// I2C
	i2cInit();
	sbi(PORTC, 0);	// i2c SCL on ATmega163,323,16,32,etc
	sbi(PORTC, 1);	// i2c SDA on ATmega163,323,16,32,etc
	cbi(TWCR, TWIE); // disable interrupt
	i2cSetBitrate(200); // todo, check if if w ecan do 200

	// SPI
	mySpiInit();

	// a2d
	a2dInit();
	a2dSetReference(ADC_REFERENCE_256V);
	a2dSetChannel(7);
	cbi(PORTA, 7);
	cbi(DDRA, 7);
	a2dStartConvert();

	_delay_ms(50);

	// accel
	BOOL accelOkay = lis3l_Reset();
	itg3200_init();


	// switch xbee to higher baudrate
//	rprintfStr("+++");
//	_delay_ms(55);
//	rprintfStr("ATBD6,CN\r");

}




void init_motors()
{
		//off
	cbi(PORTD, 4); //motor off
	cbi(PORTD, 5); //motor off
	cbi(PORTD, 7); //motor off
	cbi(PORTB, 3); //motor off

	sbi(DDRD, 4); //motor outs
	sbi(DDRD, 5); //motor outs
	sbi(DDRD, 7); //motor outs
	sbi(DDRB, 3); //motor outs


	timerInit();
	cbi(TIMSK, TOIE0); // disable timer 0 overflow interrupt
	cbi(TIMSK, TOIE1); // disable timer 1 overflow interrupt
	cbi(TIMSK, TOIE2); // disable timer 2 overflow interrupt

	timer0SetPrescaler(TIMER_CLK_DIV1);
	timer1SetPrescaler(TIMER_CLK_DIV1);
	timer2SetPrescaler(TIMER_CLK_DIV1);

	// setup PWM timer 0
	OCR0 = 0;	// duty cycle 0%
	// enable timer0 as PWM phase correct, todo: use fast pwm
	sbi(TCCR0,WGM00);
	cbi(TCCR0,WGM01);
	// turn on channel (OC0) PWM output
	// set OC0 as non-inverted PWM
	cbi(TCCR0,COM00);
	sbi(TCCR0,COM01);


	// setup timer 1A/B
	timer1PWMInit(8);	// pwm 8 bit

	timer1PWMASet(0);	// duty cycle 0%
	timer1PWMBSet(0);	// duty cycle 0%
	timer1PWMAOn();
	timer1PWMBOn();

	//setup PWM timer 2
	OCR2 = 0;	// duty cycle 0%
	// enable timer2 as PWM phase correct, todo: use fast pwm
	sbi(TCCR2,WGM20);
	cbi(TCCR2,WGM21);
	// turn on channel (OC0) PWM output
	// set OC0 as non-inverted PWM
	cbi(TCCR2,COM20);
	sbi(TCCR2,COM21);


	// enable timer interrupt
	sbi(TIMSK, TOIE0); //enable timer 0 overflow interrupt
}


union InData{

	u08 bytes[4];
	u16 words[2];
	u32 integer;
};

void checkForInput()
{
static u08 var = 0;
static union InData tmp;

	u08 c;
	int in;
	while( (in = uartGetByte()) != -1)
	{
		//uartSendByte(in);
		c = in;
		if(c == '#') // start of message
		{
			tmp.integer = 0;
			var = 0;
		}
		else if(var == 0)
		{
			var = c;
		}
		else if(c >= '0' && c <= '9')
		{
			tmp.integer = (tmp.integer<<4) | (c - '0');
		}
		else if(c >= 'a' && c <= 'f')
		{
			tmp.integer = (tmp.integer<<4) | (c - 'a' + 10);
		}
		else if(c == '*') // end of message
		{

			if(var == 'C')
			{
				Thrust = tmp.bytes[3];
				Yaw   = (tmp.bytes[2] - 50);
				Pitch = (tmp.bytes[1] - 50);
				Roll  = (tmp.bytes[0] - 50);
			}
			else if(var == 'P')
				PID_Kp = (float)tmp.words[0] / 10.0f;
			else if(var == 'I')
				PID_Ki = (float)tmp.words[0] / 10.0f;
			else if(var == 'D')
				PID_Kd = (float)tmp.words[0] / 10.0f;
			else if(var == 'Q')//P_yaw
				PID_Kp_yaw = (float)tmp.words[0] / 10.0f;
			else if(var == 'J')//I_yaw
				PID_Ki_yaw = (float)tmp.words[0] / 10.0f;
			else if(var == 'E')//D_yaw
				PID_Kd_yaw = (float)tmp.words[0] / 10.0f;
			else if(var == 'G')//D_yaw
				Gain = (float)tmp.words[0] / 100.0f;
			else if(var == 'M')
				Thrust = tmp.bytes[0];
//			setMotorSpeeds();
		}
	}
}


u16 getBatteryVoltage()
{
	if(a2dIsComplete())
	{
		u16 ad = (inb(ADCL) | (inb(ADCH)<<8));	// read ADC (full 10 bits);
		a2dStartConvert();
		return ad;
	}
	else
		return 0;
}





void calibrate_sensors()
{

	// disable interrupts
//	cli();


	PORTC |= (1<<2);
	_delay_ms(2000);
	PORTC &= ~(1<<2);

	//after power up, wait for the user put it down and come to a rest
	// then we take the reading of the sensors and define a zero point
	{
		s16 acc[3], last[3];
		u08 dur = 0;

		last[0] = last[1] = last[2] = acc[0] = acc[1] = acc[2] = 0;

		while(dur < 50)
		{
			acc[0] = lis3l_GetAccel(0);
			acc[1] = lis3l_GetAccel(1);
			acc[2] = lis3l_GetAccel(2);
		
			if(abs(acc[0] - last[0]) < 100 &&
					abs(acc[1] - last[1]) < 100 &&
					abs(acc[2] - last[2]) < 100 )
				dur++; 
			else
				dur=0;

			last[0] = acc[0];
			last[1] = acc[1];
			last[2] = acc[2];
			_delay_ms(10);
//			rprintf("%d,%d,%d,%d*\n",dur,acc[0], acc[1], acc[2]);

		}
	}

	// okay, all quiet
	// let's take an average
	{
		PORTC |= (1<<2);

		s32 acc[3], gyr[3];
		gyr[0] = gyr[1] = gyr[2] = acc[0] = acc[1] = acc[2] = 0;

		u08 dat[6];
		for(u08 c = 0; c < 128; c++)
		{
			acc[0] += lis3l_GetAccel(0);
			acc[1] += lis3l_GetAccel(1);
			acc[2] += lis3l_GetAccel(2);

			itg3200_read_register(ITG3200_REG_GYRO_XOUT_H, dat, 6);
			for(u08 i = 0; i < 3; ++i) 
				gyr[i] += (dat[i*2] << 8) | dat[i*2+1];

			_delay_ms(15);
		}

		gyro_0x = gyr[0] / 128;
		gyro_0y = gyr[1] / 128;
		gyro_0z = gyr[2] / 128;
		
		accel_0x= acc[0] / 128;
		accel_0y= acc[1] / 128;
		accel_0z= 0;//acc[2] / 50; can't find z zero under the influence of gravity

		PORTC &= ~(1<<2);
	}

	// enable interrupts
	//sei();

	//rprintf("zero values\n acc: %d,%d,%d\n gyro:%d,%d,%d",accel_0x, accel_0y, accel_0z, gyro_0x, gyro_0y, gyro_0z);
}





int main()
{
	init_motors();
	init_peripherals();



	// find out how many cells are connected

	u16 batteryCutoff;
	while((batteryCutoff = getBatteryVoltage()) == 0);
	if(batteryCutoff > 720) // voltage is higher than 6V -> 2cells
		batteryCutoff = 720;	// cutoff 6V
	else
		batteryCutoff = 360; 	// cutoff 3V


	calibrate_sensors();
	// enable interrupt driven gyro acquisation
	startGyro();



	u08 c = 0, maxThrust = (batteryCutoff>360)?70:100;
	u08 cutBattery = 0;
	while(1) {

		if(Thrust > maxThrust)
			Thrust = maxThrust;
		calculate_balance(Thrust, Yaw, Pitch, Roll);


		checkForInput();


		// check battery
		if(((c%100)==0)) {

			u16 bat = getBatteryVoltage();
			if(bat > 0 && bat <= batteryCutoff) {
				if(maxThrust > 0)
					maxThrust -= 5;
				cutBattery = 1;
			}
		}

		if(!cutBattery || !(c % 50))
			PORTC ^= (1<<2);
	
		c++;
		//_delay_ms(100);
	}

}

//TIMER_INTERRUPT_HANDLER(SIG_OVERFLOW0)
//{
//  return;
//}


