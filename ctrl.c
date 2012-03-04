//
// This code is distributed under the GNU General Public License
// which can be found at http://www.gnu.org/licenses/gpl.txt
// It is meant for information purposes only and is in no way guaranteed
// to be accurate, bug free or even working.
// Use it at your own risk.
// 

#include "ctrl.h"
#include "global.h"
#include "ITG3200.h"
#include "lis3lv02_spi.h"
#include "rprintf.h"
#include "util/delay.h"
#include "math.h"
#include "uart.h"

#define min(A, B) (((A)<(B))?(A):(B))
#define max(A, B) (((A)>(B))?(A):(B))
#define clamp(MIN, MAX, VAL) max(MIN, min(MAX, VAL))


#define CTRL_FREQ 100.0f // (Hz)
#define CTRL_DT (1.0f / CTRL_FREQ)

// these values map rpm in % to dutycycles
// (not currently used)
static u08 motor_lut[4][11] = 
{
// 0, 10, 20, 30, 40, 50, 60, 60, 70 90, 100% 

//MotorA
{0,6,13,23,35,51,72,98,132,175,223},

//MotorB
{0,7,13,26,39,53,74,105,142,187,244},

//MotorC
{0,12,22,33,48,65,89,120,158,213,252},

//MotorD
{0,7,14,24,38,57,83,114,151,202,252}
};


volatile s32  accu_gyro[3] = {0,0,0};
volatile u16  accu_dt = 0;
volatile BOOL accu_sampling = TRUE;
volatile u16  accu_test = 0;



extern s16 accel_0x, accel_0y, accel_0z;
extern s16 gyro_0x, gyro_0y, gyro_0z;


extern float PID_Kp, PID_Kp_yaw;
extern float PID_Kd, PID_Kd_yaw;
extern float PID_Ki, PID_Ki_yaw;
extern float Gain;

extern unsigned long Timer0Reg0;


u08 rpm_to_duty(u08 motorNo, s16 rpmPercent10)
{

//	rpmPercent10 = clamp(0,999, rpmPercent10);
	
	// test. since thrust is rpm squared
	//  pwm and thrust coule actually be more or less linear
	// hence the inverse lookup might not be needed
	return clamp(0,25000, rpmPercent10*25) / 100;

//	rpmPercent10 = clamp(0,800, rpmPercent10);

	u08 i0 = (rpmPercent10) / 100;
	u08 i1 = i0+1;

	u08 v0 = motor_lut[motorNo][i0];
	u08 v1 = motor_lut[motorNo][i1];

	u08 fact10 = rpmPercent10 % 100;
	u08 fact10inv = 100 - rpmPercent10 % 100;

	u16 ret = fact10inv * v0 + fact10 * v1;

	return ret / 100;
}

/*
// this works well to a certain degree at which it behaves a tad unlinear
void calc_tilt_roll(s16* ac, float *outTilt, float *outRoll)
{
	// normalize the accelerometer heading
	float x = ac[0];
	float y = ac[1];
	float z = ac[2];
	float inv_magxz = -1.0f / sqrt( x*x + z*z );
	float inv_magyz = -1.0f / sqrt( y*y + z*z );

	// tilt and roll are angles between gravity (0,0,-1) and accel in the planes xz and yz
	// angle = acos( dot(a, g) ) (pre normalized)

	// tilt (xz)
	// 	float tilt = acos( (0.0f) * x/magxz + (-1.0f) * z/magxz ) 
	//	= acos( (-1)*z/magxz )
	//  = acos ( z * inv_magxz )
	float tilt = acos( z * inv_magxz );
	if(ac[0] < 0)
		tilt = -tilt;

	// roll (yz)
	float roll = acos( z * inv_magyz );
	if(ac[1] < 0)
		roll = -roll;

	*outRoll = roll / PI * 180.0;
	*outTilt = tilt / PI * 180.0;
}

*/

void calc_tilt_roll(s16* ac, float *outTilt, float *outRoll)
{
	// normalize the accelerometer heading
	float x = ac[0];
	float y = ac[1];
	float z = ac[2];
	float magacc = sqrt( x*x + y*y + z*z );

	//tilt angle, angle = acos( dot(a, g) ) with g=gravity=(0,0,-1)
	//float ang = acos( x/magacc*0 + y/magacc*0 + z/magacc*(-1.0) )
	float ang = (magacc > 0.0f)?(acos( (z/magacc)*(-1.0) ) / PI * 180.0):0.0f;

	// extract 2d direction of the tilt and change the magnitude to the angle
	float magaccxy = (ang > 0.1)?(ang / sqrt( x*x + y*y )):0.0f;
	x = x * magaccxy;
	y = y * magaccxy;


	*outTilt = x;
	*outRoll = y;
}









/*inline
float PID(const float Error, float* lastError, float *Integ, 
			const float Kp, const float Ki, const float Kd)
{
float Output;

//	*Integ += Error;
	float D = Error - *lastError;
	*lastError = Error;

	// test D squared
    Output = Kp * Error;

//    Output += Ki * *Integ;
    Output += Kd * D;
     
    //return Output;
	return clamp(-30.f, 30.f, Output);
}*/




static u08 motorDuty[5] = {0,0,0,0, 0}; // +1 as a security buffer

void setMotorSpeeds()
{
	OCR1A = motorDuty[2]; //<- thats 16 bit no?
	OCR1B = motorDuty[3];
	OCR0  = motorDuty[1];
	OCR2  = motorDuty[0]; //<- thats 16 bit no?
}

/*
// old calculations, doesn't work so great
void calculate_balance(const u08 Thrust, const s08 Yaw, const s08 Pitch, const s08 Roll)
{
static float lastErr[3] ={0,0,0}, intErr[3]= {0,0,0}; // pid variables
static float g_bias[3] ={0,0,0};	// gyro bias
static float angle[3] ={0,0,0};	// gyro integration result


	// read the sensors
	//

	s16 acc[3];
	acc[0] = lis3l_GetAccel(0) - accel_0x;
	acc[1] = lis3l_GetAccel(1) - accel_0y;
	acc[2] = lis3l_GetAccel(2) - accel_0z;

	s16 gyro[3];
	u08 dat[6];
	itg3200_read_register(ITG3200_REG_GYRO_XOUT_H, dat, 6);
	for(u08 i = 0; i < 3; ++i) 
		gyro[i] = (dat[i*2] << 8) | dat[i*2+1];

	gyro[0] -= gyro_0x;
	gyro[1] -= gyro_0y;
	gyro[2] -= gyro_0z;

	// the time past
	u16 tim = 	Timer0Reg0;
	Timer0Reg0 = 0;
	const float dt = ((float)tim) / 14400.f;


	// integrate gyro
	// val -> degrees/s -> degrees / time past
	// (degrees per LSB = 14.375 deg/s from gyro datasheet)
	// (clock ticks per second 14400, CPU_CLK / 256(overflow int) / 2(timer is running in phase correct mode)

	const float scl = dt / 14.375f;
	angle[0] -= (float)gyro[0] * scl;
	angle[1] += (float)gyro[1] * scl;
	angle[2] += (float)gyro[2] * scl;

	float tilt, roll;
	calc_tilt_roll(acc, &tilt, &roll); // from accel

		
	// calculate the gyro bias
	//
	const float bweight = 0.995, inv_bweight = 1.0f - bweight;
	g_bias[0] = g_bias[0]*bweight + (angle[0] - tilt) * inv_bweight;
	g_bias[1] = g_bias[1]*bweight + (angle[1] - roll) * inv_bweight;
//	g_bias[2] = 0; // don't know how to define this yet, no heading lock, just use the zero measure from calibration





	float errorT = -(float)Pitch*Gain + (angle[0] - g_bias[0]); 
	float errorR = -(float)Roll*Gain  - (angle[1] - g_bias[1]); 
	float errorY = (float)Yaw*Gain    - angle[2];

	// generate the correct motor rpms
	//
	u16 a, b, c, d; //rpms
	a = b = c = d = (u16)Thrust*10;

	if(Thrust > 20)
	{
		const float Kd_dt = PID_Kd / dt;
		const float Kd_yaw_dt = PID_Kd_yaw / dt;

		s16 outT = PID(errorT, &lastErr[0], &intErr[0], PID_Kp, PID_Ki, Kd_dt)*10;
		s16 outR = PID(errorR, &lastErr[1], &intErr[1], PID_Kp, PID_Ki, Kd_dt)*10;
		s16 outY = PID(errorY, &lastErr[2], &intErr[2], PID_Kp_yaw, PID_Ki_yaw, Kd_yaw_dt)*10;

		//s16 outY = Yaw*8;
		//rprintf("%d,%d\n", (int)outT, (int)outR);

		// do tilt a,b vs c,d
		a += outT;
		b += outT;
		c -= outT;
		d -= outT;

		// do roll a,c vs b,d
		a += outR;
		b -= outR;
		c -= outR;
		d += outR;

		// do yaw
		a -= outY;
		b += outY;
		c -= outY;
		d += outY;

	}
	else
	{
		lastErr[0] = lastErr[1] = lastErr[2] = 0.0f; 
		intErr[0] = intErr[1] = intErr[2] = 0.0f;
//		angle[0] = angle[1] = angle[2] = 0;
		angle[2] = 0; //reset yaw when on ground

	}
	
	motorDuty[0] = rpm_to_duty(0, a);
	motorDuty[1] = rpm_to_duty(0, b);
	motorDuty[2] = rpm_to_duty(0, c);
	motorDuty[3] = rpm_to_duty(0, d);
	setMotorSpeeds();


//	static u08 foo = 0;
//	if(xbee_readyToSend() && ((foo % 20) == 0))
//		rprintf("$MT,%f, %f, %f*\n", PID_Kp, PID_Ki, PID_Kd);

//	if(xbee_readyToSend())
//		rprintf("$GY,%d,%d,%d*\n", gyro[0], gyro[1], gyro[2]);

//	if(xbee_readyToSend())
//		rprintf("$MT,%d,%d,%d,%d*\n", motorDuty[0], motorDuty[1], motorDuty[2], motorDuty[3]);
//	if(xbee_readyToSend())
//		rprintf("$MT,%d,%d, %d, %d*\n", (s16)(tilt*10), gyro[0], (s16)(roll*10), gyro[1]);


}

*/

inline
float PD(const float Error, const float dError, 
			const float Kp, const float Kd,
			const float maxP, const float maxD)
{
	return clamp(-maxP, maxP, -Error * Kp) + clamp(-maxD, maxD, dError * Kd);
}
inline
float PDcl(const float Error, const float dError, 
			const float Kp, const float Kd,
			const float max)
{
	return clamp(-max, max, (-Error * Kp) + (dError * Kd));
}

inline
float PID(const float Error, const float dError, const float iError,
			const float Kp, const float Kd, const float Ki,
			const float maxP, const float maxD, const float maxI)
{
	return clamp(-maxP, maxP, -Error * Kp) + clamp(-maxD, maxD, dError * Kd) + clamp(-maxI, maxI, -iError * Ki);
}
inline
float PIDcl(const float Error, const float dError, const float iError,
			const float Kp, const float Kd, const float Ki,
			const float max)
{
	return clamp(-max, max, (-Error * Kp) + (dError * Kd) + (-iError * Ki));
}


inline
float signedSqrt(const float val)
{
  if(val < 0)
  	return -sqrt(-val);
  else
  	return sqrt(val);
}

/*
// accel (P) and gyro (D) fed directly to the controller
// this works well for hovering but starts to oscillate once the errors get too big
// during large acceleration periods
void calculate_balance(const u08 Thrust, const s08 Yaw, const s08 Pitch, const s08 Roll)
{

	// read the sensors
	//
	s16 acc[3];
	s16 gyro[3];

	u08 dat[6];
	while(!itg3200_dataReady());
	itg3200_read_register(ITG3200_REG_GYRO_XOUT_H, dat, 6);
	for(u08 i = 0; i < 3; ++i) 
		gyro[i] = (dat[i*2] << 8) | dat[i*2+1];

	// todo: pull all 3 regs in one go
	acc[0] = -(lis3l_GetAccel(0) - accel_0x);
	acc[1] = (lis3l_GetAccel(1) - accel_0y);
	acc[2] = lis3l_GetAccel(2);


	// convert sensor data to degrees(/s)
	float tilt, roll, yaw;
	calc_tilt_roll(acc, &tilt, &roll);
	float dTilt, dRoll, dYaw;
	dTilt = (float)gyro[0] / -14.375;
	dRoll = (float)gyro[1] / -14.375;
	dYaw  = (float)gyro[2] / -14.375;

	// calculate the error and response
	int rtilt = tilt;
	int rroll = roll;
	tilt = tilt + Pitch;
	roll = roll + Roll;
	yaw  = -(float)Yaw;
	float resTilt = PD(tilt, dTilt, PID_Kp, PID_Kd, 5000.0f, 5000.0f);  // clamp to sqrt(5000+5000)=100
	float resRoll = PD(roll, dRoll, PID_Kp, PID_Kd, 5000.0f, 5000.0f);
	float resYaw  = PD( yaw, dYaw,  PID_Kp_yaw, PID_Kd_yaw, 5000.0f, 5000.0f);
	
	//response is thrust, thrust = rpm^2
	// convert thrust to rpm
	s08 outT, outR, outY;
	outT = round(signedSqrt(resTilt));  // +-100
	outR = round(signedSqrt(resRoll));
	outY = round(signedSqrt(resYaw));

	// cross mix motor response
	s16 a, b, c, d, thr; //rpms
	thr = ((s16)Thrust * 255)/100;
	a = b = c = d = thr;

	// do tilt a,b vs c,d
	a += outT;
	b += outT;
	c -= outT;
	d -= outT;

	// do roll a,c vs b,d
	a += outR;
	b -= outR;
	c -= outR;
	d += outR;

	// do yaw
	a -= outY;
	b += outY;
	c -= outY;
	d += outY;
	

	u08 bottom = 10;
	if(thr <= bottom)
	{
		motorDuty[0] = thr;
		motorDuty[1] = thr;
		motorDuty[2] = thr;
		motorDuty[3] = thr;
	}
	else
	{
		motorDuty[0] = clamp(bottom, 180, a);
		motorDuty[1] = clamp(bottom, 180, b);
		motorDuty[2] = clamp(bottom, 180, c);
		motorDuty[3] = clamp(bottom, 180, d);
	}
	setMotorSpeeds();
	

	static u08 skip = 0;
	if(skip & 8)
	{
		skip = 0;

		if(xbee_readyToSend()) {

			rprintf("$S,%d,%d,%d,%d*\n", rtilt, rroll, (int)outT, (int)outR);
			uartSendTxBuffer();
		}
	}
	else
		skip++;


}
*/

// from wikipedia  (doesn't quite work for large numbers. to be confirmed)
// could also be an incompatibility with the floating point format implemented for avr
float sqrt_approx(float z)
{
    union
    {
        s32 tmp;
        float f;
    } u;
 
    u.f = z;
 
    u.tmp -= (s32)1 << 23; 	/* Subtract 2^m. */
    u.tmp >>= 1; 			/* Divide by 2. */
    u.tmp += (s32)1 << 29; 	/* Add ((b + 1) / 2) * 2^m. */
 
    return u.f;
}


// gyro and accel , linear mixed
// the idea being that the gyro does most of the work but the accelerometer
// pulls/corrects the gyro data slowly towards the absolute 
// works well too, but is prone to stable oscillation
void calculate_balance(const u08 Thrust, const s08 Yaw, const s08 Pitch, const s08 Roll)
{
static float accu_tilt = 0;
static float accu_roll = 0;
static float gyro_weight = 1;
static float iTilt = 0, iRoll = 0;

	// read the sensors
	//
	s16 acc[3];
	s16 gyro[3];

	u08 dat[6];
	while(!itg3200_dataReady());
	itg3200_read_register(ITG3200_REG_GYRO_XOUT_H, dat, 6);
	for(u08 i = 0; i < 3; ++i) 
		gyro[i] = (dat[i*2] << 8) | dat[i*2+1];

	// todo: pull all 3 regs in one go
	acc[0] = -(lis3l_GetAccel(0) - accel_0x);
	acc[1] = (lis3l_GetAccel(1) - accel_0y);
	acc[2] = lis3l_GetAccel(2);

	gyro[0] -= gyro_0x;
	gyro[1] -= gyro_0y;
	gyro[2] -= gyro_0z;

	// convert sensor data to degrees(/s)
	float dTilt, dRoll, dYaw;
	dTilt = (float)gyro[0] / -14.375;
	dRoll = (float)gyro[1] / -14.375;
	dYaw  = (float)gyro[2] / -14.375;

	// integrate the gyro input
	accu_tilt -= dTilt * CTRL_DT; // this loop runs at 100Hz
	accu_roll -= dRoll * CTRL_DT;

	// calculate tilt/roll from accel
	float tilt, roll, yaw;
	calc_tilt_roll(acc, &tilt, &roll);

	gyro_weight = 0.01;
	float invgw = 1.0f - gyro_weight;
	accu_tilt = accu_tilt * invgw + tilt * gyro_weight;
	accu_roll = accu_roll * invgw + roll * gyro_weight;


	// calculate the error and response
	int rtilt = tilt;
	int rroll = roll;
	tilt = accu_tilt + Pitch * Gain;
	roll = accu_roll + Roll * Gain;
	yaw  = -(float)Yaw;

	//integrate the error
	iTilt += tilt * CTRL_DT;
	iRoll += roll * CTRL_DT;

//	float resTilt = PID(tilt, dTilt, iTilt, PID_Kp, PID_Kd, PID_Ki, 4000.0f, 3000.0f, 3000.0f);  // clamp to sqrt(4000+3000+3000)=100
//	float resRoll = PID(roll, dRoll, iRoll, PID_Kp, PID_Kd, PID_Ki, 4000.0f, 3000.0f, 3000.0f);
	float resTilt = PIDcl(tilt, dTilt, iTilt, PID_Kp, PID_Kd, PID_Ki, 2500.0f);  // clamp to sqrt(2500)=50
	float resRoll = PIDcl(roll, dRoll, iRoll, PID_Kp, PID_Kd, PID_Ki, 2500.0f);

//	float resTilt = PD(tilt, dTilt, PID_Kp, PID_Kd, 5000.0f, 5000.0f);  // clamp to sqrt(2500)=50
//	float resRoll = PD(roll, dRoll, PID_Kp, PID_Kd, 5000.0f, 5000.0f);
//	float resYaw  = PD( yaw, dYaw,  PID_Kp_yaw, PID_Kd_yaw, 5000.0f, 5000.0f);

//	float resTilt = PDcl(tilt, dTilt, PID_Kp, PID_Kd, 2500.0f);  // clamp to sqrt(2500)=50
//	float resRoll = PDcl(roll, dRoll, PID_Kp, PID_Kd, 2500.0f);
	float resYaw  = PDcl( yaw, dYaw,  PID_Kp_yaw, PID_Kd_yaw, 2500.0f);

	//response is thrust, thrust = rpm^2
	// convert thrust to rpm
	s08 outT, outR, outY;
	outT = round(signedSqrt(resTilt));  // +-50
	outR = round(signedSqrt(resRoll));
	outY = round(signedSqrt(resYaw));

	// cross mix motor response
	s16 a, b, c, d, thr; //rpms
	thr = ((s16)Thrust * 255)/100;
	a = b = c = d = thr;

	// do tilt a,b vs c,d
	a += outT;
	b += outT;
	c -= outT;
	d -= outT;

	// do roll a,c vs b,d
	a += outR;
	b -= outR;
	c -= outR;
	d += outR;

	// do yaw
	a -= outY;
	b += outY;
	c -= outY;
	d += outY;


	u08 bottom = 10;
	if(thr <= bottom) // switch off stabilizer when on the ground
	{
		motorDuty[0] = thr;
		motorDuty[1] = thr;
		motorDuty[2] = thr;
		motorDuty[3] = thr;

		// reset integrator
		iTilt = iRoll = 0;
	}
	else
	{
		motorDuty[0] = clamp(bottom, 255, a);
		motorDuty[1] = clamp(bottom, 255, b);
		motorDuty[2] = clamp(bottom, 255, c);
		motorDuty[3] = clamp(bottom, 255, d);
	}
	setMotorSpeeds();




	static u08 skip = 0;
	if(skip++ & 16)
	{
		skip = 0;

		if(xbee_readyToSend()) {

			rprintf("$S,%d,%d,%d,%d*\n", (int)(resTilt), (int)(resRoll), (int)accu_tilt, (int)accu_roll);
			uartSendTxBuffer();
		}
	}


}







// gyro only
// best flight characteristics but without auto leveling
/*
void calculate_balance(const u08 Thrust, const s08 Yaw, const s08 Pitch, const s08 Roll)
{

	// read the sensors
	//
	s16 gyro[3];

	u08 dat[6];
	while(!itg3200_dataReady());
	itg3200_read_register(ITG3200_REG_GYRO_XOUT_H, dat, 6);
	for(u08 i = 0; i < 3; ++i) 
		gyro[i] = (dat[i*2] << 8) | dat[i*2+1];


	gyro[0] -= gyro_0x;
	gyro[1] -= gyro_0y;
	gyro[2] -= gyro_0z;

	float dTilt, dRoll, dYaw;
	dTilt = (float)gyro[0] / -14.375;
	dRoll = (float)gyro[1] / -14.375;
	dYaw  = (float)gyro[2] / -14.375;



	// calculate the error and response
	float tilt, roll, yaw;
	tilt = Pitch * Gain;
	roll = Roll * Gain;
	yaw  = -(float)Yaw;
	float resTilt = PD(tilt, dTilt, PID_Kp, PID_Kd, 5000.0f, 5000.0f);  // clamp to sqrt(5000+5000)=100
	float resRoll = PD(roll, dRoll, PID_Kp, PID_Kd, 5000.0f, 5000.0f);
	float resYaw  = PD( yaw, dYaw,  PID_Kp_yaw, PID_Kd_yaw, 5000.0f, 5000.0f);
	
	//response in thrust, thrust = some constant * rpm^2
	// convert thrust to rpm (pwm)
	s08 outT, outR, outY;
	outT = round(signedSqrt(resTilt));  // +-100
	outR = round(signedSqrt(resRoll));
	outY = round(signedSqrt(resYaw));

	// cross mix motor response
	s16 a, b, c, d, thr; //rpms
	thr = ((s16)Thrust * 255)/100;
	a = b = c = d = thr;

	// do tilt a,b vs c,d
	a += outT;
	b += outT;
	c -= outT;
	d -= outT;

	// do roll a,c vs b,d
	a += outR;
	b -= outR;
	c -= outR;
	d += outR;

	// do yaw
	a -= outY;
	b += outY;
	c -= outY;
	d += outY;
	

	u08 bottom = 10;
	if(thr <= bottom) // shut of the imu on the ground
	{
		motorDuty[0] = thr;
		motorDuty[1] = thr;
		motorDuty[2] = thr;
		motorDuty[3] = thr;
	}
	else
	{
		motorDuty[0] = clamp(bottom, 180, a);
		motorDuty[1] = clamp(bottom, 180, b);
		motorDuty[2] = clamp(bottom, 180, c);
		motorDuty[3] = clamp(bottom, 180, d);
	}
	setMotorSpeeds();
	

	static u08 skip = 0;
	if(skip & 16)
	{
		skip = 0;

		if(xbee_readyToSend()) {

			rprintf("$S,%d,%d*\n", (int)dTilt, (int)dRoll);
			uartSendTxBuffer();
		}
	}
	else
		skip++;


}
*/

//Timer0 overflow interrupt
SIGNAL(SIG_OVERFLOW0)
{
static u16 globalTime = 0;

	globalTime++;
	return;
/*
	sei(); // reenable interrupts
	// avoid recursion of another irq hit
	if(!accu_sampling && itg3200_dataReady())
	{
		accu_sampling = TRUE;
		PORTC |= (1<<2);
		accu_dt += globalTime;
		globalTime = 0;

		// read gyro
		s16 gyro[3];
		u08 dat[6];
		itg3200_read_register(ITG3200_REG_GYRO_XOUT_H, dat, 6);
		for(u08 i = 0; i < 3; ++i) 
			gyro[i] = (dat[i*2] << 8) | dat[i*2+1];

		gyro[0] -= gyro_0x;
		gyro[1] -= gyro_0y;
		gyro[2] -= gyro_0z;

		// integrate the rotation, 1st order for now
		// dt is constant, it's the sample rate of the gyro, so we don't have to divide

		accu_gyro[0] += gyro[0];
		accu_gyro[1] += gyro[1];
		accu_gyro[2] += gyro[2];
		accu_test++;

		PORTC &= ~(1<<2);
		accu_sampling = FALSE;	
	}
*/

}

