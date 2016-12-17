/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/



#include "binary.h"
#include "sixaxis.h"
#include "drv_time.h"
#include "util.h"
#include "config.h"
#include "led.h"
#include "drv_serial.h"

#include "drv_i2c.h"

#include "bmi055.h"

#include <math.h>
#include <stdio.h>
#include <inttypes.h>

// Useful function pointer types.
typedef void (*func_void_void_t)(void);

// gyro orientation
// the expected orientation is with the gyro dot in the front-left corner
// use this to rotate to the correct orientation 
// rotations performed in order
//#define SENSOR_ROTATE_45_CCW
//#define SENSOR_ROTATE_90_CW
//#define SENSOR_ROTATE_90_CCW
#define SENSOR_ROTATE_180
//#define SENSOR_FLIP_180


// temporary fix for compatibility between versions
#ifndef GYRO_ID_1 
#define GYRO_ID_1 0x68 
#endif
#ifndef GYRO_ID_2
#define GYRO_ID_2 0x78
#endif
#ifndef GYRO_ID_3
#define GYRO_ID_3 0x7D
#endif
#ifndef GYRO_ID_4
#define GYRO_ID_4 0x72
#endif


extern void loadcal(void);


static void sixaxis_init_6xxx(void);
static void sixaxis_read_6xxx(void);
static void gyro_read_6xxx(void);
static void gyro_cal_6xxx(void);

static void sixaxis_init_bmi055(void);
static void sixaxis_read_bmi055(void);
static void gyro_read_bmi055(void);
static void gyro_cal_bmi055(void);

typedef struct {
	func_void_void_t sixaxis_init;
	func_void_void_t sixaxis_read;
	func_void_void_t gyro_read;
	func_void_void_t gyro_cal;
} sixaxis_t;

sixaxis_t sixaxis_funcs =
  {
	  sixaxis_init_6xxx,
	  sixaxis_read_6xxx,
		gyro_read_6xxx,
		gyro_cal_6xxx
	};

void sixaxis_init(void)
{
	sixaxis_funcs.sixaxis_init();
}



void sixaxis_init_6xxx( void)
{
// gyro soft reset
	
	
	i2c_writereg(ADDRESS_6XXX,  107 , 128);
	 
 delay(40000);
	

// set pll to 1, clear sleep bit old type gyro (mpu-6050)	
	i2c_writereg(ADDRESS_6XXX,  107 , 1);
	
	int newboard = !(0x68 == i2c_readreg(ADDRESS_6XXX, 117) );
	
	i2c_writereg(ADDRESS_6XXX,  28, B00011000);	// 16G scale

// acc lpf for the new gyro type
//       0-6 ( same as gyro)
	if (newboard) i2c_writereg(ADDRESS_6XXX, 29, ACC_LOW_PASS_FILTER);
	
// gyro scale 2000 deg (FS =3)

	i2c_writereg(ADDRESS_6XXX, 27 , 24);
	
// Gyro DLPF low pass filter

	i2c_writereg(ADDRESS_6XXX, 26 , GYRO_LOW_PASS_FILTER);
}

static void sixaxis_init_bmi055(void)
{
  // gyro soft reset.
  i2c_writereg(BMI055_GYR_ADDRESS, BMI055_BGW_SOFTRESET, BMI055_BGW_SOFTRESET_RESET);
	
	// ACC reset.
  i2c_writereg(BMI055_ACC_ADDRESS, BMI055_BGW_SOFTRESET, BMI055_BGW_SOFTRESET_RESET);
	
	delay(40000); // Is this delay required for BMI055 ?

	i2c_writereg(BMI055_ACC_ADDRESS, BMI055_PMU_RANGE, BMI055_PMU_RANGE_16G); // 16G scale
	i2c_writereg(BMI055_ACC_ADDRESS, BMI055_PMU_BW, ACC_LOW_PASS_FILTER_BMI055); // Filter
	
	i2c_writereg(BMI055_GYR_ADDRESS, BMI055_RANGE,BMI055_RANGE_2000DPS);
	i2c_writereg(BMI055_GYR_ADDRESS, BMI055_BW, GYRO_LOW_PASS_FILTER_BMI055);
}

int sixaxis_check(void)
{
#ifndef DISABLE_GYRO_CHECK
	// read "who am I" register
	// new board returns 78h (unknown gyro maybe mpu-6500 compatible) marked m681
	// old board returns 68h (mpu - 6050)
	// a new (rare) gyro marked m540 returns 7Dh
	int id = i2c_readreg(ADDRESS_6XXX, 117);

	if (!(GYRO_ID_1==id||GYRO_ID_2==id||GYRO_ID_3==id||GYRO_ID_4==id))
	{
		// It was not a known 6xxx could it be a BMI055 ?
		if((i2c_readreg(BMI055_GYR_ADDRESS, BMI055_CHIP_ID) == BMI055_CHIP_ID_VAL) && (i2c_readreg(BMI055_ACC_ADDRESS, BMI055_BGW_CHIPID) == BMI055_BGW_CHIPID_VAL))
		{
			// It is a BMI055 !
			sixaxis_funcs.sixaxis_init = sixaxis_init_bmi055;
			sixaxis_funcs.sixaxis_read = sixaxis_read_bmi055;
			sixaxis_funcs.gyro_read = gyro_read_bmi055;
			sixaxis_funcs.gyro_cal = gyro_cal_bmi055;
			return 1;
		}
		// No idea what chip it is.
		return 0;
	}

  // It was a 6xxx.
  return 1;
#else
  return 1;
#endif
}


float accel[3];
float gyro[3];

float accelcal[3];
float gyrocal[3];


float lpffilter(float in, int num);

void sixaxis_read(void)
{
	sixaxis_funcs.sixaxis_read();
}

void sixaxis_read_6xxx(void)
{
	int data[16];


	float gyronew[3];

	
	i2c_readdata(ADDRESS_6XXX, 59 , data , 14 );
		
	accel[0] = -(int16_t) ((data[0] << 8) + data[1]);
	accel[1] = -(int16_t) ((data[2] << 8) + data[3]);
	accel[2] = (int16_t) ((data[4] << 8) + data[5]);


// this is the value of both cos 45 and sin 45 = 1/sqrt(2)
#define INVSQRT2 0.707106781f
		
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = accel[0];
		accel[0] = (accel[0] * INVSQRT2 + accel[1] * INVSQRT2);
		accel[1] = -(temp * INVSQRT2 - accel[1] * INVSQRT2);	
		}
#endif
	
#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = accel[1];
		accel[1] = accel[0];
		accel[0] = -temp;	
		}
#endif
		
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = accel[1];
		accel[1] = -accel[0];
		accel[0] = temp;	
		}
#endif
				
#ifdef SENSOR_ROTATE_180
		{
		accel[1] = -accel[1];
		accel[0] = -accel[0];	
		}
#endif		
		
#ifdef SENSOR_FLIP_180
		{
		accel[2] = -accel[2];
		accel[0] = -accel[0];	
		}
#endif	
//order
	gyronew[1] = (int16_t) ((data[8] << 8) + data[9]);
	gyronew[0] = (int16_t) ((data[10] << 8) + data[11]);
	gyronew[2] = (int16_t) ((data[12] << 8) + data[13]);


gyronew[0] = gyronew[0] - gyrocal[0];
gyronew[1] = gyronew[1] - gyrocal[1];
gyronew[2] = gyronew[2] - gyrocal[2];
	
	
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0] * INVSQRT2 + gyronew[1] * INVSQRT2;
		gyronew[0] = gyronew[0] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif	
		
#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = gyronew[1];
		gyronew[1] = -gyronew[0];
		gyronew[0] = temp;	
		}
#endif
		
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0];
		gyronew[0] = -temp;	
		}
#endif
		
					
#ifdef SENSOR_ROTATE_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[0] = -gyronew[0];	
		}
#endif		
		
#ifdef SENSOR_FLIP_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[2] = -gyronew[2];	
		}
#endif	

//gyronew[0] = - gyronew[0];
gyronew[1] = - gyronew[1];
gyronew[2] = - gyronew[2];

	for (int i = 0; i < 3; i++)
	  {


		  gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;
#ifndef SOFT_LPF_NONE
		  gyro[i] = lpffilter(gyronew[i], i);
#else
		  gyro[i] = gyronew[i];
#endif
	  }




}

static void sixaxis_read_bmi055(void)
{
	int data[6];

	int error = 0;

	float gyronew[3];

	error = (i2c_readdata(BMI055_ACC_ADDRESS, BMI055_ACCD_X_LSB, data, 6));
// 2nd attempt at an i2c read   
	if (error)
	  {
		  error = (i2c_readdata(BMI055_ACC_ADDRESS, BMI055_ACCD_X_LSB, data, 6));

	  }
		
	accel[0] = (int16_t) ((data[1] << 8) + data[0]);
	accel[1] = (int16_t) ((data[3] << 8) + data[2]);
	accel[2] = (int16_t) ((data[5] << 8) + data[4]);
		
	error = (i2c_readdata(BMI055_GYR_ADDRESS, BMI055_RATE_X_LSB, data, 6));
// 2nd attempt at an i2c read   
	if (error)
	  {
		  error = (i2c_readdata(BMI055_GYR_ADDRESS,BMI055_RATE_X_LSB, data, 6));

	  }


	gyronew[1] = (int16_t) ((data[1] << 8) + data[0]);
	gyronew[0] = (int16_t) ((data[2] << 8) + data[2]);
	gyronew[2] = (int16_t) ((data[3] << 8) + data[4]);

	for (int i = 0; i < 3; i++)
	  {

		  gyronew[i] = gyronew[i] - gyrocal[i];
// Full range is 32767 to -32767
		  gyronew[i] = gyronew[i] * 0.06103701895199438459425641651662f * 0.017453292f;
			
#ifndef SOFT_LPF_NONE
		  gyro[i] = lpffilter(gyronew[i], i);
#else
		  gyro[i] = gyronew[i];
#endif
	  }


	gyro[0] = -gyro[0];
	gyro[2] = -gyro[2];


}




void gyro_read_6xxx( void)
{
int data[6];
	
	i2c_readdata(ADDRESS_6XXX, 67 , data , 6 );
	
float gyronew[3];
	// order
gyronew[1] = (int16_t) ((data[0]<<8) + data[1]);
gyronew[0] = (int16_t) ((data[2]<<8) + data[3]);
gyronew[2] = (int16_t) ((data[4]<<8) + data[5]);

		
gyronew[0] = gyronew[0] - gyrocal[0];
gyronew[1] = gyronew[1] - gyrocal[1];
gyronew[2] = gyronew[2] - gyrocal[2];
	
	
		
#ifdef SENSOR_ROTATE_45_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0] * INVSQRT2 + gyronew[1] * INVSQRT2;
		gyronew[0] = gyronew[0] * INVSQRT2 - temp * INVSQRT2;	
		}
#endif
			
#ifdef SENSOR_ROTATE_90_CW
		{
		float temp = gyronew[1];
		gyronew[1] = -gyronew[0];
		gyronew[0] = temp;	
		}
#endif

				
#ifdef SENSOR_ROTATE_90_CCW
		{
		float temp = gyronew[1];
		gyronew[1] = gyronew[0];
		gyronew[0] = -temp;	
		}
#endif
	
					
#ifdef SENSOR_ROTATE_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[0] = -gyronew[0];	
		}
#endif		
		
							
#ifdef SENSOR_FLIP_180
		{
		gyronew[1] = -gyronew[1];
		gyronew[2] = -gyronew[2];	
		}
#endif		
	


		
//gyronew[0] = - gyronew[0];
gyronew[1] = - gyronew[1];
gyronew[2] = - gyronew[2];
	
	
for (int i = 0; i < 3; i++)
	  {
		  gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;
#ifndef SOFT_LPF_NONE
		  gyro[i] = lpffilter(gyronew[i], i);
#else
		  gyro[i] = gyronew[i];
#endif
	  }

}

static void gyro_read_bmi055(void)
{
	int data[6];

	i2c_readdata(BMI055_GYR_ADDRESS, BMI055_RATE_X_LSB, data, 6);

	float gyronew[3];

	gyronew[1] = (int16_t) ((data[1] << 8) + data[0]);
	gyronew[0] = (int16_t) ((data[3] << 8) + data[2]);
	gyronew[2] = (int16_t) ((data[5] << 8) + data[4]);


	gyronew[0] = gyronew[0] - gyrocal[0];
	gyronew[1] = gyronew[1] - gyrocal[1];
	gyronew[2] = gyronew[2] - gyrocal[2];

	gyronew[0] = -gyronew[0];
	gyronew[2] = -gyronew[2];


	for (int i = 0; i < 3; i++)
	  {
		  gyronew[i] = gyronew[i] * 0.061035156f * 0.017453292f;
#ifndef SOFT_LPF_NONE
		  gyro[i] = lpffilter(gyronew[i], i);
#else
		  gyro[i] = gyronew[i];
#endif
	  }

}
 


#define CAL_TIME 2e6

void gyro_cal(void)
{
	sixaxis_funcs.gyro_cal();
}

void gyro_cal_6xxx(void)
{
int data[6];
float limit[3];	
unsigned long time = gettime();
unsigned long timestart = time;
unsigned long timemax = time;
unsigned long lastlooptime = time;

float gyro[3];	
	
 for ( int i = 0 ; i < 3 ; i++)
			{
			limit[i] = gyrocal[i];
			}

// 2 and 15 seconds
while ( time - timestart < CAL_TIME  &&  time - timemax < 15e6 )
	{	
		
		unsigned long looptime; 
		looptime = time - lastlooptime;
		lastlooptime = time;
		if ( looptime == 0 ) looptime = 1;

	i2c_readdata(ADDRESS_6XXX,  67 , data , 6 );	

			
	gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
	gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
	gyro[2] = (int16_t) ((data[4]<<8) + data[5]);
		

	
#ifdef OLD_LED_FLASH
		  if ((time - timestart) % 200000 > 100000)
		    {
			    ledon(B00000101);
			    ledoff(B00001010);
		    }
		  else
		    {
			    ledon(B00001010);
			    ledoff(B00000101);
		    }
#else
static int ledlevel = 0;
static int loopcount = 0;

loopcount++;
if ( loopcount>>5 )
{
	loopcount = 0;
	ledlevel = ledlevel + 1;
	ledlevel &=15;
}

if ( ledlevel > (loopcount&0xF) ) 
{
	ledon( 255);
}
else 
{
	ledoff( 255);
}
#endif



		 for ( int i = 0 ; i < 3 ; i++)
			{

					if ( gyro[i] > limit[i] )  limit[i] += 0.1f; // 100 gyro bias / second change
					if ( gyro[i] < limit[i] )  limit[i] -= 0.1f;
				
					limitf( &limit[i] , 800);
				
					if ( fabsf(gyro[i]) > 100+ fabsf(limit[i]) ) 
					{										
						timestart = gettime();
							#ifndef OLD_LED_FLASH
							ledlevel = 1;
							#endif
					}
					else
					{						
					lpf( &gyrocal[i] , gyro[i], lpfcalc( (float) looptime , 0.5 * 1e6) );
				
					}

			}

while ( (gettime() - time) < 1000 ) delay(10); 				
time = gettime();
		}
	
	
if (time - timestart > 15e6 - 5000)
	  {
		  for (int i = 0; i < 3; i++)
		    {
			    gyrocal[i] = 0;

		    }

		  loadcal();
	  }

		


	
#ifdef SERIAL_INFO	
printf("gyro calibration  %f %f %f \n "   , gyrocal[0] , gyrocal[1] , gyrocal[2]);
#endif
	
}


static void gyro_cal_bmi055(void)
{
	int data[6];

	unsigned long time = gettime();
	unsigned long timestart = time;
	unsigned long timemax = time;
	unsigned long lastlooptime = time;

	float gyro[3];
	float limit[3];

	for (int i = 0; i < 3; i++)
	  {
		  limit[i] = gyrocal[i];
	  }

// 2 and 15 seconds
	while (time - timestart < CAL_TIME && time - timemax < 15e6)
	  {

		  unsigned long looptime;
		  looptime = time - lastlooptime;
		  lastlooptime = time;
		  if (looptime == 0)
			  looptime = 1;

		  i2c_readdata(BMI055_GYR_ADDRESS, 67, data, 6);

		  gyro[0] = (int16_t) ((data[3] << 8) + data[2]);
		  gyro[1] = (int16_t) ((data[1] << 8) + data[0]);
		  gyro[2] = (int16_t) ((data[5] << 8) + data[4]);


		  if ((time - timestart) % 200000 > 100000)
		    {
			    ledon(B00000101);
			    ledoff(B00001010);
		    }
		  else
		    {
			    ledon(B00001010);
			    ledoff(B00000101);
		    }

		  for (int i = 0; i < 3; i++)
		    {

			    if (gyro[i] > limit[i])
				    limit[i] += 0.1f;	// 100 gyro bias / second change
			    if (gyro[i] < limit[i])
				    limit[i] -= 0.1f;

			    limitf(&limit[i], 800);

			    if (fabs(gyro[i]) > 100 + fabs(limit[i]))
			      {
				      timestart = gettime();
			      }
			    else
			      {
				      lpf(&gyrocal[i], gyro[i], lpfcalc((float)looptime, 0.5 * 1e6));

			      }

		    }

		  while ((gettime() - time) < 1000)
			  delay(10);
		  time = gettime();

	  }



	if (time - timestart < CAL_TIME)
	  {
		  for (int i = 0; i < 3; i++)
		    {
			    gyrocal[i] = 0;

		    }

		  loadcal();
	  }


#ifdef SERIAL
	printf("gyro calibration  %f %f %f \n ", gyrocal[0], gyrocal[1], gyrocal[2]);
#endif

}

void acc_cal(void)
{
	accelcal[2] = 2048;
	for (int y = 0; y < 500; y++)
	  {
		  sixaxis_read();
			delay(800);
		  for (int x = 0; x < 3; x++)
		    {
			    lpf(&accelcal[x], accel[x], 0.92);
		    }
		  gettime();	// if it takes too long time might overflow so we call it here

	  }
	accelcal[2] -= 2048;


	for (int x = 0; x < 3; x++)
	  {
		  limitf(&accelcal[x], 500);
	  }

}







