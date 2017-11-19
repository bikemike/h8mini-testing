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


#include "hardware.h"

#ifdef IMU_BMI055


#include "config.h"
#include "binary.h"
#include "sixaxis.h"
#include "drv_time.h"
#include "util.h"
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
//#define SENSOR_ROTATE_180
//#define SENSOR_FLIP_180

static const uint8_t bmi055_acc_range = BMI055_PMU_RANGE_16G;
static int8_t        bmi055_acc_scale = 1; // dynamically set based on acc range

extern void loadcal(void);


extern unsigned int liberror;

void sixaxis_init(void)
{
	// gyro soft reset.
	i2c_writereg(BMI055_GYR_ADDRESS, BMI055_BGW_SOFTRESET, BMI055_BGW_SOFTRESET_RESET);

	// ACC reset.
	i2c_writereg(BMI055_ACC_ADDRESS, BMI055_BGW_SOFTRESET, BMI055_BGW_SOFTRESET_RESET);

	liberror = 0; // resetting causes liberror so reset the error counter

	delay(40000); // Is this delay required for BMI055 ?
	i2c_writereg(BMI055_ACC_ADDRESS, BMI055_PMU_RANGE, bmi055_acc_range); // 16G scale
	i2c_writereg(BMI055_ACC_ADDRESS, BMI055_PMU_BW, ACC_LOW_PASS_FILTER_BMI055); // Filter

	i2c_writereg(BMI055_GYR_ADDRESS, BMI055_RANGE,BMI055_RANGE_2000DPS);
	i2c_writereg(BMI055_GYR_ADDRESS, BMI055_BW, GYRO_LOW_PASS_FILTER_BMI055);

	if (bmi055_acc_range == BMI055_PMU_RANGE_16G)
	{
		bmi055_acc_scale = 1;
	}
	else if (bmi055_acc_range == BMI055_PMU_RANGE_8G)
	{
		bmi055_acc_scale = 2;
	}
	else if (bmi055_acc_range == BMI055_PMU_RANGE_4G)
	{
		bmi055_acc_scale = 4;
	}
	else if (bmi055_acc_range == BMI055_PMU_RANGE_2G)
	{
		bmi055_acc_scale = 8;
	}
}

int sixaxis_check(void)
{
#ifndef DISABLE_GYRO_CHECK
	uint8_t accel;
	uint8_t gyro;

	accel = i2c_readreg(BMI055_ACC_ADDRESS, BMI055_BGW_CHIPID);
	gyro = i2c_readreg(BMI055_GYR_ADDRESS, BMI055_CHIP_ID);
	if(( gyro == BMI055_CHIP_ID_VAL) && ( accel == BMI055_BGW_CHIPID_VAL))
	{
		// It is a BMI055 !
		return 1;
	}
	// Not a BMI055
	return 0;
#else
	return 1;
#endif
}


float accel[3];
float gyro[3];

float accelcal[3];
float gyrocal[3];


float lpffilter(float in, int num);

void sixaxis_read(sixaxis_readtype read_type)
{
	int data[12];

	float gyronew[3];

	if (SIXAXIS_GYRO_ONLY != read_type)
	{
		i2c_readdata(BMI055_ACC_ADDRESS, BMI055_ACCD_X_LSB, data, 6);

		accel[1] = -(float)((int16_t) (((uint16_t)data[1] << 8) + ((uint16_t)data[0]&0xF0))/bmi055_acc_scale);
		accel[0] =  (float)((int16_t) (((uint16_t)data[3] << 8) + ((uint16_t)data[2]&0xF0))/bmi055_acc_scale);
		accel[2] =  (float)((int16_t) (((uint16_t)data[5] << 8) + ((uint16_t)data[4]&0xF0))/bmi055_acc_scale);

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

	}

	i2c_readdata(BMI055_GYR_ADDRESS,BMI055_RATE_X_LSB, data+6, 6);


	gyronew[0] =  (int16_t) ((data[7] << 8) + data[6]);
	gyronew[1] = -(int16_t) ((data[9] << 8) + data[8]);
	gyronew[2] =  (int16_t) ((data[11] << 8) + data[10]);

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

	gyronew[1] = -gyronew[1];
	gyronew[2] = -gyronew[2];

	for (int i = 0; i < 3; i++)
	{
		// Full range is 32767 to -32767
		//gyronew[i] = gyronew[i] * 0.06103701895199438459425641651662f * 0.017453292f;
		gyronew[i] = gyronew[i] * .00106529691457869198f;
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

		i2c_readdata(BMI055_GYR_ADDRESS,BMI055_RATE_X_LSB, data, 6);

		gyro[0] =  (int16_t) ((data[1] << 8) + data[0]);
		gyro[1] = -(int16_t) ((data[3] << 8) + data[2]);
		gyro[2] =  (int16_t) ((data[5] << 8) + data[4]);


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
}

void acc_cal(void)
{
	accelcal[2] = 2048;
	for (int y = 0; y < 500; y++)
	  {
		  sixaxis_read(SIXAXIS_GYRO_AND_ACCEL);
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



#endif



