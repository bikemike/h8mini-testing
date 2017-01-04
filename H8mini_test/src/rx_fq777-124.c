/*
The MIT License (MIT)

Copyright (c) 2016 bikemike

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

#include "config.h"


#ifdef RX_FQ777_124

#include "hardware.h"
#include "binary.h"
#include "drv_spi.h"

#include "gd32f1x0.h"
#include "xn297.h"
#include "drv_time.h"
#include <stdio.h>
#include "defines.h"

#include "rx_bayang.h"

#include "util.h"

#define _BV(bit) (1 << (bit))


// radio settings

// packet period in uS
#define PACKET_PERIOD 2000
#define PAYLOAD_LENGTH 8

// because it's from the cg023 port
#define RADIO_XN297


#define TX_POWER 3

// global variables
float rx[4];
char aux[AUXNUMBER];
char lastaux[AUXNUMBER];
char auxchange[AUXNUMBER];


// rx code variables
char lasttrim[4];
char rfchannel[4] = {0x4D, 0x43, 0x27, 0x07};
//char rfchannel[4] = {0x4D, 0x43, 0x4D, 0x43};
int rxaddress[5] = { 0xe7, 0xe7, 0xe7, 0xe7,0x67 };
int rxmode = 0;
int rf_chan = 0;
// received a packet 0x20 0x15 0x05 0x06 0xd3 0x45 0x00 0x18
// 55 67 70 77* 80 83*
   //37 43 46 4d  50 53
// 77, 67, 39 7


enum {
    // flags going to packet[6]
    // H7_FLAG_RATE0, // default rate, no flag
    FQ777124_FLAG_RETURN     = 0x40,  // 0x40 when not off, !0x40 when one key return
    FQ777124_FLAG_HEADLESS   = 0x04,
    FQ777124_FLAG_EXPERT     = 0x01,
    FQ777124_FLAG_FLIP       = 0x80,
};


int ssv7241_reg_init[] = 
{
	0x1F,0x1,0x00,
	0x1B,0x4,0x10,0xE1,0xD3,0x3D,
	0x19,0x4,0x06,0xAA,0xA2,0xDB,
	0x1A,0x4,0x27,0x61,0x01,0xF8,
	0x1F,0x1,0x01,
	0x18,0x4,0xBF,0x94,0x00,0xDF,
	0x19,0x4,0x77,0x48,0x9A,0xE8,
	0x1B,0x4,0x76,0x87,0xCA,0x01,
	0x1F,0x1,0x02,
	0x1B,0x4,0xA0,0x00,0x18,0xA0,
	0x1F,0x1,0x04,
	0x18,0x4,0x01,0x00,0xF0,0x00,
	0x1F,0x1,0x05,
	0x18,0x4,0x84,0x03,0x2A,0x03,
	0x19,0x4,0x90,0xBF,0x00,0x00,
	0x1A,0x4,0xA0,0x0F,0x00,0x00,
	0x1F,0x1,0x00,
};


void writeregs(uint8_t data[], uint8_t size)
{
	spi_cson();
	for (uint8_t i = 0; i < size; i++)
	{
		spi_sendbyte(data[i]);
	}
	spi_csoff();
	delay(1000);
}

void ssv_init_reg()
{
	int size = sizeof(ssv7241_reg_init)/sizeof(ssv7241_reg_init[0]);
	for (int i = 0; i < size; )
	{
		int reg = ssv7241_reg_init[i++];
		int sz  = ssv7241_reg_init[i++];
		if (1 == sz)
		{
			xn_writereg(reg, ssv7241_reg_init[i]);
		}
		else
		{
			xn_writereg_multi(reg, ssv7241_reg_init+i, sz);
		}
		i += sz;
	}
}


void rx_init()
{
	// always on (CH_ON) channel set 1
	aux[AUXNUMBER - 2] = 1;
	// always off (CH_OFF) channel set 0
	aux[AUXNUMBER - 1] = 0;
#ifdef AUX1_START_ON
	aux[CH_AUX1] = 1;
#endif

#ifdef AUX4_START_ON
	aux[CH_AUX4] = 1;
#endif

	ssv_init_reg();

	xn_writereg(CONFIG,_BV(PRIM_RX) | _BV(PWR_UP) | _BV(EN_CRC) | _BV(CRCO));

	xn_writerxaddress(rxaddress);

	xn_writereg(EN_AA, 0);  // aa disabled
	xn_writereg(EN_RXADDR, 1);  // pipe 0 only
	xn_writereg(RF_CH, 0x4D);  // bind on channel 4D
	xn_writereg(FEATURE, 0x04); // enable dynamic payload
	xn_writereg(DYNPD, 0x01);  // enable dynamic payload on p0
	xn_writereg(RF_SETUP, 0x20); // bitrate 256k, power 00(-18dBm)
	xn_writereg(RX_PW_P0, PAYLOAD_LENGTH);  // payload size
	xn_writereg(SETUP_RETR, 0); // no retransmissions ( redundant?)
	xn_writereg(SETUP_AW, 3);   // address size (5 bits)
	xn_command(FLUSH_RX);
	xn_writereg( STATUS , 0x70);



#ifdef RADIO_CHECK
	int rxcheck = xn_readreg(0x0f); // rx address pipe 5   
	// should be 0xc6
	extern void failloop(int);
	if (rxcheck != 0xc6)
		failloop(3);
#endif
	// set CE pin high
	GPIO_SetBits( RADIO_CE_PORT, RADIO_CE_PIN);
}



void nextchannel(void);

int loopcounter = 0;
int oldchan = 0;

static char checkpacket()
{
	int status = xn_command(NOP);

	if (status & _BV(MASK_RX_DR))
	{
		//RX packet received
		return 1;
	}

	return 0;
}


int rxdata[PAYLOAD_LENGTH];


static float packettodata(int *data)
{
	// 50 center, 0-100
	// convert to percent (-1 to 1)
	return ((*data) - 50)*.02f;
}


static int decodepacket(void)
{
	uint8_t sum = 0;
	for (int i = 0; i < PAYLOAD_LENGTH-1; i++)
	{
		sum += rxdata[i];
	}
	if (sum == rxdata[PAYLOAD_LENGTH - 1])
	{
		// rxdata 0-3 = TREA
		// rx 0-3 = AERT
		rx[0] = packettodata(&rxdata[3]);
		rx[1] = packettodata(&rxdata[2]);
		rx[2] = packettodata(&rxdata[1]);
		// throttle
		rx[3] = rxdata[0]*.01f;

		aux[CH_RTH] = rxdata[5] & FQ777124_FLAG_RETURN; 
		aux[CH_HEADFREE] = rxdata[5] & FQ777124_FLAG_HEADLESS; 
		aux[CH_EXPERT] = rxdata[5] & FQ777124_FLAG_EXPERT; 
		aux[CH_FLIP] = rxdata[5] & FQ777124_FLAG_FLIP; 
		// rxdata[4] - trim 0x20 +- 1F (yaw), 60(roll), A0(pitch)
		/* rxdata[5] flags
		 */

		for (int i = 0; i < AUXNUMBER - 2; i++)
		{
			auxchange[i] = 0;
			if (lastaux[i] != aux[i])
				auxchange[i] = 1;
			lastaux[i] = aux[i];
		}

		return 1;   // valid packet 
	}
	return 0; // sum fail
}



void nextchannel()
{
	// lower CE (must be done to change channels properly with the ssv7241)
	GPIO_ResetBits( RADIO_CE_PORT, RADIO_CE_PIN);

	rf_chan++;
	rf_chan &= 3;
	xn_writereg(RF_CH, rfchannel[rf_chan]);
	xn_command(FLUSH_RX);
	xn_writereg( STATUS , 0x70);

	// raise CE
	GPIO_SetBits( RADIO_CE_PORT, RADIO_CE_PIN);
}


unsigned long failsafetime;

int failsafe = 0;



void checkrx(void)
{
	int packetreceived = checkpacket();
	int pass = 0;
	if (packetreceived)
	{
		if (rxmode == RX_MODE_BIND)
		{   // rx startup , bind mode

			xn_readpayload(rxdata, PAYLOAD_LENGTH);
			uint8_t checksum = rxdata[4] + rxdata[5] + rxdata[6];
			if (checksum == rxdata[7])
			{

				rxaddress[0] = rxdata[4];
				rxaddress[1] = rxdata[5];
				rxaddress[2] = rxdata[6];

				xn_writerxaddress(rxaddress);
				xn_writereg(RF_CH, rfchannel[rf_chan]);    // Set channel frequency 
				xn_command(FLUSH_RX);
				xn_writereg( STATUS , 0x70);

				rxmode = RX_MODE_NORMAL;

#ifdef DEBUG
				printf(" BIND \n");
#endif
			}
		}
		else
		{   // normal mode  

			unsigned long temptime = gettime();

			xn_readpayload(rxdata, PAYLOAD_LENGTH);

			pass = decodepacket();

			if (pass)
			{
				failsafetime = temptime;
				failsafe = 0;
			}

			nextchannel();
		}   // end normal rx mode

	} // end packet received


	unsigned long time = gettime();


	if (time - failsafetime > FAILSAFETIME)
	{
		//  failsafe
		failsafe = 1;
		rx[0] = 0;
		rx[1] = 0;
		rx[2] = 0;
		rx[3] = 0;
	}

}


#endif

