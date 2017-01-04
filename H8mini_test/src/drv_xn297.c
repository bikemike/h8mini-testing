
#include "binary.h"
#include "drv_spi.h"

#include "gd32f1x0.h"
#include "config.h"
#include "xn297.h"

#define gpioset( port , pin) port->BOR = (0x0001 << pin)
#define gpioreset( port , pin) port->BCR = (0x0001 << pin)

#define SPIOFF gpioset( GPIOB, 5)

#ifdef XN297_EMULATION
///////////////
// XN297 emulation layer
enum {
	XN297_UNSCRAMBLED = 0,
	XN297_SCRAMBLED
};
#define _BV(bit) (1 << (bit))

uint8_t xn297_scramble_enabled=XN297_SCRAMBLED;	//enabled by default
uint8_t xn297_addr_len;
uint8_t xn297_tx_addr[5];
uint8_t xn297_rx_addr[5];
uint8_t xn297_crc = 0;

static const uint8_t xn297_scramble[] = {
	0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
	0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
	0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc,
	0x1b, 0x5d, 0x19, 0x10, 0x24, 0xd3, 0xdc, 0x3f,
	0x8e, 0xc5, 0x2f};

const uint16_t xn297_crc_xorout_scrambled[] = {
	0x0000, 0x3448, 0x9BA7, 0x8BBB, 0x85E1, 0x3E8C,
	0x451E, 0x18E6, 0x6B24, 0xE7AB, 0x3828, 0x814B,
	0xD461, 0xF494, 0x2503, 0x691D, 0xFE8B, 0x9BA7,
	0x8B17, 0x2920, 0x8B5F, 0x61B1, 0xD391, 0x7401,
	0x2138, 0x129F, 0xB3A0, 0x2988};

const uint16_t xn297_crc_xorout[] = {
	0x0000, 0x3d5f, 0xa6f1, 0x3a23, 0xaa16, 0x1caf,
	0x62b2, 0xe0eb, 0x0821, 0xbe07, 0x5f1a, 0xaf15,
	0x4f0a, 0xad24, 0x5e48, 0xed34, 0x068c, 0xf2c9,
	0x1852, 0xdf36, 0x129d, 0xb17c, 0xd5f5, 0x70d7,
	0xb798, 0x5133, 0x67db, 0xd94e};

static uint8_t bit_reverse(uint8_t b_in)
{
    uint8_t b_out = 0;
    for (uint8_t i = 0; i < 8; ++i)
	{
        b_out = (b_out << 1) | (b_in & 1);
        b_in >>= 1;
    }
    return b_out;
}

static const uint16_t polynomial = 0x1021;
static uint16_t crc16_update(uint16_t crc, uint8_t a)
{
	crc ^= a << 8;
    for (uint8_t i = 0; i < 8; ++i)
        if (crc & 0x8000)
            crc = (crc << 1) ^ polynomial;
		else
            crc = crc << 1;
    return crc;
}
#endif

void xn_writereg(int reg, int val)
{
	reg = reg & 0x0000003F;
	reg = reg | 0x00000020;
	spi_cson();
	spi_sendbyte(reg);
	spi_sendbyte(val);
	spi_csoff();
}
void xn_writereg_multi(int reg, int* bytes, int size)
{
	reg = reg & 0x0000003F;
	reg = reg | 0x00000020;
	spi_cson();
	spi_sendbyte(reg);
	for (int i = 0; i < size; ++i)
		spi_sendbyte(bytes[i]);
	spi_csoff();
}

int xn_readreg(int reg)
{
	reg = reg & 0x1F;
	spi_cson();
	spi_sendrecvbyte(reg);
	reg = spi_sendrecvbyte(0);
	spi_csoff();
	return reg;
}

int xn_command(int command)
{
	spi_cson();
	int status = spi_sendrecvbyte(command);
	spi_csoff();
	return status;
}


void _spi_write_address(int reg, int val)
{
	spi_cson();
	spi_sendbyte(reg);
	spi_sendbyte(val);
	spi_csoff();
}


void xn_configure(int flags)
{
#ifdef XN297_EMULATION
	xn297_crc = !!((uint8_t)flags & _BV(EN_CRC));
	flags &= ~(_BV(EN_CRC) | _BV(CRCO));
#endif
	xn_writereg(CONFIG, flags & 0xFF);
}


/*
void xn_readpayload2( int *data , int size )
{
//	int index = 0;
	spi_cson();
	spi_sendbyte( B01100001 ); // read rx payload
	while(size!=0)
	{
	data++[0]=	spi_sendzerorecvbyte(  );
	//data++;
	size--;
	}
	spi_csoff();
}
*/

void xn_readpayload(int *data, int size)
{
	int index = 0;
	spi_cson();
	spi_sendbyte(B01100001);	// read rx payload
	while (index < size)
	  {
		  data[index] = spi_sendzerorecvbyte();
		  index++;
	  }
	spi_csoff();
#ifdef XN297_EMULATION
for(uint8_t i=0; i<size; i++)
	{
		uint8_t d = (uint8_t)data[i];
		if(xn297_scramble_enabled)
			d ^= xn297_scramble[i+xn297_addr_len];
		d = bit_reverse(d);
		data[i] =  d;
	}
#endif
}


void xn_writerxaddress(int *addr)
{
	int len = 5;
#ifdef XN297_EMULATION
	if (len > 5) len = 5;
	if (len < 3) len = 3;
	uint8_t buf[] = { 0, 0, 0, 0, 0 };
	int buf2[] = { 0, 0, 0, 0, 0 };
	xn297_addr_len = len;
	for (int i = 0; i < len; ++i)
	{
		xn297_rx_addr[i] = (uint8_t)addr[i];
	}
	for (uint8_t i = 0; i < xn297_addr_len; ++i)
	{
		buf[i] = xn297_rx_addr[i];
		if(xn297_scramble_enabled)
			buf[i] ^= xn297_scramble[xn297_addr_len-i-1];
	}
	for (int i = 0; i < len; ++i)
		buf2[i] = buf[i];
#else
	int* buf2 = addr;	
#endif
	xn_writereg(SETUP_AW, len-2);
	xn_writereg_multi(RX_ADDR_P0, buf2, 5);
}

void xn_writetxaddress(  int *addr)	
{
	int len = 5;
#ifdef XN297_EMULATION
	if (len > 5) len = 5;
	if (len < 3) len = 3;
	int buf[] = { 0x55, 0x0F, 0x71, 0x0C, 0x00 }; // bytes for XN297 preamble 0xC710F55 (28 bit)
	xn297_addr_len = len;
	if (xn297_addr_len < 4)
		for (uint8_t i = 0; i < 4; ++i)
			buf[i] = buf[i+1];
#else
	int* buf = addr;
#endif
	xn_writereg(SETUP_AW, len-2);
	xn_writereg_multi(TX_ADDR, buf, 5);
#ifdef XN297_EMULATION
	for (int i = 0; i < len; ++i)
		xn297_tx_addr[i] = (uint8_t)addr[i];
#endif
}


void xn_writepayload( int msg[] , int size )
{
	int index = 0;
#ifdef XN297_EMULATION
	uint8_t buf[64];
	uint8_t last = 0;
	int data[64];

	if (xn297_addr_len < 4)
	{
		// If address length (which is defined by receive address length)
		// is less than 4 the TX address can't fit the preamble, so the last
		// byte goes here
		buf[last++] = 0x55;
	}
	for (uint8_t i = 0; i < xn297_addr_len; ++i)
	{
		buf[last] = xn297_tx_addr[xn297_addr_len-i-1];
		if(xn297_scramble_enabled)
			buf[last] ^=  xn297_scramble[i];
		last++;
	}
	for (uint8_t i = 0; i < size; ++i)
	{
		// bit-reverse bytes in packet
		uint8_t b_out = bit_reverse((uint8_t)msg[i]);
		buf[last] = b_out;
		if(xn297_scramble_enabled)
			buf[last] ^= xn297_scramble[xn297_addr_len+i];
		last++;
	}
	if (xn297_crc)
	{
		uint8_t offset = xn297_addr_len < 4 ? 1 : 0;
		uint16_t crc = 0xb5d2;
		for (uint8_t i = offset; i < last; ++i)
			crc = crc16_update(crc, buf[i]);
		if(xn297_scramble_enabled)
			crc ^= xn297_crc_xorout_scrambled[xn297_addr_len - 3 + size];
		else
			crc ^= xn297_crc_xorout[xn297_addr_len - 3 + size];
		buf[last++] = crc >> 8;
		buf[last++] = crc & 0xff;
	}
	for (int i = 0; i < last; ++i)
		data[i] = buf[i];
	size = last;
#else
	int* data = msg;
#endif
	spi_cson();
	spi_sendrecvbyte( 0xA0 ); // write tx payload
	while(index<size)
	{
	spi_sendrecvbyte( data[index] );
	index++;
	}
	spi_csoff();
}

