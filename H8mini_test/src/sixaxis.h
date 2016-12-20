
typedef enum 
{
	SIXAXIS_GYRO_ONLY,
	SIXAXIS_GYRO_AND_ACCEL
} sixaxis_readtype;

void sixaxis_init( void);
int sixaxis_check( void);
void sixaxis_read(sixaxis_readtype read_type);
void gyro_cal( void);
void acc_cal(void);



