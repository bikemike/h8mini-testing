// HARDWARE PINS SETTING
//
// do not change hardware pins below
// make sure you don't set SWDIO or SWDCLK pins (programming pins)
// if you do, you lose board programmability without a reset pin
//
// example: pin "PB2" ( port b , pin 2 )
// pin: GPIO_PIN_2
// port: GPIOB


// setting procedure:
// set led number zero, led aux number zero
// uncomment DISABLE_SOFTI2C_PINS , DISABLE_SPI_PINS and DISABLE_PWM_PINS
// this will prevent any pins to be used, so that there are no conflicts
// set pins starting with leds, in order to see working status
// do not set PA13 , PA14 (stm32f031) as this will break the programming interface
// to disable led pins set number to zero


//#define IMU_6XXX //(mpu-6050 etc)
#define IMU_BMI055
#define ADDRESS_6XXX       0x68
#define BMI055_ACC_ADDRESS 0x19  // 0x18 or 0x19
#define BMI055_GYR_ADDRESS 0x69  // 0x68 or 0x69


// always on pin ( for vreg if present)
// used by cx-10 boards and other quads with switches
// comment out to disable
//#define ENABLE_VREG_PIN


#define VREG_PIN_1       GPIO_PIN_1
#define VREG_PORT_1      GPIOA


#define FPV_PIN          GPIO_PIN_14 // SWCLK
#define FPV_PIN_PORT     GPIOA


//#define BUZZER_PIN       GPIO_PIN_13 // SWDAT
#define BUZZER_PIN       GPIO_PIN_14 // SWCLK
#define BUZZER_PIN_PORT  GPIOA
#define BUZZER_DELAY     5e6 // 5 seconds after loss of tx or low bat before buzzer starts


//*** DO NOT ENABLE ESC DRIVER WITH BRUSHED MOTORS CONNECTED ***
// output driver type , esc settings in drv_esc.c file

#define USE_PWM_DRIVER
//#define USE_ESC_DRIVER
//#define USE_DSHOT_DRIVER

// settings in file drv_servo.c
//#define SERVO_DRIVER

// enable serial out on back-left LED
//#define SERIAL

// set zero to disable (0 - 4)
#define LED_NUMBER       4

#define LED1PIN          GPIO_PIN_10
#define LED1PORT         GPIOA

#define LED2PIN          GPIO_PIN_0
#define LED2PORT         GPIOA

#define LED3PIN          GPIO_PIN_4
#define LED3PORT         GPIOB

#define LED4PIN          GPIO_PIN_15
#define LED4PORT         GPIOA

// aux leds
// set zero to disable (0 - 2)
#define AUX_LED_NUMBER   0

#define AUX_LED1PIN      GPIO_PIN_2
#define AUX_LED1PORT     GPIOB

#define AUX_LED2PIN      GPIO_PIN_x
#define AUX_LED2PORT     GPIOx

// invert - leds turn on when high
//#define LED1_INVERT
//#define LED2_INVERT
//#define LED3_INVERT
//#define LED4_INVERT
//#define AUX_LED1_INVERT
//#define AUX_LED2_INVERT

// i2c driver to use ( dummy - disables i2c )
// hardware i2c used PB6 and 7 by default ( can also use PA9 and 10)

#define USE_HARDWARE_I2C
//#define USE_SOFTWARE_I2C
//#define USE_DUMMY_I2C

// I2C speed: fast = no delays 
// slow1 = for i2c without pull-up resistors
// slow2 = i2c failsafe speed
#define SOFTI2C_SPEED_FAST
//#define SOFTI2C_SPEED_SLOW1
//#define SOFTI2C_SPEED_SLOW2

// hardware i2c speed ( 1000, 400 , 200 , 100Khz)
#define HW_I2C_SPEED_FAST2
//#define HW_I2C_SPEED_FAST
//#define HW_I2C_SPEED_SLOW1
//#define HW_I2C_SPEED_SLOW2

// pins for hw i2c , select one only
// select pins PB6 and PB7 OR select pins PA9 and PA10
#define HW_I2C_PINS_PB67
//#define HW_I2C_PINS_PA910

// softi2c pins definitons
#define SOFTI2C_SDAPIN   GPIO_PIN_7
#define SOFTI2C_SDAPORT  GPIOB

#define SOFTI2C_SCLPIN   GPIO_PIN_6
#define SOFTI2C_SCLPORT  GPIOB

#define SOFTI2C_GYRO_ADDRESS 0x68
//#define SOFTI2C_GYRO_ADDRESS 0x69

// disable the check for known gyro that causes the 4 times flash
//#define DISABLE_GYRO_CHECK

// gyro ids for the gyro check
#define GYRO_ID_1 0x68
#define GYRO_ID_2 0x78
#define GYRO_ID_3 0x7D
#define GYRO_ID_4 0x72

// disable lvc functions
//#define DISABLE_LVC

// Analog battery input pin and adc channel

#define BATTERYPIN       GPIO_PIN_8
#define BATTERYPORT      GPIOA
#define BATTERY_ADC_CHANNEL ADC_Channel_8

// divider setting for adc uses 2 measurements
// the adc readout can be found in debug mode , debug.adcfilt
// #enable DEBUG should be in config.h
// default for 1/2 divider
#define ADC_BATT_VOLTAGE 3.77
#define ADC_READOUT 2727.0


// SPI PINS DEFINITONS ( for radio ic )
// MOSI , CLK , SS - outputs , MISO input

//disable pins so they don't interfere with other pins 
//#define DISABLE_SPI_PINS

#define SPI_MOSI_PIN     GPIO_PIN_7
#define SPI_MOSI_PORT    GPIOA

#define SPI_MISO_PIN     GPIO_PIN_6
#define SPI_MISO_PORT    GPIOA

#define SPI_CLK_PIN      GPIO_PIN_5
#define SPI_CLK_PORT     GPIOA

#define SPI_SS_PIN       GPIO_PIN_4
#define SPI_SS_PORT      GPIOA

#define RADIO_CE_PIN       GPIO_PIN_1
#define RADIO_CE_PORT      GPIOF

// check for radio chip ( 3 times flash = not found)
#define RADIO_CHECK

// radio type
#define RADIO_SSV7241
//#define RADIO_XN297
//#define RADIO_XN297L

// PWM PINS DEFINITIONS 
// currently pins PA0 to PA3 , PA5 , PA8 to PA11 supported

// pwm driver = brushed motors
// esc driver = servo type signal for brushless esc

#define USE_PWM_DRIVER
//#define USE_ESC_DRIVER

// pwm pins disable
// disable all pwm pins / function
//#define DISABLE_PWM_PINS

// pwm pin initialization
// enable the pwm pins to be used here ( multiple pins ok)
#define PWM_PA0
#define PWM_PA1
//#define PWM_PA2
//#define PWM_PA3
//#define PWM_PA4
//#define PWM_PA5
//#define PWM_PA6
//#define PWM_PA7
#define PWM_PA8
#define PWM_PA9
//#define PWM_PA10
//#define PWM_PA11
//#define PWM_PB0
//#define PWM_PB1


// Assingment of pin to motor
// Assign one pin to one motor

// back-left motor
// motor 0 pin

#define MOTOR0_PIN_PA0
//#define MOTOR0_PIN_PA1
//#define MOTOR0_PIN_PA2
//#define MOTOR0_PIN_PA3
//#define MOTOR0_PIN_PA4
//#define MOTOR0_PIN_PA5
//#define MOTOR0_PIN_PA6
//#define MOTOR0_PIN_PA7
//#define MOTOR0_PIN_PA8
//#define MOTOR0_PIN_PA9
//#define MOTOR0_PIN_PA10
//#define MOTOR0_PIN_PA11
//#define MOTOR0_PIN_PB0
//#define MOTOR0_PIN_PB1

// front-left motor
// motor 1 pin

//#define MOTOR1_PIN_PA0
#define MOTOR1_PIN_PA1
//#define MOTOR1_PIN_PA2
//#define MOTOR1_PIN_PA3
//#define MOTOR1_PIN_PA4
//#define MOTOR1_PIN_PA5
//#define MOTOR1_PIN_PA6
//#define MOTOR1_PIN_PA7
//#define MOTOR1_PIN_PA8
//#define MOTOR1_PIN_PA9
//#define MOTOR1_PIN_PA10
//#define MOTOR1_PIN_PA11
//#define MOTOR1_PIN_PB0
//#define MOTOR1_PIN_PB1

// back-right motor
// motor 2 pin

//#define MOTOR2_PIN_PA0
//#define MOTOR2_PIN_PA1
//#define MOTOR2_PIN_PA2
//#define MOTOR2_PIN_PA3
//#define MOTOR2_PIN_PA4
//#define MOTOR2_PIN_PA5
//#define MOTOR2_PIN_PA6
//#define MOTOR2_PIN_PA7
#define MOTOR2_PIN_PA8
//#define MOTOR2_PIN_PA9
//#define MOTOR2_PIN_PA10
//#define MOTOR2_PIN_PA11
//#define MOTOR2_PIN_PB0
//#define MOTOR2_PIN_PB1

// front-right motor
// motor 3 pin

//#define MOTOR3_PIN_PA0
//#define MOTOR3_PIN_PA1
//#define MOTOR3_PIN_PA2
//#define MOTOR3_PIN_PA3
//#define MOTOR3_PIN_PA4
//#define MOTOR3_PIN_PA5
//#define MOTOR3_PIN_PA6
//#define MOTOR3_PIN_PA7
//#define MOTOR3_PIN_PA8
#define MOTOR3_PIN_PA9
//#define MOTOR3_PIN_PA10
//#define MOTOR3_PIN_PA11
//#define MOTOR3_PIN_PB0
//#define MOTOR3_PIN_PB1


// gyro orientation
// the expected orientation is with the dot in the front-left corner
// use this to rotate to the correct orientation 
// rotations performed in order
//#define SENSOR_ROTATE_45_CCW
//#define SENSOR_ROTATE_90_CW
//#define SENSOR_ROTATE_90_CCW
//#define SENSOR_ROTATE_180
//#define SENSOR_FLIP_180


// RGB led type ws2812 - ws2813
// numbers over 8 could decrease performance
#define RGB_LED_NUMBER 0

// pin / port for the RGB led ( programming port ok )
#define RGB_PIN GPIO_PIN_13 // SWDAT
#define RGB_PORT GPIOA



