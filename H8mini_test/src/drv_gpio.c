#include "gd32f1x0.h"
#include "drv_gpio.h"
#include "config.h"
#include "hardware.h"

void gpio_init(void)
{
// clocks on to all ports               
	RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_GPIOA | RCC_AHBPERIPH_GPIOB | RCC_AHBPERIPH_GPIOF, ENABLE);

	GPIO_InitPara GPIO_InitStructure;

// common settings to set ports
	GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
	GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_NOPULL;	// GPIO_PUPD_NOPULL


#ifdef ENABLE_VREG_PIN	
	GPIO_InitStructure.GPIO_Pin = VREG_PIN_1;	
	GPIO_Init(VREG_PORT_1, &GPIO_InitStructure); 
	GPIO_SetBits( VREG_PORT_1, VREG_PIN_1);
#endif


#if (LED_NUMBER > 0)
	GPIO_InitStructure.GPIO_Pin = LED1PIN;
	GPIO_Init(LED1PORT, &GPIO_InitStructure);
#if (LED_NUMBER > 1)
	GPIO_InitStructure.GPIO_Pin = LED2PIN;
	GPIO_Init(LED2PORT, &GPIO_InitStructure);
#if (LED_NUMBER > 2)
	GPIO_InitStructure.GPIO_Pin = LED3PIN;
	GPIO_Init(LED3PORT, &GPIO_InitStructure);
#if (LED_NUMBER > 3)
	GPIO_InitStructure.GPIO_Pin = LED4PIN;
	GPIO_Init(LED4PORT, &GPIO_InitStructure);
#endif
#endif
#endif
#endif

#ifdef RADIO_CE_PIN
	GPIO_InitStructure.GPIO_Pin = RADIO_CE_PIN;
	GPIO_Init(RADIO_CE_PORT, &GPIO_InitStructure);
#endif


#if ( AUX_LED_NUMBER > 0 )
  GPIO_InitStructure.GPIO_Pin = AUX_LED1PIN;	
  GPIO_Init(AUX_LED1PORT, &GPIO_InitStructure); 
#endif	
#if ( AUX_LED_NUMBER > 1 )
  GPIO_InitStructure.GPIO_Pin = AUX_LED2PIN;	
  GPIO_Init(AUX_LED2PORT, &GPIO_InitStructure); 
#endif	

}


#ifdef FPV_ON
// init fpv pin separately because it may use SWDAT/SWCLK don't want to enable it right away
int gpio_init_fpv(void)
{
	// only repurpose the pin after rx/tx have bound
	extern int rxmode;
	if (rxmode == RX_MODE_NORMAL)
	{
		// set gpio pin as output
		GPIO_InitPara GPIO_InitStructure;

		// common settings to set ports
		GPIO_InitStructure.GPIO_Pin =  FPV_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
		GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_NOPULL;

		GPIO_Init(FPV_PIN_PORT,&GPIO_InitStructure);
		return 1;
	}
	return 0;
}
#endif
