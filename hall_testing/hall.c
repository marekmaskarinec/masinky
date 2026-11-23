/* Small example showing how to use the SWIO programming pin to
   do printf through the debug interface */

#include "ch32fun.h"
#include "ch32v20xhw.h"
#include <stdio.h>

uint32_t count;

static volatile int g_awd_count = 0;

__attribute__((interrupt)) __attribute__((section(".srodata"))) void
ADC1_2_IRQHandler(void)
{
	if (ADC1->STATR & ADC_AWD) {
		ADC1->STATR &= ~ADC_AWD;
		g_awd_count++;
	}
}

int last = 0;
void
handle_debug_input(int numbytes, uint8_t *data)
{
	last = data[0];
	count += numbytes;
}

static void
adc_init(void)
{
	// Divide clock by 16, should be safe for now.
	RCC->CFGR0 |= 0b11 << 14;

	// Enable ADC
	RCC->APB2PCENR |= RCC_APB2Periph_ADC1;

	funPinMode(PA2, GPIO_CFGLR_IN_ANALOG);

	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

	// Set up one conversion channel
	ADC1->RSQR1 = ADC_L_0;
	// Set up channel 2 as 1st conversion channel
	ADC1->RSQR3 = 2;

	// Set sampling time for channel 2 to 239 cycles
	ADC1->SAMPTR2 = ADC_SampleTime_239Cycles5 << (2 * 3);

	ADC1->WDLTR = 1350;
	ADC1->WDHTR = 1450;
	ADC1->CTLR1 |= ADC_AWDEN;
	ADC1->CTLR1 |= ADC_AWDIE;
	NVIC_EnableIRQ(ADC_IRQn);
	// ADC1->CTLR1 |= ADC_AWDSGL;
	// ADC1->CTLR1 |= ADC_AWDCH_2;

	// turn on ADC and set rule group to sw trig
	ADC1->CTLR2 |= ADC_ADON;

	printf("reset calibration\n");
	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while (ADC1->CTLR2 & ADC_RSTCAL)
		;

	printf("calibrate\n");
	// Calibrate
	ADC1->CTLR2 |= ADC_CAL;
	while (ADC1->CTLR2 & ADC_CAL)
		;
}

/*
 * start conversion, wait and return result
 */
uint16_t
adc_get(void)
{
	// start sw conversion (auto clears)
	// ADC1->CTLR2 |= ADC_SWSTART;
	ADC1->CTLR2 |= ADC_ADON;

	// wait for conversion complete
	while (!(ADC1->STATR & ADC_EOC))
		;

	// get result
	return ADC1->RDATAR;
}

// Regular value: 1400
// Train value: 1700

int
main()
{
	SystemInit();

	Delay_Ms(200);
	printf("--------------\n");

	// Enable GPIOs
	funGpioInitAll();
	adc_init();

	while (1) {
		// uint16_t val = adc_get();
		printf("%d, AWD=%d, g_awd_count=%d\n", 0, !!(ADC1->STATR & ADC_AWD), g_awd_count);
		Delay_Ms(10);
	}
}
