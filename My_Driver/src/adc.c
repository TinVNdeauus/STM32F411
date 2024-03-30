/*
 * adc.c
 *
 *  Created on: Mar 30, 2024
 *      Author: Acer
 */
#include <stdint.h>
#define RCC_BASE_ADDR    0x40023800
#define ADC1_BASE_ADDR   0x40012000

void adc_init()
{
    //for external GPIO
	// enable clock for GPIOA
	//set GPIOA_MODDER for PA0  in analog mode by 11


	//for internal ADC
	// enable clock ADC1
	uint32_t* RCC_APB2ENR = (uint32_t*)(RCC_BASE_ADDR + 0x44);
	*RCC_APB2ENR |= 1 << 8;

	uint32_t* ADC_SMPR1 = (uint32_t*)(ADC1_BASE_ADDR + 0x0C);
	*ADC_SMPR1 |= 0b111 << 18;        // Channel 16 sampling time selection as 16 cycles

	uint32_t* ADC_JSQR = (uint32_t*)(ADC1_BASE_ADDR + 0x38);
	*ADC_JSQR |= 16 << 15;           //select channel 16(temp sensor) for JSQR4

	uint32_t* ADC_CCR = (uint32_t*)(ADC1_BASE_ADDR + 0x300 + 0x04);
	*ADC_CCR |= 1<<23;               // enable temperature sensor

	uint32_t* ADC_CR2 = (uint32_t*)(ADC1_BASE_ADDR + 0x08);
	*ADC_CR2 |= 1 << 0;              // enable ADC
}

float get_temp()
{
	uint32_t* ADC_CR2 = (uint32_t*)(ADC1_BASE_ADDR + 0x08);
	uint32_t* ADC_SR = (uint32_t*)(ADC1_BASE_ADDR + 0x00);
	uint32_t* ADC_JDR1 = (uint32_t*)(ADC1_BASE_ADDR + 0x3C);
	*ADC_CR2 |= 1<<22;        // start conversion
	while(((*ADC_SR >> 2) & 1)==0)     // wait JEOC (Injected End Of Conversion) flag is set
	*ADC_SR &= ~(1<<2);         // clear JEOC
	uint32_t adc_raw_data = *ADC_JDR1 & 0x0fff;
	float VSENSE = (adc_raw_data * 3.0) / 4095.0;
	const float V25 = 0.76;           //Vol
	const float Avg_Slope = 0.0025;   //Vol/oC
	float temp = ((VSENSE - V25) / Avg_Slope) + 25;
	return temp;

}
