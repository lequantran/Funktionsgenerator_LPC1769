/*
===============================================================================
 Name        : Init.h
 Author      : Andreas Gebel % Le Quan Tran
 Version     : 10.12.2018
===============================================================================
*/

#ifndef INIT_H
#define INIT_H

//GPIO Defines

#define RGB_PINSEL ~(0x3f << 14)	//Pin P2[7]-P2[9] als GPIO Ports einstellen
#define RGB_LED (7 << 7)			//RBG-LED Ports als Ausgänge
#define TASTER_PINSEL ~(0x3f << 6)	//Pin P2[3]-P2[5] als GPIO Ports einstellen
#define A0_PINSEL ~(0x3 << 22)		//Pin P1[27] als GPIO Port einstellen
#define A0_OUT (1 << 27)			//Digitaler Ausgang A0 als Ausgang


// AD Defines

#define PCADC 12				// Einschalten im PCONP-Register
#define PDN 21					// Einschalten im ADCR-Register
#define PCLK_ADC 22				// Takt des AD-Wandler
#define ADC0_0 14				// Pin als ADC0.0
#define AD_CLKDIV 8				// Bits des CLKDIV des AD zur Einstellung des Taktes (optional)

//Timer0 defines
#define RC5_PINMODE	30				//Port des RC5
#define PCTIM0 1			//Bit zum Einschalten des Timer0
#define PCLK_TIMER0 2		//Bit zur Takteinstellung

//DA Defines

#define PCLK_DAC 22			//Takt des DA-Wandler
#define AOUT 20				//Pin als AOUT einstellen

#define MAX7311 0x20

//RIT defines
#define PCRIT 16			//Einschalten des RIT
#define PCLK_RIT 26			//Bit zur Takteinstellung
#define RITENCLR 1			//Reset



void GPIO_init(void);
void LCD_init(void);
void I2C_init(void);
void AD_init(void);
void AOUT_init(void);
void Timer0_init(void);
void RIT_init(void);

#endif /* INIT_H_ */
