/*
===============================================================================
 Name        : Funktionsgenerator.h
 Author      : Andreas Gebel & Le Quan Tran
 Version     : 1.2
 Description : 
===============================================================================
*/

#ifndef FUNKTIONSGENERATOR_H_
#define FUNKTIONSGENERATOR_H_



#define RC5 15					//Port des GPIO für das lesen des RC5
#define RC5_AUS_TASTE 0xc
#define RC5_PLAY 0x32
#define RC5_RUECKWAERTS 0x37
#define RC5_VORWAERTS 0x34

// Allgemeine Defines
#define FREQ 1
#define AMPL 2
#define BLAU 1
#define ROT 2
#define GRUEN 3


// Globale Variblen

static volatile uint16_t value_adc = 0; 	//Wert des Potis (noch aendern und entfernen)
static volatile uint16_t val = 1;			//Gibt den Index der Arrays an
static volatile uint8_t function = 0;		//Fuer den Funktionsangabe und wechsel
static volatile uint32_t new_code = 0;		//RC5 Code aus dem Timer0
static volatile uint32_t flip;				//Flip-Flag des RC5 Codes
static volatile uint32_t rc5_option;		//RC5 Code fuer die Einstellung der Funktionsausgabe
static volatile uint8_t scale = 0;			//Fuer die Skalierung der Funktionsausgabe
static volatile uint8_t new_cmd;


// Prototypen


void rgb_led_set(uint8_t);
void red_leds_set(uint16_t);
void LCD_write_form(char*);
void LCD_write_freq_ampl(uint8_t, uint16_t);
void DOUT_write(void);
void AD_write(uint16_t value);
void button_read(void);
void AD_read(void);
void TIMER0_IRQHandler(void);
void RIT_IRQHandler(void);
void frequency_get(void);
void frequency_set(uint16_t);

#endif /* FUNKTIONSGENERATOR_H_ */
