/*
===============================================================================
 Name        : Init.c
 Author      : Andreas Gebel % Le Quan Tran
 Version     : 10.12.2018
===============================================================================
*/

#include "LPC17xx.h"
#include "i2c_1769.h"
#include "lcdlib_1769.h"
#include "Init.h"

/**
 *
 *  \file     Init.c
 *
 *  <HR>
 * *  \section sec1 Init.c / Init.h
 *	Diese Datei dient zur Initialisierung aller benötigten GPIO-Ports,
 * 	die verwendeten Interrupts, den LCD-Display, I2C-Portexpander sowie
 * 	den Digita-Analog- sowie Analog-Digital-Wandler.
 *
 *  \subsection sec2_1 GPIOs
 *	<b>GPIO_init():</b>Dieses Unterprogramm dient zur Einstellung und Initialisierung der benötigten GPIO Ports. <br>
 *	Mit Hilfe der PINSEL-Funktion werden die Funktionen der Ports für die drei Taster, die RGB-LEDs
 *	und der Digitale Ausgang A0 als GPIO-Port Funktionen eingestellt.  <br>
 *	Danach werden die Ports für die RBG-LEDs sowie des Digitalen Ausgangs A0 als Ausgang eingestellt,
 *	indem mit FIODIR eine 1 auf das entsprechende Pin-Bit gesetzt wird.
 *
 *  \subsection sec2_2 LCD-Display
 *	<b>LCD_init():</b>Dieses Unterprogramm dient zu Einstellung und Initialisierung des LCD-Display. <br>
 *	Zuerst wird mit <b>lcd_init(4)</b> das LCD als Display mit 4 Zeilen initialisiert.
 *	Daraufhin löschen wir mit <b>lcd_clrscr()</b> das Display und schreiben mit <b>lcd_write_4lines()</b>
 *	unsere Standardausgabe.
 *
 *  \subsection sec2_3 I2C-Portexpander
 *	<b>I2C_init():</b> Dieses Unterprogramm dient zur Einstellung und Initialisierung des I2C-Portexpanders
 *	für die Ausgabe an den 6 roten LEDs.<br>
 *	Mit der Funktion <b>i2c_init(I2C_SM)</b> initialisieren wir den Bus. Der Parameter gibt an, dass wir
 *	den Standard Modus verwenden und somit eine Übertragungsrate von 0,1 Mbit/s haben. Die Übertragung auf den
 *	I2C-Bus wird daraufhin mit <b>i2c_start()</b> gestartet. Als nächstes geben wir an, dass auf den Baustein
 *	MAX7311 geschrieben werden soll und stellen Port 1 und Port 2 des Bausteins als Ausgang ein. Zuletzt wird die
 *	Übertragung mit <b>i2c_stop()</b> gestoppt.
 *
 *  \subsection sec2_4 Analog-Digital-Wandler
 *	<b>AD_init():</b> Dieses Unterprogramm dient zur Einstellung und Initialisierung des Analog-Digital-Wandler. <br>
 *	Beim Reset ist der AD-Wandler ausgestellt. Um diesen einzuschalten setzen wir eine 1 auf das PCADC Bit des PCONP-Registers
 *	und daraufhin eine 1 auf das PDN Bit im AD0CR-Register.<br>
 *	Der Takt wird nun auf auf 1/8 der CCLK eingestellt, indem eine "0b11" auf die Bits PCLK_ADC des PCLKSEL0-Registers geschrieben
 *	wird. Danach wird die Taktrate nochmal mit dem Wert des CLKDIV-Bits im ADCR-Registers geteilt. Im ganzen haben wir eine
 *	Taktrate von (100MHz / 8) / (1+1) = 6,25MHz.<br>
 *	Anschließend wird der Pin P0.23 mit der PINSEL1-Funktion als AD0.0 eingestellt und mit dem PINMODE1-Funkion die
 *	PullUp- und PullDown-Widerstände deaktiviert.
 *
 *  \subsection sec2_5 Digital-Analog-Wandler
 *	<b>AOUT_init():</b> Dieses Unterpogramm dient zur Konfiguration des Digital-Analog-Wandler.<br>
 *	Der Digital-Analog-Wandler ist mit dem VDDA verbunden und muss nicht extra eingeschaltet werden.
 * 	Die Bits PCLK_DAC des PCLKSEL0-Registers werden auf "0b00" gesetzt und somit ein Takt von 100/4 = 25MHz eingestellt.
 *	Der Pin P0.23 wird mit der PINSEL1-Funktion als AOUT eingestellt und mit dem PINMODE1-Funkion die
 *	PullUp- und PullDown-Widerstände deaktiviert.
 *
 *  \subsection sec2_6 Timer0 für die Decodierung des RC5-Codes
 *	<b>Timer0_init():</b> Dieses Unterprogramm dient zur Konfiguration des Interrupts Timer0, der zur Decodierung
 *	des RC5-Signals benötigt wird.<br>
 *	Um den Timer0 einzuschalten wird das PCTIM0 Bit im PCONP-Register auf 1 gesetzt. Danach wird der Takt auf
 * 	100MHz / 8 = 12,5MHz eingestellt, indem die Bits PCLK_TIMER0 im Register PCLKSEL0 auf "0b11" gesetzt werden.<br>
 *	Das Match Register des Timers0 setzen wir auf 3700. Dieser Wert wird die ganze Zeit mit dem Wert im Timer Control verglichen.
 *	Das TC-Register wird dabei bei jeden Takt erhöht bis der Wert 3700 erreicht wird. Somit wurde ein Interrupt mit einem Takt von
 * 	12,5MHz / 3700 = ca 3,378kHz eingestellt.
 *	Die Bits des Timer0 Match Control Register werden auf "0b11" gesetzt. Dies bewirkt, dass wenn der Timer-Counter den Wert des
 *	Match Register 0 erreicht, zurückgesetzt wird und ein Interrupt erzeugt wird.
 *	Im Register Timer Control Register setzen wir eine 1 auf das zweite Bit um den Timer-Counter des Timer0 zu stoppen und
 *	zurück zu setzen. Danach wird mit <b>NVIC_EnableIRQ(TIMER0_IRQn)</b> die Funktion angegeben, die bei einem Interrupt ausgeführt werden soll.
 *	Mit der PINMODE-Funktion werden die Widerstände des Port-Eingangs des Infrafrots-Sensor deaktiviert.
 *	Zuletzt wird im Timer Control Register eine 1 gesetzt, um den Timer zu starten.
 *
 *  \subsection sec2_7 Repetitive Interrupt Timer
 *	<b>RIT_init():</b> Dieses Unterprogramm dient zur Konfiguration des RITS, der zur Einstellung der Frequenz und Ausgabe
 *	des Funktionssignals benutzt wird. <br>
 *	Um den RIT einzuschalten wird der PCRIT-Bit im PCONP-Register auf 1 gesetzt. Anschließend wird der Takt auf 100MHz eingstellt.
 *	Dafür werden die Bits PCLK_RIT im PCLKSEL1 zuerst auf 0 gesetzt und danach auf "0b01".
 *	Im Register RICTRL setzen wir das Bit RITENCLR auf 1. Damit wird der Timer zurück auf 0 gesetzt, wenn dieser den Wert im
 *	RITCOMPVAL-Register erreicht. Der Interrupt wird gleichzeitig ausgeführt.
 *	Mit <b>NVIC_EnableIRQ(RIT_IRQn)</b> geben wir die Funktion an, die bei einem Interrupt ausgeführt werden soll.
 *
 *  <HR>
 *
 *   \author	Andreas Gebel & Le Quan Tran
 *   \date		10.12.2018
 *   \version   aktuelle Version 10.12.18
 */


 /*********************************************************************/
/**
Funktion zur Einstellung der Ports als GPIO Ports und
Initialisierung der GPIO Ports als Ausgänge.

\param   -

\return  -

\version 04.12.2018

\todo    
\bug     keine Fehler bekannt
**********************************************************************/
void GPIO_init(void){

	LPC_PINCON->PINSEL4 &= RGB_PINSEL;			//Pin P2[7]-P2[9] als GPIO Ports einstellen
	LPC_GPIO2->FIODIR |= RGB_LED;				//RBG-LED Ports als Ausgänge

	LPC_PINCON->PINSEL4 &= TASTER_PINSEL;		//Pin P2[3]-P2[5] als GPIO Ports einstellen

	LPC_PINCON->PINSEL3 &= A0_PINSEL;			//Pin P1[27] als GPIO Port einstellen
	LPC_GPIO1->FIODIR |= A0_OUT;				//Digitaler Ausgang A0 als Ausgang

}

/*********************************************************************/
/**
Funktion Initialisierung des LCD-Displays.

\param   -

\return  -

\version 04.12.2018

\todo    
\bug     keine Fehler bekannt
**********************************************************************/
void LCD_init(void){
	lcd_init(4);	//LCD-Display mit 4 Zeilen initialisieren
	lcd_clrscr();	//LCD-Display löschen/clearen
	lcd_write_4lines("Funktionsgenerator", "Form: " , "Freq:       Hz" , "Ampl:       mV");	//4 Zeilen Ausgabe an LCD-Display, jede Zeile mit einen Komma getrennt
}

/*********************************************************************/
/**
Funktion zur Initialisierung der 16 roten LEDs mit Hilfe des I2C

\param   -

\return  -

\version 04.12.2018

\todo    
\bug     keine Fehler bekannt
**********************************************************************/
void I2C_init(void){

	i2c_init(I2C_SM);							//Standard-Modus, Übertragungsrate 0,1 Mbit/s
	i2c_start();								//Übertragung starten
		i2c_write_byte(MAX7311 | I2C_WRITE);	//Schreibvorgang auf Baustein MAX7311
		i2c_write_byte(0x06);					//Adresse, Port 1 Konfiguration
		i2c_write_byte(0x00);					//Port 1 als Ausgang einstellen
		i2c_write_byte(0x00);					//Port 2 als Ausgang einstellen, Adresse 0x07 bzw. nächstes Byte
	i2c_stop();									//Schreibvorgang beenden
}

/*********************************************************************/
/**
Funktion zur Initialisierung des Analog-Digital-Wandler.

\param   -

\return  -

\version 04.12.2018

\todo    
\bug     keine Fehler bekannt
**********************************************************************/
void AD_init(void){

	LPC_SC->PCONP |= (1 << PCADC);						// AD Einschalten
	LPC_ADC->ADCR |= (1 << PDN);						// AD Einschalten
	LPC_SC->PCLKSEL0 |= (0b11 << PCLK_ADC);				// CCLK / 8 einstellen
	LPC_ADC->ADCR |= (0b01 << AD_CLKDIV);				// Takt  nochmal um 2 teilen => 100MHz / 8*2 = 6,25MHz
	LPC_PINCON->PINSEL1 |= (0b01 << ADC0_0);			// ADC0.0 auswählen
	LPC_PINCON->PINMODE1 |= (0b10 << ADC0_0);			// Weder Pull-up noch Pull-Down

}

/*********************************************************************/
/**
Funktion Initialisierung des analogen Ausgangs

\param   -

\return  -

\version 04.12.2018

\todo
\bug     keine Fehler bekannt
**********************************************************************/
void AOUT_init(void){

	LPC_SC->PCLKSEL0 &= ~(0b11 << PCLK_DAC); //Takt: CCLK / 4
	LPC_PINCON->PINSEL1 |= (0b10 << AOUT);	 //Pin als DAC einstellen
	LPC_PINCON->PINMODE1 |= (0b10 << AOUT);	 //Weder Pull-Up- noch Pull-Down-Widerstand

}

/*********************************************************************/
/**
Funktion Initialisierung des Timer0 Interrupts für die Decodierung des RC5-Codes bzw.
Fernbedienung

\param   -

\return  -

\version 04.12.2018

\todo
\bug     keine Fehler bekannt
**********************************************************************/
void Timer0_init(void){

	LPC_SC->PCONP |= (1 << PCTIM0) ; 			// Einschaltung des Timer0
	LPC_SC->PCLKSEL0 |= (0b11 << PCLK_TIMER0);	// Takt-aktivierung CCLK / 8 -> 100MHz / 8 = 12,5MHz

	LPC_TIM0->MR0  = 3700;  					// MR Wert -> 12,5 MHz / 3700 = 3,378kHz
	LPC_TIM0->MCR  |= (0b11);					// TC löschen wenn MR0 erreicht wurde und Interrupt generieren
	LPC_TIM0->TCR  |= (1 << 1);

	NVIC_EnableIRQ(TIMER0_IRQn); 				// Funktion die beim Interrupt ausgefuehrt wird

	LPC_PINCON->PINMODE0|= (2 << RC5_PINMODE); 			// Weder Pull-UP noch Pull-Down
	LPC_TIM0->TCR  = 1;        					// Timer0 aktivieren
}

/*********************************************************************/
/**
Funktion Initialisierung des Repetitive Interrupt Timer fuer
die Einstellung der Frequenz

\param   -

\return  -

\version 04.12.2018

\todo
\bug     keine Fehler bekannt
**********************************************************************/
void RIT_init(void) {
	LPC_SC->PCONP |= (1 << PCRIT);						// Einschalten
	LPC_SC->PCLKSEL1 &= ~(0b11 << PCLK_RIT);			// Takt Einstellungen löschen
	LPC_SC->PCLKSEL1 |= (0b01 << PCLK_RIT);				// Takt auf 100MHz stellen
	LPC_RIT->RICTRL |= (1 << RITENCLR);					// RIT Clearen

	NVIC_EnableIRQ(RIT_IRQn);
}
