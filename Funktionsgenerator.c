/*
===============================================================================
 Name        : Funktionsgenerator.c
 Author      : Andreas Gebel % Le Quan Tran
 Version     :	
 Description : 
===============================================================================
*/


#include "LPC17xx.h"
#include "Funktionsgenerator.h"
#include "i2c_1769.h"
#include "lcdlib_1769.h"
#include "functions.h"
#include "Init.h"


/**
 *
 *  \file     Funktionsgenerator.c
 *  \mainpage Funktionsgenerator
 *
 *  <HR>
 *
 *  \section sec1 Einleitung
 *	Dieses Programm dient zur Ausgabe eines Sinus-, Dreieck- oder Saegezahn-Signals dessen
 * 	Frequenz mit Hilfe einer Fernbedienung eingestellt werden kann und die Amplitude durch
 *	den AD-Wandler eingestellt werden kann.
 *
 *	\section sec2 Unterprogramme
 *	In dieser Sektion sind alle Unterprogramme die zur Funktion des Funktionsgenerators gebraucht werden.
 *
 * 	\subsection sec2_1 rgb_led_set()
 *  Dem Unterprogramm wird die Farbe als Parameter übergeben und mit einer
 *  switch-case werden die betroffenen RGB-LEDs mittels des FIOPIN Registers an-/ausgeschaltet
 *
 *	\subsection sec2_2 red_leds_set()
 *  Der übergebene Parameter wird auf die 16 Bits des Portexpanders skaliert und mit Hilfe des I2C-Bus
 *  übergeben.
 *
 *  \subsection sec2_3 LCD_write_form()
 *  Der übergebene String wird mit der der Funktion <b>lcd_write_string_xy()</b>, die mit einer Bibliothek 
 *  zur Verfügung gestellt wurde, an dem LC-Display ausgegeben.
 *
 *  \subsection sec2_4 LCD_write_freq_ampl()
 *  Es wird zwischen einer Ausgabe der Frequenz oder Amplitude unterschieden und dann auf die jeweilige
 *  Zeile eine Zahl mit maximal 5 Zeichen ausgeben. Frequenz im Bereich 0-10kHz und Amplitude im Bereich
 *  0-4095 (ADC-Wert) der skaliert wird.
 *  
 *  \subsection sec2_5 DOUT_write()
 *  Ausgabe eines Rechtecksignals indem der digitale Ausgang in einer bestimmten Takt gesetzt und wieder
 *  gelöscht wird mit FIOSET und FIOCLR. Dieser ist phasengleich zur Signalausgabe des analogen Ausgangs.
 *  
 *  \subsection sec2_6 AD_read()
 *  Liest einen analogen Wert über dem AD-Wandler und wandelt diesen, in einen digitalen Wert um.
 *  Dafür wird speziell der ADC Eingang gesetzt, von dem gelesen werden soll. Dieser wird gestartet und 
 *  es wird darauf gewartet, dass die Umwandlung fertig ist. Der Wert wird dann noch gefiltert, um 
 *  große Schwankungen zu minimieren.
 *
 *  \subsection sec2_7 AD_write()
 *  Die Form des Signals wird anhand der globalen Variable <b>function</b> ausgewählt. Mit den beiden 
 *  rechtsshifts skalieren wir die Funktionsausgabe mit dem ADC-Wert.  
 *  
 *  \subsection sec2_8 button_read()
 *  Die Funktion dient zur Einlesung der Taster T1-T3 sowie der Auswertung des RC5-Codes.
 *  Falls ein Taster oder die PLAY/VORWAERTS/RUECKWARTS-Taste auf der Fernbedienung gedrückt wird, 
 *  wird die entsprechene Funktion in der globalen Variable <b>function</b> 
 *  gesetzt und die entsprechenden Ausgaben für den LCD ausgeführt. 
 *
 *  \subsection sec2_9 TIMER0_IRQHandler()
 *  Die Timer0-Serviceroutine dient zur Decodierung des R5-Signals. 
 *
 *  \subsection sec2_10 RIT_IRQHandler()
 *  Die RIT-Serviceroutine ruft in einem bestimmten Takt AD_write(), die für die Ausgabe des Signals am 
 *  analogen Ausgang dient, und DOUT_write(), die für die Ausgabe eines Rechtecksignals am digitalen Ausgang dient,
 *  auf.
 *
 *  \subsection sec2_11 frequency_get()
 *  Das Unterprogramm übernimmt den aktuellen RC5-Code und prüft nach, ob dieser eine
 *  neu betätigte Taste oder gedrückt gehaltene Taste ist. 
 *  Danach wird die Frequenz aktualisiert und wenn die RC5-Aus-Taste betätigt wurde, wird
 *  Die Frequenz mit der frequency_set() Funktion geändert.
 * 
 *  \subsection sec2_12 frequency_set()
 *  Prüft nach, welche Frequenz übergeben wurde und setzt die scale-Variable auf den benötigten Wert, 
 *  um noch eine saubere Signaldarstellung zu ermöglichen.
 *  Der RIT wird auf die angegebene Frequenz angepasst.
 *  
 *
 *
 *  <HR>
 *
 *   \author	Andreas Gebel & Le Quan Tran
 *   \date		10.12.2018
 *   \version 	1.19	        aktuelle Version 10.12.18
 *   \version	1.18			05.12.18
 */

int main(void) {

	GPIO_init();
	LCD_init();
	I2C_init();
	AD_init();
	AOUT_init();
	Timer0_init();
	RIT_init();

	rgb_led_set(BLAU);					//RGB beim Start auf Blau stellen
	LCD_write_form("Sinus");			//Sinus als Startsignal
	LCD_write_freq_ampl(FREQ, 1000);	// 1kHz als Startfrequenz
	frequency_set(1000);					//		""

	while(1){
		button_read();							//Taster / RC5-Code lesen und Einstellung des Funktionssignals
		AD_read();								//ADC-Wert lesen
		red_leds_set(value_adc);				//ADC-Wert and den 16 LEDs ausgeben
		LCD_write_freq_ampl(AMPL, value_adc);	//Ausgabe des ADC-Wertes als AMPL auf dem LC Display
		if (new_cmd == 1) {						//Nur wenn ein neues Kommando/RC5-Taste gedrück wurde, wird die Funktion zur Frequenzänderung aufgerufen
			frequency_get();
			new_cmd = 0;						//Kommando zurücksetzten bzw. bearbeitet worden
		}
	}

    return 0 ;
}


/*********************************************************************/
/**
RGB LED Setter, je nachdem welche Form angezeigt wird

\param  color
			Eingabe der Farbe (ROT, BLAU, GRUEN)
				Wertebereich BLAU bzw 1
							 ROT bzw 2
							 GRUEN bzw 3

\return  -

\version 08.11.2018

\todo
\bug     keine Fehler bekannt
**********************************************************************/
void rgb_led_set(uint8_t color){

	switch(color){
	case BLAU:
		LPC_GPIO2->FIOPIN = (~(1) << 7);	//P2[7] auf 0 setzten und die anderen P2 auf 1
			break;

	case ROT:
		LPC_GPIO2->FIOPIN = (~(2) << 7);	//P2[8] auf 0 setzten und die anderen P2 auf 1
			break;

	case GRUEN:
		LPC_GPIO2->FIOPIN = (~(4) << 7);	//P2[9] auf 0 setzten und die anderen P2 auf 1
			break;
	}
}

/*********************************************************************/
/**
Funktion Ausgabe der Amplitude auf den 16 roten LEDs über I2C

\param	byte
			eine Zahl bzw eine Bitmaske die in dem Bereich 0-2^16 ist


\return  -

\version 04.12.2018

\todo		
\bug     keine Fehler bekannt
**********************************************************************/

void red_leds_set(uint16_t byte){

	uint16_t leds = 0;
	leds = 1 << ((15* byte)/4095);		// Skalieren von 12 auf 16 Bits

	i2c_start();
	i2c_write_byte(MAX7311 | I2C_WRITE);// Schreibvorgang an MAX7311 starten
	i2c_write_byte(0x02);				// Kommando Bit zum Start
	i2c_write_byte(~leds);				// Die ersten 8 Bits schreiben
	i2c_write_byte(~(leds >> 8));		// Die letzten 8 Bits schreiben
	i2c_stop();

}


/*********************************************************************/
/**
Funktion Ausgabe der Form auf den LC Display

\param	string
			String zur Ausgabe am LCD
			Strings:
			Sinus
			Dreieck
			Saegezahn


\return  -

\version 08.11.2018

\todo
\bug     keine Fehler bekannt
**********************************************************************/
void LCD_write_form(char_t *string) {
	lcd_write_string_xy(string, 7 , 2);
}

/*********************************************************************/
/**
Funktion Ausgabe der Frequenz und Amplitude auf dem LC Display

\param  option
			FREQ = die Frequenz auf dem LC Display wird geaendert
			AMPL = die Amplitude auf dem LC wird geaendert
\param	num
			Die Zahl die ausgegeben werden soll:
				bei option FREQ = Zahl in Bereich von 0-10000
				bei option AMPL = Zahl in Bereich von 0-4095

\return  -

\version 08.11.2018

\todo
\bug     keine Fehler bekannt
**********************************************************************/
void LCD_write_freq_ampl(uint8_t option, uint16_t num){


	switch(option){
		case 1:
			lcd_gotoxy(7, 3);		
			lcd_write_uint(num, 5);	//Maximal 5 Zeichen ausgeben
			break;

		case 2:
			num = (3040*num/4095);	//Skalierung des ADC wertes auf die mV Anzeige des Oszilloskop
			lcd_gotoxy(7, 4);
			lcd_write_uint(num, 5); //Maximal 5 Zeichen ausgeben
			break;
	}
}

/*********************************************************************/
/**
Funktion Ausgabe eines Rechtecksignals mit dem DA Wandler am Digitalen Ausgang

\param   -

\return  -

\version 08.11.2018

\todo
\bug     keine Fehler bekannt
**********************************************************************/
void DOUT_write(void){

	if(val <= 180)							//Bei der hälfte des val setzen oder clearen für phasengleiches Rechtecksignal
		LPC_GPIO1->FIOSET = (1 << 27);		//Ausgang setzen
	else
		LPC_GPIO1->FIOCLR = (1 << 27);		//Ausgang löschen

}

/*********************************************************************/
/**
Funktion A/D-Wandler lesen und den Wert in value_adc speichern

\param   -

\return  -

\version 28.11.2018

\todo
\bug     keine Fehler bekannt
**********************************************************************/
void AD_read(void){

	uint16_t new_value;
	uint32_t done;

	LPC_ADC->ADCR &= ~(0xff);					//Auswahl eines AD-Eingangs löschen
	LPC_ADC->ADCR |= (1 << 0);					//ADC0.0 auswählen
	LPC_ADC->ADCR |= (1 << 24);					//ADC an ADC0.0 starten

	do
	{
		done = LPC_ADC->ADGDR >> 31;
	} while(done != 1);							//Auf Done-Bit = 1warten

	LPC_ADC->ADCR &= ~((1 << 24)|(1 << 0));		//Start-Bit und Auswahl-Bit löschen

	new_value = (LPC_ADC->ADGDR >> 4) & 0xFFF;	//Bits shiften und Maskieren

	value_adc = (0.1*new_value) + (0.9* value_adc);	// Filtern

}

/*********************************************************************/
/**
Funktion zur Ausgabe des Signals am analogen Ausgang mit einer
bestimmten Frequenz und Amplitude

\param  value
			Index des Arrays, der ausgegeben werden soll
				Wertebereich: 0-360
\param	function_option
			Für die Einstellung des Funktionssignals
				Wertebereich: 0-2
\return  -

\version 10.12.2018

\todo    
\bug     keine Fehler bekannt
**********************************************************************/
void AD_write(uint16_t value){

	uint16_t amplitude;

    amplitude = (value_adc >> 4);		//Berechnung der Amplitude mit dem ADC Wert

    switch(function){

    case 0:
    	value = (SINUS[val]);
    	break;
    case 1:
    	value = (SAEGEZAHN[val]);
		break;
    case 2:
    	value = (DREIECK[val]);
		break;
    }

	value = ((value * amplitude) >> 8);	//Berechnung der Amplitude mit dem ADC Wert

	LPC_DAC->DACCTRL |= (0b11 << 1);	//Bit DBLBUF_ENA und CNT_ENA setzen
	LPC_DAC->DACR = (value << 6);		//Ausgabe des Wertes an AOUT

}

/*********************************************************************/
/**
Funktion Taster oder RC5 lesen und Veraenderung der Form auf dem LC Display sowie
der RBG LED und Ausgabe der entsprechenden Signalform.
\param   -


\return  -

\version 05.12.2018

\todo	
\bug     keine Fehler bekannt
**********************************************************************/
void button_read(void) {

	static uint8_t taster;							//T1-3
	static uint8_t option;


	taster = (~(LPC_GPIO2->FIOPIN >> 3) & 0b111);	// Taster einlesen

	option = rc5_option;							// letzte Fernbedienungaktion kopieren

	if(taster == 1){								// T1
		option = RC5_RUECKWAERTS;
	}
	else if (taster == 2){							// T2
		option = RC5_PLAY;
	}
	else if (taster == 4){							// T3
		option = RC5_VORWAERTS;
	}
	
	if(option == RC5_RUECKWAERTS){					//Option für T1 bzw RC5_RUECKWAERTS
		function = 0;
		rgb_led_set(BLAU);
		LCD_write_form("Sinus    ");
	}
	else if( option == RC5_PLAY){					//Option für T2 bzw RC5_PLAY
		function = 1;
		rgb_led_set(ROT);
		LCD_write_form("Saegezahn");
	}
	else if (option == RC5_VORWAERTS){				//Option für T3 bzw RC5_VORWAERTS
		function = 2;
		rgb_led_set(GRUEN);
		LCD_write_form("Dreieck  ");
	}

	rc5_option = 0;
}


/*********************************************************************/
/**
Ausführung der Funktion bei Interrupt des Timer0-Interrupts.
Dient zur Decodierung des RC5-Signal.

\param   -

\return  -

\version 05.12.2018

\todo	

\bug     keine Fehler bekannt
**********************************************************************/
void TIMER0_IRQHandler(void){

	static volatile uint32_t start = 0;
	static volatile uint32_t counter = 0;
	static volatile uint32_t takt = 0;
	static volatile uint32_t code = 0;
	static volatile uint32_t old_bit = 0;
	static volatile uint32_t old_flip = 0;
	uint32_t bit = 0, new_bit = 0;

    LPC_TIM0->IR |= 0b10;         		// Interrupt zuruecksetzen

	bit = ((LPC_GPIO0->FIOPIN  >> RC5) & 0b1) ^ 1;	// Port des RC5 Einganges lesen, maskieren und invertieren

	if(bit == 1 && start == 0){			// Schaue nach ob ein neues Bit am Eingang liegt und das zu einer neuen
		code = 0;						// Übertragung gehoert. Wenn der Ausdruck wahr ist werden alle Variablen
		takt = 7;						// für eine Uebertragung vorbereitet.
		counter = 0;
		start = 1;
	}

	if((takt > 4) && (old_bit != bit)){ //
		new_bit = bit;					// Eingelesenes Bit zwischenspeichern
		counter++;						// Bit-Zaehler inkrementieren
		takt = 0;						// Takt zuruecksetzen
		if (counter == 3)				// Flip-Bit sichern
			flip = new_bit;
		code = (code << 1) | new_bit;	// Vorhandenen Code um 1 nach links schieben und neuen Bit hinten dransetzen

	}else if(takt > 10){				// Sollte die Uebertragund mitten drin abbrechen werden alle Variablen
		takt = 0;						// zurueckgesetzt.
		counter = 0;
		start = 0;
	}

	if(counter == 14){					// Sobald 14 Bits eingelesen sind wird der Befehl gespeichert
		new_code   = code & 0x3f;		// Befehl wird gespeichert und so maskiert das nur der eigentliche Befehl gespeichert wird
		rc5_option = code & 0x3f;		// Wie oben nur in eine 2. Variable
		if (old_flip == flip) {			// Flag zur Signalisierung eines neuen Befehls
			new_cmd = 1;
		}
		counter = 0;					// Vorbereitung auf neue Übertragung
		start = 0;
	}
	old_flip = flip;					// Sichern von Variablen fuer Abfragen
	old_bit = bit;
	
	if(start){							// Erhöhe den Taktzähler
		takt++;
	}
}

/*********************************************************************/
/**
Ausführung der Funktion bei Interrupt des RIT.
Dient zur Ausfuehrung der Signaleausgabe am DOUT und AOUT.

\param   -

\return  -

\version 05.12.2018

\todo	

\bug     keine Fehler bekannt
**********************************************************************/
void RIT_IRQHandler(void) {

	LPC_RIT->RICTRL |= (1<<0);					//Interrupt zuruecksetzten
    AD_write(val);								
    DOUT_write();
   	val = val + scale;
    if(val >= 360)                              //Bei einem Array-Überlauf => von vorne beginnen
    	val = val % 360;

}


/*********************************************************************/
/**
Diese Funktion dient dabei, die Frequenz mittels der Fernbedienung einlesen
zu können.

\param   -

\return  -

\version 05.12.2018

\todo	

\bug     keine Fehler bekannt
**********************************************************************/
void frequency_get(void) {

	static uint32_t old_flip = 0;
	static uint32_t old_code;
	static uint32_t code;
	static uint32_t frequency = 0;
	uint8_t mul = 10;
	
	NVIC_DisableIRQ(TIMER0_IRQn);
	code = new_code;
	NVIC_EnableIRQ(TIMER0_IRQn);

	//Nur wenn eine neue Zahl eingelesen wird oder wenn die gleiche Zahl nochmal eingegeben wird
	if ((old_code != code) || ((old_code == code) && (old_flip != flip))) {
		if ((code < 10) && (code >= 0)) {		//Wenn die gedrueckte Taste eine Zahl von 0-9 ist
			frequency = (frequency*mul) + code;	//Frequenz einstellen, mit 10 multiplizieren immer wenn ein neuer gueltiger Wert eingelesen wird
			if(frequency > 10000){					
				frequency = 0;
			}
			LCD_write_freq_ampl(FREQ, frequency);	// Eingegeben Frequenz anzeigen
		}

		//Wenn die Aus-Taste gedrueckt wird, wird die eingestellte Frequenz übernommen
		if (code == RC5_AUS_TASTE && frequency != 0) {
			frequency_set(frequency);
			frequency = 0;
		}
	}
	

	old_code = code;	//Taste zwischenspeichern fuer die Ueberpruefung
	old_flip = flip;		//Flip-Flag zwischenspeichern fuer die Ueberpruefung
}

/*********************************************************************/
/**
Diese Funktion dient dabei, die Frequenz und Scale Wert einzustellen

\param   frequency
			Die Frequenz die eingestellt werden soll	
				Wertebereich 0-10000

\return  -

\version 05.12.2018

\todo	

\bug     keine Fehler bekannt
**********************************************************************/
void frequency_set(uint16_t frequency){

	//Scale in Abhängigkeit der Frequenz bestimmen 
	//Scale legt anzahl der auszugebenen Werte an
	if(frequency <= 1000)
		scale = 2;
	else if(frequency <= 2000)
		scale = 4;
	else if(frequency <= 3000)
		scale = 6;
	else if(frequency <= 4000)
		scale = 8;
	else if(frequency <= 5000)
		scale = 10;
	else if(frequency <= 6000)
		scale = 12;
	else if(frequency <= 9000)
		scale = 18;
	else
		scale = 20;

	LPC_RIT->RICTRL &= ~(1 << 3);												//RIT stoppen
	LPC_RIT->RICOMPVAL = (100000000/((360/scale)*frequency));					//Einstellung des COMPARE Values um die gewuenschte Frequenz zu bekommen
	LPC_RIT->RICOUNTER = 0;														//RIT Counter wieder auf 0 Stellen
	LPC_RIT->RICTRL |= (1 << 3);												//RIT starten

}
