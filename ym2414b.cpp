// some code grabbed from YM2151 Library, see http://www.ooishoo.org/?page_id=15
// see http://sr4.sakura.ne.jp/fmsound/opz.html for YM2414B register analysis
#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>

#include "opz.h"
#include "types.h"


void setup()
{
	// init pins
	//pinMode(pinIC, OUTPUT);
	//pinMode(13, OUTPUT);

	YM_CTRL_DDR |= _BV(YM_CS) | _BV(YM_RD) | _BV(YM_WR) | _BV(YM_A0);	// output mode for control pins
	//YM_DATA_DDR = 0xff;													// output mode for data bus pins
	DDRD |= B11111100;													// pins 2-7 output
	DDRB |= B00000011;													// pins 8-9 output
	YM_CTRL_PORT |= _BV(YM_WR) | _BV(YM_RD); 							// WR and RD high by default

	// reset YM
	/*
	digitalWrite(pinIC, LOW);
	delay(100);
	digitalWrite(pinIC, HIGH);
	*/

	//Serial.begin(9600);
	//Serial.println("YM2414B demo");

	delay(1000);

	// setup as TZ81Z does during poweron

	setreg(0x09, 0x00);
	setreg(0x0f, 0x00);
	setreg(0x1c, 0x00);
	setreg(0x1e, 0x00);
	setreg(0x0a, 0x04);
	setreg(0x14, 0x70);
	setreg(0x15, 0x01);

	load_patch(0);

	for (uint8_t j = 0; j < 8; j++)
	{
		setreg(0x08, 0x00 + j);	// Key OFF channel j
	}

	/*
	for (uint16_t i = 0; i <= 255; i++)
	{
	Serial.print("0x");
	Serial.print(i, HEX);
	Serial.print(" = 0x");
	Serial.print(yamaha[i], HEX);
	if (((i >= 0x40) && (i <= 0x5f)) || ((i >= 0x80) && (i <= 0x9f)) || ((i >= 0xc0) && (i <= 0xdf)))
	{
	Serial.print(" / 0x");
	Serial.print(yamaha2[i], HEX);
	}
	Serial.println();
	}
	delay(100000);
	*/
}

void loop()
{

	uint8_t notes[] = {0, 2, 4, 5, 7, 9, 11};

	for (uint8_t i = 0; i < 7; i++)
	{
		set_note(i, 48 + notes[i], 127, 127, 1);
		delay(300);
		unset_note(i);
		delay(300);
	}
	//digitalWrite(2, LOW);
	delay(1000);
	//digitalWrite(2, HIGH);

	//load_patch(0);
	//process_encoders();
	//update_display();
	//MIDI.read();
}

