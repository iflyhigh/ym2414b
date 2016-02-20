// some code grabbed from YM2151 Library, see http://www.ooishoo.org/?page_id=15
// see http://sr4.sakura.ne.jp/fmsound/opz.html for YM2414B register analysis
#include "opz.h"
#include "types.h"


void setup()
{
	// init pins
	pinMode(pinIC, OUTPUT);
	pinMode(13, OUTPUT);

	YM_CTRL_DDR |= _BV(YM_CS) | _BV(YM_RD) | _BV(YM_WR) | _BV(YM_A0);	// output mode for control pins
	YM_DATA_DDR = 0xff;													// output mode for data bus pins
	YM_CTRL_PORT |= _BV(YM_WR) | _BV(YM_RD); 							// WR and RD high by default

	// reset YM
	digitalWrite(pinIC, LOW);
	delay(100);
	digitalWrite(pinIC, HIGH);

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
	Serial.begin(9600);

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

	for (uint8_t i = 0; i < 8; i++)
	{
		set_note(i, 24 + i);
		delay(300);
		unset_note(i);
		delay(300);
	}
	delay(1000);

	//load_patch(0);
	//process_encoders();
	//update_display();
	//MIDI.read();
}

