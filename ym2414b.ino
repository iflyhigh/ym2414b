// Some code grabbed from YM2151 Library, see http://www.ooishoo.org/?page_id=15

#define YM_CTRL_DDR DDRC
#define YM_CTRL_PORT PORTC
#define YM_DATA_DDR DDRD
#define YM_DATA_PORT PORTD
#define YM_DATA_PIN PIND

#define YM_CS (3) // PC2 (= pin A2 for Arduino UNO)
#define YM_RD (2) // PC2 (= pin A2 for Arduino UNO)
#define YM_WR (1) // PC1 (= pin A1 for Arduino UNO)
#define YM_A0 (0) // PC0 (= pin A0 for Arduino UNO)

const int pinIC = 12;

void wait(uint8_t loop)
{
	uint8_t wi;
	for (wi = 0; wi < loop; wi++)
	{
		// 16MHz  nop = @60nSec
		asm volatile("nop\n\t nop\n\t nop\n\t nop\n\t");
	}
}

static void setreg(uint8_t reg, uint8_t data)
{
	static uint8_t last_write_addr = 0x00;

	if (last_write_addr != 0x20)
	{
		YM_DATA_DDR = 0x00; 					// input mode for data bus pins
		wait(8);
		YM_CTRL_PORT &= ~_BV(YM_A0); 			// A0 low - read
		wait(4);
		for (uint8_t i = 0; i < 32; i++)
		{
			YM_CTRL_PORT &= ~_BV(YM_RD);		// RD low - read data
			wait(4);

			if ((YM_DATA_PIN & _BV(7)) == 0) 	// D7 contains 0 when write has completed, 1 - when not
			{
				YM_CTRL_PORT |= _BV(YM_RD);		// RD high - stop reading data if D7 == 0
				wait(4);
				break;
			}

			YM_CTRL_PORT |= _BV(YM_RD);			// RD high - stop reading data anyway
			wait(8);							// wait some more
			if (i > 16)
			{
				delayMicroseconds(1);
			}
		}
		YM_DATA_DDR = 0xff;				// output mode for data bus pins
		wait(8);
	}
	YM_CTRL_PORT &= ~_BV(YM_A0); 	// A0 low - write register address
	YM_DATA_PORT = reg;				// register address
	wait(4);
	YM_CTRL_PORT &= ~_BV(YM_WR);	// WR low - write data
	wait(4);						// wait for address to be read by YM chip
	YM_CTRL_PORT |= _BV(YM_WR);		// WR high - data written
	wait(2);
	YM_CTRL_PORT |= _BV(YM_A0);		// A0 high - write register data
	YM_DATA_PORT = data;			// register data
	wait(4);
	YM_CTRL_PORT &= ~_BV(YM_WR);	// WR low - write data
	wait(4);						// wait for address to be read by YM chip
	YM_CTRL_PORT |= _BV(YM_WR);		// WR high - data written
	wait(2);

	last_write_addr = reg;
}

void setup()
{
	// init pins
	pinMode(pinIC, OUTPUT);

	YM_CTRL_DDR |= _BV(YM_CS) | _BV(YM_RD) | _BV(YM_WR) | _BV(YM_A0);
	YM_DATA_DDR = 0xFF;
	YM_CTRL_PORT |= _BV(YM_CS) | _BV(YM_WR) | _BV(YM_RD); /* CS and WR HIGH by default */

	YM_CTRL_PORT &= ~_BV(YM_A0); 	/* A0 LOW by default */
	YM_CTRL_PORT &= ~_BV(YM_CS); // CS LOW


	// reset YM
	digitalWrite(pinIC, LOW);
	delay(100);
	digitalWrite(pinIC, HIGH);

	for (uint8_t j = 0; j < 8; j++)
	{
		setreg(0x08, 0x00 + j);	// Key OFF channel j
	}

	setreg(0x0f, 0x00);	// Noise OFF
	setreg(0x18, 0x00);	// LFO frequency = 0
	setreg(0x19, 0x00);	// Amplitude modulation depth = 0
	setreg(0x19, 0x80);	// Phase modulation depth = 0
	setreg(0x1B, 0x00);	// LFO wafeform = sawtooth

	load_patch(0);

	delay(1000);
	/*

		for (uint8_t j = 0; j < 8; j++)									// we support only single mode => iterate settings across all 8 channels
		{
			setreg(0x38 + j, (0x07 << 4) | 0x00);						// PMS = 4 + AMS = 0 channel J
		}

		setreg(0x19, 0x80 | 0xff);	// Phase modulation depth = f
		setreg(0x18, 0xd0);	// LFO frequency = 0
	*/
	//Serial.begin(9600);
}

void loop()
{
	set_note(0, 47);
	//Serial.println("1");
	delay(1000);
	unset_note(0);
	//Serial.println("0");
	//delay(1000);
	//load_patch(0);
	//process_encoders();
	//update_display();
	//MIDI.read();
}

void load_patch(uint16_t i)
{
	uint8_t patch[38] =
	{

		6, 3,
		31, 5, 5, 5, 2, 33, 1, 8, 3,
		27, 11, 0, 6, 15, 0, 1, 2, 3,
		31, 6, 7, 6, 5, 0, 2, 0, 7,
		31, 11, 8, 6, 3, 0, 1, 1, 7

		/*
				4, 3,
				30, 0, 0, 0, 0, 23, 0, 1, 3,
				27, 4, 0, 7, 1, 0, 0, 1, 3,
				30, 0, 0, 0, 0, 18, 0, 1, 7,
				25, 4, 0, 7, 1, 0, 0, 1, 7
		*/
	};

	/*
	AL  FB
	AR1 DR1 SR1 RR1 SL1 OL1 KS1 ML1 DT1
	AR2 DR2 SR2 RR2 SL2 OL2 KS2 ML2 DT2
	AR3 DR3 SR3 RR3 SL3 OL3 KS3 ML3 DT3
	AR4 DR4 SR4 RR4 SL4 OL4 KS4 ML4 DT4

	AL  - Algorithm
	FB  - FeedBack
	AR* - Attack Rate
	DR* - Decay Rate
	SR* - Sustain Rate
	RR* - Release Rate
	SL* - Sustain Level
	OL* - Operator Level
	KS* - Key Scale
	ML* - Multiple
	DT* - Detune
	*/

	setreg(0x20, 0xc0 | (patch[i + 1] << 3) | patch[i + 0]);		// RL + FB + CONECT

	for (uint8_t j = 0; j < 8; j++)									// we support only single mode => iterate settings across all 8 channels
	{
		setreg(0x38 + j, (0x00 << 4) | 0x00);						// PMS = 0 + AMS = 0 channel J
		setreg(0x40 + j, (patch[i + 10] << 4) | patch[i + 9]);		// Detune(1) + Phase mulitply channel J op 0
		setreg(0x48 + j, (patch[i + 19] << 4) | patch[i + 18]);		// Detune(1) + Phase mulitply channel J op 1
		setreg(0x50 + j, (patch[i + 28] << 4) | patch[i + 27]);		// Detune(1) + Phase mulitply channel J op 2
		setreg(0x58 + j, (patch[i + 37] << 4) | patch[i + 36]);		// Detune(1) + Phase mulitply channel J op 3
		setreg(0x60 + j, patch[i + 7]);								// Total level channel J op 0
		setreg(0x68 + j, patch[i + 16]);							// Total level channel J op 1
		setreg(0x70 + j, patch[i + 25]);							// Total level channel J op 2
		setreg(0x78 + j, patch[i + 34]);							// Total level channel J op 3
		setreg(0x80 + j, (patch[i + 8] << 6) | patch[i + 2]);		// Key scaling + Attack rate channel J op 0
		setreg(0x88 + j, (patch[i + 17] << 6) | patch[i + 11]);		// Key scaling + Attack rate channel J op 1
		setreg(0x90 + j, (patch[i + 26] << 6) | patch[i + 20]);		// Key scaling + Attack rate channel J op 2
		setreg(0x98 + j, (patch[i + 35] << 6) | patch[i + 29]);		// Key scaling + Attack rate channel J op 3
		setreg(0xa0 + j, 0x80 | patch[i + 3]);						// Amplitude modulation sensitivity = 1 + Decay(1) rate channel J op 0
		setreg(0xa8 + j, 0x80 | patch[i + 12]);						// Amplitude modulation sensitivity = 1 + Decay(1) rate channel J op 1
		setreg(0xb0 + j, 0x80 | patch[i + 21]);						// Amplitude modulation sensitivity = 1 + Decay(1) rate channel J op 2
		setreg(0xb8 + j, 0x80 | patch[i + 30]);						// Amplitude modulation sensitivity = 1 + Decay(1) rate channel J op 3
		setreg(0xc0 + j, 0x00 | patch[i + 4]);						// Detune(2) = 0 + Decay(2)/Sustain rate channel J op 0
		setreg(0xc8 + j, 0x00 | patch[i + 13]);						// Detune(2) = 0 + Decay(2)/Sustain rate channel J op 1
		setreg(0xd0 + j, 0x00 | patch[i + 22]);						// Detune(2) = 0 + Decay(2)/Sustain rate channel J op 2
		setreg(0xd8 + j, 0x00 | patch[i + 31]);						// Detune(2) = 0 + Decay(2)/Sustain rate channel J op 3
		setreg(0xe0 + j, (patch[i + 6] << 4) | patch[i + 5]);		// Decay(1) level + Release rate channel J op 0
		setreg(0xe8 + j, (patch[i + 15] << 4) | patch[i + 14]);		// Decay(1) level + Release rate channel J op 1
		setreg(0xf0 + j, (patch[i + 24] << 4) | patch[i + 23]);		// Decay(1) level + Release rate channel J op 2
		setreg(0xf8 + j, (patch[i + 33] << 4) | patch[i + 32]);		// Decay(1) level + Release rate channel J op 2
	}

}

void set_note(uint8_t voice, uint8_t midi_note)
{
	uint8_t octave;
	uint8_t note;
	uint8_t fraction = 0;
	// For some reason YM2414B supports 8 ocataves (0-7) each starting with C# and ending with C
	// A4 = 440Hz according to application notes (2.1.2) && MIDI A4 = 440Hz => we support MIDI notes
	// from 1 to 96 inclusive and ignore others
	if ((midi_note > 0) && (midi_note < 97))						// ignore notes that are not supported by YM2151
	{
		octave = (midi_note - 1) / 12;
		note = (midi_note - 1) % 12;
		if (note > 2) {note++;};									// YM2151 note numbers in octave are 0,1,2,4,5,6,8,9,10,12,13,14
		if (note > 6) {note++;};									// Why?
		if (note > 10) {note++;};									// Because we can! :)
		setreg(0x28 + voice, (octave << 4) | note);					// Set channel note
		setreg(0x30 + voice, fraction << 2);						// Set channel note fraction (not implemented yet)
		setreg(0x08, 0x78 | voice);									// Key ON for channel (all 4 OPs are running)
	}
}

void unset_note(uint8_t voice)
{
	setreg(0x08, 0x00 | voice);
}