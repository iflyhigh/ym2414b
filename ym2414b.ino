// some code grabbed from YM2151 Library, see http://www.ooishoo.org/?page_id=15
// see http://sr4.sakura.ne.jp/fmsound/opz.html for YM2414B register analysis

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

uint8_t yamaha[256];

struct 	op_t	// FM operator settings
{
	uint8_t	ar;
	uint8_t	d1r;
	uint8_t	d2r;
	uint8_t	rr;
	uint8_t	d1l;
	uint8_t	ls;
	uint8_t	ame_ebs_kvs;
	uint8_t	out;
	uint8_t	f;
	uint8_t	rs_det;
};				// 10 bytes

struct aop_t
{
	uint8_t	egshft_fix_fixrg;
	uint8_t	osw_fine;
};			// 2 bytes

struct voice_t
{
	op_t	op[4];
	uint8_t	sy_fbl_alg;
	uint8_t	lfs;
	uint8_t	lfd;
	uint8_t	pmd;
	uint8_t	amd;
	uint8_t	pms_ams_lfw;
	uint8_t	trps;
	uint8_t	pbr;
	uint8_t	ch_mo_su_po_pm;
	uint8_t	port;
	uint8_t	fc_vol;
	uint8_t	mw_pitch;
	uint8_t mw_ampli;
	uint8_t	bc_pitch;
	uint8_t	bc_ampli;
	uint8_t	bc_p_bias;
	uint8_t	bc_e_bias;
	char 	voice_name[10];
	uint8_t pr1;
	uint8_t pr2;
	uint8_t pr3;
	uint8_t pl1;
	uint8_t pl2;
	uint8_t pl3;
	aop_t	aop[4];
	uint8_t	rev;
	uint8_t fc_pitch;
	uint8_t fc_ampli;
};		// 84 bytes

voice_t voice;

uint8_t lfs_vmem_reg[100] =
{
	1, 46, 71, 87, 104, 112, 123, 131, 136, 141,
	146, 151, 155, 158, 163, 165, 169, 171, 173, 176,
	179, 182, 183, 186, 187, 188, 192, 193, 197, 196,
	198, 199, 201, 203, 204, 204, 205, 208, 209, 210,
	211, 212, 214, 215, 216, 217, 218, 219, 220, 220,
	220, 222, 224, 224, 225, 226, 227, 228, 229, 229,
	230, 231, 232, 232, 233, 234, 235, 235, 236, 236,
	236, 237, 237, 239, 240, 240, 241, 242, 242, 244,
	245, 245, 245, 245, 246, 248, 248, 248, 249, 249,
	249, 250, 250, 251, 251, 251, 252, 252, 252, 252
};

uint8_t lfs_vmem_reg_ym[256] =
{
	0x00, 0x08, 0x0C, 0x13, 0x18, 0x1D, 0x20, 0x27, 0x2B, 0x30, 0x33, 0x38,	0x3C, 0x3F, 0x43, 0x47,
	0x4B, 0x50, 0x54, 0x58, 0x5B, 0x5F, 0x63, 0x65, 0x68, 0x6C, 0x6F, 0x72, 0x75, 0x78, 0x7B, 0x7D,
	0x80, 0x83,	0x9A, 0x9C, 0x8B, 0x8D, 0x90, 0x92, 0x94, 0x96, 0x99, 0x9B, 0x9D, 0x9F, 0xA1, 0xA3,
	0xA5, 0xA7, 0xA9, 0xAB, 0xAD, 0xAE, 0xB0, 0xB2,	0xB3, 0xB5, 0xB7, 0xB8, 0xBA, 0xBB, 0xBD, 0xBE,
	0xBF, 0xC1, 0xC2, 0xC4, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD1, 0xD1,
	0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDC, 0xDD, 0xDE, 0xDE,
	0xDF, 0xE0, 0xE1, 0xE1,	0xE2, 0xE2, 0xE3, 0xE4, 0xE4, 0xE5, 0xE5, 0xE6, 0xE6, 0xE7, 0xE7, 0xE8,
	0xE8, 0xE9, 0xE9, 0xEA, 0xEA, 0xEB, 0xEB, 0xEB, 0xEC, 0xEC,	0xED, 0xED, 0xED, 0xEE, 0xEE, 0xEE,
	0xEF, 0xEF, 0xF0, 0xF0, 0xF0, 0xF0, 0xF1, 0xF1, 0xF1, 0xF2, 0xF2, 0xF2, 0xF2, 0xF3, 0xF3, 0xF3,
	0xF3, 0xF4, 0xF4, 0xF4, 0xF4, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xF6, 0xF6, 0xF6, 0xF6, 0xF6, 0xF6,
	0xF7, 0xF7, 0xF7, 0xF7, 0xF7, 0xF7, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF9, 0xF9, 0xF9,
	0xF9, 0xF9, 0xF9, 0xF9, 0xF9, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA,
	0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFC, 0xFC, 0xFC,
	0xFC, 0xFC,	0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
	0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD,	0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD,
	0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFE,	0xFE, 0xFF
};

uint8_t tl_vmem_reg[100] =
{
	119, 114, 110, 106, 102, 99, 96, 94, 92, 90,
	88, 86, 84, 82, 80, 78, 77, 76, 74, 73,
	71, 70, 69, 68, 67, 66, 65, 64, 63, 62,
	61, 60, 59, 58, 57, 56, 55, 54, 53, 52,
	51, 50, 49,	48, 47, 46, 45, 44, 43, 42,
	41, 40, 39, 38, 37, 36, 35, 34, 33, 32,
	31, 30, 29, 28, 27, 26, 25, 36, 23, 22,
	29, 20, 19, 18, 26, 16, 15, 14, 13, 12,
	11, 10, 9, 8, 7, 6, 5, 4, 3, 2,
	1, 0, 0, 0, 0, 0, 0, 0, 0, 21
};

void wait(uint8_t loop)
{
	uint8_t wi;
	for (wi = 0; wi < loop; wi++)
	{
		// 16MHz  nop = @60nSec
		asm volatile("nop\n\t");
	}
}

static void setreg(uint8_t reg, uint8_t data)
{

	YM_CTRL_PORT &= ~_BV(YM_A0); 	// A0 low - write register address
	YM_CTRL_PORT &= ~_BV(YM_WR);	// WR low - write data
	wait(1);
	YM_CTRL_PORT &= ~_BV(YM_CS);	// CS low - chip select
	YM_DATA_PORT = reg;				// register address
	wait(3);
	YM_CTRL_PORT |= _BV(YM_CS);		// CS high - chip unselect
	YM_CTRL_PORT |= _BV(YM_WR);		// WR high - data written
	YM_CTRL_PORT |= _BV(YM_A0);		// A0 high - write register data
	wait(15);
	YM_CTRL_PORT &= ~_BV(YM_WR);	// WR low - write data
	wait(1);
	YM_CTRL_PORT &= ~_BV(YM_CS);	// CS low - chip select
	YM_DATA_PORT = data;			// register data
	wait(3);
	YM_CTRL_PORT |= _BV(YM_CS);		// CS high - chip unselect
	YM_CTRL_PORT |= _BV(YM_WR);		// WR high - data written
	wait(50);


	/*
		yamaha[reg] = data;

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
		wait(8);
		YM_DATA_DDR = 0xff;				// output mode for data bus pins
		wait(8);
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
		wait(8);
	*/
}

void setup()
{
	// init pins
	pinMode(pinIC, OUTPUT);

	YM_CTRL_DDR |= _BV(YM_CS) | _BV(YM_RD) | _BV(YM_WR) | _BV(YM_A0);	// output mode for control pins
	YM_DATA_DDR = 0xff;													// output mode for data bus pins
	YM_CTRL_PORT |= _BV(YM_WR) | _BV(YM_RD); 							// WR and RD high by default
	YM_CTRL_PORT &= ~_BV(YM_A0); 										// A0 low by default
	YM_CTRL_PORT &= ~_BV(YM_CS); 										// CS always low

	// reset YM
	digitalWrite(pinIC, LOW);
	delay(100);
	digitalWrite(pinIC, HIGH);

	// setup as TZ81Z does during poweron

	setreg(0x09, 0x00);
	setreg(0x0a, 0x04);
	setreg(0x0f, 0x00);
	setreg(0x14, 0x70);
	//setreg(0x15, 0x01);
	setreg(0x1c, 0x00);
	setreg(0x1e, 0x00);

	for (uint8_t j = 0; j < 8; j++)
	{
		setreg(0x08, 0x00 + j);	// Key OFF channel j
	}

//	Serial.begin(9600);
	load_patch(0);

	delay(1000);
	/*
		for (uint16_t i = 0; i <= 255; i++)
		{
			Serial.print(i, HEX);
			Serial.print(" - ");
			Serial.println(yamaha[i], HEX);
		}
	*/
}

void loop()
{
	for (uint8_t i = 0; i < 8; i++)
	{
		set_note(i, 37);
		delay(1000);
		unset_note(i);
		delay(1000);
	}
	delay(3000);

//load_patch(0);
//process_encoders();
//update_display();
//MIDI.read();
}


void load_patch(uint16_t i)
{
	unsigned char data[84] =
	{
		0x1F, 0x01, 0x00, 0x08, 0x0A, 0x00, 0x03, 0x43, 0x0A, 0x1E, 0x1F, 0x01, 0x00, 0x08, 0x07, 0x00,
		0x00, 0x46, 0x00, 0x10, 0x1F, 0x09, 0x06, 0x08, 0x0F, 0x1B, 0x07, 0x4A, 0x04, 0x1E, 0x1F, 0x09,
		0x00, 0x09, 0x0F, 0x00, 0x01, 0x63, 0x04, 0x03, 0x3A, 0x1C, 0x00, 0x00, 0x00, 0x52, 0x0C, 0x04,
		0x0D, 0x00, 0x63, 0x4B, 0x00, 0x00, 0x00, 0x32, 0x00, 0x4D, 0x6F, 0x6E, 0x6F, 0x70, 0x68, 0x42,
		0x61, 0x73, 0x73, 0x63, 0x63, 0x63, 0x32, 0x32, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00,
		0x00, 0x00, 0x00, 0x00
	};
	memcpy(&voice, data, 84);


	//	setreg(0x16, lfs_vmem_reg[voice.lfs]);														// UNUSED !!! LFRQ2 = LFO2 Speed
	setreg(0x17, 0x26);
	setreg(0x17, 0x80);
	//setreg(0x17, 0x00 | voice.amd);																	// AMD2 = LFO2 Amplitude Modulation Depth
	//setreg(0x17, 0x80 | voice.pmd);																	// PMD2 = LFO2 Pitch Modulation Depth
	setreg(0x18, lfs_vmem_reg[voice.lfs]);															// LFRQ1 = LFO1 Speed
	// N = 2.**(1./16.)
	// setreg(0x19, 0x00 | min(127, int(round(113.66 * (N ** (voice.amd - 99))))));					// AMD = LFO1 Amplitude Modulation Depth
	setreg(0x19, 0x00);														// PMD = LFO1 Pitch Modulation Depth
	setreg(0x19, 0x80);
//	setreg(0x19, 0x00 | voice.amd);																	// AMD2 = LFO2 Amplitude Modulation Depth
	setreg(0x19, 0x80 | voice.pmd);																	// PMD2 = LFO2 Pitch Modulation Depth
//	setreg(0x1b, 0x00 | ((voice.sy_fbl_alg & 0x40) >> 1) | ((voice.sy_fbl_alg & 0x40) >> 2) |		// UNUSED !!! LFO2 Sync + LFO1 Sync ...
//	       ((voice.pms_ams_lfw & 0x03) << 2) | (voice.pms_ams_lfw & 0x03));							// UNUSED !!! ... + LFO2 Waveform + LFO1 Waveform
	setreg(0x1b, 0x00 | ((voice.sy_fbl_alg & 0x40) >> 2) | (voice.pms_ams_lfw & 0x03));				// LFO1 Sync + LFO1 Waveform

	for (uint8_t j = 0; j < 8; j++)																	// we support only single mode => iterate settings across all 8 channels
	{
		setreg(0x20 + j, 0xc0 | (voice.sy_fbl_alg & 0x3f));											// R + UNK1 + Feedback + Algorithm, UNK1=1 when no output, =0 when playing note
		setreg(0x38 + j, 0x00 | (voice.pms_ams_lfw & 0x70) | ((voice.pms_ams_lfw & 0x0c) >> 2));	// PMS1 + AMS1 = LFO1 Pitch/Amplitude Modulation Sensitivity
		//	setreg(0x38 + j, 0x84 | (voice.pms_ams_lfw & 0x70) | ((voice.pms_ams_lfw & 0x0c) >> 2));	// UNUSED !!! PMS2 + AMS2 = LFO2 Pitch/Amplitude Modulation Sensitivity

		for (uint8_t k = 0; k < 4; k++)																// iterate across operators
		{
			if ((voice.aop[k].egshft_fix_fixrg & 0x08) == 0)										// RATIO mode
			{
				uint8_t dt1_local;

				if ((voice.op[k].rs_det & 0x07) >= 3)
				{
					dt1_local = ((voice.op[k].rs_det & 0x07) - 3);
				}
				else
				{
					dt1_local = (7 - (voice.op[k].rs_det & 0x07));
				}
				setreg(0x40 + j + 0x08 * k, 0x00 | (dt1_local << 4) |
				       ((voice.op[k].f & 0x3c) >> 2));												// DT1 + MUL = Detune 1 + 4 upper bits of CRS (coarse frequency)
			}
			else																					// FIX mode
			{
				setreg(0x40 + j + 0x08 * k, 0x00 | ((voice.aop[k].egshft_fix_fixrg & 0x07) << 4) |
				       ((voice.op[k].f & 0x3c) >> 2));												// FXR + FXF = Fixed range + 4 upper bits of fixed frequency
			}
			setreg(0x40 + j + 0x08 * k, 0x80 | voice.aop[k].osw_fine);								// OW + FINE = Oscillator waveform + fine frequency tuning

			// WRONG setreg(0x60 + j + 0x08 * k, (135 - min(127, int(round(1.54 *  voice.op[k].out)))));		// TL = Operator output level
			setreg(0x60 + j + 0x08 * k, (tl_vmem_reg[voice.op[k].out]));							// TL = Operator output level
			setreg(0x80 + j + 0x08 * k, ((voice.op[k].rs_det & 0x18) << 3) |						// KRS + FIX + AR = Key rate scaling ...
			       ((voice.aop[k].egshft_fix_fixrg & 0x08) << 1) | (voice.op[k].ar & 0x1f) );		// ... + fix/ratio mode + operator attack rate
			setreg(0xa0 + j + 0x08 * k, (voice.op[k].ame_ebs_kvs & 0x80) | voice.op[k].d1r);		// AME + D1R = Amplitude modulation enable + Operator Decay 1 Rate
			setreg(0xc0 + j + 0x08 * k, ((voice.op[k].f & 0x03) << 6) | voice.op[k].d2r);			// DT2 + D2R = Detune 2 + Operator Decay 2 Rate
			setreg(0xc0 + j + 0x08 * k, ((voice.aop[k].egshft_fix_fixrg & 0x20) << 2) | 0x28 |		// EGS + REV = EG shift + 1 magic bit + ...
			       voice.rev);																		// ... + reverb rate
			setreg(0xe0 + j + 0x08 * k, ((15 - voice.op[k].d1l) << 4) | voice.op[k].rr);			// D1L + RR = Operator Decay 1 Level + Release Rate
		}
	}
}


void set_note(uint8_t channel, uint8_t midi_note)
{
	uint8_t octave;
	uint8_t note;
	uint8_t fraction = 0x0f;	// measured !
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
		/*
				for (uint8_t k = 0; k < 4; k++)
				{
					setreg(0xe0 + channel + 0x08 * k, 0xfe);					// D1L + RR
					setreg(0xc0 + channel + 0x08 * k, 0x28);					// EGS + REV
				}

				setreg(0x08, 0x00 | channel);

				for (uint8_t k = 0; k < 4; k++)
				{
					setreg(0x60 + channel + 0x08 * k, (tl_vmem_reg[voice.op[k].out]));							// TL = Operator output level
				}
		*/
		setreg(0x28 + channel, (octave << 4) | note);					// Set channel note
		setreg(0x30 + channel, (fraction << 2) | 0x01);					// Set channel note fraction + MONO bit=1
		/*				setreg(0x20 + channel, 0xc0 | (voice.sy_fbl_alg & 0x3f));		// R + UNK1 + Feedback + Algorithm, UNK1=1 when no output, =0 when playing note

						for (uint8_t k = 0; k < 4; k++)
						{
							setreg(0xe0 + channel + 0x08 * k, ((15 - voice.op[k].d1l) << 4) | voice.op[k].rr);			// D1L + RR = Operator Decay 1 Level + Release Rate
							setreg(0xc0 + channel + 0x08 * k, ((voice.aop[k].egshft_fix_fixrg & 0x20) << 2) | 0x28 |		// EGS + REV = EG shift + 1 magic bit + ...
							       voice.rev);																		// ... + reverb rate
						}

				setreg(0x20 + channel, 0x80 | (voice.sy_fbl_alg & 0x3f));		// R + UNK1 + Feedback + Algorithm, UNK1=1 when no output, =0 when playing note
		*/
		setreg(0x08, 0x78 | channel);									// Key ON for channel (all 4 OPs are running)
		//setreg(0x14, 0x40);												// TIMER
	}
}

void unset_note(uint8_t channel)
{
	/*
	for (uint8_t k = 0; k < 4; k++)
	{
		setreg(0xe0 + channel + 0x08 * k, 0xfe);					// D1L + RR
		setreg(0xc0 + channel + 0x08 * k, 0x28);					// EGS + REV
	}
	*/
	setreg(0x08, 0x00 | channel);
	//setreg(0x20 + channel, 0xc0 | (voice.sy_fbl_alg & 0x3f));			// R + UNK1 + Feedback + Algorithm, UNK1=1 when no output, =0 when playing note
}