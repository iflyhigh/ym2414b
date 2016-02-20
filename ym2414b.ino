// some code grabbed from YM2151 Library, see http://www.ooishoo.org/?page_id=15
// see http://sr4.sakura.ne.jp/fmsound/opz.html for YM2414B register analysis
#include "tables.h"

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
uint8_t yamaha2[256];

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

uint8_t pmd[100] =
{
	0,  0,  2,  3,  4,  5,  7,  8,  9, 11,
	12, 13, 14, 16, 17, 18, 20, 21, 22, 23,
	25, 26, 27, 29, 30, 31, 33, 34, 35, 36,
	38, 39, 40, 42, 43, 44, 45, 47, 48, 49,
	51, 52, 53, 54, 56, 57, 58, 60, 61, 62,
	63, 65, 66, 67, 69, 70, 71, 72, 74, 75,
	76, 78, 79, 80, 82, 83, 84, 85, 87, 88,
	89, 91, 92, 93, 94, 96, 97, 98, 100, 101,
	102, 103, 105, 106, 107, 109, 110, 111, 112, 114,
	115, 116, 118, 119, 120, 121, 123, 124, 125, 127
};

uint8_t amd[100] =
{
	0,  0,  0,  0,  0,  1,  1,  1,  1,  2,
	2,  2,  2,  3,  3,  3,  3,  4,  4,  4,
	5,  5,  5,  6,  6,  6,  7,  7,  7,  7,
	8,  8,  8,  9,  9,  9, 10, 10, 10, 11,
	11, 12, 12, 12, 13, 13, 14, 14, 15, 15,
	16, 16, 17, 17, 18, 18, 19, 19, 20, 20,
	21, 21, 22, 23, 23, 24, 25, 25, 26, 27,
	28, 28, 29, 30, 31, 32, 33, 34, 35, 36,
	37, 38, 40, 41, 42, 44, 45, 48, 49, 51,
	54, 56, 60, 62, 67, 70, 77, 86, 96, 127
};

uint8_t mul[64] =
{
	0, 0, 0, 0, 1, 1, 1, 1,
	2, 2, 3, 2, 2, 4, 3, 3,
	5, 3, 4, 6, 4, 4, 7, 5,
	5, 8, 6, 5, 9, 6, 7, 10,
	6, 7, 11, 8, 12, 7, 8, 9,
	13,	8, 14, 10, 9, 15, 9, 11,
	10,	12, 11, 10, 13, 12, 11, 14,
	13,	12, 15, 14, 13, 15, 14, 15
};

uint8_t dt2[64] =
{
	0, 1, 2, 3, 0, 1, 2, 3,
	0, 1, 0, 2, 3, 0, 1, 2,
	0, 3, 1, 0, 2, 3, 0, 1,
	2, 0, 1, 3, 0, 2, 1, 0,
	3, 2, 0, 1, 0, 3, 2, 1,
	0, 3, 0, 1, 2, 0, 3, 1,
	2, 1, 2, 3, 1, 2, 3, 1,
	2, 3, 1, 2, 3, 2, 3, 3
};


uint8_t tl(uint8_t op, uint8_t vmem_tl, uint8_t vmem_alg)
{
	/*
	* ALG OP1 OP2 OP3 OP4
	* -------------------
	* 0   a   a   a   a
	* 1   a   a   a   a
	* 2   a   a   a   a
	* 3   a   a   a   a
	* 4   b   a   b   a
	* 5   c   c   c   a
	* 6   c   c   c   a
	* 7   d   d   d   d
	*
	* mind the operator order in VMEM (4-2-3-1)!
	*/

	uint8_t *tl_alg[8][4] =
	{
		{basic_tl_a, basic_tl_a, basic_tl_a, basic_tl_a},
		{basic_tl_a, basic_tl_a, basic_tl_a, basic_tl_a},
		{basic_tl_a, basic_tl_a, basic_tl_a, basic_tl_a},
		{basic_tl_a, basic_tl_a, basic_tl_a, basic_tl_a},
		{basic_tl_a, basic_tl_a, basic_tl_b, basic_tl_b},
		{basic_tl_a, basic_tl_c, basic_tl_c, basic_tl_c},
		{basic_tl_a, basic_tl_c, basic_tl_c, basic_tl_c},
		{basic_tl_d, basic_tl_d, basic_tl_d, basic_tl_d}
	};

	if (vmem_tl < 20)
	{
		return tl_alg[vmem_alg][op][vmem_tl];
	}
	else
	{
		return tl_alg[vmem_alg][op][20] - (vmem_tl - 20);
	}
}

uint8_t lfo(uint8_t speed, uint8_t waveform)
{
	if (waveform == 3)
	{
		return lfo_sh[speed];
	}
	else
	{
		return lfo[speed];
	}
}

void wait(uint8_t loop)
{
	uint8_t wi;
	for (wi = 0; wi < loop; wi++)
	{
		// 16MHz  nop = @60nSec
		asm volatile("nop\n\t");
	}
}

static void setreg2(uint8_t reg, uint8_t data)
{
	setreg3(reg, data);
	yamaha2[reg] = data;
}

static void setreg(uint8_t reg, uint8_t data)
{
	setreg3(reg, data);
	yamaha[reg] = data;
}

static void setreg3(uint8_t reg, uint8_t data)
{
	uint8_t ym_busy = 1;

	YM_DATA_DDR = 0x00; 				// input mode for data bus pins
	YM_CTRL_PORT |= _BV(YM_A0);
	YM_CTRL_PORT |= _BV(YM_WR);

	while (ym_busy)
	{
		YM_CTRL_PORT &= ~_BV(YM_RD);	// RD low - read data
		wait(1);
		YM_CTRL_PORT &= ~_BV(YM_CS);	// CS low - chip select
		wait(3);
		ym_busy = (YM_DATA_PIN & _BV(7));
		YM_CTRL_PORT |= _BV(YM_CS);		// CS high - chip unselect
		YM_CTRL_PORT |= _BV(YM_RD);		// RD high - stop reading data
		wait(50);
	}

	ym_busy = 1;
	YM_DATA_DDR = 0xff;
	wait(50);

	YM_CTRL_PORT &= ~_BV(YM_A0); 	// A0 low - write register address
	YM_CTRL_PORT &= ~_BV(YM_WR);	// WR low - write data
	wait(1);
	YM_CTRL_PORT &= ~_BV(YM_CS);	// CS low - chip select
	YM_DATA_PORT = reg;				// register address
	wait(3);
	YM_CTRL_PORT |= _BV(YM_CS);		// CS high - chip unselect
	YM_CTRL_PORT |= _BV(YM_WR);		// WR high - data written
	YM_CTRL_PORT |= _BV(YM_A0);		// A0 high - write register data
	wait(60);
	YM_CTRL_PORT &= ~_BV(YM_WR);	// WR low - write data
	wait(1);
	YM_CTRL_PORT &= ~_BV(YM_CS);	// CS low - chip select
	YM_DATA_PORT = data;			// register data
	wait(3);
	YM_CTRL_PORT |= _BV(YM_CS);		// CS high - chip unselect
	YM_CTRL_PORT |= _BV(YM_WR);		// WR high - data written
	wait(60);
}

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

void load_patch(uint16_t i)
{
	unsigned char data[84] =
	{
		0x1F, 0x09, 0x09, 0x0F, 0x0C, 0x27, 0x02, 0x35, 0x37, 0x10, 0x1F, 0x04, 0x03, 0x04, 0x0D, 0x19,
		0x04, 0x56, 0x05, 0x16, 0x1F, 0x0F, 0x08, 0x0D, 0x0C, 0x00, 0x43, 0x60, 0x04, 0x0E, 0x1F, 0x0C,
		0x03, 0x07, 0x0E, 0x00, 0x43, 0x63, 0x05, 0x10, 0x3C, 0x14, 0x08, 0x02, 0x1B, 0x56, 0x05, 0x02,
		0x04, 0x00, 0x63, 0x32, 0x00, 0x00, 0x00, 0x32, 0x00, 0x4C, 0x6F, 0x54, 0x69, 0x6E, 0x65, 0x38,
		0x31, 0x5A, 0x20, 0x63, 0x63, 0x63, 0x32, 0x32, 0x32, 0x00, 0x7F, 0x00, 0x01, 0x00, 0x18, 0x00,
		0x01, 0x00, 0x00, 0x00
	};
	memcpy(&voice, data, 84);

	//setreg(0x16, lfo_vmem_reg[voice.lfs]);														// UNUSED !!! LFRQ2 = LFO2 Speed
	//setreg(0x17, amd_vmem_reg[voice.amd]);														// AMD2 = LFO2 Amplitude Modulation Depth
	//setreg(0x17, 0x80 | pmd_vmem_reg[voice.pmd]);													// PMD2 = LFO2 Pitch Modulation Depth
	setreg(0x18, lfo(voice.lfs, (voice.pms_ams_lfw & 0x03)));										// LFRQ1 = LFO1 Speed
	setreg(0x19, amd_vmem_reg[voice.amd]);															// AMD1 = LFO1 Amplitude Modulation Depth
	setreg2(0x19, 0x80 | pmd_vmem_reg[voice.pmd]);													// PMD1 = LFO1 Pitch Modulation Depth
	//setreg(0x1b, 0x00 | ((voice.sy_fbl_alg & 0x40) >> 1) | ((voice.sy_fbl_alg & 0x40) >> 2) |		// UNUSED !!! LFO2 Sync + LFO1 Sync ...
	//       ((voice.pms_ams_lfw & 0x03) << 2) | (voice.pms_ams_lfw & 0x03));						// UNUSED !!! ... + LFO2 Waveform + LFO1 Waveform
	setreg(0x1b, ((voice.sy_fbl_alg & 0x40) >> 2) | (voice.pms_ams_lfw & 0x03));					// LFO1 Sync + LFO1 Waveform

	for (uint8_t j = 0; j < 8; j++)																	// we support only single mode => iterate settings across all 8 channels
	{
		setreg(0x20 + j, 0xc0 | (voice.sy_fbl_alg & 0x3f));											// R + UNK1 + Feedback + Algorithm, UNK1=1 when no output, =0 when playing note
		setreg(0x38 + j, (voice.pms_ams_lfw & 0x70) | ((voice.pms_ams_lfw & 0x0c) >> 2));			// PMS1 + AMS1 = LFO1 Pitch/Amplitude Modulation Sensitivity
		//setreg(0x38 + j, 0x84 | (voice.pms_ams_lfw & 0x70) | ((voice.pms_ams_lfw & 0x0c) >> 2));	// UNUSED !!! PMS2 + AMS2 = LFO2 Pitch/Amplitude Modulation Sensitivity

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
				//setreg(0x40 + j + 0x08 * k, (dt1_local << 4) | ((voice.op[k].f & 0x3c) >> 2));		// DT1 + MUL = Detune 1 + 4 upper bits of CRS (coarse frequency)
				setreg(0x40 + j + 0x08 * k, (dt1_local << 4) | mul_vmem_reg[voice.op[k].f]);		// DT1 + MUL = Detune 1 + 4 upper bits of CRS (coarse frequency)
			}
			else																					// FIX mode
			{
				setreg(0x40 + j + 0x08 * k, ((voice.aop[k].egshft_fix_fixrg & 0x07) << 4) |
					   ((voice.op[k].f & 0x3c) >> 2));												// FXR + FXF = Fixed range + 4 upper bits of fixed frequency
			}
			setreg2(0x40 + j + 0x08 * k, 0x80 | voice.aop[k].osw_fine);								// OW + FINE = Oscillator waveform + fine frequency tuning
			setreg(0x60 + j + 0x08 * k, tl(k, voice.op[k].out, (voice.sy_fbl_alg & 0x07)));			// TL = Operator output level
			setreg(0x80 + j + 0x08 * k, ((voice.op[k].rs_det & 0x18) << 3) |						// KRS + FIX + AR = Key rate scaling ...
				   ((voice.aop[k].egshft_fix_fixrg & 0x08) << 1) | (voice.op[k].ar & 0x1f) );		// ... + fix/ratio mode + operator attack rate
			setreg(0xa0 + j + 0x08 * k, (voice.op[k].ame_ebs_kvs & 0x80) | voice.op[k].d1r);		// AME + D1R = Amplitude modulation enable + Operator Decay 1 Rate
			//setreg(0xc0 + j + 0x08 * k, ((voice.op[k].f & 0x03) << 6) | voice.op[k].d2r);			// DT2 + D2R = Detune 2 + Operator Decay 2 Rate
			setreg(0xc0 + j + 0x08 * k, (dt2_vmem_reg[voice.op[k].f] << 6) | voice.op[k].d2r);			// DT2 + D2R = Detune 2 + Operator Decay 2 Rate
			setreg2(0xc0 + j + 0x08 * k, ((voice.aop[k].egshft_fix_fixrg & 0x20) << 2) | 0x28 |		// EGS + REV = EG shift + 1 magic bit + ...
					voice.rev);																		// ... + reverb rate
			setreg(0xe0 + j + 0x08 * k, ((15 - voice.op[k].d1l) << 4) | voice.op[k].rr);			// D1L + RR = Operator Decay 1 Level + Release Rate
		}
	}
}

void set_note(uint8_t channel, uint8_t midi_note)
{
	uint8_t octave;
	uint8_t note;
	uint8_t fraction = 0xf1;	// measured !
	// For some reason YM2414B supports 8 ocataves (0-7) each starting with C# and ending with C
	// A4 = 440Hz according to application notes (2.1.2) && MIDI A4 = 440Hz => we support MIDI notes
	// from 1 to 96 inclusive and ignore others
	if ((midi_note > 0) && (midi_note < 97))						// ignore notes that are not supported by YM2151
	{
		octave = (midi_note - 1) / 12;
		note = (midi_note - 1) % 12;
		if (note > 2) {note++;};									// YM2414B note numbers in octave are 0,1,2,4,5,6,8,9,10,12,13,14
		if (note > 6) {note++;};									// Why?
		if (note > 10) {note++;};									// Because we can! :)

		for (uint8_t k = 0; k < 4; k++)
		{
			setreg(0x60 + channel + 0x08 * k, (tl(k, voice.op[k].out, (voice.sy_fbl_alg & 0x07))));							// TL = Operator output level
		}

		setreg(0x28 + channel, (octave << 4) | note);					// Set channel note
		setreg(0x30 + channel, (fraction << 2) | 0x01);					// Set channel note fraction + MONO bit=1
		setreg(0x08, 0x78 | channel);									// Key ON for channel (all 4 OPs are running)
	}
}

void unset_note(uint8_t channel)
{
	setreg(0x08, 0x00 | channel);
}