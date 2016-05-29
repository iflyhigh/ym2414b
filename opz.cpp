#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "opz.h"
#include "types.h"
#include "tables.h"

#define SET_NOTE_OK 1
#define SET_NOTE_NOT_IN_RANGE 0

amem_t amem;

void wait(uint8_t loop)
{
	uint8_t wi;
	for (wi = 0; wi < loop; wi++)
	{
		// 16MHz  nop = @60nSec
		//asm volatile("nop\n\t nop\n\t nop\n\t nop\n\t");
		asm volatile("nop\n\t");
	}
}

void setreg(uint8_t reg, uint8_t data)
{
	uint8_t ym_busy = 1;

	DDRD &= B00000011;					// input mode for data bus pins
	DDRB &= B11111100;					// input mode for data bus pins

	wait(4);							// some time is needed to setup

	while (ym_busy > 0)
	{
		YM_CTRL_PORT &= ~_BV(YM_RD);	// RD low - read data
		YM_CTRL_PORT &= ~_BV(YM_CS);	// CS low - chip select
		wait(3);						// minimal delay 3
		ym_busy = (PIND & _BV(7));
		YM_CTRL_PORT |= _BV(YM_CS) | _BV(YM_RD);;		// CS high - chip unselect, RD high - stop reading data
		wait(15);
	}
	ym_busy = 1;

	DDRD |= B11111100;				// output mode for data bus pins
	DDRB |= B00000011;				// output mode for data bus pins

	YM_CTRL_PORT &= ~_BV(YM_A0) & ~_BV(YM_WR); 	// A0 low - write register address, WR low - write data
	PORTD = ((reg & B11111100) | B00000011);
	PORTB = ((reg & B00000011) | B11111100);
	YM_CTRL_PORT &= ~_BV(YM_CS);	// CS low - chip select

	wait(3);						// do as tx81z

	YM_CTRL_PORT |= _BV(YM_CS);		// CS high - chip unselect
	YM_CTRL_PORT |= _BV(YM_WR) | _BV(YM_A0); //  WR high - data written, A0 high - write register data

	wait(11); 						// minimal delay 7

	YM_CTRL_PORT &= ~_BV(YM_WR);	// WR low - write data
	PORTD = ((data & B11111100) | B00000011);
	PORTB = ((data & B00000011) | B11111100);
	YM_CTRL_PORT &= ~_BV(YM_CS);	// CS low - chip select

	wait(3);						// do as tx81z

	YM_CTRL_PORT |= _BV(YM_CS);		// CS high - chip unselect
	YM_CTRL_PORT |= _BV(YM_WR);		// WR high - data written

	wait(24);
}

uint8_t tl(uint8_t op, uint8_t vmem_tl, uint8_t vmem_alg, uint8_t vmem_kvs, uint8_t vmem_kls, uint8_t seq_note, uint8_t seq_velocity, uint8_t midi_volume)
{
	uint16_t _tl;
	uint8_t _kvs = 0;
	uint8_t _kls = 0;

	if (vmem_tl < 21)
	{
		_tl = basic_tl[vmem_tl] + tl_alg[vmem_alg][op] + (tl_vol[vmem_alg][op] ? pgm_read_byte(&volume_tl[midi_volume]) : 0);
	}
	else
	{
		_tl = basic_tl[20] + tl_alg[vmem_alg][op] - vmem_tl + 20 + (tl_vol[vmem_alg][op] ? pgm_read_byte(&volume_tl[midi_volume]) : 0);
	}

	if (vmem_kvs > 0)
	{
		_kvs = pgm_read_byte(&kvs[vmem_kvs - 1][127 - seq_velocity]);
		if ((seq_note > 20) && (vmem_kls > 0))
		{
			_kls = pgm_read_byte(&kls_kvs17[vmem_kls - 1][(seq_note / 3) - 1]);
		}
	}
	else
	{
		if ((seq_note > 17) && (vmem_kls > 0))
		{
			_kls = pgm_read_byte(&kls_kvs0[vmem_kls - 1][(seq_note / 3) - 1]);
		}
	}

	_tl = _tl + _kvs + _kls;

	if (_tl > 127) { _tl = 127; }

	return (uint8_t)_tl;

}

uint8_t lfo(uint8_t speed, uint8_t waveform)
{
	if (waveform == 3)
	{
		return pgm_read_byte(&lfo_sh[speed]);
	}
	else
	{
		return pgm_read_byte(&lfo_other[speed]);
	}
}

void init_voice()
{
	/*
		unsigned char data[84] =
		{
			0x19, 0x0F, 0x0F, 0x07, 0x0F, 0x05, 0x00, 0x62, 0x0D, 0x00, 0x19, 0x00, 0x0F, 0x07, 0x0F, 0x05,
			0x01, 0x63, 0x04, 0x00, 0x19, 0x00, 0x0F, 0x07, 0x0F, 0x05, 0x02, 0x63, 0x0D, 0x06, 0x1F, 0x00,
			0x06, 0x02, 0x0F, 0x05, 0x01, 0x63, 0x04, 0x06, 0x47, 0x23, 0x00, 0x4B, 0x00, 0x71, 0x1E, 0x0C,
			0x04, 0x00, 0x63, 0x32, 0x00, 0x00, 0x00, 0x32, 0x00, 0x53, 0x70, 0x61, 0x63, 0x65, 0x20, 0x56,
			0x69, 0x62, 0x65, 0x63, 0x63, 0x63, 0x32, 0x32, 0x32, 0x00, 0x10, 0x00, 0x00, 0x00, 0x50, 0x00,
			0x50, 0x00, 0x00, 0x00
		};
		memcpy(&amem, data, 84);
	*/
	setreg(0x16, 0x00);																				// UNUSED !!! LFRQ2 = LFO2 Speed
	setreg(0x17, 0x00);																				// AMD2 = LFO2 Amplitude Modulation Depth
	setreg(0x17, 0x80);																				// PMD2 = LFO2 Pitch Modulation Depth
	setreg(0x18, lfo(amem.lfs, (amem.pms_ams_lfw & 0x03)));										// LFRQ1 = LFO1 Speed
	setreg(0x19, pgm_read_byte(&amd[amem.amd]));													// AMD1 = LFO1 Amplitude Modulation Depth
	setreg(0x19, 0x80 | pgm_read_byte(&pmd[amem.pmd]));											// PMD1 = LFO1 Pitch Modulation Depth
	//setreg(0x1b, 0x00 | ((amem.sy_fbl_alg & 0x40) >> 1) | ((amem.sy_fbl_alg & 0x40) >> 2) |		// UNUSED !!! LFO2 Sync + LFO1 Sync ...
	//       ((amem.pms_ams_lfw & 0x03) << 2) | (amem.pms_ams_lfw & 0x03));						// UNUSED !!! ... + LFO2 Waveform + LFO1 Waveform
	setreg(0x1b, ((amem.sy_fbl_alg & 0x40) >> 2) | (amem.pms_ams_lfw & 0x03));					// LFO1 Sync + LFO1 Waveform

	for (uint8_t j = 0; j < 8; j++)																	// we support only single mode => iterate settings across all 8 channels
	{
		setreg(0x20 + j, 0xc0 | (amem.sy_fbl_alg & 0x3f));											// R + UNK1 + Feedback + Algorithm, UNK1=1 when no output, =0 when playing note
		setreg(0x38 + j, (amem.pms_ams_lfw & 0x70) | ((amem.pms_ams_lfw & 0x0c) >> 2));			// PMS1 + AMS1 = LFO1 Pitch/Amplitude Modulation Sensitivity
		//setreg(0x38 + j, 0x84 | (amem.pms_ams_lfw & 0x70) | ((amem.pms_ams_lfw & 0x0c) >> 2));	// UNUSED !!! PMS2 + AMS2 = LFO2 Pitch/Amplitude Modulation Sensitivity

		for (uint8_t k = 0; k < 4; k++)																// iterate across operators
		{
			if ((amem.aop[k].egshft_fix_fixrg & 0x08) == 0)										// RATIO mode
			{
				uint8_t dt1_local;

				if ((amem.op[k].rs_det & 0x07) >= 3)
				{
					dt1_local = ((amem.op[k].rs_det & 0x07) - 3);
				}
				else
				{
					dt1_local = (7 - (amem.op[k].rs_det & 0x07));
				}
				setreg(0x40 + j + 0x08 * k, (dt1_local << 4) | pgm_read_byte(&mul[amem.op[k].f]));		// DT1 + MUL = Detune 1 + table func
			}
			else																						// FIX mode
			{
				setreg(0x40 + j + 0x08 * k, ((amem.aop[k].egshft_fix_fixrg & 0x07) << 4) |
				       ((amem.op[k].f & 0x3c) >> 2));													// FXR + FXF = Fixed range + 4 upper bits of fixed frequency
			}
			setreg(0x40 + j + 0x08 * k, 0x80 | amem.aop[k].osw_fine);									// OW + FINE = Oscillator waveform + fine frequency tuning
			setreg(0x80 + j + 0x08 * k, ((amem.op[k].rs_det & 0x18) << 3) |							// KRS + FIX + AR = Key rate scaling ...
			       ((amem.aop[k].egshft_fix_fixrg & 0x08) << 2) | (amem.op[k].ar & 0x1f) );			// ... + fix/ratio mode + operator attack rate
			setreg(0xa0 + j + 0x08 * k, ((amem.op[k].ame_ebs_kvs & 0x40) << 1) | amem.op[k].d1r);		// AME + D1R = Amplitude modulation enable + Operator Decay 1 Rate
			setreg(0xc0 + j + 0x08 * k, (pgm_read_byte(&dt2[amem.op[k].f]) << 6) | amem.op[k].d2r);	// DT2 + D2R = Detune 2 + Operator Decay 2 Rate
			setreg(0xc0 + j + 0x08 * k, ((amem.aop[k].egshft_fix_fixrg & 0x20) << 2) | 0x28 |			// EGS + REV = EG shift + 1 magic bit + ...
			       amem.rev);																			// ... + reverb rate
			setreg(0xe0 + j + 0x08 * k, ((15 - amem.op[k].d1l) << 4) | amem.op[k].rr);				// D1L + RR = Operator Decay 1 Level + Release Rate
		}
	}
}

uint8_t set_note(uint8_t channel, int16_t midi_note, uint8_t midi_velocity, uint8_t midi_volume, uint8_t micro_tuning)
{
	uint8_t opz_octave;
	uint8_t opz_note;
	uint8_t opz_fraction = 0;

	midi_note = midi_note + (amem.transpose - 24) - 12;	// TRPS = transpose according to middle C, shift by 12 down for simpler calculations

	if (((midi_note % 12) > 0) && (midi_note > 0))
	{
		if (note_tuning[micro_tuning][(midi_note % 12)] < 0)
		{
			opz_fraction = 63 + note_tuning[micro_tuning][(midi_note % 12)];
			midi_note--;
		}
		else
		{
			opz_fraction = note_tuning[micro_tuning][(midi_note % 12)];
		}
	}

	if ((midi_note > 0) && (midi_note < 97))				// ignore notes that are not supported by YM2414B. TX81z wraps notes, we'll do better
	{
		opz_octave = (midi_note - 1) / 12;
		opz_note = (midi_note - 1) % 12;
		if (opz_note > 2) { opz_note++; }									// YM2414B note numbers in octave are 0,1,2,4,5,6,8,9,10,12,13,14
		if (opz_note > 6) { opz_note++; }
		if (opz_note > 10) { opz_note++; }

		for (uint8_t k = 0; k < 4; k++)
		{
			setreg(0x60 + channel + 0x08 * k, ( tl(k, amem.op[k].out, (amem.sy_fbl_alg & 0x07), 		// TL = Operator output level
			                                       (amem.op[k].ame_ebs_kvs & 0x07), amem.op[k].kls, midi_note, midi_velocity, midi_volume)));
		}

		setreg(0x28 + channel, (opz_octave << 4) | opz_note);				// Set channel note
		setreg(0x30 + channel, (opz_fraction << 2) | 0x01);					// Set channel note fraction + MONO bit=1
		setreg(0x08, 0x78 | channel);										// Key ON for channel (all 4 OPs are running)
		setreg(0x20 + channel, 0x80 | (amem.sy_fbl_alg & 0x3f));					// R + UNK1 + Feedback + Algorithm, UNK1=1 when no output, =0 when playing note
		setreg(0x1b, ((amem.sy_fbl_alg & 0x40) >> 2) | (amem.pms_ams_lfw & 0x03));					// LFO1 Sync + LFO1 Waveform

		return SET_NOTE_OK;
	}
	else
	{
		return SET_NOTE_NOT_IN_RANGE;
	}
}

void unset_note(uint8_t channel)
{
	setreg(0x20 + channel, 0xc0 | (amem.sy_fbl_alg & 0x3f));						// R + UNK1 + Feedback + Algorithm, UNK1=1 when no output, =0 when playing note
	setreg(0x08, 0x00 | channel);
}

void modify_running_note(uint8_t channel, int16_t midi_note, uint8_t midi_velocity, uint8_t midi_volume)
{
	for (uint8_t k = 0; k < 4; k++)
	{
		setreg(0x60 + channel + 0x08 * k, ( tl(k, amem.op[k].out, (amem.sy_fbl_alg & 0x07), 		// TL = Operator output level
		                                       (amem.op[k].ame_ebs_kvs & 0x07), amem.op[k].kls, midi_note, midi_velocity, midi_volume)));
	}
}