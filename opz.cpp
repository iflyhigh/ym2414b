#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "opz.h"
#include "types.h"
#include "tables.h"

voice_t voice;

void wait(uint8_t loop)
{
	uint8_t wi;
	for (wi = 0; wi < loop; wi++)
	{
		// 16MHz  nop = @60nSec
		asm volatile("nop\n\t");
	}
}

void setreg(uint8_t reg, uint8_t data)
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

uint8_t tl(uint8_t op, uint8_t vmem_tl, uint8_t vmem_alg, uint8_t vmem_kvs, uint8_t vmem_kls, uint8_t opz_note, uint8_t midi_velocity)
{
	uint16_t _tl;
	uint8_t _kvs = 0;
	uint8_t _kls = 0;
	uint8_t _note = 0;

	if (vmem_tl < 21)
	{
		_tl = tl_alg[vmem_alg][op][vmem_tl];
	}
	else
	{
		_tl = tl_alg[vmem_alg][op][20] - (vmem_tl - 20);
	}

	if (vmem_kls > 0)
	{
		uint8_t _opz_note;
		_opz_note = (opz_note & 0x0f);
		if (_opz_note > 10) { _opz_note--; }
		if (_opz_note > 7) { _opz_note--; }
		_note = (opz_note >> 4) * 10 + _opz_note;
	}

	if (vmem_kvs > 0)
	{
		_kvs = pgm_read_byte(kvs[vmem_kvs][127 - midi_velocity]);
		if ((_note > 0) && (vmem_kls > 0))
		{
			_kls = pgm_read_byte(kls_kvs17[vmem_kls][_note]);
		}
	}
	else
	{
		// _kvs = 0
		if ((_note > 0) && (vmem_kls > 0))
		{
			_kls = pgm_read_byte(kls_kvs0[vmem_kls][_note]);
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
		return pgm_read_byte(lfo_sh[speed]);
	}
	else
	{
		return pgm_read_byte(lfo_other[speed]);
	}
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
	setreg(0x19, pgm_read_byte(amd[voice.amd]));													// AMD1 = LFO1 Amplitude Modulation Depth
	setreg(0x19, 0x80 | pgm_read_byte(pmd[voice.pmd]));												// PMD1 = LFO1 Pitch Modulation Depth
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
				setreg(0x40 + j + 0x08 * k, (dt1_local << 4) | pgm_read_byte(mul[voice.op[k].f]));		// DT1 + MUL = Detune 1 + 4 upper bits of CRS (coarse frequency)
			}
			else																						// FIX mode
			{
				setreg(0x40 + j + 0x08 * k, ((voice.aop[k].egshft_fix_fixrg & 0x07) << 4) |
					   ((voice.op[k].f & 0x3c) >> 2));													// FXR + FXF = Fixed range + 4 upper bits of fixed frequency
			}
			setreg(0x40 + j + 0x08 * k, 0x80 | voice.aop[k].osw_fine);									// OW + FINE = Oscillator waveform + fine frequency tuning
			setreg(0x80 + j + 0x08 * k, ((voice.op[k].rs_det & 0x18) << 3) |							// KRS + FIX + AR = Key rate scaling ...
				   ((voice.aop[k].egshft_fix_fixrg & 0x08) << 1) | (voice.op[k].ar & 0x1f) );			// ... + fix/ratio mode + operator attack rate
			setreg(0xa0 + j + 0x08 * k, (voice.op[k].ame_ebs_kvs & 0x80) | voice.op[k].d1r);			// AME + D1R = Amplitude modulation enable + Operator Decay 1 Rate
			setreg(0xc0 + j + 0x08 * k, (pgm_read_byte(dt2[voice.op[k].f]) << 6) | voice.op[k].d2r);	// DT2 + D2R = Detune 2 + Operator Decay 2 Rate
			setreg(0xc0 + j + 0x08 * k, ((voice.aop[k].egshft_fix_fixrg & 0x20) << 2) | 0x28 |			// EGS + REV = EG shift + 1 magic bit + ...
				   voice.rev);																			// ... + reverb rate
			setreg(0xe0 + j + 0x08 * k, ((15 - voice.op[k].d1l) << 4) | voice.op[k].rr);				// D1L + RR = Operator Decay 1 Level + Release Rate
		}
	}
}

void set_note(uint8_t channel, uint8_t midi_note, uint8_t midi_velocity)
{
	uint8_t opz_octave;
	uint8_t opz_note;
	uint8_t opz_fraction = 0xf1;	// measured !
	// For some reason YM2414B supports 8 ocataves (0-7) each starting with C# and ending with C
	// A4 = 440Hz according to application notes (2.1.2) && MIDI A4 = 440Hz => we support MIDI notes
	// from 1 to 96 inclusive and ignore others
	if ((midi_note > 0) && (midi_note < 97))						// ignore notes that are not supported by YM2151
	{
		opz_octave = (midi_note - 1) / 12;
		opz_note = (midi_note - 1) % 12;
		if (opz_note > 2) {opz_note++;};									// YM2414B note numbers in octave are 0,1,2,4,5,6,8,9,10,12,13,14
		if (opz_note > 6) {opz_note++;};									// Why?
		if (opz_note > 10) {opz_note++;};									// Because we can! :)

		for (uint8_t k = 0; k < 4; k++)
		{
			setreg(0x60 + channel + 0x08 * k, ( tl(k, voice.op[k].out, (voice.sy_fbl_alg & 0x07), 		// TL = Operator output level
												   (voice.op[k].ame_ebs_kvs & 0x07), voice.op[k].kls, opz_note, midi_velocity)));
		}

		setreg(0x28 + channel, (opz_octave << 4) | opz_note);				// Set channel note
		setreg(0x30 + channel, (opz_fraction << 2) | 0x01);					// Set channel note fraction + MONO bit=1
		setreg(0x08, 0x78 | channel);										// Key ON for channel (all 4 OPs are running)
	}
}

void unset_note(uint8_t channel)
{
	setreg(0x08, 0x00 | channel);
}

