#include "Arduino.h"
#include <MIDI.h>

#define YM_CTRL_DDR DDRC
#define YM_CTRL_PORT PORTC
#define YM_DATA_DDR DDRD
#define YM_DATA_PORT PORTD
#define YM_DATA_PIN PIND


#define YM_CS (3) // PC2 (= pin A2 for Arduino UNO)
#define YM_RD (2) // PC2 (= pin A2 for Arduino UNO)
#define YM_WR (1) // PC1 (= pin A1 for Arduino UNO)
#define YM_A0 (0) // PC0 (= pin A0 for Arduino UNO)

//const int pinIC = 12;

struct 	op_t	// FM operator settings
{
	uint8_t	ar;
	uint8_t	d1r;
	uint8_t	d2r;
	uint8_t	rr;
	uint8_t	d1l;
	uint8_t	kls;
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

struct amem_t
{
	op_t	op[4];
	uint8_t	sy_fbl_alg;
	uint8_t	lfs;
	uint8_t	lfd;
	uint8_t	pmd;
	uint8_t	amd;
	uint8_t	pms_ams_lfw;
	uint8_t	transpose;
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

struct opz_rt_note_t
{
	uint8_t midi_note;
	uint8_t midi_velocity;
	uint8_t midi_volume;
	uint8_t microtuning;
};

extern amem_t amem;

void setreg(uint8_t reg, uint8_t data);
void init_voice(void);
uint8_t set_note(uint8_t channel, int16_t midi_note, uint8_t midi_velocity, uint8_t midi_volume, uint8_t micro_tuning);
void unset_note(uint8_t channel);
void modify_running_note(uint8_t channel, int16_t midi_note, uint8_t midi_velocity, uint8_t midi_volume);