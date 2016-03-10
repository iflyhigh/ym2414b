#include "Arduino.h"

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

void setreg(uint8_t reg, uint8_t data);
void load_patch(uint16_t i);
uint8_t set_note(uint8_t channel, int16_t midi_note, uint8_t midi_velocity, uint8_t midi_volume, uint8_t micro_tuning);
uint8_t set_note2(uint8_t channel, int16_t midi_note, uint8_t midi_velocity);
void unset_note(uint8_t channel);
