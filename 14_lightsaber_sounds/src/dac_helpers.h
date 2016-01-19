// dac_helpers.h
// Function prototypes for i2s_helpers.c

#ifndef DACHELPERS_H_
#define DACHELPERS_H_

#define TEST_SOUND4	"sweet4.wav"
#define TEST_SOUND5 "sweet5.wav"
#define SABER_IDLE	"idle1.wav"
#define SABER_SWING	"swing0.wav"

void DMA_setup(void);
void DAC_setup(void);

void create_gpio_clock(void);
//void prepare_microsd_card(char *filename);
void open_file(char * filename);
void play_sound(char * filename);
void DAC_TIMER_setup();
void DAC_setup(void);
void add_track(char * filename);

typedef struct filename_data
{
	char filename[15];
} wav_files;

#endif
