// dac_helpers.h
// Function prototypes for i2s_helpers.c

#ifndef DACHELPERS_H_
#define DACHELPERS_H_

void DMA_setup(void);
void DAC_setup(void);

void create_gpio_clock(void);
void prepare_microsd_card(char *filename);
void play_sound(const int track);
int get_next_track();
void DAC_TIMER_setup();
void DAC_setup(void);

typedef struct filename_data
{
	char filename[15];
} wav_files;

#endif
