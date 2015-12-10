// i2s_helpers.h
// Function prototypes for i2s_helpers.c

#ifndef I2SHELPERS_H_
#define I2SHELPERS_H_

void DMA_setup(void);
void I2S_init(void);
void I2S_setup(void);

void create_gpio_clock(void);
void prepare_microsd_card(char *filename);
void play_sound(const int track);
int get_next_track();

typedef struct filename_data
{
	char filename[15];
} wav_files;

#endif
