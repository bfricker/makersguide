// i2s_helpers.c
// Implementation file for I2S helper functions

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>

#include "em_device.h"
#include "em_common.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_dac.h"
#include "em_prs.h"
#include "em_timer.h"
#include "em_dma.h"
#include "em_usart.h"
#include "dmactrl.h"
#include "ff.h"
#include "microsd.h"
#include "diskio.h"
#include "bsp.h"
#include "utilities.h"
#include "i2s_helpers.h"

#define MCLK_PORT		gpioPortD
#define MCLK_PIN		7

/** Remove this #define if you want to use DAC0 instead of the I2S dac. */
#define USE_I2S

/** Filename to open from SD-card */
//#define WAV_FILENAME             "sweet3.wav"

/** Ram buffers
 * BUFFERSIZE should be between 512 and 1024, depending on available ram on efm32
 */
#define BUFFERSIZE               512

/** DMA callback structure */
static DMA_CB_TypeDef DMAcallBack;

/* Temporary buffer for use when source is mono, can't put samples directly in
 * stereo DMA buffer with f_read(). */
static int16_t ramBufferTemporaryMono[BUFFERSIZE];

/* Buffers for DMA transfer, 32 bits are transfered at a time with DMA.
 * The buffers are twice as large as BUFFERSIZE to hold both left and right
 * channel samples. */
static int16_t ramBufferDacData0Stereo[2 * BUFFERSIZE];
static int16_t ramBufferDacData1Stereo[2 * BUFFERSIZE];

/** Bytecounter, need to stop DMA when finished reading file */
static uint32_t ByteCounter = 0;

/** File system specific */
static FATFS Fatfs;

/** File to read WAV audio data from */
static FIL WAVfile;

/** WAV header structure */
typedef struct
{
  uint8_t  id[4];                   /** should always contain "RIFF"      */
  uint32_t totallength;             /** total file length minus 8         */
  uint8_t  wavefmt[8];              /** should be "WAVEfmt "              */
  uint32_t format;                  /** Sample format. 16 for PCM format. */
  uint16_t pcm;                     /** 1 for PCM format                  */
  uint16_t channels;                /** Channels                          */
  uint32_t frequency;               /** sampling frequency                */
  uint32_t bytes_per_second;        /** Bytes per second                  */
  uint16_t bytes_per_capture;       /** Bytes per capture                 */
  uint16_t bits_per_sample;         /** Bits per sample                   */
  uint8_t  data[4];                 /** should always contain "data"      */
  uint32_t bytes_in_data;           /** No. bytes in data                 */
} WAV_Header_TypeDef;

/** Wav header. Global as it is used in callbacks. */
static WAV_Header_TypeDef wavHeader;

bool need_to_rewind = true;
bool add_dead_time = true;
bool send_zeros = true;

UINT    bytes_read = 0;

#define MAX_TRACKS 3
wav_files file_array[MAX_TRACKS] = {{"sweet4.wav"}, {"sweet5.wav"}, {"sweet6.wav"}};

/***************************************************************************//**
 * @brief
 *   Initialize MicroSD driver.
 * @return
 *   Returns 0 if initialization succeded, non-zero otherwise.
 ******************************************************************************/
int initFatFS(void)
{
  MICROSD_Init();
  if (f_mount(0, &Fatfs) != FR_OK)
    return -1;
  return 0;
}

/***************************************************************************//**
 * @brief
 *   This function is required by the FAT file system in order to provide
 *   timestamps for created files. Since we do not have a reliable clock we
 *   hardcode a value here.
 *
 *   Refer to reptile/fatfs/doc/en/fattime.html for the format of this DWORD.
 * @return
 *    A DWORD containing the current time and date as a packed datastructure.
 ******************************************************************************/
DWORD get_fattime(void)
{
  return (28 << 25) | (2 << 21) | (1 << 16);
}

/**************************************************************************//**
 * @brief
 *   This function fills up the memory buffers with data from SD card.
 * @param stereo
 *   Input is in stereo.
 * @param primary
 *   Fill primary or alternate DMA buffer with data.
 *****************************************************************************/
void FillBufferFromSDcard(bool stereo, bool primary)
{
  UINT     bytes_read;
  int16_t  * buffer;
  int      i, j;
  uint16_t tmp;

  /* Set buffer pointer correct ram buffer */
  if (primary)
  {
    buffer = ramBufferDacData0Stereo;
  }
  else /* Alternate */
  {
    buffer = ramBufferDacData1Stereo;
  }

#define FUDGE 9000
  if (need_to_rewind && ByteCounter > FUDGE)
  {
	  f_lseek(&WAVfile, f_tell(&WAVfile) - FUDGE);
	  need_to_rewind = false;
	  ByteCounter -= FUDGE;
  }

  if (stereo)
  {
    /* Stereo, Store Left and Right data interlaced as in wavfile */
    /* DMA is writing the data to the combined register as interlaced data*/

    /* First buffer is filled from SD-card */
    f_read(&WAVfile, buffer, 4 * BUFFERSIZE, &bytes_read);
    ByteCounter += bytes_read;

    for (i = 0; i < 2 * BUFFERSIZE; i++)
    {
//      /* Adjust volume */
//      buffer[i] = (buffer[i] * (int32_t) volumeAdjustFactor) / 100;

#ifndef USE_I2S
      tmp = buffer[i];

      /* Convert from signed to unsigned */
      tmp = buffer[i] + 0x8000;

      /* Convert to 12 bits */
      tmp >>= 4;

      buffer[i] = tmp;
#endif
    }
  }
  else /* Mono */
  {
    /* Read data into temporary buffer. */
	f_read(&WAVfile, ramBufferTemporaryMono, BUFFERSIZE, &bytes_read);
    ByteCounter += bytes_read;

    j = 0;
    for (i = 0; i < (2 * BUFFERSIZE) - 1; i += 2)
    {
//      /* Adjust volume */
//      ramBufferTemporaryMono[j] = (ramBufferTemporaryMono[j] *
//                                   (int32_t) volumeAdjustFactor) / 100;

      tmp = ramBufferTemporaryMono[j];

#ifndef USE_I2S
      /* Convert from signed to unsigned */
      tmp = ramBufferTemporaryMono[j] + 0x8000;

      /* Convert to 12 bits */
      tmp >>= 4;
#endif

	  /* Make sample 12 bits and unsigned */
	  buffer[ i     ] = tmp;
	  buffer[ i + 1 ] = tmp;
      j++;
    }
  }
}

/**************************************************************************//**
 * @brief
 *   Callback function called when the DMA finishes a transfer.
 * @param channel
 *   The DMA channel that finished.
 * @param primary
 *   Primary or Alternate DMA descriptor
 * @param user
 *   User defined pointer (Not used in this example.)
 *****************************************************************************/
void PingPongTransferComplete(unsigned int channel, bool primary, void *user)
{
  (void) channel;              /* Unused parameter */
  (void) user;                 /* Unused parameter */

  FillBufferFromSDcard((bool) wavHeader.channels, primary);

  if ( DMA->IF & DMA_IF_CH0DONE )           /* Did a DMA complete while   */
  {                                         /* reading from the SD Card ? */
    /* If FillBufferFromSDcard() takes too much time, we need to restart  */
    /* the pingpong machinery. This results in an audible click, which is */
    /* acceptable once in a while...                                      */
    DMA->IFC = DMA_IFC_CH0DONE;
    DMA_ActivatePingPong( 0,
                          false,
#ifdef USE_I2S
                          (void *) & (USART1->TXDOUBLE),
                          (void *) &ramBufferDacData0Stereo,
                          (2 * BUFFERSIZE) - 1,
                          (void *) &(USART1->TXDOUBLE),
                          (void *) &ramBufferDacData1Stereo,
                          (2 * BUFFERSIZE) - 1);
#else
                          (void *) &(DAC0->COMBDATA),
                          (void *) &ramBufferDacData0Stereo,
                          BUFFERSIZE - 1,
                          (void *) &(DAC0->COMBDATA),
                          (void *) &ramBufferDacData1Stereo,
                          BUFFERSIZE - 1);
#endif
    return;
  }

  /* Stop DMA if bytecounter is equal to datasize or larger */
  bool stop = false;

  if (ByteCounter >= wavHeader.bytes_in_data)
  {
	stop = true;
	f_close(&WAVfile);
  }

  /* Refresh the DMA control structure */
  DMA_RefreshPingPong(0,
                      primary,
                      false,
                      NULL,
                      NULL,
#ifdef USE_I2S
                      (2 * BUFFERSIZE) - 1,
#else
                      BUFFERSIZE - 1,
#endif
                      stop);
}

#ifndef USE_I2S
/**************************************************************************//**
 * @brief
 *   DAC Setup.
 * @details
 *   Setup DAC in stereo mode and triggered by PRS.
 *****************************************************************************/
void DAC_setup(void)
{
  DAC_Init_TypeDef        init        = DAC_INIT_DEFAULT;
  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;

  /* Calculate the DAC clock prescaler value that will result in a DAC clock
   * close to 1 MHz. Second parameter is zero, if the HFPERCLK value is 0, the
   * function will check what the HFPERCLK actually is. */
  init.prescale = DAC_PrescaleCalc(1000000, 0);

  /* Initialize the DAC. */
  DAC_Init(DAC0, &init);

  /* Enable prs to trigger samples at the right time with the timer */
  initChannel.prsEnable = true;
  initChannel.prsSel    = dacPRSSELCh0;

  /* Both channels can be configured the same
   * and be triggered by the same prs-signal. */
  DAC_InitChannel(DAC0, &initChannel, 0);
  DAC_InitChannel(DAC0, &initChannel, 1);

  DAC_Enable(DAC0, 0, true);
  DAC_Enable(DAC0, 1, true);
}
#endif

#ifndef USE_I2S
/**************************************************************************//**
 * @brief
 *   Setup TIMER for prs triggering of DAC conversion
 * @details
 *   Timer is set up to tick at the same frequency as the frequency described
 *   in the global .wav header. This will also cause a PRS trigger.
 *****************************************************************************/
void TIMER_setup(void)
{
  uint32_t timerTopValue;
  /* Use default timer configuration, overflow on counter top and start counting
   * from 0 again. */
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

  TIMER_Init(TIMER0, &timerInit);

  /* PRS setup */
  /* Select TIMER0 as source and TIMER0OF (Timer0 overflow) as signal (rising edge) */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER0, PRS_CH_CTRL_SIGSEL_TIMER0OF, prsEdgePos);

  /* Calculate the proper overflow value */
  timerTopValue = CMU_ClockFreqGet(cmuClock_TIMER0) / wavHeader.frequency;

  /* Write new topValue */
  TIMER_TopBufSet(TIMER0, timerTopValue);
}
#endif

/**************************************************************************//**
 * @brief
 *   Setup DMA in ping pong mode
 * @details
 *   The DMA is set up to transfer data from memory to the DAC, triggered by
 *   PRS (which in turn is triggered by the TIMER). When the DMA finishes,
 *   it will trigger the callback (PingPongTransferComplete).
 *****************************************************************************/
void DMA_setup(void)
{
  /* DMA configuration structs */
  DMA_Init_TypeDef       dmaInit;
  DMA_CfgChannel_TypeDef chnlCfg;
  DMA_CfgDescr_TypeDef   descrCfg;

  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);

  /* Set the interrupt callback routine */
  DMAcallBack.cbFunc = PingPongTransferComplete;

  /* Callback doesn't need userpointer */
  DMAcallBack.userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri   = false; /* Can't use with peripherals */
  chnlCfg.enableInt = true;  /* Interrupt needed when buffers are used */

  /* channel 0 and 1 will need data at the same time,
   * can use channel 0 as trigger */

#ifdef USE_I2S
  chnlCfg.select = DMAREQ_USART1_TXBL;
#else
  chnlCfg.select = DMAREQ_DAC0_CH0;
#endif

  chnlCfg.cb = &DMAcallBack;
  DMA_CfgChannel(0, &chnlCfg);

  /* Setting up channel descriptor */
  /* Destination is DAC/USART register and doesn't move */
  descrCfg.dstInc = dmaDataIncNone;

  /* Transfer 32/16 bit each time to DAC_COMBDATA/USART_TXDOUBLE register*/
#ifdef USE_I2S
  descrCfg.srcInc = dmaDataInc2;
  descrCfg.size   = dmaDataSize2;
#else
  descrCfg.srcInc = dmaDataInc4;
  descrCfg.size   = dmaDataSize4;
#endif

  /* We have time to arbitrate again for each sample */
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;

  /* Configure both primary and secondary descriptor alike */
  DMA_CfgDescr(0, true, &descrCfg);
  DMA_CfgDescr(0, false, &descrCfg);

  /* Enabling PingPong Transfer*/
  DMA_ActivatePingPong(0,
                       false,
#ifdef USE_I2S
                       (void *) & (USART1->TXDOUBLE),
                       (void *) &ramBufferDacData0Stereo,
                       (2 * BUFFERSIZE) - 1,
                       (void *) &(USART1->TXDOUBLE),
                       (void *) &ramBufferDacData1Stereo,
                       (2 * BUFFERSIZE) - 1);
#else
                       (void *) &(DAC0->COMBDATA),
                       (void *) &ramBufferDacData0Stereo,
                       BUFFERSIZE - 1,
                       (void *) &(DAC0->COMBDATA),
                       (void *) &ramBufferDacData1Stereo,
                       BUFFERSIZE - 1);
#endif
}

#ifdef USE_I2S

void I2S_init(void)
{
	CMU_ClockEnable(cmuClock_USART1, true);

	/* Use location 1: TX  - Pin D0, (RX - Pin D1) */
	/*                 CLK - Pin D2, CS - Pin D3   */

	GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);
}
/**************************************************************************//**
 * @brief
 *   Setup USART1 in I2S mode.
 * @details
 *   USART1 is initialized in I2S mode to feed the DS1334 I2S dac.
 *   Baudrate is set based on information from the WAV file header.
 *****************************************************************************/
void I2S_setup(void)
{
  USART_InitI2s_TypeDef init = USART_INITI2S_DEFAULT;

  /* Configure USART for basic I2S operation */
  //init.sync.baudrate = 23438 * 16; //24000 * 16;
  init.sync.baudrate = wavHeader.frequency * 16 * wavHeader.channels * 0.976583333;

  USART_InitI2s(USART1, &init);

  /* Enable pins at location 1 */
  USART1->ROUTE = USART_ROUTE_TXPEN |
                  USART_ROUTE_CSPEN |
                  USART_ROUTE_CLKPEN |
                  USART_ROUTE_LOCATION_LOC1;

  // Reset this every time
  need_to_rewind = true;

}
#endif

/**************************************************************************//**
 * @brief
 *   Toggle GPIO pin PD7 for MCLK
 * @details
 *   Uses TIMER1, since DAC solution already uses TIMER0
 *****************************************************************************/
void create_gpio_clock(void)
{
	CMU_ClockEnable(cmuClock_TIMER1, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	// Enable MCLK output
	GPIO_PinModeSet(MCLK_PORT, MCLK_PIN, gpioModePushPull, 0);

	// Create the object initializer for compare channel
	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
	timerCCInit.mode = timerCCModeCompare;
	timerCCInit.cufoa = timerOutputActionNone;
	timerCCInit.cofoa = timerOutputActionToggle;
	timerCCInit.cmoa = timerOutputActionNone;
	timerCCInit.filter = true;

	// Configure TIMER1 CC channel 1
	TIMER_InitCC(TIMER1, 1, &timerCCInit);

	// Route CC1 to location 1 (PD7) and enable pin for cc1
	TIMER1->ROUTE |= (TIMER_ROUTE_CC1PEN | TIMER_ROUTE_LOCATION_LOC4);

	// Set TIMER Top value to 2, to generate 8MHz clock
	TIMER_TopSet(TIMER1, 2);

	/* Select timer parameters */
	TIMER_Init_TypeDef timerInit =
	{
	.enable     = true,
	.debugRun   = true,
	.prescale   = timerPrescale1,
	.clkSel     = timerClkSelHFPerClk,
	.fallAction = timerInputActionNone,
	.riseAction = timerInputActionNone,
	.mode       = timerModeUp,
	.dmaClrAct  = false,
	.quadModeX4 = false,
	.oneShot    = false,
	.sync       = false,
	};

	/* Configure timer */
	TIMER_Init(TIMER1, &timerInit);
}

/**************************************************************************//**
 * @brief
 *   Opens MICROSD card for use
 * @details
 *   Opens card, reads filename, fills buffers for the first time
 *****************************************************************************/
void prepare_microsd_card(char *filename)
{
	/* Initialize filesystem */
	MICROSD_Init();

	FRESULT res = f_mount(0, &Fatfs);
	if (res != FR_OK)
	{
	/* No micro-SD with FAT32 is present */
		DEBUG_BREAK
	}

	/* Open wav file from SD-card */
	if (f_open(&WAVfile, filename, FA_READ) != FR_OK)
	{
	/* No micro-SD with FAT32, or no WAV_FILENAME found */
		DEBUG_BREAK
	}

	ByteCounter = 0;

	/* Read header and place in header struct */
	f_read(&WAVfile, &wavHeader, sizeof(wavHeader), &bytes_read);

	/* Fill both primary and alternate RAM-buffer before start */
	FillBufferFromSDcard((bool) wavHeader.channels, true);
	FillBufferFromSDcard((bool) wavHeader.channels, false);
}

void play_sound(const int track)
{
	char * s = file_array[track].filename;
	prepare_microsd_card(s);

	/* Setup USART1 as I2S master */
	I2S_setup();

	/* Setup DMA and peripherals */
	DMA_setup();
}

int get_next_track()
{
	static int track = 0;
	int result = track;
	track++;
	if (track == MAX_TRACKS) track = 0;
	return result;
}
