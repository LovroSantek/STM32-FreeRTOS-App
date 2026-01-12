#ifndef INC_AUDIO_H_
#define INC_AUDIO_H_

#include "main.h"

#define AUDIO_RESET_PIN GPIO_PIN_4
#define AUDIO_I2C_ADDRESS 0x94

#define SINE_BUFFER_SIZE 256
#define SAMPLE_RATE 48000

extern uint16_t dmaBuffer[SINE_BUFFER_SIZE];

void init_AudioReset(void);
void configAudio(void);
void changeAudioFrequency(uint32_t frequency);
void changeAudioAmplitude(float amplitude);

#endif /* INC_AUDIO_H_ */
