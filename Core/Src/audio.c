/*
 * audio.c
 *
 *  Created on: Jan 5, 2026
 *      Author: lovro
 */

#include "audio.h"

void generateSineTable();
void fillDmaBuffer();

uint32_t sineFrequency = 440;
float sineAmplitude = 0.5f;   // 0.0 to 1.0
uint16_t sineTable[SINE_BUFFER_SIZE] = {0};
uint16_t dmaBuffer[SINE_BUFFER_SIZE];

void init_AudioReset(){
	HAL_GPIO_WritePin(GPIOD, AUDIO_RESET_PIN, GPIO_PIN_SET);
}

void configAudio(){
	uint8_t bytes [2];
	init_AudioReset();

	/** Power sequence **/
	// Set Power Control Register to "on" state
	bytes [0] = 0x02;
	bytes [1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100) ;

	/** Initialisation sequence **/
	bytes [0] = 0x00;
	bytes [1] = 0x99;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x47;
	bytes [1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x32;
	bytes [1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x32;
	bytes [1] = 0x0;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x00;
	bytes [1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	/** Ctl registers configuration **/
	bytes [0] = 0x04;
	bytes [1] = 0xAF;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x0D;
	bytes [1] = 0x70;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x05;
	bytes [1] = 0x81;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x06;
	bytes [1] = 0x07;
	HAL_I2C_Master_Transmit(&hi2c1 , AUDIO_I2C_ADDRESS , bytes , 2, 100);

	bytes [0] = 0x0A;
	bytes [1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x27;
	bytes [1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x1F;
	bytes [1] = 0x0F;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x22;
	bytes [1] = 0xC0;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x14;
	bytes [1] = 2;
	HAL_I2C_Master_Transmit (&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x15;
	bytes [1] = 2;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x20;
	bytes [1] = 24;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	bytes [0] = 0x21;
	bytes [1] = 24;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	/** Power up **/
	bytes [0] = 0x02;
	bytes [1] = 0x9E;
	HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, bytes, 2, 100);

	generateSineTable();
}

void generateSineTable()
{
    for (int i = 0; i < SINE_BUFFER_SIZE; i++)
    {
        float t = (float)i / SINE_BUFFER_SIZE;   // 0..1
        float sample = sinf(2.0f * M_PI * t);      // -1..1
        sineTable[i] = (uint16_t)((sample + 1.0f) * 16000.f);
    }
    fillDmaBuffer();
}

void fillDmaBuffer()
{
    static float phase = 0.0f;  // phase accumulator
    float phaseStep = sineFrequency * SINE_BUFFER_SIZE / SAMPLE_RATE; // samples per output

    for (int i = 0; i < SINE_BUFFER_SIZE; i++)
    {
        uint16_t index = (uint16_t)phase % SINE_BUFFER_SIZE;
        float sample = sineTable[index];

        // amplitude scaling
        sample = sample * sineAmplitude + (1.0f - sineAmplitude) * 16000.f;
        dmaBuffer[i] = (uint16_t)sample;

        // advance phase
        phase += phaseStep;
        if (phase >= SINE_BUFFER_SIZE)
            phase -= SINE_BUFFER_SIZE;
    }
}

void changeAudioFrequency(uint32_t frequency){
	sineFrequency = frequency;
	fillDmaBuffer();
}

void changeAudioAmplitude(float amplitude){
	sineAmplitude = amplitude;
	fillDmaBuffer();
}
