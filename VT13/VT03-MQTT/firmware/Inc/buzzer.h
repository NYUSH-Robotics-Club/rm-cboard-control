/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    buzzer.h
  * @brief   This file contains all the function prototypes for
  *          the buzzer.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUZZER_H__
#define __BUZZER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */


// TIM4 clock frequency (APB1 bus: 72MHz / 2 = 36MHz)
#define TIM4_CLOCK_FREQ      36000000

// Musical note frequency definitions (Hz)
#define NOTE_C3  131    // Do (low octave)
#define NOTE_D3  147    // Re (low octave)
#define NOTE_E3  165    // Mi (low octave)
#define NOTE_F3  175    // Fa (low octave)
#define NOTE_G3  196    // Sol (low octave)
#define NOTE_A3  220    // La (low octave)
#define NOTE_B3  247    // Si (low octave)
#define NOTE_C4  262    // Do
#define NOTE_D4  294    // Re
#define NOTE_E4  330    // Mi
#define NOTE_F4  349    // Fa
#define NOTE_G4  392    // Sol
#define NOTE_A4  440    // La
#define NOTE_B4  494    // Si
#define NOTE_C5  523    // Do (octave)
#define NOTE_D5  587    // Re (octave)
#define NOTE_E5  659    // Mi (octave)
#define NOTE_F5  698    // Fa (octave)
#define NOTE_G5  784    // Sol (octave)
#define NOTE_A5  880    // La (octave)

// Note duration definitions (milliseconds)
#define DURATION_QUARTER  300  // Quarter note (1 beat)
#define DURATION_HALF     600  // Half note (2 beats)
#define DURATION_WHOLE    1200 // Whole note (4 beats)
#define DURATION_EIGHTH   150  // Eighth note (0.5 beat)

// Rest duration (silence between notes)
#define NOTE_REST_GAP     50   // Gap between notes

// Musical note structure
typedef struct {
    uint32_t frequency; // Note frequency (Hz)
    uint32_t duration;  // Note duration (ms)
} MusicalNote;

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/**
 * @brief Initialize buzzer
 * @retval None
 */
void Buzzer_Init(void);

/**
 * @brief Play Twinkle Twinkle Little Star (full version)
 * @retval None
 */
void Buzzer_PlayTwinkleStarFull(void);

/**
 * @brief Play a short beep sound
 * @retval None
 */
void Buzzer_PlayBeep(void);

/**
 * @brief Handle button press for music control
 * @retval None
 */
void Buzzer_HandleButtonPress(void);

/**
 * @brief Update music playback (call this in main loop)
 * @retval None
 */
void Buzzer_Update(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_H__ */
