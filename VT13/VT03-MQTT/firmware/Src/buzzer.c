/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    buzzer.c
  * @brief   This file provides code for buzzer control functionality
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

/* Includes ------------------------------------------------------------------*/
#include "buzzer.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define C3 131
#define C3s 139
#define D3 147
#define D3s 156
#define E3 165
#define F3 175
#define F3s 185
#define G3 196
#define G3s 208
#define A3 220
#define A3s 233
#define B3 247
#define C4 262
#define C4s 277
#define D4 294
#define D4s 311
#define E4 330
#define F4 349
#define F4s 370
#define G4 392
#define G4s 415
#define A4 440
#define A4s 466
#define B4 494
#define C5 523
#define C5s 554
#define D5 587
#define D5s 622
#define E5 659
#define F5 698
#define F5s 740
#define G5 784
#define G5s 831
#define A5 880
#define A5s 932
#define B5 988
#define C6 1047
/* USER CODE END Private defines */

/* USER CODE BEGIN Private variables */

static bool buzzer_initialized = false;
static uint32_t last_button_press_time = 0;
static bool music_playing = false;
static uint32_t music_note_index = 0;
static uint32_t music_note_start_time = 0;
static uint32_t music_note_duration = 0;

/* USER CODE END Private variables */

/* USER CODE BEGIN Private function prototypes */

/**
 * @brief Calculate PWM parameters
 * @param frequency Target frequency
 * @param prescaler Prescaler pointer
 * @param period Period value pointer
 * @retval None
 */
static void Buzzer_CalculatePWM(uint32_t frequency, uint32_t *prescaler, uint32_t *period);

/* USER CODE END Private function prototypes */

/* USER CODE BEGIN Private user code */

// Twinkle Twinkle Little Star melody (full version - enhanced contrast)
static const MusicalNote twinkle_star_full_melody[] = {
    // Twinkle, twinkle, little star (增强对比度)
    {NOTE_C4, DURATION_QUARTER}, {NOTE_C4, DURATION_QUARTER}, {NOTE_G5, DURATION_QUARTER}, {NOTE_G5, DURATION_QUARTER},
    {NOTE_A5, DURATION_QUARTER}, {NOTE_A5, DURATION_QUARTER}, {NOTE_G5, DURATION_HALF},
    
    // How I wonder what you are
    {NOTE_F5, DURATION_QUARTER}, {NOTE_F5, DURATION_QUARTER}, {NOTE_E5, DURATION_QUARTER}, {NOTE_E5, DURATION_QUARTER},
    {NOTE_D5, DURATION_QUARTER}, {NOTE_D5, DURATION_QUARTER}, {NOTE_C5, DURATION_HALF},
    
    // Up above the world so high
    {NOTE_G5, DURATION_QUARTER}, {NOTE_G5, DURATION_QUARTER}, {NOTE_F5, DURATION_QUARTER}, {NOTE_F5, DURATION_QUARTER},
    {NOTE_E5, DURATION_QUARTER}, {NOTE_E5, DURATION_QUARTER}, {NOTE_D5, DURATION_HALF},
    
    // Like a diamond in the sky
    {NOTE_G5, DURATION_QUARTER}, {NOTE_G5, DURATION_QUARTER}, {NOTE_F5, DURATION_QUARTER}, {NOTE_F5, DURATION_QUARTER},
    {NOTE_E5, DURATION_QUARTER}, {NOTE_E5, DURATION_QUARTER}, {NOTE_D5, DURATION_HALF},
    
    // Twinkle, twinkle, little star
    {NOTE_C4, DURATION_QUARTER}, {NOTE_C4, DURATION_QUARTER}, {NOTE_G5, DURATION_QUARTER}, {NOTE_G5, DURATION_QUARTER},
    {NOTE_A5, DURATION_QUARTER}, {NOTE_A5, DURATION_QUARTER}, {NOTE_G5, DURATION_HALF},
    
    // How I wonder what you are
    {NOTE_F5, DURATION_QUARTER}, {NOTE_F5, DURATION_QUARTER}, {NOTE_E5, DURATION_QUARTER}, {NOTE_E5, DURATION_QUARTER},
    {NOTE_D5, DURATION_QUARTER}, {NOTE_D5, DURATION_QUARTER}, {NOTE_C5, DURATION_HALF}
};

// Approximate 8-bar right-hand melody for "哈基米哈基米"
// Uses mostly eighth notes, with a longer note at the very end.
// Make sure you have DURATION_EIGHTH defined in your code.

static const MusicalNote hajimi_melody_8bars[] = {
    // Bar 1
    {NOTE_E5, DURATION_QUARTER}, {NOTE_G5, DURATION_QUARTER}, {NOTE_A5, DURATION_QUARTER}, 
        {NOTE_E5, DURATION_QUARTER}, {NOTE_G5, DURATION_QUARTER}, {NOTE_A5, DURATION_QUARTER},     
        {NOTE_E5, DURATION_QUARTER}, {NOTE_G5, DURATION_QUARTER}, {NOTE_A5, DURATION_QUARTER}, 
        {NOTE_E5, DURATION_QUARTER}, {NOTE_G5, DURATION_QUARTER}, {NOTE_A5, DURATION_QUARTER}, 
        {NOTE_E5, DURATION_QUARTER}, {NOTE_G5, DURATION_QUARTER}, {NOTE_A5, DURATION_QUARTER}, 
    {NOTE_C5, DURATION_QUARTER}, {NOTE_D5, DURATION_QUARTER}, {NOTE_E5, DURATION_HALF}, 
    {NOTE_D5, DURATION_QUARTER}, {NOTE_C5, DURATION_QUARTER}, {NOTE_A4, DURATION_QUARTER}, 
    {NOTE_D5, DURATION_HALF},
    

};


/**
 * @brief Calculate PWM parameters
 * @param frequency Target frequency
 * @param prescaler Prescaler pointer
 * @param period Period value pointer
 * @retval None
 */
static void Buzzer_CalculatePWM(uint32_t frequency, uint32_t *prescaler, uint32_t *period)
{
    // PWM frequency = TIM4 clock frequency / ((Prescaler + 1) * (Period + 1))
    // TIM4 is on APB1 bus with 36MHz clock
    // For better accuracy, we select an appropriate prescaler value
    
    uint32_t target_period = TIM4_CLOCK_FREQ / frequency;
    
    // If target period is too large, need to increase prescaler
    if (target_period > 65535) {
        *prescaler = (target_period / 65535) - 1;
        *period = target_period / (*prescaler + 1) - 1;
    } else {
        *prescaler = 0;
        *period = target_period - 1;
    }
    
    // Ensure values are within valid range
    if (*prescaler > 65535) {
        *prescaler = 65535;
    }
    if (*period > 65535) {
        *period = 65535;
    }
}

/* USER CODE END Private user code */

/* USER CODE BEGIN 0 */

/**
 * @brief Initialize buzzer
 * @retval None
 */
void Buzzer_Init(void)
{
    if (!buzzer_initialized) {
        // Start TIM4 PWM channel 3
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
        buzzer_initialized = true;
    }
}



/**
 * @brief Start buzzer
 * @param frequency Buzzer frequency (Hz)
 * @retval None
 */
void Buzzer_Start(uint32_t frequency)
{
    if (!buzzer_initialized) {
        Buzzer_Init();
    }
    
    // Stop PWM before changing parameters
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    
    uint32_t prescaler, period;
    Buzzer_CalculatePWM(frequency, &prescaler, &period);
    
    // Set prescaler and period
    __HAL_TIM_SET_PRESCALER(&htim4, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim4, period);
    
    // Set duty cycle to 50%
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period / 2);
    
    // Restart PWM with new parameters
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

/**
 * @brief Stop buzzer
 * @retval None
 */
void Buzzer_Stop(void)
{
    if (buzzer_initialized) {
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    }
}

/**
 * @brief Play Twinkle Twinkle Little Star (full version)
 * @retval None
 */
void Buzzer_PlayTwinkleStarFull(void)
{
    uint32_t melody_length = sizeof(twinkle_star_full_melody) / sizeof(MusicalNote);
    
    for (uint32_t i = 0; i < melody_length; i++) {
        // Play the note
        Buzzer_Start(twinkle_star_full_melody[i].frequency);
        HAL_Delay(twinkle_star_full_melody[i].duration);
        Buzzer_Stop();
        
        // Add small gap between notes for better sound
        if (i < melody_length - 1) { // Don't delay after the last note
            HAL_Delay(NOTE_REST_GAP);
        }
    }
}

/**
 * @brief Play a short beep sound
 * @retval None
 */
void Buzzer_PlayBeep(void)
{
    Buzzer_Start(NOTE_B4);
    HAL_Delay(200);
    Buzzer_Stop();
}

/**
 * @brief Handle button press for music control
 * @retval None
 */
void Buzzer_HandleButtonPress(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // Software debouncing: ignore button presses within 20ms
    if (current_time - last_button_press_time < 20) {
        return;
    }
    
    last_button_press_time = current_time;
    
    // Start music playback if not already playing
    if (!music_playing) {
        music_playing = true;
        music_note_index = 0;
        music_note_start_time = current_time;
        music_note_duration = 0;
    }
}

/**
 * @brief Update music playback (call this in main loop)
 * @retval None
 */
void Buzzer_Update(void)
{
    if (!music_playing) {
        return;
    }
    
    uint32_t current_time = HAL_GetTick();
    uint32_t melody_length = sizeof(hajimi_melody_8bars) / sizeof(MusicalNote);
    
    // Check if current note duration has elapsed
    if (music_playing && (current_time - music_note_start_time >= music_note_duration) ){
        // Stop current note
        Buzzer_Stop();
        
        // Move to next note
        music_note_index++;
        
        if (music_note_index >= melody_length) {
            // Music finished
            music_playing = false;
            return;
        }
        
        // Start next note
        Buzzer_Start(hajimi_melody_8bars[music_note_index].frequency);
        music_note_start_time = current_time;
        music_note_duration = hajimi_melody_8bars[music_note_index].duration;
    }
}

/* USER CODE END 0 */
