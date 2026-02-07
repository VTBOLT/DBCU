/**
 * @file ws2812b.c
 * @brief WS2812B LED strip driver implementation
 * 
 * PWM+DMA method:
 * - Timer generates PWM signal at 800kHz (1.25us period)
 * - DMA updates the duty cycle for each bit
 * - Bit 0: ~0.4us high, ~0.85us low (32% duty cycle)
 * - Bit 1: ~0.8us high, ~0.45us low (64% duty cycle)
 * 
 * Timer Configuration:
 * - For 250MHz system clock, prescaler = 0, ARR = 312 gives 800kHz
 * - Adjust based on your actual system clock
 */

#include "ws2812b.h"
#include <stdlib.h>
#include <string.h>

/* PWM values for WS2812B timing */
#define WS2812B_0_CODE      100     // ~32% duty cycle for bit 0
#define WS2812B_1_CODE      200     // ~64% duty cycle for bit 1

/* Helper function to convert byte to PWM values */
static void byte_to_pwm(uint8_t byte, uint16_t *pwm_data) {
    for (int i = 0; i < 8; i++) {
        if (byte & (1 << (7 - i))) {
            pwm_data[i] = WS2812B_1_CODE;
        } else {
            pwm_data[i] = WS2812B_0_CODE;
        }
    }
}

int WS2812B_Init(WS2812B_Handle_t *handle, TIM_HandleTypeDef *htim, 
                 uint32_t tim_channel, uint16_t num_leds) {
    if (handle == NULL || htim == NULL || num_leds == 0 || num_leds > WS2812B_MAX_LEDS) {
        return -1;
    }

    handle->htim = htim;
    handle->tim_channel = tim_channel;
    handle->num_leds = num_leds;
    handle->transfer_complete = 1;

    /* Allocate memory for color buffer */
    handle->colors = (WS2812B_Color_t *)calloc(num_leds, sizeof(WS2812B_Color_t));
    if (handle->colors == NULL) {
        return -1;
    }

    /* Allocate PWM buffer: 24 bits per LED + reset pulses */
    uint32_t buffer_size = (num_leds * 24) + WS2812B_RESET_PULSES;
    handle->pwm_buffer = (uint16_t *)calloc(buffer_size, sizeof(uint16_t));
    if (handle->pwm_buffer == NULL) {
        free(handle->colors);
        return -1;
    }

    /* Initialize all LEDs to off */
    WS2812B_Clear(handle);

    return 0;
}

void WS2812B_SetLED(WS2812B_Handle_t *handle, uint16_t led_index, 
                    uint8_t r, uint8_t g, uint8_t b) {
    if (handle == NULL || led_index >= handle->num_leds) {
        return;
    }

    handle->colors[led_index].r = r;
    handle->colors[led_index].g = g;
    handle->colors[led_index].b = b;
}

void WS2812B_SetLEDColor(WS2812B_Handle_t *handle, uint16_t led_index, 
                         WS2812B_Color_t *color) {
    if (handle == NULL || color == NULL || led_index >= handle->num_leds) {
        return;
    }

    handle->colors[led_index] = *color;
}

void WS2812B_SetAll(WS2812B_Handle_t *handle, uint8_t r, uint8_t g, uint8_t b) {
    if (handle == NULL) {
        return;
    }

    for (uint16_t i = 0; i < handle->num_leds; i++) {
        handle->colors[i].r = r;
        handle->colors[i].g = g;
        handle->colors[i].b = b;
    }
}

void WS2812B_Clear(WS2812B_Handle_t *handle) {
    WS2812B_SetAll(handle, 0, 0, 0);
}

int WS2812B_Update(WS2812B_Handle_t *handle) {
    if (handle == NULL || handle->transfer_complete == 0) {
        return -1;
    }

    uint16_t pwm_idx = 0;

    /* Convert colors to PWM values */
    /* WS2812B expects GRB order */
    for (uint16_t led = 0; led < handle->num_leds; led++) {
        byte_to_pwm(handle->colors[led].g, &handle->pwm_buffer[pwm_idx]);
        pwm_idx += 8;
        byte_to_pwm(handle->colors[led].r, &handle->pwm_buffer[pwm_idx]);
        pwm_idx += 8;
        byte_to_pwm(handle->colors[led].b, &handle->pwm_buffer[pwm_idx]);
        pwm_idx += 8;
    }

    /* Add reset pulses (low) */
    for (uint16_t i = 0; i < WS2812B_RESET_PULSES; i++) {
        handle->pwm_buffer[pwm_idx++] = 0;
    }

    /* Start DMA transfer */
    handle->transfer_complete = 0;
    
    if (HAL_TIM_PWM_Start_DMA(handle->htim, handle->tim_channel, 
                              (uint32_t *)handle->pwm_buffer, pwm_idx) != HAL_OK) {
        handle->transfer_complete = 1;
        return -1;
    }

    return 0;
}

int WS2812B_WaitComplete(WS2812B_Handle_t *handle, uint32_t timeout_ms) {
    if (handle == NULL) {
        return -1;
    }

    uint32_t start_tick = HAL_GetTick();
    
    while (handle->transfer_complete == 0) {
        if ((HAL_GetTick() - start_tick) > timeout_ms) {
            return -1;  // Timeout
        }
    }

    return 0;
}

void WS2812B_DMA_Complete(WS2812B_Handle_t *handle) {
    if (handle == NULL) {
        return;
    }

    /* Stop PWM */
    HAL_TIM_PWM_Stop_DMA(handle->htim, handle->tim_channel);
    handle->transfer_complete = 1;
}
