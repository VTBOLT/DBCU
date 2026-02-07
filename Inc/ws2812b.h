/**
 * @file ws2812b.h
 * @brief WS2812B LED strip driver for STM32H523
 * 
 * This library uses PWM+DMA to control WS2812B LED strips.
 * The timing is generated using a timer's PWM output.
 */

#ifndef WS2812B_H
#define WS2812B_H

#include "stm32h5xx_hal.h"
#include <stdint.h>

/* WS2812B Configuration */
#define WS2812B_MAX_LEDS        64      // Maximum number of LEDs
#define WS2812B_RESET_PULSES    50      // Reset period pulses

/* Color structure */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} WS2812B_Color_t;

/* Library handle */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;
    uint16_t num_leds;
    WS2812B_Color_t *colors;
    uint16_t *pwm_buffer;
    volatile uint8_t transfer_complete;
} WS2812B_Handle_t;

/* Function prototypes */

/**
 * @brief Initialize WS2812B driver
 * @param handle Pointer to WS2812B handle
 * @param htim Pointer to timer handle
 * @param tim_channel Timer channel (TIM_CHANNEL_1, TIM_CHANNEL_2, etc.)
 * @param num_leds Number of LEDs in the strip
 * @return 0 on success, -1 on error
 */
int WS2812B_Init(WS2812B_Handle_t *handle, TIM_HandleTypeDef *htim, 
                 uint32_t tim_channel, uint16_t num_leds);

/**
 * @brief Set color of a single LED
 * @param handle Pointer to WS2812B handle
 * @param led_index LED index (0-based)
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 */
void WS2812B_SetLED(WS2812B_Handle_t *handle, uint16_t led_index, 
                    uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set color of a single LED using color structure
 * @param handle Pointer to WS2812B handle
 * @param led_index LED index (0-based)
 * @param color Pointer to color structure
 */
void WS2812B_SetLEDColor(WS2812B_Handle_t *handle, uint16_t led_index, 
                         WS2812B_Color_t *color);

/**
 * @brief Set all LEDs to the same color
 * @param handle Pointer to WS2812B handle
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 */
void WS2812B_SetAll(WS2812B_Handle_t *handle, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Clear all LEDs (turn off)
 * @param handle Pointer to WS2812B handle
 */
void WS2812B_Clear(WS2812B_Handle_t *handle);

/**
 * @brief Update LED strip with current colors
 * @param handle Pointer to WS2812B handle
 * @return 0 on success, -1 on error
 */
int WS2812B_Update(WS2812B_Handle_t *handle);

/**
 * @brief Wait for update to complete
 * @param handle Pointer to WS2812B handle
 * @param timeout_ms Timeout in milliseconds
 * @return 0 on success, -1 on timeout
 */
int WS2812B_WaitComplete(WS2812B_Handle_t *handle, uint32_t timeout_ms);

/**
 * @brief DMA transfer complete callback (call from HAL_TIM_PWM_PulseFinishedCallback)
 * @param handle Pointer to WS2812B handle
 */
void WS2812B_DMA_Complete(WS2812B_Handle_t *handle);

#endif /* WS2812B_H */
