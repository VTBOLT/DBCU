#pragma once

#include <stdbool.h>

/**
 * @brief Opens the SD card for writing.
 *
 * @return true on success,
 * @return false on failure.
 */
bool SD_Open(void);

/**
 * @brief Creates a new file on the SD card.
 * @param filename Pointer to a null-terminated string containing the filename.
 * @return true if file creation was successful or it already exists, false
 * otherwise.
 */
bool SD_CreateFile(char *filename);

/**
 * @brief Writes data to a file on the SD card.
 * @param filename Pointer to a null-terminated string containing the filename.
 * @param data Pointer to the data buffer to write.
 * @param length Number of bytes to write.
 * @return true if write operation was successful, false otherwise.
 */
bool SD_WriteFile(char *filename, const uint8_t *data, uint32_t length);

/**
 * @brief Closes the SD card device
 * @return true if device closure was successful, false otherwise.
 */
bool SD_Close(void);

/**
 * @brief Flushes all pending data to the SD card.
 * @return true if flush operation was successful, false otherwise.
 */
bool SD_FlushData(void);