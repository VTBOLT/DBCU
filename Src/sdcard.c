#include "sdcard.h"
#include "app_filex.h"
#include "fx_api.h"

FX_MEDIA sd_disk;
UCHAR media_memory[512]; // Buffer for media operations

bool SD_Open(void) {
  UINT status;
  status = fx_media_open(&sd_disk, "SD Card",
                         fx_stm32_sd_driver, // Your driver function
                         0,                  // Driver info pointer
                         media_memory,       // Media buffer pointer
                         512);               // Media buffer size
  return (status == FX_SUCCESS);
}

bool SD_CreateFile(char *filename) {
  UINT status;
  status = fx_file_create(&sd_disk, filename);
  return (status == FX_SUCCESS || status == FX_ALREADY_CREATED);
}

bool SD_WriteFile(char *filename, const uint8_t *data, uint32_t length) {
  UINT status;
  FX_FILE tmp_file;

  status = fx_file_open(&sd_disk, &tmp_file, filename, FX_OPEN_FOR_WRITE);
  if (status != FX_SUCCESS) {
    // Handle error
    return 0;
  }

  // 3. Write data
  status = fx_file_write(&tmp_file, (void *)data, length);
  if (status != FX_SUCCESS) {
    // Handle error
    return 0;
  }

  // 4. Close the file
  fx_file_close(&tmp_file);
  return 1;
}

bool SD_Close(void) {
  UINT status = fx_media_close(&sd_disk);
  return (status == FX_SUCCESS);
};

bool SD_FlushData(void) {
  UINT status = fx_media_flush(&sd_disk);
  return (status == FX_SUCCESS);
}