#ifndef SENSOR_CONFIG_CLI_H
#define SENSOR_CONFIG_CLI_H


#define ANGLE_CALIB_INFO_LEN       163
#define FLASH_PAGE_SIZE            256

/* This Marco is used to control all the sensor config related command-line,
   comment out this Marco can size RAM size but make all the sensor config command-line can no longer be used */
#define SENSOR_CONFIG_CLI

void sensor_config_cli_commands(void);
void ang_calib_data_init(float *data_buf);
long sensor_cfg_command_handler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

#endif
