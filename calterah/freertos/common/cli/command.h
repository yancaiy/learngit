#ifndef COMMAND_H
#define COMMAND_H

#include "FreeRTOS.h"


void command_interpreter_task(void *parameters);
void freertos_cli_init();

#endif
