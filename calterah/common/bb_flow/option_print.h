#ifndef OPTION_PRINT_H
#define OPTION_PRINT_H


#include "embARC_debug.h"


#define  PRINT_EN  false // enable print debug message

#if PRINT_EN
#define OPTION_PRINT EMBARC_PRINTF
#else
#define OPTION_PRINT NO_PRINTF
#endif
void NO_PRINTF(const char *format,...);





#endif
