
#include <stdlib.h>
#include <string.h>
#include "sharedVar.h"

uint8_t gProfileCount;  //配置的射频芯片的使用到的多少个profile，默认4个
uint8_t frame_type_profile[4];
float gHILPara[3] = {0};

void initVar(void)
{	
	frame_type_profile[0] = 0;
	frame_type_profile[1] = 1;
	frame_type_profile[2] = 2;
	frame_type_profile[3] = 3;

	gProfileCount = 4;
}

