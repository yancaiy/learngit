
#include "FreeRTOS.h"
#include "cfg.h"


//陀螺仪初始化状态
int gGyroCaliFlag = 0;

static radarTimestamp gTimestamp; //时间戳


void setCyroCaliFlag(uint8_t flag)
{
	gGyroCaliFlag = flag;
}

uint8_t getCyroCaliFlag(void)
{
	return gGyroCaliFlag;
}

void setTimestamp(uint64_t timestamp)
{
	gTimestamp.tick = xTaskGetTickCount();
	gTimestamp.timestamp = timestamp;
}

void updateTimestamp(void)
{
	TickType_t tick = xTaskGetTickCount();
	gTimestamp.timestamp += (tick - gTimestamp.tick);
    gTimestamp.tick = tick;
}

uint64_t getTimestamp(void)
{
	return gTimestamp.timestamp;
}

