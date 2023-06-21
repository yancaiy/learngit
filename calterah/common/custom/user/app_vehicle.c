
#include <string.h>
#include <math.h>
#include "sharedVar.h"
#include "sharedDef.h"
#include "cfg.h"
#include "app_vehicle.h"

#define MIN_RADIUS          10
#define MAX_RADIUS          0x7FFE


static TVehicleInfo gVehicleInfo;


float VehicleGetSpeed(void)
{
	return gVehicleInfo.SpeedSource ? gVehicleInfo.Speed : gVehicleInfo.estSpeed[1];
}

void VehicleSetSpeed(float speed, INT32 direction, int source)
{
	if (RADAR_MODE == RCW_MODE)
	{
		if (direction == 1)
		{
			gVehicleInfo.Speed = speed / 3.6;
		}
		else
		{
			gVehicleInfo.Speed = -speed / 3.6;
		}
	}
	else if(RADAR_MODE == BSD_MODE)
	{
		//for BSD , process in app
		gVehicleInfo.Speed = speed / 3.6;
		
		if (direction == 1)
		{
			gVehicleInfo.Speed = -speed / 3.6;
		}
		else
		{
			gVehicleInfo.Speed = speed / 3.6;
		}
		
	}
	else
	{
		if (direction == 1)
		{
			gVehicleInfo.Speed = -speed / 3.6;
		}
		else
		{
			gVehicleInfo.Speed = speed / 3.6;
		}
	}

	if (source)
	{
		gVehicleInfo.speedLostCnt = 0;
		gVehicleInfo.SpeedSource = radar_config_using->speedSource;
		gVehicleInfo.isStill = speed < 3 ? 1 : 0;
		gVehicleInfo.recvInfo |= 0x1;		
	}
	else
	{
		gVehicleInfo.recvInfo |= 0x1;
	}
}

float VehicleGetYawRate(void)
{
	return (gVehicleInfo.YawRate);
}

void VehicleSetYawRate(float yawrate, INT32 flag)
{
	if (flag == 1)
	{
		gVehicleInfo.YawRate = yawrate; // YawRate is available
		float RadiusCurvatureSpeed = VehicleGetSpeed();
		float RadiusCurvature = 0x7FFF;
		
		if (fabsf(yawrate) > 1e-3 && RadiusCurvatureSpeed > 1)
		{
				RadiusCurvature = (float)(RadiusCurvatureSpeed / gVehicleInfo.YawRate * 360 / 2 / 3.14159f);

				if(fabsf(RadiusCurvature) > MIN_RADIUS) //不再最小值范围内减才有效
				{
					RadiusCurvature = RadiusCurvature > 0 ? RadiusCurvature - 15 : RadiusCurvature + 15;
				}
		}

		VehicleSetRadiusCurvature( RadiusCurvature );

		gVehicleInfo.yawrateLostCnt = 0;
		gVehicleInfo.recvInfo |= 0x2;
	}
	else
	{
		// if YawRate is invalid
		gVehicleInfo.recvInfo |= 0x2;
		return;
	}
	return;
}

float VehicleGetSteeringAngle(void)
{
	return gVehicleInfo.SteeringAngle;
}

void VehicleSetSteeringAngle(float angle, INT32 flag)
{
	if (flag == 1)
	{
		gVehicleInfo.SteeringAngle = angle;
	}

	gVehicleInfo.recvInfo |= 0x8;
}

float VehicleGetSteeringAngleSpd(void)
{
	return gVehicleInfo.SteeringAngleSpd;
}

void VehicleSetSteeringAngleSpd(float angleRage, INT32 flag)
{
	if (flag)
	{
		gVehicleInfo.SteeringAngleSpd = angleRage;
	}
}

float VehicleGetRadiusCurvature(void)
{
	return gVehicleInfo.RadiusCurvature;
}

void VehicleSetRadiusCurvature(float rc)
{
	gVehicleInfo.RadiusCurvature = rc;
}

//void recordVehicleInfoAk()
//{
//	memcpy((void *)&(gVehicleInfoAk), (char *)&(gVehicleInfo), sizeof(gVehicleInfoAk));
//}

void initVehicleInfo(void)
{
	memset(&gVehicleInfo, 0, sizeof(gVehicleInfo));

	gVehicleInfo.isStill = 1;
	gVehicleInfo.Speed = 0;
	gVehicleInfo.SpeedSource = radar_config_using->speedSource;
	gVehicleInfo.EstCarSpeed = 0;
	gVehicleInfo.YawRate = 0;
	gVehicleInfo.RadiusCurvature = 0xffff;
	gVehicleInfo.engineValid = 0;
	gVehicleInfo.keyStatus = KEY_STATUS_ON;

	//没有获取凯翼车辆的时间时，将时间默认值设置为2000年01月01日00时00分00秒
	gVehicleInfo.time[0] = 0x00;	//秒
	gVehicleInfo.time[1] = 0x00;	//分
	gVehicleInfo.time[2] = 0x00;	//时
	gVehicleInfo.time[3] = 0x01;	//日
	gVehicleInfo.time[4] = 0x01;	//月
	gVehicleInfo.time[5] = 0x00;	//年
}

//冒泡排序算法 ->升序
void do_BubbleSort(float *arr, int length)
{
	float temp; //临时变量
	for (int i = 0; i < length; i++)
	{ //表示趟数，一共length次。
		for (int j = length - 1; j > i; j--)
		{
			if (arr[j] < arr[j - 1])
			//if ((arr[j] - arr[j - 1]) < 1e-6)
			{
				temp = arr[j];
				arr[j] = arr[j - 1];
				arr[j - 1] = temp;
			}
		}
	}
}

float doYawRateMedianFilter(float yawrate)
{
    static float YawrateBuffer[7] = {0,0,0,0,0,0,0};
    float TempBuffer[7];
    //滑窗
    for(int i = 0; i < 7 - 1; i++)
    {
        YawrateBuffer[i] = YawrateBuffer[i + 1];
    }
    YawrateBuffer[6] = yawrate;

    memcpy(TempBuffer,YawrateBuffer, sizeof(YawrateBuffer));
    //排序
    do_BubbleSort(TempBuffer,7);

    //取中值
    return TempBuffer[3];
}

