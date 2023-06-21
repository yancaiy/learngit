
#ifndef APP_APP_VEHICLE_H_
#define APP_APP_VEHICLE_H_

#include "typedefs.h"

#define KEY_STATUS_NONE    0
#define KEY_STATUS_OFF     1
#define KEY_STATUS_ACC     2
#define KEY_STATUS_ON      3

typedef struct tagVehicleInfo
{
    uint8_t SpeedSource;            //0 -- 自测, 1 -- obd
    uint8_t sendSpeed;              //是否外发估计的车速
    uint8_t yawrateLostCnt;         //yawrate丢失次数
    uint8_t speedLostCnt;           //速度丢失次数
	uint8_t isStill;                //静止状态
	uint8_t engineValid;            //引擎状态消息
	uint8_t time[6];                //车身时间
	uint8_t keyStatus;              //钥匙状态
	uint8_t recvInfo;               //接收到车身信号标志
    float Speed;                    //m
    float YawRate;                  //deg/s
	float SteeringAngle;          //deg
    float SteeringAngleSpd;      //deg/s
    float RadiusCurvature;          //m
    float LongitudinalAcceleration; //m/s/s
    float LateralAcceleration;      //m/s/s
    float EstCarSpeed;
	float WheelSpeed[4];             //右前轮速     KPH    //左前轮速 //右后轮速  //左后轮速 
    float estSpeed[2];              //估计的车速, 0是用mag值估算的车速，1是用目标点估算的车速
	uint32_t odoMeter;
    uint8_t actualGear;             //实际挡位信号  n
} TVehicleInfo;


extern float VehicleGetSpeed(void);
extern void VehicleSetSpeed(float speed, INT32 direction, int source);
extern float VehicleGetYawRate(void);
extern void VehicleSetYawRate(float yawrate, INT32 flag);
extern float VehicleGetSteeringAngle(void);
extern void VehicleSetSteeringAngle(float angle, INT32 flag);
extern float VehicleGetSteeringAngleSpd(void);
extern void VehicleSetSteeringAngleSpd(float angleRage, INT32 flag);
extern float VehicleGetRadiusCurvature(void);
extern void VehicleSetRadiusCurvature(float rc);
extern void initVehicleInfo(void);
void do_BubbleSort(float *arr, int length);
float getYawRateOffset(float yawrate, float vehicleSpd);


#endif /* APP_APP_VEHICLE_H_ */

