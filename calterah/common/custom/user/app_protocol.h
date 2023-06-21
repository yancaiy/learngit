#ifndef __APP_PROTOCOL_H__
#define __APP_PROTOCOL_H__

#include "typedefs.h"
#include "can_trans.h"
#include "sharedVar.h"
#include "flash.h"

#define CALC_TYPE_INSTALL_CALC		0
#define CALC_TYPE_SELF_ANGLE_CALC	1

#if KAIYI_VCAN_TYPE == 0
typedef struct
{
    uint64_t RollCount : 4;
    uint64_t Resv0 : 4;
    uint64_t Resv1 : 8;
    uint64_t check : 8;
    uint64_t VehSpd : 13; //Vehicle speed 车速值 *0.0625 kmh
    uint64_t Resv2 : 1;
    uint64_t VehSpdValid : 1;
    uint64_t Resv3 : 25;
} msgABSESP1;

typedef struct
{
    uint64_t Resv0 : 13;    
    uint64_t engineStatus : 3;
    uint64_t Resv1 : 48;      
} msgENG1;

//凯翼方向盘角度
typedef struct
{
    uint64_t checksum : 8;
    uint64_t Rollingcount : 4;
    uint64_t Resv0 : 4;
    uint64_t SteerAngleValid : 1;
    uint64_t SteerAngleSpeedValid : 1;
    uint64_t TorsionBarTorqueValid : 1;
    uint64_t LASCtrlSts : 2;
    uint64_t TorsionBarTorque : 11;
    uint64_t SteerAngleSpeed : 16;
    uint64_t SteerAngle : 16;
} msgEPS1;

//凯翼偏航率
typedef struct
{
    uint64_t Checksum : 8;
    uint64_t Resv0 : 8;
    uint64_t YawRate : 16;        //曲率 0x0~0x2710 0xD8F0~0xFFFF *0.01 degree/second Error:0x2711~0xD8EF
    uint64_t Resv1 : 20;
    uint64_t YawRateVd : 2;       //曲率有效位 0:Not valid 1:valid
    uint64_t Resv2 : 2;
    uint64_t Rollingcounter : 8;
} msgYAS1;

typedef struct
{
    uint64_t Resv0 : 32;
    uint64_t DrvDoorStatus : 1; //左前门状态 0:close 1:open
    uint64_t PsgDoorStatus : 1; //右前门状态 0:close 1:open
    uint64_t Resv1 : 2;
    uint64_t RRDoorStatus : 1;  //右后门状态 0:close 1:open
    uint64_t RLDoorStatus : 1;  //左后门状态 0:close 1:open
    uint64_t Resv2 : 1;
    uint64_t DriverDoorOpenValid : 1;
    uint64_t DirectionIndRight : 1;
    uint64_t DirectionIndLeft : 1;
    uint64_t Resv3 : 19;
    uint64_t KeyState : 2; //0-off 1-acc 2-on 3-Crank+on  前向只用到这一个信号
    uint64_t Resv4 : 1;
} msgBCMDoorLight;

//凯翼时间
typedef struct
{
    uint64_t Resv0 : 5;
    uint64_t year : 8;       
    uint64_t mouth : 4;      
    uint64_t day : 5;
    uint64_t Resv1 : 14; 
    uint64_t second : 6;        
    uint64_t Resv2 : 6;    
    uint64_t minuite : 8;    
    uint64_t hour : 8;     
} MsgTime;

//凯翼IMC1
typedef struct
{
    uint64_t OdometerInfo : 20;       //里程信息 km
    uint64_t Resv0 : 20;              //
    uint64_t VehicBatteryVoltage : 8; //整车电压
    uint64_t Resv1 : 16;              //
} msgICM1;
#else
typedef struct
{
    uint64_t Resv0 : 24;
    uint64_t VehSpd : 13; //Vehicle speed 车速值 *0.0625 kmh
    uint64_t Resv1 : 1;
    uint64_t VehSpdValid : 1;
    uint64_t Resv2 : 25;
} msgABSESP1;

typedef struct
{
    uint64_t Resv0 : 13;    
    uint64_t engineStatus : 2;
    uint64_t Resv1 : 39;      
} msgENG1;

//凯翼偏航率
typedef struct
{
    uint64_t Resv0 : 16;
    uint64_t YawRate : 16;        //曲率 0x0~0x2710 0xD8F0~0xFFFF *0.01 degree/second Error:0x2711~0xD8EF
    uint64_t Resv1 : 20;
    uint64_t YawRateVd : 1;       //曲率有效位 0:Not valid 1:valid
    uint64_t Resv2 : 11;
} msgYAS1;

//凯翼时间
typedef struct
{
    uint64_t Resv0 : 5;
    uint64_t year : 8;       
    uint64_t mouth : 4;      
    uint64_t day : 5;
    uint64_t Resv1 : 14; 
    uint64_t second : 6;        
    uint64_t Resv2 : 6;    
    uint64_t minuite : 6;
    uint64_t Resv3 : 2;      
    uint64_t hour : 5;
    uint64_t Resv4 : 3;
} MsgTime;

//凯翼IMC1
typedef struct
{
    uint64_t OdometerInfo : 24;       //里程信息 km
    uint64_t Resv0 : 40;              //
} msgICM1;
#endif

///uds

typedef struct 
{
    uint32_t ID;                //MSG ID
    uint8_t CommCnt;            //信号计数器
    uint8_t LostLimitTimes;     //报文丢失限值
    uint16_t MsgCycleTime;      //报文周期
    uint8_t RollCommCntFlag;    //rolling counter flage
    uint8_t ChecksumFlag;       //check flage
    uint8_t Validflage;
    uint8_t Validflage1;
    uint8_t RollPreviousInit;
    uint8_t RollPreviousCount;
    uint32_t RollPreviousTime;
    uint32_t PreviousMsgTime;
} CommDetectNode;

#define SIG_VEH_SPD_INVALID 0x0   //车速invalid
#define SIG_YAW_RATE_INVALID 0x1  //横摆角速度invalid
#define SIG_STR_ANGLE_INVALID 0x2 //方向盘转角invalid
#define SIG_GEAR_POS_INVALID 0x3  //档位invalid
#define SIG_IGN_STATE_INVALID 0x4 //点火信号invalid
#define SIG_CAMERA_LOST 0x5       //摄像头信号丢失

// extern void CommDetectIncrease(uint32_t MsgID);
// extern void CommDetectClear(void);

bool CommTimeoutDetect(uint32_t MsgID);
void CommDetectIncrease(void);
void CommDetectClear(uint32_t MsgID);

bool CommCheckDetect(uint32_t MsgID,int type);
bool check_0x127(void *data);
bool check_0x2E9(void *data);
bool check_0x2A4(void *data);

///uds end

typedef struct tgRadarInfo1
{
    uint64_t frame_cnt : 8;
    uint64_t resv0 : 11;
    uint64_t tx2_obj_num : 8;
    uint64_t tx1_obj_num : 8;
    uint64_t object_cnt : 8;//max 128
    uint64_t volt : 10;//voltage*10,max 204.8
    uint64_t temp : 11; //gMCUTemp*10,max 204.8
} msgRadarInfo_301;

typedef struct tgRadarInfo2
{
    uint64_t resv0 : 9;
    uint64_t can1_busoff_cnt : 3;
    uint64_t can0_busoff_cnt : 3;
    uint64_t time_gap_pcan_send : 8;
    uint64_t time_gap_vcan_send : 8;
    uint64_t time_gap_data_proc : 8;
    uint64_t time_gap_signal_proc : 8;
    uint64_t can1_send_err_cnt : 5;
    uint64_t can0_send_err_cnt : 6;
    uint64_t can1_reinit_cnt : 3;
    uint64_t can0_reinit_cnt : 3;
} msgRadarInfo_302;
typedef struct tgRadarInfo4
{
    uint64_t resv0 : 14;
    uint64_t reference_pll_status : 1; //1-lock 0-unlock
    uint64_t fmcw_pll_status : 1;      //1-lock 0-unlock
    uint64_t tx2_power : 8;            //天线2的发射功率
    uint64_t tx1_power : 8;            //天线1的发射功率
    uint64_t system_time : 32;         //系统时间
} msgRadarInfo_304;

typedef struct tgRadarDtcInfo
{
    uint64_t resv0 : 62;
    uint64_t dtc_2 : 1;
    uint64_t dtc_1 : 1;
} msgRadarDtc_302;

typedef struct tgRadarInfo3
{
    uint64_t sw_byte3 : 8;
    uint64_t sw_byte2 : 8;
    uint64_t sw_byte1 : 8;
    uint64_t sw_byte0 : 8;
    uint64_t hw_byte3 : 8;
    uint64_t hw_byte2 : 8;
    uint64_t hw_byte1 : 8;
    uint64_t hw_byte0 : 8;
} msgRadarInfo_303;

//速度上报结构体
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t ecUSpeed     	: 16;
    uint64_t EstCarSpeedV1  : 16;
    uint64_t EstCarSpeedV2  : 16;
    uint64_t resv           : 16;
#else

    uint64_t resv           : 8;
    uint64_t installElevated: 8;
    uint64_t yawRate        : 16;
    uint64_t EstCarSpeedV1  : 16;
    uint64_t ecUSpeed       : 16;
#endif
}stSpeedInfo;


//速度上报结构体
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t p0  : 16;
    uint64_t p1  : 16;
    uint64_t p2  : 16;
    uint64_t p3  : 16;
#else

    uint64_t p3  : 16;
    uint64_t p2  : 16;
    uint64_t p1  : 16;
    uint64_t p0  : 16;
#endif
}stNve;

typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t swBestFeqCnt  	: 16;
    uint64_t startFeqCnt  	: 4;
    uint64_t sw_statue 	  	: 4;
    uint64_t value_LR  		: 4;
    uint64_t value_SR  		: 4;
    uint64_t res  			: 32;
#else
    uint64_t res  			:24;
    uint64_t swFeqSilentCnt : 4;
    uint64_t nveDiff  		: 4;
    uint64_t valueSR  		: 4;
    uint64_t valueLR  		: 4;
    uint64_t swStatue 	  	: 4;
    uint64_t startFeqCnt  	: 4;
    uint64_t swBestFeqCnt  	: 16;
#endif
}stjamming;

//信息查询结构体
typedef struct
{
#if IS_BIG_ENDIAN == 1
    uint64_t queryVersion : 1;
    uint64_t queryRadarSN : 1;
    uint64_t queryInstallClacInfo : 1;
    uint64_t queryWatchDogState : 1;
    uint64_t resv : 60;
#else

    uint64_t resv         			: 60;
    uint64_t queryWatchDogState  	:  1;
    uint64_t queryInstallClacInfo  	:  1;
    uint64_t queryRadarSN 			:  1;
    uint64_t queryVersion 			:  1;
#endif
}stQueryMsg;

//版本信息结构体
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t softVersion 	: 16;
    uint64_t hwVersion   	: 16;
    uint64_t BootVersion 	: 16;
    uint64_t APcalcVersion 	: 8;
    uint64_t resv        	: 8;
#else
    uint64_t resv        	: 8;
    uint64_t APcalcVersion 	: 8;
    uint64_t BootVersion 	: 16;
    uint64_t hwVersion   	: 16;
    uint64_t softVersion 	: 16;
#endif
}stVersionMsg;

//版本信息结构体
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t cmd_type    : 8;
    uint64_t softVersion : 16;
    uint64_t hwVersion   : 16;
    uint64_t resv        : 24;
#else
	uint64_t resv		 : 24;
	uint64_t hwVersion	 : 16;
	uint64_t softVersion : 16;
	uint64_t cmd_type	 : 8;
#endif
}ETC_stVersionMsg;

//雷达配置结构体
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint8_t outputMode  :  2;
    uint8_t carVelMode  :  2;
    uint8_t workMode    :  4;
#else
	uint8_t workMode	:  4;
	uint8_t carVelMode	:  2;
	uint8_t outputMode	:  2;
#endif
}ETC_stRadarModeMsg;

//雷达配置结构体
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t outputMode               : 2;
    uint64_t carVelMode               : 2;
    uint64_t workMode                 : 4;
    uint64_t sendExtInfo              : 2;
    uint64_t estimatedSpeedCfgExpand  : 2;
    uint64_t angleMapperMode  		  : 2;
    uint64_t sendYawRateMode		  : 2;
    uint64_t canBaudrateMode		  : 2;
    uint64_t sendExtInfo2 			  : 2;
    uint64_t dbgMode                  : 2;
    uint64_t cohesionMode             : 2;
    uint64_t canMode                  : 2;
    uint64_t protVer                  : 4;
    uint64_t resv        			  : 34;
#else
    uint64_t resv                     : 34;
    uint64_t protVer                  : 4;
    uint64_t canMode                  : 2;
    uint64_t cohesionMode             : 2;
    uint64_t dbgMode                  : 2;
    uint64_t sendExtInfo2 			  : 2;
    uint64_t canBaudrateMode		  : 2;
    uint64_t sendYawRateMode		  : 2;
    uint64_t angleMapperMode  		  : 2;
    uint64_t estimatedSpeedCfgExpand  : 2;
    uint64_t sendExtInfo              : 2;
    uint64_t workMode                 : 4;
    uint64_t carVelMode               : 2;
    uint64_t outputMode               : 2;
#endif
}stWorkModeMsg;

//雷达启动帧
typedef struct
{
#if IS_BIG_ENDIAN==1
	uint64_t radarId  	 :  4;
	uint64_t swtResetCnt :  28;
	uint64_t resetCnt	 :  32;
#else
	uint64_t resetCnt	 :  32;
	uint64_t swtResetCnt :  28;
	uint64_t radarId  	 :  4;
#endif
}stRadarStartMsg;

//雷达配置结构体
typedef struct
{
#if IS_BIG_ENDIAN == 1
    uint64_t passWord : 40;
    uint64_t cmd : 8;
    uint64_t value1 : 8; //cmd =1 时 为 hight
    uint64_t value2 : 8; //cmd =1 时 为Angle
#else

	uint64_t value2 	 :	8; //cmd =1 时 为Angle
	uint64_t value1 	 :	8; //cmd =1 时 为 hight
	uint64_t cmd		 :	8;
	uint64_t passWord	 :	40;
	
#endif
}ETC_stWorkModeMsg;

//目标列表头信息结构体
typedef struct
{
#if IS_BIG_ENDIAN==1

    uint64_t objectNum   : 8;   //目标数量
    uint64_t measCounter : 16;  //周期计数
    uint64_t version     : 4;   //CAN协议版本
    uint64_t AgcSW       : 1;   //AGC开关
    uint64_t noise       : 11;  //底噪
    uint64_t senceFlag   : 16;  //场景标识
    uint64_t resv2       : 8;  //保留
#else
	uint64_t resv2		 : 8;  //保留
	uint64_t senceFlag   : 16;  //场景标识
	uint64_t noise       : 11;  //底噪
	uint64_t AgcSW       : 1;   //AGC开关
	uint64_t version     : 4;   //CAN协议版本
	uint64_t measCounter : 16;  //周期计数
	uint64_t objectNum   : 8;   //目标数量

#endif
}stObjStartMsg;

//服务校准信息
typedef union
{
    struct stServiceCaliDeb1
    {
        uint64_t leastRFactor3  : 8;  //
        uint64_t caliAngle3	        :12;  //阶段3
        uint64_t leastRFactor2  : 8;  //
        uint64_t caliAngle2	        :12;  //阶段2
        uint64_t leastRFactor1  : 8;  //
        uint64_t caliAngle1	        :12;  //阶段1
        uint64_t packnum	    : 4;  //包信息
    }pack1;
    struct stServiceCaliDeb2
    {
        uint64_t leastRFactor6  : 8;  //
        uint64_t caliAngle6	        :12;  //阶段6
        uint64_t leastRFactor5  : 8;  //
        uint64_t caliAngle5	        :12;  //阶段5
        uint64_t leastRFactor4  : 8;  //
        uint64_t caliAngle4	        :12;  //阶段4
        uint64_t packnum	    : 4;  //包信息
    }pack2;
    struct stServiceCaliDeb3
    {
        uint64_t elapsedTime        :16;
        uint64_t calcState	        : 8;  //状态
        uint64_t currentObjNumber   : 8;
        uint64_t dbaaAllObjNumber   :16;
        uint64_t warningInfo        : 8;  //
        uint64_t caliDepthStatue    : 4;
        uint64_t packnum	        : 4;  //包信息
    }pack3;
    struct stServiceCaliDeb4
    {
        uint64_t startTime          :60;  //启动时间
        uint64_t packnum	        : 4;  //包信息
    }pack4;
    struct stServiceCaliDeb5
    {
        uint64_t ObjFlag            :60;  //启动时间
        uint64_t packnum	        : 4;  //包信息
    }pack5;
    struct stServiceCaliDeb6
    {
        uint64_t flag               :32;  //
        uint64_t res                :28;  //
        uint64_t packnum	        : 4;  //包信息
    }objFlag;
}stServiceCaliDebMsg;

//目标列表头信息结构体
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t PassWord    : 40;  //保留
    uint64_t objectNum   : 24;   //目标数量
#else
	uint64_t objectNum	 : 24;	 //目标数量
	uint64_t PassWord	 : 40;	//保留
#endif
}ETC_stObjStartMsg;

//目标信息结构体
typedef union//struct
{
    struct stObjPolor
    {
    #if IS_BIG_ENDIAN==1
        uint64_t objId    :  8;  //目标ID
        uint64_t range    : 13;  //距离
        uint64_t velocity : 11;  //速度
        uint64_t angle    : 10;  //角度
        uint64_t latVel   :  9;  //横向速度
        uint64_t resv     :  2;
        uint64_t dynProp  :  3;  //运动状态
        uint64_t snr      :  8;  //SNR
    #else
        uint64_t snr	  :  8;  //SNR
        uint64_t dynProp  :  3;  //运动状态
        uint64_t resv	  :  2;
        uint64_t latVel   :  9;  //横向速度
        uint64_t angle	  : 10;  //角度
        uint64_t velocity : 11;  //速度
        uint64_t range	  : 13;  //距离
        uint64_t objId	  :  8;  //目标ID
    #endif
    }polor;

    struct stObjCartesian
    {
    #if IS_BIG_ENDIAN==1
        uint64_t objId    :  8;  //目标ID
        uint64_t rangeY   : 13;  //纵向距离
        uint64_t speedY   : 11;  //纵向速度
        uint64_t rangeX   : 10;  //横向距离
        uint64_t speedX   :  9;  //横向速度
        uint64_t resv     :  2;
        uint64_t dynProp  :  3;  //运动状态
        uint64_t snr      :  8;  //SNR
    #else
        uint64_t snr	  :  8;  //SNR
        uint64_t dynProp  :  3;  //运动状态
        uint64_t resv	  :  2;
        uint64_t speedX   :  9;  //纵向距离
        uint64_t rangeX	  : 10;  //纵向速度
        uint64_t speedY   : 11;  //横向距离
        uint64_t rangeY	  : 13;  //横向速度
        uint64_t objId	  :  8;  //目标ID
    #endif
    }cartesian;
    
}stObjInfoMsg;

typedef struct
{
    uint64_t msg_cnt : 2;       //消息计数
    uint64_t resv2 : 1;         //占位
    uint64_t cipv_flag : 1;     //cipv标志
    uint64_t resv1 : 2;         //占位
    uint64_t aeb_cipv_flag : 1; //aeb cipv标志
    uint64_t acc_cipv_flag : 1; //acc cipv标志
    uint64_t target_id : 8;     //目标id
    uint64_t vel_lat : 12;      //横向速度
    uint64_t vel_lon : 12;      //纵向速度
    uint64_t pos_lat : 12;      //横向位置
    uint64_t pos_lon : 12;      //纵向位置

} stTargetInfoAMsg;

typedef struct
{
    uint64_t msg_cnt : 2;       //消息计数
    uint64_t target_type : 6;   //目标类型
    uint64_t prob_of_exist : 2; //存在可能性
    uint64_t resv1 : 22;        //占位
    uint64_t dyn_prob : 3;      //动态属性
    uint64_t resv2 : 2;         //占位
    uint64_t meas_state : 3;    //测量状态
    uint64_t target_id : 8;     //目标id
    uint64_t resv3 : 4;         //占位
    uint64_t acc_lon : 12;      //纵向加速度
} stTargetInfoBMsg;

typedef struct
{
    uint64_t resv1 : 40;     //占位
    uint64_t cipv_id : 8;    //cipv id
    uint64_t resv2 : 10;     //占位
    uint64_t number_obj : 6; //目标数量
} stRadarHeaderMsg;

typedef struct
{
    uint64_t resv1 : 31;               //reserved
    uint64_t rad_yawrate_valid : 1;    //Radar Yaw Rate Raw Validity
    uint64_t sig_gear_invalid : 1;     //Invalid Gear position signal
    uint64_t sig_ign_invalid : 1;      //Invalid IGN Sta signal
    uint64_t radar_block : 1;          //Radar block
    uint64_t mount_error : 1;          //Radar mount position shift error
    uint64_t resv2 : 4;                //reserved
    uint64_t mcu_error : 1;            //MCU Error
    uint64_t mcu_memory_error : 1;     //MCU Memory Error
    uint64_t g_sensor_fault : 1;       //G-sensor fault
    uint64_t rf_sensor_fault : 1;      //RF sensor fault
    uint64_t mis_calib : 1;            //Missing calibration
    uint64_t sig_veh_spd_invalid : 1;  //Invalid Vehicle Speed signal
    uint64_t sig_yawrate_invalid : 1;  //Invalid Yaw Rate signal
    uint64_t sig_str_ang_invalid : 1;  //Invalid Steering Angle signal
    uint64_t rf_vol_1_high : 1;        //RF voltage1 too high
    uint64_t rf_vol_2_low : 1;         //RF voltage2 too low
    uint64_t rf_vol_2_high : 1;        //RF voltage2 too high
    uint64_t bat_vol_high : 1;         //Battery voltage too high
    uint64_t bat_vol_low : 1;          //Battery voltage too low
    uint64_t rf_temp_high : 1;         //RF temperature too high
    uint64_t rf_temp_low : 1;          //RF temperature too low
    uint64_t mcu_master_clk_error : 1; //MCU Master Clock Error
    uint64_t rad_sleep_flag : 1;       //Radar_Sleep_Flag
    uint64_t bus_off : 1;              //Communication Bus Off
    uint64_t camera_lost : 1;          //Lost Communication with Image processing module
    uint64_t mcu_temp_high : 1;        //MCU temperature too high
    uint64_t mcu_temp_low : 1;         //MCU temperature too low
    uint64_t mcu_vol_high : 1;         //MCU voltage too high
    uint64_t mcu_vol_low : 1;          //MCU voltage too low
    uint64_t rf_vol_1_low : 1;         //RF voltage1 too low
} stRadarStatusMsg;

typedef struct
{
	uint64_t radar_yaw_rate_raw : 16; //Radar Yaw Rate Raw data
	uint64_t resv0 			    : 48;              //占位
} stRadarVehInfo2Msg;

typedef struct
{
	uint64_t radar_yaw_rate_raw : 16; //Radar Yaw Rate Raw data
	uint64_t radar_yaw_Cali_Flag:  8;              //占位
	uint64_t absv 			    : 16;              //占位
	uint64_t speed 			    : 16;              //占位
	uint64_t resv0 			    :  8;              //占位
} stCtRadarVehInfo2Msg;

typedef struct
{
    uint64_t resv1 : 16;            //占位
    uint64_t str_angle : 16;        //方向盘转角
    uint64_t yaw_rate : 16;         //横摆角速度
    uint64_t str_angle_valid : 1;   //方向盘转角有效性
    uint64_t resv2 : 2;             //方向盘转角有效性
    uint64_t yaw_rate_valid : 1;    //横摆角速度有效性
    uint64_t vehicle_spd : 11;      //车速
    uint64_t vehicle_spd_valid : 1; //车速有效性
} stVehicleInfo1Msg;

typedef struct
{
    uint64_t resv1 : 52;          //占位
    uint64_t ign_sta : 1;         //点火状态
    uint64_t ign_sta_invalid : 1; //点火状态有效性
    uint64_t resv2 : 5;           //方向盘转角有效性
    uint64_t gear_sta_valid : 1;  //档位有效性
    uint64_t gear_sta : 3;        //档位
    uint64_t resv3 : 1;           //占位
} stVehicleInfo2Msg;

//原始目标信息扩展结构体
typedef struct
{
#if IS_BIG_ENDIAN == 1
    uint64_t elevated : 8;  //高度角
    uint64_t range : 13;    //距离
    uint64_t velocity : 11; //速度
    uint64_t angle : 10;    //角度
    uint64_t mag : 9;       //信号mag值
    uint64_t resv : 5;
    uint64_t snr : 8; //SNR
#else
    uint64_t snr : 8;       //SNR
    uint64_t TxCh : 2;      //区分是哪个天线
    uint64_t resv : 3;
    uint64_t mag : 9;       //信号mag值
    uint64_t angle : 10;    //角度
    uint64_t velocity : 11; //速度
    uint64_t range : 13;    //距离
    uint64_t elevated : 8;  //高度角
#endif
}stRawObjInfoMsg;

//原始目标信息扩展结构体
typedef struct
{
#if IS_BIG_ENDIAN == 1
    uint64_t elevated : 8;  //高度角
    uint64_t range : 13;    //距离
    uint64_t velocity : 11; //速度
    uint64_t angle : 10;    //角度
    uint64_t mag : 9;       //信号mag值
    uint64_t resv : 5;
    uint64_t snr : 8; //SNR
#else
    uint64_t TxCh  		: 2;    //区分是哪个天线 -- 0-未知、1-Tx1、2-Tx3、3-未进跟踪的目标的目标
    //uint64_t using 		: 1;	//跟踪是否使用 //可以使用速度最大值代替 ，直观可以看到
    uint64_t elevated 	: 8;  	//高度角  精度0.3
    uint64_t range 		: 13;   //距离     精度0.05      范围0-409.55
    uint64_t velocity 	: 12; 	//速度  	精度0.05   -102.375 ~ 102.3
    uint64_t angle 		: 12;   //角度     精度0.05   -102.375 ~ 102.3
    uint64_t snr 		: 10;   //SNR   精度0.2
    uint64_t mag 		: 7;    //mag值  精度1	   0-127
#endif
}stRawObjInfoMsg_V5;  //要注意协议反转

//原始目标信息扩展结构体
typedef struct
{
	uint64_t rcs		:10;
    uint64_t firstvelocity :12; 	//速度  	精度0.05   -102.375 ~ 102.3
	uint64_t res		:42;
}stRawPhaseObjInfoMsg_V5;  //要注意协议反转


//目标信息扩展结构体
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t objId    			: 8;  //目标ID
    uint64_t rcs        		: 8;  //rcs
    uint64_t objHeightAngle    	: 8;  //高度角
    uint64_t trkReiability     	: 8;
    uint64_t obj_acc_x     		: 8;
    uint64_t obj_acc_y	     	: 8;
    uint64_t objSort     		: 16;  //高度角
#else
	uint64_t objSort			: 16;  //高度角
	uint64_t obj_acc_y			: 8;
	uint64_t obj_acc_x			: 8;
	uint64_t trkReiability		: 8;
	uint64_t objHeightAngle 	: 8;  //高度角
	uint64_t rcs			    : 8;  //rcs
	uint64_t objId				: 8;  //目标ID
#endif
}stObjExtMsg;

//目标信息扩展结构体 2
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t objId    			: 8;  //目标ID
    uint64_t objlength    		: 8;  //高度
    uint64_t objWidth    		: 8;  //宽度
    uint64_t objXRms    		: 5;  //rms
    uint64_t objYRms    		: 5;  //rms
    uint64_t objVxRms    		: 5;  //rms
    uint64_t objVyRms    		: 5;  //rms
    uint64_t objAxRms    		: 5;  //rms
    uint64_t objAyRms    		: 5;  //rms
    uint64_t objAngleRms    	: 5;  //rms
    uint64_t objMeasState    	: 3;  //测量状态、卡尔曼状态
    uint64_t res		    	: 2;  //
#else
    uint64_t res		    	: 2;  //
    uint64_t objMeasState    	: 3;  //测量状态、卡尔曼状态
    uint64_t objAngleRms    	: 5;  //rms
    uint64_t objAyRms    		: 5;  //rms
    uint64_t objAxRms    		: 5;  //rms
    uint64_t objVyRms    		: 5;  //rms
    uint64_t objVxRms    		: 5;  //rms
    uint64_t objYRms    		: 5;  //rms
    uint64_t objXRms    		: 5;  //rms
    uint64_t objWidth    		: 8;  //宽度
    uint64_t objlength    		: 8;  //高度
    uint64_t objId    			: 8;  //目标ID
#endif
}stObjExtMsg2;


//雷达配置，大陆协议
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t storeInNVMValid    : 1;  //允许存储到存储器中
    uint64_t sortIndexValid     : 1;  //允许改变排序列表
    uint64_t sendExtInfoValid   : 1;  //允许改变拓展信息操作
    uint64_t sendQualityValid   : 1;  //允许改变发送重要信息
    uint64_t outTypeValid       : 1;  //允许改变输出类型
    uint64_t radarPowerValid    : 1;  //允许改变雷达输出功率
    uint64_t sensorIdValid      : 1;  //允许传感器ID变化
    uint64_t maxDistanceValid   : 1;  //允许改变最多距离
    uint64_t maxDistanceValue   : 10; //最大距离
    uint64_t res3               : 14; //
    uint64_t radarPowerValue    : 3;  //雷达输出功率
    uint64_t outputTypeValue    : 2;  //输出类型
    uint64_t sensorIdValue      : 3;  //雷达ID号
    uint64_t storeInNVM         : 1;  //参数是否保存到非易失存储器
    uint64_t sortIndexValue     : 3;  //排序方式
    uint64_t sendExtInfo        : 1;  //是否发送拓展信息
    uint64_t sendQualityInfo    : 1;  //是否发送重要信息
    uint64_t ctrlRelay          : 1;  //控制继电器消息
    uint64_t ctrlRelayValid     : 1;  //是否允许控制继电器
    uint64_t res2               : 4;  //
    uint64_t rcsThreshold       : 3;  //RCS阈值
    uint64_t rcsThresholdValid  : 1;  //是否允许设置RCS阈值
    uint64_t res1               : 8;
#else
    uint64_t res1               : 8;
    uint64_t rcsThresholdValid  : 1;  //是否允许设置RCS阈值
    uint64_t rcsThreshold       : 3;  //RCS阈值， 0x0：标准， 0x1：高灵敏度
    uint64_t res2               : 4;  //
    uint64_t ctrlRelayValid     : 1;  //是否允许控制继电器
    uint64_t ctrlRelay          : 1;  //控制继电器消息
    uint64_t sendQualityInfo    : 1;  //是否发送重要信息
    uint64_t sendExtInfo        : 1;  //是否发送拓展信息
    uint64_t sortIndexMethod    : 3;  //排序方式， 0x0：没排序， 0x1：按距离排序， 0x2：按RCS排序
    uint64_t storeInNVM         : 1;  //参数是否保存到存储器
    uint64_t sensorIdValue      : 3;  //雷达ID号， 0-7
    uint64_t outputType         : 2;  //输出类型， 0x0：空， 0x1：发送目标， 0x2：发送集群
    uint64_t radarPower         : 3;  //雷达输出功率，0x0：标准， 0x1：-3dB Tx gain， 0x2：-6dB Tx gain， 0x3：-9dB Tx gain
    uint64_t res3               : 14; //
    uint64_t maxDistance        : 10; //最大距离， 0-2046m
    uint64_t maxDistanceValid   : 1;  //允许改变最多距离
    uint64_t sensorIdValid      : 1;  //允许传感器ID变化
    uint64_t radarPowerValid    : 1;  //允许改变雷达输出功率
    uint64_t outTypeValid       : 1;  //允许改变输出类型
    uint64_t sendQualityValid   : 1;  //允许改变发送重要信息
    uint64_t sendExtInfoValid   : 1;  //允许改变拓展信息操作
    uint64_t sortIndexValid     : 1;  //允许改变排序列表
    uint64_t storeInNVMValid    : 1;  //允许存储到存储器中
#endif
}Conti_stSetRadarCfg;

//雷达状态输出，大陆协议
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t nvmWriteStatus     : 1;  //写参数状态， 0：失败， 1：成功
    uint64_t nvmReadStatus      : 1;  //读参数状态， 0：失败， 1：成功
    uint64_t res5               : 6;  //测量计数器
    uint64_t maxDistanceCfg     : 10; //雷达最远扫描距离
    uint64_t res4    		    : 12; //
    uint64_t radarPowerCfg      : 3;  //雷达发射功率， 0：标准， 1：-3dB Tx gain， 2：-6dB Tx gain， 3：-9dB Tx gain
    uint64_t sortIndexCfg       : 3;  //排序方式， 0：无， 1：按距离排序， 2：按RCS排序
    uint64_t res3               : 1;  //
    uint64_t sensorIdCfg        : 3;  //雷达ID
    uint64_t motionRxState      : 2;  //速度和偏航角速度输入标记的状态， 0：正常， 1：速度缺失， 2：偏航角速度缺失， 3：两者皆缺失
    uint64_t sendExtInfoCfg     : 1;  //是否发送拓展信息
    uint64_t sendQualityInfoCfg : 1;  //是否发送重要信息
    uint64_t outputTypeCfg      : 2;  //0：空， 1：发送目标， 2：发送集群
    uint64_t ctrlRelayCfg       : 1;  //继电器控制
    uint64_t res2               : 12; //
    uint64_t rcsThresholdCfg    : 3;  //RCS阈值， 0：标准， 1：高灵敏度
    uint64_t res1               : 2;  //
#else
    uint64_t res1               : 2;  //
    uint64_t rcsThresholdCfg    : 3;  //RCS阈值， 0：标准， 1：高灵敏度
    uint64_t res2               : 12; //
    uint64_t ctrlRelayCfg       : 1;  //继电器控制
    uint64_t outputTypeCfg      : 2;  //0：空， 1：发送目标， 2：发送集群
    uint64_t sendQualityInfoCfg : 1;  //是否发送重要信息
    uint64_t sendExtInfoCfg     : 1;  //是否发送拓展信息
    uint64_t motionRxState      : 2;  //速度和偏航角速度输入标记的状态， 0：正常， 1：速度缺失， 2：偏航角速度缺失， 3：两者皆缺失
    uint64_t sensorIdCfg        : 3;  //雷达ID
    uint64_t res3               : 1;  //
    uint64_t sortIndexCfg       : 3;  //排序方式， 0：无， 1：按距离排序， 2：按RCS排序
    uint64_t radarPowerCfg      : 3;  //雷达发射功率， 0：标准， 1：-3dB Tx gain， 2：-6dB Tx gain， 3：-9dB Tx gain
    uint64_t res4    		    : 12; //
    uint64_t maxDistanceCfg     : 10; //雷达最远扫描距离
    uint64_t res5               : 6;  //测量计数器
    uint64_t nvmReadStatus      : 1;  //读参数状态， 0：失败， 1：成功
    uint64_t nvmWriteStatus     : 1;  //写参数状态， 0：失败， 1：成功
#endif
}Conti_stRadarStatusMsg;

//集群列表状态，大陆协议
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t nofTargetsNear     : 8;  //近距离目标数
    uint64_t nofTargetsFar      : 8;  //远距离目标数
    uint64_t measCounter    	: 16; //测量计数器
    uint64_t interfaceVersion   : 4;  //接口版本
    uint64_t res    		    : 28; //
#else
    uint64_t res    		    : 28; //
    uint64_t interfaceVersion   : 4;  //接口版本
    uint64_t measCounter    	: 16; //测量计数器
    uint64_t nofTargetsFar      : 8;  //远距离目标数
    uint64_t nofTargetsNear     : 8;  //近距离目标数
#endif
}Conti_stRawListStatusMsg;

//集群基本信息，大陆协议
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t objId              : 8;  //目标ID
    uint64_t distLong           : 13; //纵向距离
    uint64_t res2               : 1;  //
    uint64_t distLat            : 10; //横向距离
    uint64_t vrelLong           : 10; //纵向速度
    uint64_t vrelLat            : 9;  //横向速度
    uint64_t res1               : 2;  //
    uint64_t dynProp            : 3;  //运动状态
    uint64_t rcs                : 8;  //散射截面
#else
    uint64_t rcs                : 8;  //散射截面
    uint64_t dynProp            : 3;  //运动状态
    uint64_t res1               : 2;  //
    uint64_t vrelLat            : 9;  //横向速度
    uint64_t vrelLong           : 10; //纵向速度
    uint64_t distLat            : 10; //横向距离
    uint64_t res2               : 1;  //
    uint64_t distLong           : 13; //纵向距离
    uint64_t objId              : 8;  //目标ID
#endif
}Conti_stRawListGeneralMsg;

//集群重要信息，大陆协议
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t objId              : 8;  //集群数ID
    uint64_t distLongRms        : 5;  //纵向距离标准差
    uint64_t distLatRms         : 5;  //横向距离标准差
    uint64_t vrelLongRms        : 5;  //纵向速度标准差
    uint64_t vrelLatRms         : 5;  //横向速度标准差
    uint64_t res2               : 1;  //
    uint64_t TargetPdH0         : 3;  //集群虚警概率
    uint64_t inValidState       : 5;  //集群的有效状态
    uint64_t ambigState         : 3;  //多普勒（径向速度）不确定的状态
    uint64_t res1               : 24;
#else
    uint64_t res1               : 24;
    uint64_t ambigState         : 3;  //多普勒（径向速度）不确定的状态
    uint64_t inValidState       : 5;  //集群的有效状态
    uint64_t targetPdH0         : 3;  //集群虚警概率
    uint64_t res2               : 1;  //
    uint64_t vrelLatRms         : 5;  //横向速度标准差
    uint64_t vrelLongRms        : 5;  //纵向速度标准差
    uint64_t distLatRms         : 5;  //横向距离标准差
    uint64_t distLongRms        : 5;  //纵向距离标准差
    uint64_t objId              : 8;  //集群数ID
#endif
}Conti_stRawListQualityMsg;

//目标列表头信息结构体（大陆协议）
typedef struct
{
#if IS_BIG_ENDIAN==1

    uint64_t objectNum   : 8;   //目标数量
    uint64_t measCounter : 16;  //周期计数
    uint64_t version     : 4;   //CAN协议版本
    uint64_t res         : 36;  //保留
#else
	uint64_t res         : 36;  //保留
	uint64_t version     : 4;   //CAN协议版本
	uint64_t measCounter : 16;  //周期计数
	uint64_t objectNum   : 8;   //目标数量
#endif
}Conti_stObjListStatusMsg;

//目标基本信息，大陆协议
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t objId    :  8;  //目标ID
    uint64_t rangeX   : 13;  //距离X
    uint64_t rangeY   : 11;  //距离Y
    uint64_t speedX   : 10;  //速度X
    uint64_t speedY   :  9;  //速度Y
    uint64_t resv     :  2;
    uint64_t dynProp  :  3;  //运动状态
    uint64_t rcs      :  8;  //散射截面
#else
    uint64_t rcs      :  8;  //散射截面
    uint64_t dynProp  :  3;  //运动状态
    uint64_t res      :  2;
    uint64_t verlLat  :  9;  //横向速度
    uint64_t verlLong : 10;  //纵向速度
    uint64_t distLat  : 11;  //横向距离
    uint64_t distLong : 13;  //纵向距离
    uint64_t objId	  :  8;  //目标ID
#endif
}Conti_stObjListGerneralMsg;

//目标重要信息，大陆协议
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t objId    			: 8;  //目标ID
    uint64_t distLongRms    	: 5;  //高度
    uint64_t distLatRms    		: 5;  //宽度
    uint64_t verlLongRms    	: 5;  //rms
    uint64_t verlLatRms    		: 5;  //rms
    uint64_t arelLongRms    	: 5;  //rms
    uint64_t arelLatRms    		: 5;  //rms
    uint64_t orientationRms    	: 5;  //rms
    uint64_t res2    		    : 5;  //rms
    uint64_t probOfExist    	: 3;  //rms
    uint64_t objMeasState    	: 3;  //测量状态、卡尔曼状态
    uint64_t res1		    	: 2;  //
#else
    uint64_t res1		    	: 10;  //
    uint64_t objMeasState    	: 3;  //测量状态、卡尔曼状态
    uint64_t probOfExist        : 3;  //存在概率
    uint64_t res2               : 5;  //
    uint64_t orientationRms    	: 5;  //rms
    uint64_t arelLatRms    		: 5;  //rms
    uint64_t arelLongRms    	: 5;  //rms
    uint64_t verlLatRms    		: 5;  //rms
    uint64_t verlLongRms    	: 5;  //rms
    uint64_t distLatRms    		: 5;  //rms
    uint64_t distLongRms    	: 5;  //rms
    uint64_t objId    			: 8;  //目标ID
#endif
}Conti_stObjListQualityMsg;

//目标拓展信息，大陆协议
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t objId    			: 8;  //目标ID
    uint64_t arelLong    		: 5;  //高度
    uint64_t objRyRms    		: 5;  //宽度
    uint64_t objVxRms    		: 5;  //rms
    uint64_t objVyRms    		: 5;  //rms
    uint64_t objAxRms    		: 5;  //rms
    uint64_t objAyRms    		: 5;  //rms
    uint64_t objAngleRms    	: 5;  //rms
    uint64_t res2    		    : 5;  //rms
    uint64_t probOfExist    	: 3;  //rms
    uint64_t objMeasState    	: 3;  //测量状态、卡尔曼状态
    uint64_t res1		    	: 2;  //
#else
    uint64_t objWidth           : 8;  //目标宽度
    uint64_t objLength    	    : 8;  //目标长度
    uint64_t res1    		    : 6;  //
    uint64_t orientationAngle   : 10; //目标方位角
    uint64_t objClass    		: 3;  //目标类型
    uint64_t res2    		    : 1;  //
    uint64_t arelLat    		: 9;  //横向加速度
    uint64_t arelLong    		: 11; //纵向加速度
    uint64_t objId    			: 8;  //目标ID
#endif
}Conti_stObjListExtendMsg;


//4F5-汽车信息接入结构体
typedef struct
{
#if IS_BIG_ENDIAN==1

    uint64_t velocityValid 	: 1;//车速有效
    uint64_t doorSigValid   : 1;//开门信号有效
    uint64_t turnSigValid   : 1;//转向灯有效
    uint64_t GearValid 		: 1;//挡位有效
    uint64_t curvatureValid	: 1;//曲率有效
    uint64_t resv1        	: 4;
    uint64_t velocity        	: 15;//速度
    uint64_t doorLF        	: 1;//左前门
    uint64_t doorRF        	: 1;//右前门
    uint64_t doorLR        	: 1;//左后门
    uint64_t doorRR        	: 1;//右后门
    uint64_t turnLeft       : 1;//左转灯
    uint64_t turnRignt      : 1;//右转灯
    uint64_t GearSig        : 2;//挡位信息
    uint64_t curvature        	: 16;//曲率
    uint64_t resv        		: 16;
#else
	uint64_t resv				: 16;
	uint64_t curvature			: 16;//曲率
	uint64_t GearSig		: 2;//挡位信息
	uint64_t turnRignt		: 1;//右转灯
	uint64_t turnLeft		: 1;//左转灯
	uint64_t doorRR 		: 1;//右后门
	uint64_t doorLR 		: 1;//左后门
	uint64_t doorRF 		: 1;//右前门
	uint64_t doorLF 		: 1;//左前门
	uint64_t velocity			: 15;//速度
	uint64_t resv1			: 4;
	uint64_t curvatureValid : 1;//曲率有效
	uint64_t GearValid		: 1;//挡位有效
	uint64_t turnSigValid	: 1;//转向灯有效
	uint64_t doorSigValid	: 1;//开门信号有效
	uint64_t velocityValid	: 1;//车速有效
#endif
}stInCarInfoMsg;

//40n-配置雷达安装信息
typedef struct
{
#if IS_BIG_ENDIAN==1

    uint64_t heightValid   	  :  1;//高度有效
    uint64_t angleHValid 	  :  1;//水平角有效
    uint64_t offsetHValid 	  :  1;//距离车轴偏移补偿
    uint64_t rcsOffsetValid   :  1;//RCS补偿有效
    uint64_t resv1        	  :  5;//保留
    uint64_t horizontalOffset :  6;//距离车轴偏移补偿
    uint64_t Height			  :  9;//安装高度
    uint64_t horizontalAngle  :  11;//安装水平角
    uint64_t rcsOffset        :  8; //RCS补偿值
    uint64_t resv2        	  : 21;//保留
#else
	uint64_t resv2			  : 5;//保留
    uint64_t rcsOffset1		  : 8;//RCS补偿值1
	uint64_t elevatedOffset	  : 8;//  //高度角补偿值
    uint64_t rcsOffset        :  8;//RCS补偿值
	uint64_t horizontalAngle  : 11;//安装水平角
	uint64_t Height 		  :  9;//安装高度
	uint64_t horizontalOffset :  6;//距离车轴偏移补偿
	uint64_t resv1			  :  4;//保留
	uint64_t elevatedOffsetValid  :  1;//保留
    uint64_t rcsOffsetValid   :  1;//RCS补偿有效
	uint64_t offsetHValid	  :  1;//距离车轴偏移补偿
	uint64_t angleHValid	  :  1;//水平角有效
	uint64_t heightValid	  :  1;//高度有效
#endif
}stInstallInfoMsg;


//42n-校准信息
typedef struct
{
#if IS_BIG_ENDIAN==1
	uint64_t CalcType      	: 4;//校准类型 0-安装标定 1-自标定
    uint64_t debugFlagValid   	: 1;//debugFlag
    uint64_t angleHValid 	: 1;//水平角有效
    uint64_t HistMaxCntValid 	: 1;//Hist最大个数
    uint64_t calcStateValid	: 1;//校准状态
	uint64_t installedObjValid 	: 1;//保留
    uint64_t debugFlag 			: 8;//debugFlag
    uint64_t caliFrame			: 7;//校准周期，实际花了多少帧校准
    uint64_t horizontalAngle  	: 11;//安装水平角
    uint64_t HistMaxCnt			: 4;//Hist最大个数
    uint64_t caliFrameValid		: 1;//保留
    uint64_t calcState			: 8;//校准状态
    uint64_t installedObj_x     : 6;//
    uint64_t installedObj_y		: 10;//
#else
    uint64_t installedObj_y		: 10;//
    uint64_t installedObj_x     : 6;//
    uint64_t calcState			: 8;//校准状态
    uint64_t caliFrameValid		: 1;//保留
    uint64_t HistMaxCnt			: 4;//Hist最大个数
    uint64_t horizontalAngle  	: 11;//安装水平角
    uint64_t caliFrame			: 7;//校准周期，实际花了多少帧校准
    uint64_t debugFlag 			: 8;//debugFlag
    uint64_t installedObjValid 	: 1;//保留
    uint64_t calcStateValid	: 1;//校准状态
    uint64_t HistMaxCntValid 	: 1;//Hist最大个数
    uint64_t angleHValid 	: 1;//水平角有效
    uint64_t debugFlagValid   	: 1;//debugFlag
    uint64_t CalcType      	: 4;//校准类型 0-安装标定 1-自标定
#endif
}stCalcInfoMsg;

//0x126 大陆协议车身信息
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t resv1			  :  8;     //保留
    uint64_t velocity		  : 13;     //速度
    uint64_t velocityValid	  :  1;     //速度有效标志
    uint64_t resv2			  : 42;     //保留
#else
	uint64_t resv2			  : 42;     //保留
    uint64_t velocityValid	  :  1;     //速度有效标志
    uint64_t velocity		  : 13;     //速度
    uint64_t resv1			  :  8;     //保留
#endif
}StrCarArs410EscStats;

//0x131 大陆协议车身信息
typedef struct
{
#if IS_BIG_ENDIAN==1
    uint64_t resv0			  : 13;     //保留
    uint64_t curvatureValid	  :  2;     //曲率有效标志
    uint64_t resv1		      : 17;     //保留
    uint64_t curvature	      : 16;     //曲率
    uint64_t resv2			  : 16;     //保留
#else
	uint64_t resv2			  : 16;     //保留
    uint64_t curvature	      : 16;     //曲率
    uint64_t resv1		      : 17;     //保留
    uint64_t curvatureValid	  :  2;     //曲率有效标志
    uint64_t resv0			  : 13;     //保留
#endif
}StrCarArs410Yrs1;

//耗时统计
typedef struct
{
#if IS_BIG_ENDIAN==1
	uint64_t T0      	: 16;
	uint64_t T1      	: 16;
	uint64_t T2      	: 16;
	uint64_t T3      	: 16;
#else
	uint64_t T3      	: 16;
	uint64_t T2      	: 16;
	uint64_t T1      	: 16;
	uint64_t T0      	: 16;
#endif
}stTimeInfoMsg;


//雷达模式切换
typedef struct
{
#if IS_BIG_ENDIAN==1

    uint64_t password  		:  24;
    uint64_t radar_mode  	:  1;
    uint64_t resv        	: 39;
#else
	
	uint64_t resv			: 39;
	uint64_t radar_mode 	:  1;
	uint64_t password		:  24;

#endif
}stRadarModeMsg;

//雷达升级操作结构体
typedef struct
{
#if IS_BIG_ENDIAN==1

    uint64_t passWork  	 : 48;
    uint64_t operateMode :  8;
    uint64_t radarID     :  8;
#else
	uint64_t radarID	 :	8;
	uint64_t operateMode :	8;
	uint64_t passWork	 : 48;
#endif
}stUpgradeRadarMsg;

//雷达结束帧
typedef struct
{
#if IS_BIG_ENDIAN==1
	uint64_t selfState 		: 4;
	uint64_t res  	 		: 1;
    uint64_t angleOffset 	: 11;//自标定补偿角度
    uint64_t selfEn  	 	: 4;
    uint64_t speedMode     	: 4;//1-OBD 2-自测速
    uint64_t frameIntervalTime		:8;//帧间隔时间
    uint64_t installAngleOffset 	: 11;//安装补偿角度
    uint64_t horizontalOffset		: 6; //横向偏移补偿
    uint64_t res2			:15;
#else

    // uint64_t res2 : 15;
    // uint64_t horizontalOffset : 6;    //横向偏移补偿
    uint64_t res2 : 11;               //用于显示2次跟踪数据之间的tick差值
    uint64_t horizontalOffset : 10;   //用于显示客户协议的帧间隔
    uint64_t installAngleOffset : 11; //安装补偿角度
    uint64_t frameIntervalTime : 8;   //帧间隔时间
    uint64_t speedMode : 4;           //1-OBD 2-自测速
    uint64_t selfEn : 4;
    uint64_t angleOffset : 11; //自标定补偿角度
    uint64_t installCaliEn : 1;
    uint64_t selfState : 4;
#endif
}stEndFrameMsg;

//调试
typedef struct
{
	uint64_t res3 : 16;
	uint64_t res2 : 16;
	uint64_t res1_0 :  8;
	uint64_t res1_1 :  8;
    uint64_t res0 	: 16;
}stDebMsg;

//雷达结同步信息
typedef struct
{
#if IS_BIG_ENDIAN==1

    uint64_t car_radiusCurvature	: 16;//曲率半径
    uint64_t car_YawRate 			: 16;//曲率
    uint64_t Temperature_mcu_valid	: 1; //mcu温度有效
    uint64_t Temperature_mcu     	: 11;//mcu温度
    uint64_t Temperature_mmic_valid	: 1;//mmic
    uint64_t Temperature_mmic     	: 11;//mmic
    uint64_t res2					: 8;
#else

	uint64_t res2					: 8;
	uint64_t Temperature_mmic		: 11;//mmic
	uint64_t Temperature_mmic_valid : 1;//mmic
	uint64_t Temperature_mcu		: 11;//mcu温度
	uint64_t Temperature_mcu_valid	: 1; //mcu温度有效
	uint64_t car_YawRate			: 16;//曲率
	uint64_t car_radiusCurvature	: 16;//曲率半径

#endif
}stRadarSynInfo;

//安装校准配置
typedef struct
{
#if IS_BIG_ENDIAN == 1
    uint64_t calcEn_valid : 1;                //使能
    uint64_t calcObjRange_valid : 1;          //距离
    uint64_t calcObjRangeThreshold_valid : 1; //距离阈值
    uint64_t calcObjAngle_valid : 1;          //角度
    uint64_t calcObjAngleThreshold_valid : 1; //角度阈值
    uint64_t AngleRangeValid : 1;             //校准状态
    uint64_t calcResultSendEn_valid : 1;
    uint64_t HistParamCfgValid : 1;
    uint64_t calcEn : 1;                //使能
    uint64_t calcResultSendFlag : 1;    //
    uint64_t SNSendFlag : 1;            //
    uint64_t calcObjRange : 13;         //探测目标距离
    uint64_t calcObjRangeThreshold : 8; //距离阈值
    uint64_t calcObjAngle : 8;          //实际角度
    uint64_t calcObjAngleThreshold : 8; //角度阈值
    uint64_t AngleRange : 8;            //标定角度范围 --
    uint64_t HistAngleThr : 4;          //直方图分辨率 --
    uint64_t HistValidThr : 4;          //直方图有效个数门限 --
#else
    uint64_t HistValidThr : 4;           //直方图有效个数门限 --
    uint64_t HistAngleThr : 4;           //直方图分辨率 --
    uint64_t AngleRange : 8;             //标定角度范围 --
    uint64_t calcObjAngleThreshold : 8;  //角度阈值
    uint64_t calcObjAngle : 8;           //实际角度
    uint64_t calcObjRangeThreshold : 8;  //距离阈值
    uint64_t calcObjRange : 13;          //探测目标距离
    uint64_t SNSendFlag : 1;             //
    uint64_t calcResultSendFlag : 1;     //
    uint64_t calcEn : 1;                 //使能
    uint64_t HistParamCfgValid : 1;
    uint64_t calcResultSendEn_valid : 1;
    uint64_t AngleRangeValid : 1;             //校准状态
    uint64_t calcObjAngleThreshold_valid : 1; //角度阈值
    uint64_t calcObjAngle_valid : 1;          //角度
    uint64_t calcObjRangeThreshold_valid : 1; //距离阈值
    uint64_t calcObjRange_valid : 1;          //距离
    uint64_t calcEn_valid : 1;                //使能有效位
#endif
} stRadarInstallCalcCfg;

//安装校准扩展配置
typedef struct
{
#if IS_BIG_ENDIAN == 1
    uint64_t AngleRangeValid : 1;   //
    uint64_t SendInfoEnValid : 1;   //
    uint64_t HistAngleThrValid : 1; //
    uint64_t HistValidThrValid : 1; //
    uint64_t NoObjCntThrValid : 1;  //
    uint64_t TimeoutThrValid : 1;   //
    uint64_t WaitThrValid : 1;
    uint64_t TxSelValid : 1;
    uint64_t AngleRange : 8;   //标定角度范围
    uint64_t SendInfoEn : 8;   //发送使能
    uint64_t HistAngleThr : 4; //直方图分辨率
    uint64_t HistValidThr : 4; //直方图有效个数门限
    uint64_t Res27 : 2;        //
    uint64_t NoObjCntThr : 6;  //无目标判定帧数
    uint64_t res55 : 2;        //
    uint64_t TimeoutThr : 6;   //超时帧数
    uint64_t WaitThr : 8;      //标定等待时间
    uint64_t res10 : 4;
    uint64_t TxSel : 4; //Tx选择
#else

    uint64_t TxSel : 4; //Tx选择
    uint64_t res10 : 4;
    uint64_t WaitThr : 8;      //标定等待时间
    uint64_t TimeoutThr : 6;   //超时帧数
    uint64_t res55 : 2;        //
    uint64_t NoObjCntThr : 6;  //无目标判定帧数
    uint64_t Res27 : 2;        //
    uint64_t HistValidThr : 4; //直方图有效个数门限
    uint64_t HistAngleThr : 4; //直方图分辨率
    uint64_t SendInfoEn : 8;   //发送使能
    uint64_t AngleRange : 8;   //标定角度范围
    uint64_t TxSelValid : 1;
    uint64_t WaitThrValid : 1;
    uint64_t TimeoutThrValid : 1;   //
    uint64_t NoObjCntThrValid : 1;  //
    uint64_t HistValidThrValid : 1; //
    uint64_t HistAngleThrValid : 1; //
    uint64_t SendInfoEnValid : 1;   //
    uint64_t AngleRangeValid : 1;   //
#endif
} stInstallCalcExtCfg;


//vehicle
typedef struct
{
    uint64_t WheelSpeedFL          :12;
    uint64_t WheelSpeedFR          :12;
    uint64_t WheelSpeedRL          :12;
    uint64_t WheelSpeedRR          :12;
    uint64_t res                   :16;
}StrVehicleInfo1;

//vehicle
typedef struct
{
    uint64_t SteeringAngle              :16;
    uint64_t SteeringAngleSpd           : 8; 
    uint64_t LongitudinalAcceleration   :12;
    uint64_t LateralAcceleration        :12;            
    uint64_t actualGear                 : 4;           
    uint64_t res                        :12;
}StrVehicleInfo2;

//EndFrame
typedef struct
{
    uint64_t EndFrm_Checksum            : 8;
    uint64_t EndFrm_RollingCnt          : 3;
    uint64_t EndFrm_Reserve             :19;
    uint64_t EndFrm_SpeedMode           : 4;            //1-OBD 2-自测速
    uint64_t EndFrm_AutoCalAngleOffset  :11;            //自标定角度偏差
    uint64_t EndFrm_EOLInstallAngle     :11;            //下线/售后标定角度
    uint64_t EndFrm_IntervalTime        : 8;            //雷达目标帧间隔时间 
}StrEndFrame;

//服务校准相关
typedef struct
{
	uint64_t res1						:16;
	uint64_t warningInfo				: 8;
	uint64_t caliState					: 8;
	uint64_t timeOutThr					: 8;//分钟
    uint64_t caliAngle					: 15;//校准角度
    uint64_t ServiceEnable				: 1;//校准使能
    uint64_t res0						: 3;//校准使能
    uint64_t warningInfoValid			: 1;//校准使能
    uint64_t caliStateValid				: 1;//校准状态
    uint64_t timeOutThrValid			: 1;//超时门限配置
    uint64_t caliAngleValid				: 1;//校准角度
    uint64_t ServiceEnableValid			: 1;//使能
}stServiceCaliCfg;

//自校准配置
typedef struct
{
	uint64_t canCaliMinAngleThr			: 4;
	uint64_t caliAngleBeatThr			: 4;
	uint64_t sendFlag					: 7; //上报信息
	uint64_t sendFlagValid				: 1; //上报信息
	uint64_t stdThr						: 7;
	uint64_t stdThrValid				: 1;
	uint64_t caliMode					: 4; //0普通模式  1快速校准模式[校准人员确认环境OK]
	uint64_t caliCntThr					: 4;
	uint64_t caliObjNumberThr			: 8;
	uint64_t cycleWaitThr				: 4;
    uint64_t SelfCalcAngle			: 11;//自校准角度
    uint64_t SelfCalcEn				: 1;//校准使能
	uint64_t canCaliMinAngleThrValid    : 1;
	uint64_t caliAngleBeatThrValid		: 1;
	uint64_t caliModeValid				: 1;
	uint64_t caliCntThrValid			: 1;
	uint64_t caliObjNumberThrValid		: 1;
	uint64_t cycleWaitThrValid			: 1;
    uint64_t SelfCalcAngle_valid		: 1;//校准的角度 设置有效，默认不用配置 ，用于角度配置清零
    uint64_t SelfCalcEn_valid			: 1;//使能

}stRadarSelfCalcCfg;

//看门狗状态
typedef struct
{
#if IS_BIG_ENDIAN==1
	uint64_t QueryType 		: 8; //请求类型
	uint64_t QueryAns1 		: 8; //应答数据
	uint64_t QueryAns2 		: 8;
	uint64_t QueryAns3 		: 8;
	uint64_t QueryAns4 		: 8;
	uint64_t QueryAns5 		: 8;
	uint64_t QueryAns6 		: 8;
	uint64_t QueryAns7 		: 8;
#else
	uint64_t QueryAns7 		: 8;
	uint64_t QueryAns6 		: 8;
	uint64_t QueryAns5 		: 8;
	uint64_t QueryAns4 		: 8;
	uint64_t QueryAns3 		: 8;
	uint64_t QueryAns2 		: 8;
	uint64_t QueryAns1 		: 8; //应答数据
	uint64_t QueryType 		: 8; //请求类型
#endif
}stQueryAnsMsg;

//ars410 
typedef struct
{
    uint64_t FRS_Msg_CheckSum       : 8;
    uint64_t FRS_Msg_AliveCounter   : 4;
    uint64_t FRS_Status_MisAlign    : 3;
    uint64_t FRS_Status_HWErr       : 1;
    uint64_t FRS_Status_BlkProg     : 1;
    uint64_t FRS_Fail               : 1;
    uint64_t FRS_MeasEnabled        : 1;
    uint64_t FRS_Host_Yaw           :11;
    uint64_t FRS_HostSpeed          :12;
    uint64_t FRS_TimeStamp          :16;
    uint64_t FRS_Latency            : 6;
}FRS_Statue;

typedef struct
{
    uint64_t FRS_P1_00_Msg_CheckSum         : 8;
    uint64_t FRS_P1_00_Msg_AliveCounter     : 4;
    uint64_t FRS_P1_00_Obj_ExstProb         : 6;
    uint64_t FRS_P1_00_Obj_XVelRel_Stdev    : 7;
    uint64_t FRS_P1_00_Obj_XAccRel          : 7;
    uint64_t FRS_P1_00_Obj_ObstacleProb     : 5;
    uint64_t FRS_P1_00_Obj_MotionPattern    : 3;
    uint64_t FRS_P1_00_Obj_YPos_Stdev       : 7;
    uint64_t FRS_P1_00_Obj_ValidFlag        : 1;
    uint64_t FRS_P1_00_Obj_XPos_Stdev       : 7;
    uint64_t FRS_P1_00_Obj_UpdateFlag       : 1;
    uint64_t FRS_P1_00_Obj_ID               : 8;
}FRS_ObjPart1;

typedef struct
{
    uint64_t FRS_P2_00_Msg_CheckSum         : 8;
    uint64_t FRS_P2_00_Msg_AliveCounter     : 4;
    uint64_t FRS_P2_00_Obj_MeasFlag         : 1;
    uint64_t FRS_P2_00_Obj_YVelRel          :11;
    uint64_t FRS_P2_00_Obj_Type             : 2;
    uint64_t FRS_P2_00_Obj_XPos             :14;
    uint64_t FRS_P2_00_Obj_YPos             :13;
    uint64_t FRS_P2_00_Obj_XVelRel          :11;
}FRS_ObjPart2;

//CAN总线模式
#define  CAN_BUS_WORK_MODE_NORMAL		0
#define  CAN_BUS_WORK_MODE_ALL_Shield	1
#define  CAN_BUS_WORK_MODE_AGING_SHIELE	2

//查询类型
#define  CAN_QUERY_TYPE_WATCHDOG_STATE		0x1

void sendObjList(uint8_t canIdx , TTrackRVAList *pTrackList, int maxNum, uint8_t proVer);
void sendRawListConti(uint8_t canIdx, const objectInfo_t *pList, int num);
void sendRawObjList(uint8_t canIdx, const objectInfo_t *pList, int num);
void procRxCanFrame(uint8_t canIdx,TCANRxBuf *pBuf);
void sendTargetInfo(TTrackRVAList *pList);
void sendEndFrameInfo(uint8_t canidx);
void sendRawVelObjList(uint8_t canIdx, Target_Thres_t *pList, int num1, int num2, int num3, int num4);
void sendRadarInfo(uint8_t canIdx);
void sendRadarStatue(uint8_t canIdx);
void appProsendVehicleInfo(uint8_t canidx);
void appProsendEndFrameInfo(uint8_t canidx);
void appProsendTargetsArs410(uint8_t canIdx, objectInfo_t *pOutObject, int outNum);


#endif

