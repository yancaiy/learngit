#ifndef __SHARED_VAR_H__
#define __SHARED_VAR_H__

#include "typedefs.h"
#include "FreeRTOS.h"
#include "sharedDef.h"
#include "track_common.h"

//数值越低  优先级越低
#define TSK_PRIOR_HI (configMAX_PRIORITIES - 1)
#define TSK_PRIOR_MID (configMAX_PRIORITIES - 2)
#define TSK_PRIOR_LO (configMAX_PRIORITIES - 3)

#define MAX_PEAK_NUM 10240

#if RADAR_SERVICE_ALIGNMENT_MODE_DBAA == 1
#define DBAA_DEPTH_PARAM_BUFFER_SIZE	5
#define DBAA_ANFLE_RANGE		    10 //角度范围
#define DBAA_ANFLE_BIN			    0.1 //角度步进
#define DBAA_SAUARE_BUFFER_SIZE		201//(unsigned int)(2 * (DBAA_ANFLE_RANGE / DBAA_ANFLE_BIN) + 1)  //方差Buffer大小
#endif


extern float gHILPara[3];
extern uint8_t gProfileCount;
extern uint8_t frame_type_profile[4];

#define UNWRAP_QLEN                       10
#define UNWRAP_SHORT_RANGE                45.0f

#define UNWRAP_SHORT_RANGEMATCHTHRE       2.0f
#define UNWRAP_SHORT_VELMATCHTHRE         1.0f
#define UNWRAP_SHORT_ANGLEMATCHTHRE       6.0f

#define UNWRAP_LONG_RANGEMATCHTHRE       2.0f
#define UNWRAP_LONG_VELMATCHTHRE         1.0f
#define UNWRAP_LONG_ANGLEMATCHTHRE       6.0f

#define UNWRAPTHRE                      3.5f
#define UNWRAPRANGETHRE                 3.0f

#define UNWRAPRANGEWEIGHT               4.0f
#define UNWRAPANGLEWEIGHT               9.0f
#define UNWRAPMAGWEIGHT                 36.0f

//雷达标定成功后，标定目标的位置记录
typedef struct radarInstallCalcObj_t
{
    float x;
    float y;
    float range;
    float speed;
    float angle;
    float snr;
    int idx;
    UINT8 histMaxCnt;//实际最大直方图个数
} radarInstalledCalcObj;

/// Raw List
typedef struct tagTargetRawRVA
{
    float distance;
    float speed;
    float angle;
    int16_t mag;
} TTargetRawRVA;

typedef struct tagTargetRVAList
{
    TTargetRawRVA target[MAX_NB_OF_TARGETS];
} TTargetRVAList;

/// Track List
typedef union tagTrackState {
    UINT64 word;
    struct
    {
        UINT64 GROUPING_CHANGED : 1;
        UINT64 moveFlag : 1;
        UINT64 STATUS : 3;
        UINT64 STATUS_FLAG : 4;
        UINT64 SORT_FLAG : 32;
        UINT64 ObjOutPutFlag : 1;
        UINT64 objType : 4; //另一个目标分类
        UINT64 DynProp : 6;
        UINT64 idx_1 : 8;
        UINT64 MeasFlag : 1;
        UINT64 UpdateFlag : 1;  //用于指示 CAN 总线中新创建的对象的标志。 0x0：循环中的新对象;  0x1：对象在上一个周期中存在
        UINT64 bridge : 1;
        UINT64 ars410ObjType : 1;
    } bit_big;
    struct
    {
        UINT64 ars410ObjType : 1;
        UINT64 bridge : 1;
        UINT64 UpdateFlag : 1;  //用于指示 CAN 总线中新创建的对象的标志。 0x0：循环中的新对象;  0x1：对象在上一个周期中存在
        UINT64 MeasFlag : 1;
        UINT64 idx_1 : 8;
        UINT64 DynProp : 6;
        UINT64 objType : 4; //另一个目标分类
        UINT64 ObjOutPutFlag : 1;
        UINT64 SORT_FLAG : 32;
        UINT64 STATUS_FLAG : 4;
        UINT64 STATUS : 3;
        UINT64 moveFlag : 1;
        UINT64 GROUPING_CHANGED : 1;
    } bit;
} TTrackState;

typedef union tagTrackValue {
    UINT32 word;
    struct
    {
        UINT32 missCnt : 8;
        UINT32 hitCnt : 8;
        UINT32 trkReiability : 8;
        UINT32 trkCnt : 8;
    } bit_big;
    struct
    {
        UINT32 trkCnt : 8;
        UINT32 trkReiability : 8;
        UINT32 hitCnt : 8;
        UINT32 missCnt : 8;
    } bit;
} TTrackValue;

typedef struct tagTrackRVA
{
    TTrackState state;
    TTrackValue traValue;
    float distance;
    float speed;
    float angle;
    float heighAngle;
    float snr;
    float centerOffset;
    float objLength;
    float objWidth;
    float acc_y;
    float acc_x;
    float rcs;
    float vx;
    float vy;
    float  x;
    float  y;
    float  objObstacleProb;
    float rms[7];
    UINT8  sendId;
} TTrackRVA;

typedef struct tagRawRVA
{
    float distance;
    float speed;
    float angle;
    float angleFixed;
    float heighAngle;
    float snr;
    float rcs;
    uint8_t mag;
    uint8_t objTxCh;//远距离目标还是近距离目标
    uint8_t rawIdx;//对应的原始点, radarTarget使用
    uint8_t resv;
} TRawRVA;

typedef struct tagCaliObj
{
    float distance;
    float speed;
    float angle;
    float heighAngle;
    float snr;
} TCaliObj;

typedef struct tagRawRVAList
{
    uint64_t time;
    int32_t targetNum;
    float angleOffset;
    float StatiObjDeteRange;
    TRawRVA target[MAX_NB_OF_TARGETS];
} TRawRVAList;

typedef union tagTrackSence {
    UINT16 word;
    struct
    {
        UINT16 ruralScene : 1;  //乡村道路
        UINT16 barnScene : 1;   //地下车库
        UINT16 fenceScene : 1;  //靠栏杆场景
        UINT16 tunnelFence : 1; //隧道场景
        UINT16 ObjMaxCntAbove : 2;    //清智目标超过标志 0没有超，1，2，3都分别在不同策略下还超过
        UINT16 jam : 1;        //拥堵场景
        UINT16 REV : 9;        //其他
    } bit_big;
    struct
    {
        UINT16 REV : 9;        //其他
		UINT16 jam : 1;        //拥堵场景
        UINT16 ObjMaxCntAbove : 2;    //清智目标超过标志 0没有超，1，2，3都分别在不同策略下还超过
        UINT16 tunnelFence : 1; //隧道场景
        UINT16 fenceScene : 1;  //靠栏杆场景
        UINT16 barnScene : 1;   //地下车库
        UINT16 ruralScene : 1;  //乡村道路
    } bit;
} TTrackSenceFlag;

typedef union tagTrackObjFilter {
    UINT8 word;
    struct
    {
    	UINT8 unMoveFlag : 1;  //±3m外的，静止目标
    	UINT8 Fence : 1;   	//护栏过滤
    	UINT8 bridght : 1;  //天桥
    	UINT8 REV : 5;        //其他
    } bit_big;
    struct
    {
    	UINT8 REV : 4;        //其他
    	UINT8 harmonic : 1;        //其他
    	UINT8 bridght : 1;  //天桥
    	UINT8 Fence : 1;   	//护栏过滤
    	UINT8 unMoveFlag : 1;  //±3m外的，静止目标
    } bit;
} TTrackObjFilterFlag;

typedef struct tagTrackRVAList
{
    uint16_t targetNum;
    TTrackSenceFlag senceFlag;
    int forwardObjId;  //正前方最近非护栏目标ID
    float angleOffset; //角度偏移补偿
    float estSpeed[3]; //估计的车速, 0是用mag值估算的车速，1是用目标点估算的车速
    TTrackRVA target[MAX_NB_OF_TARGETS];
    uint32_t sendIdMask[4];//0、1、2、3---分别排序，最大共128个
} TTrackRVAList;

typedef union SendTrkObjTypeFlag {
	UINT8 word;
	struct {
		UINT8 REV 				: 6; //其他
		UINT8 CANDI				: 1; //CANDI目标
		UINT8 TRACK				: 1; //跟踪后目标
	}bit;
}SendTrkObjType;

typedef union tagradarOperate {
	UINT8 word;
	struct {
		UINT8 res 			: 7; //
		UINT8 trkInitFlag 	: 1; //是否需要清除跟踪状态
	}bit;
}trkOperate_t; //雷达操作模式

typedef enum CFAR_value
{
    OSCA,
    OS2D,
    CA2D,
    CA_SOCA,
} CFAR_t;

typedef struct
{
    int8_t valid;           //是否有效
    int8_t isUsing;         //是否使用
    int8_t angleNum;        //有几个角度，超分辨时可能有2个
    uint16_t DbfMag;        //DbfMag值
    float range;            //距离
    float velocity;         //速度
    float realVel;          //解速度模糊后的结果
    float angle;            //水平角度
    float heighAngle;       //高度角
    float sig_elv;
    uint16_t rangeBin;      //距离bin
    uint16_t velocityBin;   //速度bin
    float fftMag;           //fft mag值
    float doaMag;           //doa mag值
    float threshold;        //底噪mag值
    float mag1;             //当前fftMag与前和后一个距离bin的fftMag值平方和开根号
    float phase[4];         //相位值
    float firstvelocity; 
} Target_Thres_t;

typedef struct
{
    uint8_t isUsing;    //是否使用
    uint8_t objTxCh;    //区分远近天线的目标  , 或者哪个Profile目标
    uint16_t DbfMag;    //DbfMag值
    uint16_t fftMag;    //fft Mag值
    uint16_t doaMag;    //doa mag值
    float range;        //距离
    float velocity;     //速度
    float angle;        //水平角度
    float heighAngle;   //高度角
    float snr;          //SNR
    float mag;          //Mag
    float rcs;
    float threshold;    //底噪
    float phase[4];     //相位值
    float firstvelocity;
} objectInfo_t;

typedef struct
{
    float range_bin;
    float velocity_bin;
    float angle_bin;
} Bin_Param_t;

typedef struct 
{
    float farRange;
    float leftRange;
    float leftAngle;
    float rightRange;
    float rightAngle;
}antArea_t;

typedef struct
{
    uint16_t cohesionEn; //是否是能聚类
    uint16_t filterType; //护栏过滤类型
    uint16_t radarMode;  //雷达工作模式，FCW/AEB/NORMAL
    uint16_t debugMode;  //debug模式开关
    antArea_t antArea;   //天线的负责范围
    uint8_t selfCali;    //自标定安装角度开关
    SendTrkObjType  ObjSendType; //发送的目标类型
    trkOperate_t	trkOperate; //跟踪操作
    TTrackObjFilterFlag trkOutFilter;  //输出过滤
    uint8_t isChanged;    //结构体内容已改变
    uint8_t resved[11];
    uint32_t crc;
} radarTrkCfg_t;

typedef struct
{
    float x0;
    float y0;
    float x1;
    float y1;
    u8 IsValid;
} Area;

typedef struct
{
	Area turnArea; //掉头区域的 (x0,y0)   (x1,y1) 配置时候需要注意，规定 x0y0要在 x1y1的左下方， x0<=x1 y0<=y1
	Area stopArea;
    Area sidewalkArea1;	//人行道区域1，与车辆掉头区域说明一样
	Area sidewalkArea2; //人行道区域2，与车辆掉头区域说明一样
	Area sidewalkArea3; //人行道区域3，与车辆掉头区域说明一样
	Area sidewalkArea4; //人行道区域4，与车辆掉头区域说明一样
    u8 laneNumLeft;  //左车道数量,0时候表示不用
    u8 laneNumRight; //右车道数量。0时候表示不用
    float laneWide;
    float laneWideX[8]; //最大支持7个车道

    //目标过滤相关参数
	float gRangeThr_Y_A;
	float gRangeThr_Y_B;
	float gRangeThr_X_C;
	float gRangeThr_X_D;
	float gRangeThr_Tx1_E;
	float gRangeThr_Tx2_F;

} hwBaseCfgInfo;

typedef struct
{
    uint64_t timestamp;
    TickType_t tick;
} radarTimestamp;

//为uds诊断添加部分变量
typedef struct
{
    u8 ECUHardwareVersionNumber[5];             // F193     hex 硬件版本号
    u8 ECUSoftwareCode[9];                      // F194     ascii 软件编码
    u8 ECUSoftwareVersionNumber[6];             // F195     hex 软件版本号
    u8 DATASETSoftwareNumber[9];                // FF94     ascii DATASET软件序列号
    u8 DATASETSoftwareVersionNumber[6];         // FF95     hex DATASET软件版本号
    u8 ECUBootSoftwareVersionNumber[9];         // F180     hex 底层软件Bootload版本号
    u8 FactoryMode;                             // 3401     hex 生产模式
    u8 VariantCoding[8];                        // 2100     hex 配置字
    u8 ActiveDiagnosticSession;                 // F186 unsigned 当前诊断模式
    
    u8 VIN[17];                                 // F190 ascii 车辆识别码
    u8 ECUPowerVoltage;                          // 0285 unsigned 电源电压
    u8 SOCtemperature;                          // 028D signed ECU温度
    u8 VehicleSpeed[2];                      // 1011 signed  车辆速度
    int32_t FastHorizontalMisalignmentAngle;      // 3404 signed   快速水平偏差角
    int32_t SlowHorizontalMisalignmentAngle;      // 3403 signed   慢速水平偏差角
    int32_t FastVerticalMisalignmentAngle;        // 3441 signed   快速垂直偏差角
    int32_t SlowVerticalMisalignmentAngle;        // 3442 signed   慢速垂直偏差角
    u8 SecurityAccessInvalidCounter;            // 0103 hex         安全访问错误次数
    u8 RealTime[6];                             // 车辆时间    （缓存，不是did列表）             
    uint32_t Odometer;                             // 里程   （缓存，不是did列表）
}DID_IST_STRUCT;        //byd的did列表结构


typedef struct
{
    uint8_t led : 1;
    uint8_t volt : 1;
    uint8_t rf : 1;
    uint8_t temp : 1;
    uint8_t flash_error : 1;
    uint8_t angleagv : 1;
    uint8_t resv : 1;
    uint8_t is_1st_update : 1;
} DetectStatus_st;

void initVar(void);

#endif
