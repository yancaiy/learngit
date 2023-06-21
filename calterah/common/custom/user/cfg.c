/*
 * cfg.c
 *
 *  Created on: 2017/1/17
 *      Author: HQLIU
 */
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <math.h>
#include "FreeRTOS.h"
#include "cfg.h"
#include "crc32.h"
#include "sharedbuffer.h"
#include "radarCfg.h"
#include "flash.h"
#include "flash_mmap.h"
#include "FreeRTOS.h"
#include "baseband.h"
#include "baseband_hw.h"
//#include "includes_app_h.h"
#include "radio_reg.h"
#include "baseband_alps_FM_reg.h"
#include "arc_exception.h"
#include "embARC_error.h"
#include "baseband_task.h"
#include "radio_ctrl.h"
#include "gpio_hal.h"
#include "system_misc.h"
#include "radardsp.h"
#include "status.h"
//#include "kf_track.h"
#include "fsm_reaction.h"
#include "app_dut.h"
//#include "app_eol_dbf.h"

#ifndef PI
#define PI 3.1415926535f
#endif

#define BB_WRITE_REG(bb_hw, RN, val) baseband_write_reg(bb_hw, BB_REG_##RN, val)

static int flash_erase(uint32_t addr, uint32_t len);

//===============================================================
//不跟随版本走的配置项
#define ADD_CFG_ITEM(type, item, var, member)                                      \
    {                                                                              \
        0, type, item, &var, offsetof(__typeof__(var), member), sizeof(var.member) \
    }

//跟随版本走的配置项
#define ADD_CFG_ITEM_VER(type, item, var, member)                                  \
    {                                                                              \
        1, type, item, &var, offsetof(__typeof__(var), member), sizeof(var.member) \
    }

//直接保存整个数据结构
#define ADD_CFG_STRUCT(type, item, var)    \
    {                                      \
        0, type, item, var, 0, sizeof(var) \
    }

//直接保存整个数据结构
#define ADD_CFG_STRUCT_2(type, item, var, size) \
    {                                           \
        1, type, item, var, 0, size             \
    }

#define MAX_CFG_AREA_SIZE 0x8000

#define PAGE_SIZE 32
#define CONFIG_FLASH_SEL 0x00010000
#define CONFIG_FLASH_LOCK_MASK 0xFFFEFFFF

#define CONFIG_MAGIC_NUM 0xCACACACA
#define CONFIG_MAGIC_NUM_NEW 0xC5C5C5C5

//================================================================
typedef struct
{
    uint8_t followVer; //是否跟随版本
    uint8_t type;
    uint16_t item;
    void *pVar;
    uint16_t offSet;
    uint16_t size;
} radarCfgItem_t;

typedef struct
{
    uint16_t type;
    uint16_t item;
    uint16_t size;
    uint16_t resv;
} cfgItemInfo_t;

typedef struct
{
    uint32_t magicNum;
    uint32_t crc;
    uint16_t size;
    uint16_t itemNum;
} cfgHeader_t;

enum CFG_TYPE_E
{
    BASE_CFG,
    TRK_CFG,
    LD_CFG,
    LRR400_BASE_CFG,
    LRR400_CFG,
    CFG_TYPE_NUM
};

enum CFG_ITEM_E
{
    TRX_GAIN = 0,
    RADAR_ID = 6,
    CFG_VER = 7,
    SEND_VEL = 8,
    SPEED_SRC = 9,
    START_FREQ = 10,
    BAND_WIDTH = 11,
    BAND_WIDTH2 = 12,
    TRAMP_UP = 13,
    TRAMP_DOWM = 14,
    TRAMP_IDLE = 15,
    SAMPLE_NUM = 16,
    CHIRP_NUM = 17,
    RFFT_NUM = 18,
    DFFT_NUM = 19,
    FCW_START_VEL = 20,
    FCW_FWD_SPEED = 21,
    ANT_DISTANCE = 22,
    ANT_PHASE = 23,
    TX_PATTERN = 24,
    TX_ANT_SEL = 25,
    ANGLE_OFFSET = 26,
    RADAR_INFO = 27,
    YAWRATE = 28,
    DEBUG_MODE_SW = 29,
    HRZ_OFFSET = 30,
    INSTALL_HEIGHT = 31,
    OBJ_EXTEND_INFO_SW = 32,
    DBF_OUT_ANGLE_NUM = 33,
    SW_GAIN_EN = 34,
    ANGLE_MAPPER = 35,
    ANGLE_MAPPER_EN = 36,
    INSTALL_ANGLE = 37,
    SEND_YAWRATE = 38,
    INSTALL_CALC_STATUE = 39,
    RANDOM_ID = 40,
    AGING_TEST = 41,
    OBJ_EXTEND_INFO_SW2 = 42,
    INSTALL_CALC_OBJ = 43,
    ACC_CALC_ZERO = 44,
    DBF_MAGIC = 45,
    PROTOCOL_VER = 46,
    SR_MODE = 47,
    CAN_RATE_MODE = 48,
    TRK_TX_FILTER = 49,
    ANT_DISTANCE_Y = 50,
    RCS_OFFSET = 51,
	IS_CAN_SEND_CT_PRO = 52,
	INSTALL_ELEVATED_OFFSET = 53,
	TX_POWER = 54,
    AZIMUTH_DECAY_TX = 55, 
    ANT_CALI_MODE = 56, 

    COHESION_EN = 0x100,
    FILTER_TYPE = 0x101,
    WORK_MODE = 0x102,
    DEBUG_MODE = 0x103,
    SELF_CALI = 0x104,
    ANT_AREA  = 0x105,
	TRK_OUTPUT_FILTER = 0x106,

    MIN_SPEED = 0x200,
    MAX_SPEED = 0x201,
    GUARD_POS = 0x202,
    VEL_THRD = 0x203,

    LRR400_TX_POWER = 0X205,
    HW_BASE_CFG = 0X206,

    ALL_400_CFG = 0x210,
};

//===============================================================

int eraseRebootFlagInfo(void);

uint8_t flashCrcErrorFlag;
uint8_t flashHeadErrorFlag;

static SemaphoreHandle_t gTrkCfgMutex;

static const radarTrkCfg_t gDefaultTrkCfg =
    {
        .cohesionEn = 1,
        .filterType = 0, //FILTER_FIX_FENCE, | FILTER_STATIC_OBJ | FILTER_3_LANE,
        .radarMode = RADAR_MODE,
        .debugMode = 1,
        .selfCali = 1,
        .antArea = {
            .farRange = 60,
            .leftRange = 70,
            .leftAngle = 2,
            .rightRange = 70,
            .rightAngle = -2,
        },
        .trkOutFilter.word = 0x10, //开启谐波过滤
        .isChanged = 1,
};

static const hwBaseCfgInfo gDefaultHwCfg =
    {
        .turnArea = {-10, 55, 5, 70, 1}, // x0 , y0 ,x1 ,y1 ,IsValid
        .stopArea = {-17.5, 70, -6.3, 120, 1},
        .laneNumLeft = 3,
        .laneNumRight = 3,
        .laneWide = 3.7,
        .sidewalkArea1 = {-20, 60, 24, 65, 1},
        .sidewalkArea2 = {-20, 20, 24, 25, 1},
        .sidewalkArea3 = {-26, 25,-20, 60, 1},
        .sidewalkArea4 = { 24, 25, 30, 60, 1},

        .gRangeThr_Y_A = 20,
        .gRangeThr_Y_B = 85,
        .gRangeThr_X_C = 60,
        .gRangeThr_X_D = 30,
        .gRangeThr_Tx1_E = 60,
        .gRangeThr_Tx2_F = 80,
};

//射频参数
fmcwParam_t gFmcwParam[TX_SW_NUM][2];

static radar_config_t radar_config_user;

const radar_config_t * const radar_config_using = &radar_config_user;

//boot信息
image_header_t boot_Info;

static radarTrkCfg_t gNewTrkCfg;

static const radar_config_t radar_config_default =
{
        .configMagicNum = CONFIG_MAGIC_NUM,
        .protocolVersion = PROTOCOL_VER_USING,

        .compli_para.radarId = 0,
        .compli_para.iVersion = 1,
        .sendCarVel = 1,  //Whether send estV
        .speedSource = 1, //Which speed to use? //0-est 1-OBD
        .yawrate_zero = 0,
        .debugModeSW = 0,     //debug 开关，默认不打开
        .objExtendInfoSW = 1, //默认
        .objExtendInfoSW2 = 1,
        .fmcw_startfreq = FMCW_START_FREQ,
        .fmcw_bandwidth = FMCW_BANDWIDTH,
        .fmcw_trampup = FMCW_TRAMPUP,
        .fmcw_tdown = FMCW_TDOWN,
        .fmcw_tidle = FMCW_TIDLE,
        .bb_buf_n = SAMPLE_CNT,
        .bb_buf_l = CHIRPS_PER_FRAME,
        .bb_fft_n = SAMPLES_PER_CHIRP,
        .bb_fft_l = CHIRPS_PER_FRAME,
		.canBaudRateMode = CAN_CFG_500K, 

        .swGainEn = 0,
        .lowVgaGain = 0x24,
        .angleReverse = 1,

        .randomId	= 0,
		.agingTest	= 0,

        .fcwStartUpSpeed = 0,   //FCW启动速度
        .fcwFwdObjNegSpeed = 2, //本车道绝对来向目标速度，大于此速度的不告警
        .fmcw_bandwidth2 = 1.0,

        .anten_distance = ANT_DISTANCE_CAL,
        .anten_distance_y = ANT_DISTANCE_CAL,
        .anten_phase = ANT_PHASE_CAL,

        .tx_pattern = TX_SW_MODE,
        .tx_sel = TX_SEL,
        .tx_cw = 0,

        .angleOffset = 0.0,
        .installAngle = 0,
		.installElevatedOffset = 0,
        .horizontalOffset = 0,
        .installHeight = 0,

        .radar_info = {
            .macaddr = {0x00, 0x04, 0x9F, 0x11, 0x22, 0x33},
            .ipaddr = {192, 168, 1, 60},
            .HwVer = HW_VERSION,
            .SwVer = SW_VERSION,
            .pcbaBBSN = {"001010063C1002070000x"},
			.radarSN = {"001010063C1002070000x"},
			.imuName = {"LSM_6DS33_NULL"},
            .CalVer = {0, 0},
        },
        .srMode = 0x4,                    //默认MIMO模式
        .trkTxFilter = TX_CH_FILTER_NULL, //默认不过滤远距离天线+近距离天线
        .isSendCTCanProtocol = 1,
        .rcsOffset = {0, 0, 0, 0},
		.txPower = {
        	.tx_power = {0xFF, 0xFF, 0xFF ,0xFF},
        },
        .angleMapperEn = ANGLE_MAP_EN,
        .angleMapper = ANGLE_MAPPER_TABLE,
        .antCaliMode = {false, false, false, false},
};

//默认参数列表
static const uint32_t gDefaultCfgs[CFG_TYPE_NUM][4] =
    {
        {(uint32_t)&radar_config_user, (uint32_t)&radar_config_default, sizeof(radar_config_user)}, //BASE_CFG
        {(uint32_t)&gNewTrkCfg, (uint32_t)&gDefaultTrkCfg, sizeof(gNewTrkCfg)},                     //TRK_CFG
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}
};

static const radarCfgItem_t gCfgItems[] =
    {
        //基本配置
        ADD_CFG_ITEM_VER(BASE_CFG, TRX_GAIN, radar_config_user, compli_para.compliA),
        ADD_CFG_ITEM(BASE_CFG, RADAR_ID, radar_config_user, compli_para.radarId),
        ADD_CFG_ITEM_VER(BASE_CFG, CFG_VER, radar_config_user, compli_para.iVersion),
        ADD_CFG_ITEM(BASE_CFG, SEND_VEL, radar_config_user, sendCarVel),
        ADD_CFG_ITEM(BASE_CFG, SPEED_SRC, radar_config_user, speedSource),
        ADD_CFG_ITEM_VER(BASE_CFG, START_FREQ, radar_config_user, fmcw_startfreq),
        ADD_CFG_ITEM_VER(BASE_CFG, BAND_WIDTH, radar_config_user, fmcw_bandwidth),
        ADD_CFG_ITEM_VER(BASE_CFG, BAND_WIDTH2, radar_config_user, fmcw_bandwidth2),
        ADD_CFG_ITEM_VER(BASE_CFG, TRAMP_UP, radar_config_user, fmcw_trampup),
        ADD_CFG_ITEM_VER(BASE_CFG, TRAMP_DOWM, radar_config_user, fmcw_tdown),
        ADD_CFG_ITEM_VER(BASE_CFG, TRAMP_IDLE, radar_config_user, fmcw_tidle),
        ADD_CFG_ITEM(BASE_CFG, FCW_START_VEL, radar_config_user, fcwStartUpSpeed),
        ADD_CFG_ITEM(BASE_CFG, FCW_FWD_SPEED, radar_config_user, fcwFwdObjNegSpeed),
        ADD_CFG_ITEM(BASE_CFG, ANT_DISTANCE, radar_config_user, anten_distance),
        ADD_CFG_ITEM(BASE_CFG, ANT_DISTANCE_Y, radar_config_user, anten_distance_y),
        ADD_CFG_ITEM(BASE_CFG, ANT_PHASE, radar_config_user, anten_phase),
        ADD_CFG_ITEM_VER(BASE_CFG, TX_PATTERN, radar_config_user, tx_pattern),
        ADD_CFG_ITEM_VER(BASE_CFG, TX_ANT_SEL, radar_config_user, tx_sel),
        ADD_CFG_ITEM(BASE_CFG, ANGLE_OFFSET, radar_config_user, angleOffset),
        ADD_CFG_ITEM(BASE_CFG, INSTALL_ANGLE, radar_config_user, installAngle),
        ADD_CFG_ITEM(BASE_CFG, HRZ_OFFSET, radar_config_user, horizontalOffset),
        ADD_CFG_ITEM_VER(BASE_CFG, INSTALL_HEIGHT, radar_config_user, installHeight), //installHeight
        ADD_CFG_ITEM(BASE_CFG, RADAR_INFO, radar_config_user, radar_info),
        ADD_CFG_ITEM(BASE_CFG, YAWRATE, radar_config_user, yawrate_zero),
        ADD_CFG_ITEM(BASE_CFG, DEBUG_MODE_SW, radar_config_user, debugModeSW),
        ADD_CFG_ITEM(BASE_CFG, OBJ_EXTEND_INFO_SW, radar_config_user, objExtendInfoSW),
        ADD_CFG_ITEM_VER(BASE_CFG, SW_GAIN_EN, radar_config_user, swGainEn),
        ADD_CFG_ITEM(BASE_CFG, ANGLE_MAPPER, radar_config_user, angleMapper),
        ADD_CFG_ITEM_VER(BASE_CFG, ANGLE_MAPPER_EN, radar_config_user, angleMapperEn),
        ADD_CFG_ITEM(BASE_CFG, SEND_YAWRATE, radar_config_user, sendYawrate),
        ADD_CFG_ITEM(BASE_CFG, INSTALL_CALC_STATUE, radar_config_user, installCalcState),
        ADD_CFG_ITEM(BASE_CFG, RANDOM_ID, radar_config_user, randomId),
        ADD_CFG_ITEM(BASE_CFG, AGING_TEST, radar_config_user, agingTest),
        ADD_CFG_ITEM(BASE_CFG, OBJ_EXTEND_INFO_SW2, radar_config_user, objExtendInfoSW2), //扩展信息2
        ADD_CFG_ITEM(BASE_CFG, INSTALL_CALC_OBJ, radar_config_user, installedObj),
        ADD_CFG_ITEM(BASE_CFG, ACC_CALC_ZERO, radar_config_user, accCalcZero), //加速度校准值
        ADD_CFG_ITEM(BASE_CFG, DBF_MAGIC, radar_config_user, dbfMagicNum),
        ADD_CFG_ITEM_VER(BASE_CFG, PROTOCOL_VER, radar_config_user, protocolVersion),
        ADD_CFG_ITEM_VER(BASE_CFG, SR_MODE, radar_config_user, srMode),
        ADD_CFG_ITEM_VER(BASE_CFG, CAN_RATE_MODE, radar_config_user, canBaudRateMode),
        ADD_CFG_ITEM_VER(BASE_CFG, TRK_TX_FILTER, radar_config_user, trkTxFilter),
        ADD_CFG_ITEM(BASE_CFG, RCS_OFFSET, radar_config_user, rcsOffset),
		ADD_CFG_ITEM_VER(BASE_CFG, IS_CAN_SEND_CT_PRO, radar_config_user, isSendCTCanProtocol),
		ADD_CFG_ITEM(BASE_CFG, INSTALL_ELEVATED_OFFSET, radar_config_user, installElevatedOffset),
		ADD_CFG_ITEM_VER(BASE_CFG, TX_POWER, radar_config_user, txPower),
        ADD_CFG_ITEM(BASE_CFG, AZIMUTH_DECAY_TX, radar_config_user, azimuthDecayTx),
        ADD_CFG_ITEM(BASE_CFG, ANT_CALI_MODE, radar_config_user, antCaliMode),
        
        //跟踪参数配置
        ADD_CFG_ITEM_VER(TRK_CFG, COHESION_EN, gNewTrkCfg, cohesionEn),
        ADD_CFG_ITEM_VER(TRK_CFG, FILTER_TYPE, gNewTrkCfg, filterType),
        ADD_CFG_ITEM_VER(TRK_CFG, WORK_MODE, gNewTrkCfg, radarMode),
        ADD_CFG_ITEM_VER(TRK_CFG, DEBUG_MODE, gNewTrkCfg, debugMode),
        ADD_CFG_ITEM_VER(TRK_CFG, SELF_CALI, gNewTrkCfg, selfCali),
        ADD_CFG_ITEM_VER(TRK_CFG, ANT_AREA, gNewTrkCfg, antArea),
		ADD_CFG_ITEM_VER(TRK_CFG, TRK_OUTPUT_FILTER, gNewTrkCfg, trkOutFilter),

        //ADD_CFG_STRUCT_2(LRR400_BASE_CFG, HW_BASE_CFG, &gHwCfg, sizeof(gHwCfg)),

        //这里保存整个结构体，注意config已经是地址， ADD_CFG_STRUCT定义中就不使用地址了
        //ADD_CFG_STRUCT_2(LRR400_CFG, ALL_400_CFG,  config ,sizeof(config)),
        //其他配置
};

#define CFG_ITEM_NUM (sizeof(gCfgItems) / sizeof(gCfgItems[0]))

radar_config_t *loadRadarConfig(void)
{
    //计算使用存储长度，使用freeRTOS的内存申请
#if 1
    flashHeadErrorFlag = 1;

    int j, offset = 0;
    int i, len, hlen = 0, bootInfolen = 0;
    int ret = 0;
    cfgHeader_t header;

    hlen = sizeof(cfgHeader_t);
    hlen += (hlen % 4);

    EMBARC_PRINTF("loadradarcfg step 0\n");

    //boot信息读取
    bootInfolen = sizeof(image_header_t);
    bootInfolen += (bootInfolen % 4);
    ret = flash_memory_readb(FLASH_BOOT1_BASE, (uint8_t *)(&boot_Info), bootInfolen);
    if (ret != 0)
    {
        memset(&boot_Info, 0xFF, sizeof(image_header_t));
    }

    //ret = flash_memory_readw( CONFIG_ADDR ,  (uint32_t*)(&header) , hlen);
    ret = flash_memory_readb(CONFIG_ADDR, (uint8_t *)(&header), hlen);

    EMBARC_PRINTF("flash_memory_readw ret=%d\n", ret);

    while (ret != 0)
    {
        EMBARC_PRINTF("flash_memory_read error\n");
        return &radar_config_user;
    } //不可以失败，加入异常数据上报处理

    //return;

    EMBARC_PRINTF("loadradarcfg step 0-1\n");

    if ((header.magicNum != CONFIG_MAGIC_NUM_NEW) || (header.size >= MAX_CFG_AREA_SIZE))
    {
//        setSmFailed(IDX_ITEM_NO_CFG);
        EMBARC_PRINTF("loadradarcfg step 0-2\n");
        return &radar_config_user;
    }
    EMBARC_PRINTF("loadradarcfg step 0-3 header.magicNum=0x%x , header.size=%d\n", header.magicNum, header.size);

    // len = 0;
    // for (i = 0; i < CFG_ITEM_NUM; i++)
    // {
    //     len += gCfgItems[i].size;
    // }
    // len = sizeof(cfgItemInfo_t) * CFG_ITEM_NUM + len + hlen; // 存储数据长度+item的总量+header头数据
    len = header.size + 8;
    len += len % 4;                                          // 字为单位长度
    len &= 0xFFFF;                                           // 最大长度不能大于64k
    EMBARC_PRINTF("loadradarcfg step 1 , len=%d\n", len);
    char *pBuf = (char *)pvPortMalloc(len);

    while (pBuf == NULL)
    {
    } //不可以失败 !!!!! 或是修改为默认配置
    EMBARC_PRINTF("loadradarcfg step 2\n");
    ret = flash_memory_readb(CONFIG_ADDR, (uint8_t *)pBuf, len); // 全部读完
    while (ret != 0)
    {
        vPortFree(pBuf);
        EMBARC_PRINTF("flash_memory_read error 2\n");
        return &radar_config_user;

    } //不可以失败 !!!!! 或是修改为默认配置

    //header中的size包含了header中的itemNum, size字段以及剩余的eeprom中的配置
    u32 crc = update_crc(0, (char *)pBuf + 8, (header.size - 8));
    EMBARC_PRINTF("header.crc=0x%x  crc=0x%x\n", header.crc, crc);
    if (header.crc != crc)
    {
        vPortFree(pBuf);
        flashCrcErrorFlag = 1;
        flashHeadErrorFlag = 0; //CRC错误
//        setSmFailed(IDX_ITEM_CFG_CRC);
        return &radar_config_user;
    }

    //只有到这里了说明Flash的内容可以用---否则认为Flash中的内容不可用
    flashHeadErrorFlag = 0;
    flashCrcErrorFlag = 0;

    EMBARC_PRINTF("loadradarcfg step 4 , radar_config_user.angleoffset=%d ,radar_config_user.horizontalOffset=%d\n", 
                   radar_config_user.angleOffset, radar_config_user.horizontalOffset);

    cfgItemInfo_t *item = (cfgItemInfo_t *)((char *)pBuf + sizeof(cfgHeader_t));
    char *cfgData = (char *)(item + header.itemNum);

    for (i = 0; i < header.itemNum; i++)
    {
        if (item[i].type >= CFG_TYPE_NUM)
        {
            offset += item[i].size;
            continue;
        }

        for (j = 0; j < CFG_ITEM_NUM; j++)
        {
            //type按顺序排列
            if (gCfgItems[j].type > item[i].type)
            {
                break;
            }

            if (item[i].type == gCfgItems[j].type && item[i].item == gCfgItems[j].item && item[i].size == gCfgItems[j].size)
            {
                memcpy((char *)gCfgItems[j].pVar + gCfgItems[j].offSet, cfgData + offset, item[i].size);
                break;
            }
        }

        offset += item[i].size;
    }

    vPortFree(pBuf);

    //跟随版本走的配置要使用版本的默认值
    if (memcmp(radar_config_user.radar_info.SwVer, radar_config_default.radar_info.SwVer, sizeof(radar_config_default.radar_info.SwVer)))
    {
        memcpy(radar_config_user.radar_info.SwVer, radar_config_default.radar_info.SwVer, sizeof(radar_config_default.radar_info.SwVer));

        for (i = 0; i < CFG_ITEM_NUM; i++)
        {
            if (gCfgItems[i].followVer && gCfgItems[i].type < CFG_TYPE_NUM && gDefaultCfgs[gCfgItems[i].type][0])
            {
                memcpy((char *)gCfgItems[i].pVar + gCfgItems[i].offSet,
                       (char *)gDefaultCfgs[gCfgItems[i].type][1] + gCfgItems[i].offSet,
                       gCfgItems[i].size);
            }
        }
    }
#endif
    return &radar_config_user;
}

int saveRadarErrCfg(void)
{
    //增加看门狗喂狗时间
//    reloadSwtPeriod(APP_WATCHDOG_TIME * 15); //3s

    //计算使用存储长度，使用freeRTOS的内存申请
    int i, len = 0;
    for (i = 0; i < CFG_ITEM_NUM; i++)
    {
        len += gCfgItems[i].size;
    }
    len = sizeof(cfgHeader_t) + sizeof(cfgItemInfo_t) * CFG_ITEM_NUM + len; // 存储数据长度+item的总量+header头数据
    len += len % 4;                                                         // 字为单位长度
    len &= 0xFFFF;                                                          // 最大长度不能大于64k

    char *pBuf = (char *)pvPortMalloc(len);

    while (pBuf == NULL)
    {
    } //不可以失败

    cfgHeader_t *header = (cfgHeader_t *)pBuf;
    cfgItemInfo_t *item = (cfgItemInfo_t *)(header + 1);
    char *cfgData = (char *)(item + CFG_ITEM_NUM);
    int idx = 0;

    header->magicNum = CONFIG_MAGIC_NUM_NEW;
    header->itemNum = CFG_ITEM_NUM;

    for (i = 0; i < CFG_ITEM_NUM; i++)
    {
        item->type = gCfgItems[i].type;
        item->item = gCfgItems[i].item;
        item->size = gCfgItems[i].size;

        memcpy(cfgData + idx, (char *)gCfgItems[i].pVar + gCfgItems[i].offSet, gCfgItems[i].size);

        idx += gCfgItems[i].size;
        item++;
    }

    header->size = len;
    header->crc = update_crc(0, ((char *)pBuf) + 8, (header->size - 8));
    header->crc += 0xAAAA; //错误crc
    //擦除、写入
    eraseConfig(CONFIG_ADDR, CONFIG_ADDR_SZIE);
    int ret = flash_memory_writeb(CONFIG_ADDR, (const uint8_t *)pBuf, len);
    EMBARC_PRINTF("cfg write addr=0x%x len=0x%x header->crc=0x%x\r\n", (int)pBuf, len, header->crc);
    while (ret != 0)
    {
        //写入flash失败，不可以失败
    }
    vPortFree(pBuf);

//    reloadSwtPeriod(APP_WATCHDOG_TIME); //3s
    return 0;
}

int saveRadarCfg(void)
{
    //增加看门狗喂狗时间
//    reloadSwtPeriod(APP_WATCHDOG_TIME * 15); //3s

    //计算使用存储长度，使用freeRTOS的内存申请
    int i, len = 0;
    for (i = 0; i < CFG_ITEM_NUM; i++)
    {
        len += gCfgItems[i].size;
    }
    len = sizeof(cfgHeader_t) + sizeof(cfgItemInfo_t) * CFG_ITEM_NUM + len; // 存储数据长度+item的总量+header头数据
    len += len % 4;                                                         // 字为单位长度
    len &= 0xFFFF;                                                          // 最大长度不能大于64k

    char *pBuf = (char *)pvPortMalloc(len);
    if (pBuf == NULL)
    {
        EMBARC_PRINTF("alloc mem len %d failed\r\n", len);
        return E_SYS;
    }

    cfgHeader_t *header = (cfgHeader_t *)pBuf;
    cfgItemInfo_t *item = (cfgItemInfo_t *)(header + 1);
    char *cfgData = (char *)(item + CFG_ITEM_NUM);
    int idx = 0;

    header->magicNum = CONFIG_MAGIC_NUM_NEW;
    header->itemNum = CFG_ITEM_NUM;

    for (i = 0; i < CFG_ITEM_NUM; i++)
    {
        item->type = gCfgItems[i].type;
        item->item = gCfgItems[i].item;
        item->size = gCfgItems[i].size;

        memcpy(cfgData + idx, (char *)gCfgItems[i].pVar + gCfgItems[i].offSet, gCfgItems[i].size);

        idx += gCfgItems[i].size;
        item++;
    }

    header->size = len;
    header->crc = update_crc(0, ((char *)pBuf) + 8, (header->size - 8));
    //擦除、写入
    eraseConfig(CONFIG_ADDR, CONFIG_ADDR_SZIE);
    int ret = flash_memory_writeb(CONFIG_ADDR, (const uint8_t *)pBuf, len);
    if (ret != E_OK)
    {
        EMBARC_PRINTF("cfg write addr=0x%x len=0x%x header->crc=0x%x\r\n", (int)pBuf, len, header->crc);
    }

    vPortFree(pBuf);

    //增加看门狗喂狗时间
//    reloadSwtPeriod(APP_WATCHDOG_TIME); //3s
    
    return ret;
}

//当前flash的一个扇区的大小为 64k，该中转函数的功能是处理不够64k时候的操作
//所以使用的时候必须从64k的倍数开始，否则无法处理，一次擦除1个扇区
static int flash_erase(uint32_t addr, uint32_t len)
{
    int ret = 0;
    uint32_t sector_size = 0;

    ret = flash_get_info(addr, &sector_size);
    EMBARC_PRINTF("flash_erase ret=%d , size=%d , addr=0x%x\r\n", ret, sector_size, addr);
    if (sector_size == 0)
    {
        return ret;
    }

    if (addr & (sector_size - 1))
    {
        //不处理
        ret = 0;
    }
    else
    {
        int tmp = len % sector_size;
        if (tmp)
        {
            len += (sector_size - tmp);
        }
        ret = flash_memory_erase(addr, len);
    }
    return ret;
}

void eraseConfig(uint32_t addr, uint32_t len)
{
    flash_erase(addr, len);
}

int32_t getHwVer(void)
{
#if TX_ANT_TYPE == TX_410_1_ANT_TYPE || (TX_ANT_TYPE == TX_220P_ANT_TYPE) || TX_ANT_TYPE == TX_420_1_ANT_TYPE
    return gpio_read(GPIO_ID3) << 3 | gpio_read(GPIO_ID2) << 2 | gpio_read(GPIO_ID1) << 1 | gpio_read(GPIO_ID0);
#else
    return gpio_read(GPIO_ID2) << 2 | gpio_read(GPIO_ID1) << 1 | gpio_read(GPIO_ID0);
#endif
}

static radar_config_t *loadConfig(void)
{
    radar_config_t *pConfig;

    //配置初始化为默认值
    //注意这里暂时没有把alps的配置处理默认值
    for (int i = 0; i < (CFG_TYPE_NUM - 1); i++)
    {
        if (gDefaultCfgs[i][0] > 0)
        {
            memcpy((void *)gDefaultCfgs[i][0], (void *)gDefaultCfgs[i][1], gDefaultCfgs[i][2]);
        }
    }

    pConfig = loadRadarConfig();
#if (TX_ANT_TYPE == TX_410_1_ANT_TYPE) || (TX_ANT_TYPE == TX_220P_ANT_TYPE) || (TX_ANT_TYPE == TX_420_1_ANT_TYPE)
    pConfig->radar_info.HwVer[0] = 0x3;
#endif
    pConfig->radar_info.HwVer[1] = (char)getHwVer();
    return pConfig;
}

int configWrite(const radar_config_t *cfg)
{
    return saveRadarCfg();
}

void initFmcwParam(const sensor_config_t *cfg, int i)
{
    int txprofile = 0;
    int txchirp = 0;
    int txidx = 0;
    //根据track_frame_type确定是哪组参数，切换顺序tx1 chirp1, tx2 chirp1, 输出数据，tx1 chirp2, tx2 chirp2 , 输出数据
    //天线和chirp都切换
    switch (i)
    {
    case 0:
    {
        txprofile = 0;
        txchirp = 0;
        txidx = TX_CH1;
        break;
    }
    case 1:
    {
        txprofile = 1;
        txchirp = 0;
        txidx = TX_CH2;
        break;
    }
    case 2:
    {
        txprofile = 0;
        txchirp = 1;
        txidx = TX_CH1;
        break;
    }
    case 3:
    {
        txprofile = 1;
        txchirp = 1;
        txidx = TX_CH2;
        break;
    }
    default:
    {
        txprofile = 0;
        txchirp = 0;
        txidx = TX_CH1;
        break;
    }
    }

    //当前这个值没有意义,不会修改射频参数，初始化的时候已经配置了射频参数，中间不会修改
    gFmcwParam[txprofile][txchirp].bandWidth = cfg->fmcw_bandwidth / 1000.0; //参数单位变化
    gFmcwParam[txprofile][txchirp].tup = cfg->fmcw_chirp_rampup;
    gFmcwParam[txprofile][txchirp].tdown = cfg->fmcw_chirp_down;
    gFmcwParam[txprofile][txchirp].tidle = cfg->fmcw_chirp_period - cfg->fmcw_chirp_rampup - cfg->fmcw_chirp_down;
    gFmcwParam[txprofile][txchirp].txch = txidx;
    gFmcwParam[txprofile][txchirp].nfft_range = cfg->rng_nfft;
    gFmcwParam[txprofile][txchirp].nfft_vel = cfg->vel_nfft;
    gFmcwParam[txprofile][txchirp].nchirp = cfg->nchirp;
    gFmcwParam[txprofile][txchirp].startFrq = cfg->fmcw_startfreq;
    gFmcwParam[txprofile][txchirp].adcFreq = cfg->adc_freq;
    gFmcwParam[txprofile][txchirp].decFactor = cfg->dec_factor;
}

void initSetFmcwParam(sensor_config_t *cfg, int txprofile, int txchirp)
{
    gFmcwParam[txprofile][txchirp].bandWidth = cfg->fmcw_bandwidth / 1000.0; //参数单位变化
    gFmcwParam[txprofile][txchirp].tup = cfg->fmcw_chirp_rampup;
    gFmcwParam[txprofile][txchirp].tdown = cfg->fmcw_chirp_down;
    gFmcwParam[txprofile][txchirp].tidle = cfg->fmcw_chirp_period - cfg->fmcw_chirp_rampup - cfg->fmcw_chirp_down;
    //gFmcwParam[txprofile][txchirp].txch = txidx;
    gFmcwParam[txprofile][txchirp].nfft_range = cfg->rng_nfft;
    gFmcwParam[txprofile][txchirp].nfft_vel = cfg->vel_nfft;
    gFmcwParam[txprofile][txchirp].nchirp = cfg->nchirp;
    gFmcwParam[txprofile][txchirp].startFrq = cfg->fmcw_startfreq;
}

void antParamInit(void)
{
	//根据flash中的数据配置sensor_cfg数组的校准值
	u16 *ptrCalVer = (u16 *)radar_config_user.radar_info.CalVer;
    EMBARC_PRINTF("ptrCalVer = 0x%02x \r\n", *ptrCalVer);
	
	if((*ptrCalVer != 0 ) && (*ptrCalVer!=0xFF))
	{
        if (radar_config_user.radar_info.CalVer[1] >= 3)
        {
            radar_config_user.angleReverse = 0;
        }
        
		for (int i = 0; i < NUM_FRAME_TYPE; i++)
		{
			EMBARC_PRINTF("profile[%d]\r\n", i);
			EMBARC_PRINTF("antCaliMode[%d]\r\n", radar_config_user.antCaliMode[i]);

			sensor_config_t *cfg = sensor_config_get_config(i);
			if(radar_config_user.antCaliMode[i] == CAL_MODE_ONE_REG_OEN_CHECK /*&& dbfDataIsValid(i)*/){
				cfg->sv_read_from_flash = true; //使用存在flash中的dbf因子
				continue;
			}
			cfg->sv_read_from_flash = false;//不使用存在flash中的dbf因子
			for (int n = 0; n < 16; n++)
			{
				//EMBARC_PRINTF(" test i = %d , shift=%d , n=%d , idx=%d \n" , i , shift , n ,idx);
				//写入对应天线值
				cfg->ant_pos[n].x = radar_config_user.anten_distance[i][n] / (2.0f * PI);
				cfg->ant_pos[n].y = radar_config_user.anten_distance_y[i][n] / (2.0f * PI);
				cfg->ant_comps[n] = radar_config_user.anten_phase[i][n];
				EMBARC_PRINTF("idx=%d, x=%.3f , y=%.3f, comps=%.3f \n" , n , cfg->ant_pos[n].x , cfg->ant_pos[n].y , cfg->ant_comps[n] );
			}
		}
	}
}

static void params_init(void)
{
    initVehicleInfo();
//
//    setFcwSpeed(radar_config_user.fcwStartUpSpeed, radar_config_user.fcwFwdObjNegSpeed);
//
//	initSelfCali();
//
//	initInstallInfo();
//
//	initInstallCali();
//
//    initSrvCali();

	//默认debug模式关闭
	radar_config_user.debugModeSW = 0;

	//SR模式配置
	//swRadarSRMode(radar_config_using->srMode);

	//随机老化ID-暂时取BBSN的后7个数
	char bbsn[7];
	memcpy((char *)bbsn,(char *)radar_config_user.radar_info.pcbaBBSN + 14, 7);
	radar_config_user.randomId = (uint32_t)strtol(bbsn, NULL, 16);

	//boot版本信息
	radar_config_user.radar_info.bootSwVer[0] = boot_Info.sw_version.major_ver_id;
	radar_config_user.radar_info.bootSwVer[1] = boot_Info.sw_version.minor_ver_id;
	radar_config_user.radar_info.bootSwVer[2] = boot_Info.sw_version.stage_ver_id;
	radar_config_user.radar_info.bootSwVer[3] = boot_Info.sw_version.reserverd0;

    //天线校准值初始化
    antParamInit();
}

void config_init(UINT8 InitType)
{
    if (InitType == CONFIG_INIT_TYPE_ALL)
    {
        gen_crc_table();
    }

    loadConfig();

    //参数初始化
    params_init();

    for (int i = 0; i < 4; i++)
    {
        initFmcwParam(&config[i], i);
    }

    gTrkCfgMutex = xSemaphoreCreateMutex();
}

void setCfgAngleMap(uint8_t enable)
{
    radar_config_user.angleMapperEn = enable;
}

void setCfgAgingTest(uint8_t enable)
{
    radar_config_user.agingTest = enable;
}

void setCfgAngleMapVal(int txch, int idx, float val)
{
    radar_config_user.angleMapper[txch][idx].real = val;
}

void setCfgRadarId(uint8_t id)
{
    radar_config_user.compli_para.radarId = id;
}

void setCfgDebugMode(uint8_t mode)
{
    radar_config_user.debugModeSW = mode;
}

void setCfgTxPattern(uint8_t pattern)
{
    radar_config_user.tx_pattern = pattern;
}

//生产测试用，此函数只能用于app_dut 生产中用
radar_config_t *getRadarCfg(void)
{
    return &radar_config_user;
}

const fmcwParam_t *getFmcwParam(int txType, int chirpType)
{
    return &gFmcwParam[txType][chirpType];
}
