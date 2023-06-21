/*
 * @Author: your name
 * @Date: 2020-06-11 09:55:51
 * @LastEditTime: 2022-04-20 14:18:27
 * @LastEditors: fang yongjun
 * @Description: In User Settings Edit
 * @FilePath: cfg.h
 */

#ifndef MODULE_CFG_CFG_H_
#define MODULE_CFG_CFG_H_

#include "sharedbuffer.h"
#include "sharedVar.h"
#include "sensor_config.h"
#include "flash_header.h"

#define TRX_SYNC_INTERRUPT 0

#define SPEED_LIGTH 299792458 // speed of light

#if SAMPLE_CNT == 1024
#define T_CHIRP_SCALE ((float)11 / 10) //解速度模糊chirp时长比例
#else
#define T_CHIRP_SCALE ((float)8 / 7) //解速度模糊chirp时长比例
#endif

#define CONFIG_MAGIC_NUM 0xCACACACA

#define ANGLE_FILTER 0
#define RANGE_FILTER 1
#define MAX_ANGLE_VAL 45
//#define FILTER_BOUNDARY_RNG     2
#define MAX_RANGE_VAL 200

#define CALTERAH_ANGLE_CAL 1

#define NO_FILTER_FENCE 0	//不过滤
#define FILTER_ALL_FENCE 1	//过滤护栏
#define FILTER_FIX_FENCE 2	//修正护栏
#define FILTER_STATIC_OBJ 4 //过滤静止目标
#define FILTER_3_LANE 8		//过滤三车道外目标

#define CONFIG_INIT_TYPE_ALL		1
#define CONFIG_INIT_TYPE_LOAD_FLASH	2

extern const radar_config_t * const radar_config_using;

typedef struct
{
	uint16_t txch;
	uint16_t nfft_range;
	uint16_t nfft_vel;
	uint16_t nchirp;
	float tup;
	float tdown;
	float tidle;
	float bandWidth;
	float startFrq;
	uint8_t	adcFreq;
	uint8_t decFactor;
} fmcwParam_t;

enum TRK_CFG_E
{
	TRK_SELF_CALI,
	TRK_INIT_FLAG,
	TRK_CHSN_EN,
	TRK_RADAR_MODE,
	TRK_DEBUG_MODE,
	TRK_FILTER_TYPE
};

bool isFlashHeadError(void);
bool isFlashCrcError(void);

void config_init(UINT8 InitType);
int configWrite(const radar_config_t *cfg);
void eraseConfig(uint32_t addr , uint32_t len);
void initFmcwParam(const sensor_config_t *bb, int i);
void initSetFmcwParam(sensor_config_t *cfg, int txprofile, int txchirp);
int saveRadarCfg(void);
int saveRadarErrCfg(void);
uint8_t swRadarSRMode(uint8_t mode);
void eraseSector(uint32_t addr, uint32_t len);

void rebootFlagInfoSave(uint8_t *pFlag, int size);
void rebootFlagInfoRead(uint8_t *pFlag, int size);

void setTrkCfg(enum TRK_CFG_E type, uint32_t value);
void updateTrkCfg(radarTrkCfg_t *pDstCfg);

void setCfgInstallAngle(float angle);
void setCfgInstallHeigh(float heigh);
void setCfgAngleOffset(float angle);
void setCfgHrzOffset(float offset);
void setCfgElevateOffset(float offset);
void setCfgInstalledObj(const radarInstalledCalcObj *pObj);
void setCfgInstallCaliStatus(uint8_t status);
void setCfgRcsOffset(int8_t offset,int8_t offset1);

void setCfgRcsOffset220Plus(uint8_t profileIdx, int8_t offset);
void setCfgAntDistanceX(uint8_t profileIdx, uint8_t offset,float f);
void setCfgAntDistanceY(uint8_t profileIdx, uint8_t offset,float f);
void setCfgAntPhase(uint8_t profileIdx, uint8_t offset, float f);
void setCfgAzimuthDecay(uint8_t profileIdx, uint8_t offset, int8_t i);
void setCfgAntCaliMode(uint8_t profileIdx, int8_t value);

void setCfgSendExt(uint8_t enable);
void setCfgSendExt2(uint8_t enable);
void setCfgSpeed(uint8_t source, uint8_t send);
void setCfgSendYawrate(uint8_t enable);
void setCfgAngleMap(uint8_t enable);
void setCfgDebugMode(uint8_t mode);
void setCfgProtVersion(uint8_t version);
void setCfgInstallElevatedOffset(float angle);
void setCfgTxPattern(uint8_t pattern);
void setCfgTxSel(uint8_t tx);
void setCfgFmcwStartFreq(float freq);
void setCfgRandomId(uint32_t id);
void setCfgAgingTest(uint8_t enable);
void setCfgAngleMapVal(int txch, int idx, float val);
void setCfgRadarId(uint8_t id);
const fmcwParam_t *getFmcwParam(int txType, int chirpType);
const image_header_t *getBootInfo(void);

#endif /* SHAREDSTRUCTURE_H_ */
