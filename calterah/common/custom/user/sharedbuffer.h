/*************************************************************************
 ************************CALTERAH CONFIDENTIAL****************************
 ***********************COPYRIGHT 2018 CALTERAH***************************
 *************************ALL RIGHTS RESERVED*****************************
 *
 * THIS SOFTWARE IS PROVIDED BY CALTERAH  "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL CALTERAH OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Author: GY Han
 * Date: 2018-5-1
 * Content:
 */

#ifndef _SHAREBUFFER_H_
#define _SHAREBUFFER_H_

#include "typedefs.h"
#include "sharedVar.h"
#include "angleMapping.h"

typedef struct
{
    u8 tx_power[4];
} txPowerInfo;

typedef struct
{
        unsigned int uart0ErrorCount;
        unsigned int uart1ErrorCount;
        unsigned int canErrorCount;
        unsigned int timerErrorCount;
} error_count_t;

typedef struct
{
        unsigned int start_freq;
        unsigned int stop_freq;
        unsigned int mid_freq;
        unsigned int step_up;
        unsigned int step_down;
        unsigned int cnt_wait;
        unsigned int total_count;
        // following is for debug purpose
        unsigned int up_cycle;
        unsigned int down_cycle;
        unsigned int wait_cycle;
} fmcw_reg_setup_t;

typedef struct
{
        double start_freq;
        double bandwidth;
        double t_rampup;
        double t_down;
        double t_idle;
        double Fs;
} fmcw_params_t;

typedef struct
{
        unsigned int FreStart;
        unsigned int FreCal;
        unsigned int FreStop;
        unsigned int Upslope;
        unsigned int Downslope;
        unsigned char waittime;
        float rangemax;
        float speedmax;
        unsigned char TXCFG;
} __fmcw_set;

typedef struct
{
        unsigned char x;
        unsigned char y;
        unsigned char resv[2];
} __attribute__((packed)) _compliA;

typedef struct
{
        _compliA compliA[5];
        int radarId;
        int iVersion;
} __attribute__((packed)) _compli_para;

typedef enum
{
        NO_FRAME_OUT,
        UART_OUT,
        CAN_OUT,
        UART_AND_CAN_OUT
} frame_out_t;

typedef enum
{
	MAIN_DRIVING_CAB = 0x0,	//主驾驶室
	VICE_DRIVING_CAB = 0x1,	//副驾驶室
	UPTURNED = 0x2,			//朝上
	ADOWN = 0x3,			//朝下
} radarInstallDirection;

typedef struct
{
    char type[16];
    char pcbaBBSN[24];
    char imuName[24];
    char radarSN[24];
    char macaddr[6];
    char ipaddr[4];
    char HwVer[4];
    char SwVer[4];
    char bootSwVer[4];
    char CalVer[2];
} __attribute__((packed)) radar_info_t;

/*
 * When add items in this structure, please also add corresponding part in CLI.
 */
typedef struct
{
        unsigned int configMagicNum;
        _compli_para compli_para;
        uint8_t sendCarVel;  //是否发送速度
        uint8_t sendYawrate; //是否发送曲率
        uint8_t speedSource;
        uint8_t installCalcState;
        double fmcw_startfreq;
        double fmcw_bandwidth;
        double fmcw_trampup;
        double fmcw_tdown;
        double fmcw_tidle;
        unsigned int bb_buf_n;
        unsigned int bb_buf_l;
        unsigned int bb_fft_n;
        unsigned int bb_fft_l;

        float fcwStartUpSpeed;
        float fcwFwdObjNegSpeed;

        float fmcw_bandwidth2;
        float anten_distance[4][16];
        float anten_distance_y[4][16];
        float anten_phase[4][16];

        int8_t rcsOffset[4];

        unsigned int tx_pattern; //0-normal,1-switch between tx1 and tx2, 2-switch bandwidth, 3-切换带宽和天线，other-normal
        unsigned int tx_sel;     //used when tx_pattern = 0;  1:tx1, 2:tx2, 3:tx1&tx2
        unsigned int tx_cw;      //cw mode, 0: normal, 1 : cw

        /* System control parameters */
        uint8_t swGainEn;   //使能近距离增益切换
        uint8_t lowVgaGain; //底增益模式下gain值
        uint8_t angleReverse; //角度反转
        uint8_t angleMapperEn;  //角度补偿使能

        Angle angleMapper[2][ANG_NUM];

        float installHeight;
        float installAngle;
        float horizontalOffset; //距离中心轴偏移距离
        float angleOffset;      //角度偏移

        int16_t yawrate_zero;     //陀螺仪校准
        int16_t accCalcZero[3];   //加速度计校准,放大10^4倍保存
        uint8_t debugModeSW;      //调试模式开关，0不开，1开启
        uint8_t objExtendInfoSW;  //雷达扩展信息是能开关，0不发送，1发送
        uint8_t objExtendInfoSW2; //雷达扩展信息2是能开关，0不发送，1发送
        uint8_t protocolVersion;  //雷达协议版本

        uint32_t randomId;       //老化信息ID
        uint8_t agingTest;       //老化信息发送开关
        uint8_t canBaudRateMode; //can波特率模式配置
        radar_info_t radar_info;
        unsigned int dbfMagicNum; //DBF Coef Flag
        radarInstalledCalcObj installedObj;
        uint8_t srMode;                //雷达SR模式 //0-siso 1-MIMO 2-SISO+dml 3-MIMO+DMl 0xFF-无效，使用代码配置
        uint8_t trkTxFilter;           //远近天线过滤模式 //0-不过滤 1-过滤远天线   2过滤近天线
        uint8_t isSendCTCanProtocol;   //时候发送承泰协议 
        float installElevatedOffset; //安装俯仰角补偿
        txPowerInfo txPower;
		int8_t azimuthDecayTx[4][151];//方向图
		int8_t antCaliMode[4];			  //使能1度1校准还是拟合的方式, 0 表示使用1度1校准的方式，1表示使用拟合的方式

        unsigned int crc;              //CRC of config to prevent NVRAM damage

} __attribute__((packed)) radar_config_t;

#endif
