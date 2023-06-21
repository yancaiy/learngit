#ifndef FUNC_SAFETY_H
#define FUNC_SAFETY_H

#include "func_safety_config.h"

#ifdef UNIT_TEST
#include "calterah_unit_test.h"
#else
#include "embARC_toolchain.h"
#endif

#include "fmcw_radio.h"
#include "baseband_hw.h"
#include "func_safety_config.h"

#define LED_D1_NO                               2
#define LED_D1_ON                               1
#define LED_D1_OFF                              0

// fsm cmd out bits fields define.
#define FUNC_SAFETY_BIT_0_SHIFT                 (0)
#define FUNC_SAFETY_BIT_1_SHIFT                 (1)
#define FUNC_SAFETY_BIT_2_SHIFT                 (2)
#define FUNC_SAFETY_BIT_3_SHIFT                 (3)
#define FUNC_SAFETY_BIT_4_SHIFT                 (4)
#define FUNC_SAFETY_BIT_5_SHIFT                 (5)
#define FUNC_SAFETY_BIT_6_SHIFT                 (6)
#define FUNC_SAFETY_BIT_7_SHIFT                 (7)
#define FUNC_SAFETY_BIT_8_SHIFT                 (8)
#define FUNC_SAFETY_BIT_9_SHIFT                 (9)
#define FUNC_SAFETY_BIT_10_SHIFT                (10)
#define FUNC_SAFETY_BIT_11_SHIFT                (11)
#define FUNC_SAFETY_BIT_12_SHIFT                (12)
#define FUNC_SAFETY_BIT_13_SHIFT                (13)
#define FUNC_SAFETY_BIT_14_SHIFT                (14)
#define FUNC_SAFETY_BIT_15_SHIFT                (15)
#define FUNC_SAFETY_BIT_16_SHIFT                (16)
#define FUNC_SAFETY_BIT_17_SHIFT                (17)
#define FUNC_SAFETY_BIT_18_SHIFT                (18)
#define FUNC_SAFETY_BIT_19_SHIFT                (19)
#define FUNC_SAFETY_BIT_20_SHIFT                (20)
#define FUNC_SAFETY_BIT_21_SHIFT                (21)
#define FUNC_SAFETY_BIT_22_SHIFT                (22)

// fsm index.
#define SM_INDEX_0                 (0)
#define SM_INDEX_1                 (1)
#define SM_INDEX_2                 (2)
#define SM_INDEX_3                 (3)
#define SM_INDEX_4                 (4)
#define SM_INDEX_5                 (5)
#define SM_INDEX_6                 (6)
#define SM_INDEX_7                 (7)
#define SM_INDEX_8                 (8)
#define SM_INDEX_9                 (9)
#define SM_INDEX_10                (10)
#define SM_INDEX_11                (11)
#define SM_INDEX_12                (12)
#define SM_INDEX_13                (13)
#define SM_INDEX_14                (14)
#define SM_INDEX_15                (15)
#define SM_INDEX_16                (16)
#define SM_INDEX_17                (17)
#define SM_INDEX_18                (18)
#define SM_INDEX_19                (19)
#define SM_INDEX_20                (20)
#define SM_INDEX_21                (21)
#define SM_INDEX_22                (22)
#define SM_INDEX_23                (23)
#define SM_INDEX_24                (24)
#define SM_INDEX_25                (25)
#define SM_INDEX_26                (26)
#define SM_INDEX_27                (27)
#define SM_INDEX_28                (28)
#define SM_INDEX_29                (29)
#define SM_INDEX_30                (30)
#define SM_INDEX_31                (31)
#define SM_INDEX_32                (32)
#define SM_INDEX_33                (33)
#define SM_INDEX_34                (34)
#define SM_INDEX_35                (35)
#define SM_INDEX_36                (36)
#define SM_INDEX_37                (37)
#define SM_INDEX_38                (38)
#define SM_INDEX_39                (39)
#define SM_INDEX_40                (40)



#define MAX_FUSA_TEST_BUF_LEN      64

typedef struct func_safety {
        uint32_t dummy;
        uint8_t error_type;
} func_safety_t;

typedef enum {
        SAFE_STATE_IRQ = 0,
        SAFE_STATE_SS1,
        SAFE_STATE_SS2
} safety_error_t;

typedef enum {
        PERIODIC_SM_1_CYCLE = 0,
        PERIODIC_SM_2_CYCLE,
        PERIODIC_SM_3_CYCLE,
        PERIODIC_SM_4_CYCLE
} safety_cycle_t;

/* can send status */
typedef enum {
    CAN_SEND_STATUS_IDLE = 0,   /* can send status is idle */
    CAN_SEND_STATUS_SENDING,    /* can send status is sending */
} CAN_SEND_STATUS;


typedef struct fusa_config {
        uint8_t sm_index;
        uint16_t sm_num;
        uint8_t error_type;
        bool open_flag;
}fusa_config_t;


//EMU SAFETY TEST NUM
#define  FUNC_SAFETY_ITEM_SM1          1
#define  FUNC_SAFETY_ITEM_SM2          2
#define  FUNC_SAFETY_ITEM_SM3          3
#define  FUNC_SAFETY_ITEM_SM4          4
#define  FUNC_SAFETY_ITEM_SM5          5
#define  FUNC_SAFETY_ITEM_SM6          6
#define  FUNC_SAFETY_ITEM_SM8          8
#define  FUNC_SAFETY_ITEM_SM9          9
#define  FUNC_SAFETY_ITEM_SM10         10
#define  FUNC_SAFETY_ITEM_SM11         11
#define  FUNC_SAFETY_ITEM_SM12         12
#define  FUNC_SAFETY_ITEM_SM13         13
#define  FUNC_SAFETY_ITEM_SM14         14

#define  FUNC_SAFETY_ITEM_SM101        101
#define  FUNC_SAFETY_ITEM_SM102        102
#define  FUNC_SAFETY_ITEM_SM103        103
#define  FUNC_SAFETY_ITEM_SM104        104
#define  FUNC_SAFETY_ITEM_SM105        105
#define  FUNC_SAFETY_ITEM_SM106        106
#define  FUNC_SAFETY_ITEM_SM107        107
#define  FUNC_SAFETY_ITEM_SM108        108
#define  FUNC_SAFETY_ITEM_SM109        109
#define  FUNC_SAFETY_ITEM_SM110        110
#define  FUNC_SAFETY_ITEM_SM112        112
#define  FUNC_SAFETY_ITEM_SM113        113
#define  FUNC_SAFETY_ITEM_SM114        114
#define  FUNC_SAFETY_ITEM_SM117        117
#define  FUNC_SAFETY_ITEM_SM118        118
#define  FUNC_SAFETY_ITEM_SM120        120
#define  FUNC_SAFETY_ITEM_SM121        121
#define  FUNC_SAFETY_ITEM_SM122        122
#define  FUNC_SAFETY_ITEM_SM123        123
#define  FUNC_SAFETY_ITEM_SM124        124
#define  FUNC_SAFETY_ITEM_SM125        125
#define  FUNC_SAFETY_ITEM_SM126        126
#define  FUNC_SAFETY_ITEM_SM127        127
#define  FUNC_SAFETY_ITEM_SM128        128
#define  FUNC_SAFETY_ITEM_SM129        129
#define  FUNC_SAFETY_ITEM_SM130        130
#define  FUNC_SAFETY_ITEM_SM133        133

#define  FUNC_SAFETY_ITEM_SM201        201
#define  FUNC_SAFETY_ITEM_SM202        202
#define  FUNC_SAFETY_ITEM_SM203        203
#define  FUNC_SAFETY_ITEM_SM204        204
#define  FUNC_SAFETY_ITEM_SM205        205
#define  FUNC_SAFETY_ITEM_SM206        206
#define  FUNC_SAFETY_ITEM_SM207        207

#define  FUNC_SAFETY_ITEM_SM805        805
#define  FUNC_SAFETY_ITEM_SM901        901
#define  FUNC_SAFETY_ITEM_SM902        902
#define  FUNC_SAFETY_ITEM_SM904        904
#define  FUNC_SAFETY_ITEM_SM905        905
#define  FUNC_SAFETY_ITEM_SM906        906
#define  FUNC_SAFETY_ITEM_SM907        907
#define  FUNC_SAFETY_ITEM_SM908        908
#define  FUNC_SAFETY_ITEM_SM910        910

#define REG_EMU_SPARED_0                (REL_REGBASE_EMU + 0x0600)
#define REG_CPU_ERP_CTRL                (0x3F)
#define VGA_DIFF_VALUE                  (5.00)

#define EMU_SAVE_WDG_TO_MEM_ADDR       (0x7f0100)
#define PERIOD_TO_MS                   (3 * 100000)
#define FUNC_SAFETY_FRAME_ID           0x123

#define FUNC_SAFETY_ITEM_SM1_SET_PART    0
#define FUNC_SAFETY_ITEM_SM1_READ_PART   1

extern bool bb_frame_start_flag;
extern bool ldo_set_part_flag;
extern volatile uint8_t sm1_ldo_part_cnt;
extern bool periodic_sm_finish_flag;
extern bool sample_adc_running_flag;

#if FUNC_SAFETY_CLI == FEATURE_ON
#define log_fusa            EMBARC_PRINTF
#else
#define log_fusa(...)
#endif

void fusa_disable_all(void);
void func_safety_check_error_code(void);
void func_safety_init(func_safety_t *fsm);
void led_d1_init(void);
void func_safety_test_handler(int32_t func_safety_sm_num, uint8_t func_safety_error_type);
void func_safety_error_handler(uint16_t func_safety_sm_num, uint8_t func_safety_error_type);
void func_safety_sm_can_config_reg_protection(uint8_t func_safety_error_type);
void clear_all_bb_memory(baseband_hw_t *bb_hw);
void clear_bb_memory_by_scan_start_bb_one_time(void);
void safety_mechanism_power_on_check(fmcw_radio_t *radio);
void func_safety_sm_ldo_monitor_set_part(fmcw_radio_t *radio);
void func_safety_sm_ldo_monitor_read_part(void);
void func_safety_enable_can_ecc(uint32_t id);
uint8_t get_can_send_status(void);
void set_can_send_status(uint8_t value);


//about radio ctrl,move from firmware of RF fsm
void func_safety_sm_can_loopback_init(void);
void func_safety_sm_can_loopback_rx_handler(uint8_t *data, uint32_t len);
void func_safety_sm_periodic_run_handler(void);
float fusa_get_current_time_ms(void);
void func_safety_process(void *params);
void fusa_run_periodic_item(int32_t func_safety_sm_num, uint32_t param);
void fusa_run_periodic_items_default(void);

//QSPI TEST
#define TEST_QSPI_DAT 0x12345678

#define DMU_CMD_SRC_SEL                  0
#define DMU_CMD_OUT                      1
#define DMU_CMD_IN                       2
#define DMU_CMD_SRC_CPU                  2
#define CLKGEN_READY_PLL                 1

#define CLKGEN_DIV_AHB                  66
#define CLKGEN_DIV_APB                  67
#define CLKGEN_DIV_APB_REF              68
#define CLKGEN_DIV_CAN_0                69
#define CLKGEN_DIV_CAN_1                70
#define CLKGEN_SEL_300M                  2
#define CLKGEN_SEL_400M                  3

#define SPI_S_ADDR                      0xB80000
#define QSPI_M_ADDR                     0xB90000
#define DMU_ADDR                        0xBA0000
#define DRx                             0x60
#define SER                             0x10
#define BAUDR                           0x14
#define RXFLR                           0x24
#define SR                              0x28
#define SSIENR                          0x8
#define CTRLR0                          0x0
#define CTRLR1                          0x4
#define TXFTLR                          0x18
#define RXFTLR                          0x1c
#define IMR                             0x2C
#define SPI_CTRLR0                      0xf4
#define CLKGEN_ADDR                     0xB20000


#endif // FUNC_SAFETY_H

