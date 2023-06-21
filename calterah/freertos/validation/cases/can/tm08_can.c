#include "tm08_can.h"
#include "validation.h"
#include "vmodule.h"
#include "can_hal.h"
#include "dev_can.h"
#include "can_cli.h"
#include "dma_hal.h"
#include "can_obj.h"
#include "timer_hal.h"

#define USE_CAN_TEST_CODE 1

#define CAN_VALIDATION_SYS_CASE(cid, f_init) VALIDATION_SYS_CASE_DEFINE(MODULE_CAN_ID, cid, f_init, NULL)

extern int32_t can_cli_register(uint32_t can_id, uint32_t msg_id, can_cli_callback func);

#if (USE_CAN_TEST_CODE == 1)

typedef struct can_checksum_session {
    uint32_t msg_id;

    uint32_t rx_cnt;
    uint32_t rx_sum;
    uint32_t tx_cnt;
    uint32_t tx_sum;

    uint32_t rx_cnt_shadow;
    uint32_t tx_cnt_shadow;

    uint32_t expect_seq_num;
    uint32_t unexpect_seq_counter;
} can_checksum_session_t;
can_checksum_session_t can_node_session_id300;
can_checksum_session_t can_node_session_id301;
can_checksum_session_t can_node_session_id310;
can_checksum_session_t can_node_session_id311;
can_checksum_session_t can_node_session_id320;
can_checksum_session_t can_node_session_id321;
can_checksum_session_t can_node_session_id100;
can_checksum_session_t can_node_session_idother;

void can_check_seqence_checksum(uint32_t id, uint8_t* data, uint32_t len)
{
    uint32_t curr_seq_num = 0;

    curr_seq_num = ((uint32_t)(data[3]) << 24) |
                   ((uint32_t)(data[2]) << 16) |
                   ((uint32_t)(data[1]) << 8)  |
                    (uint32_t)(data[0]);
    //EMBARC_PRINTF("curr_seq_num :  %x\r\n", curr_seq_num);

    switch (id) {
        case 0x300:
            if (curr_seq_num != can_node_session_id300.expect_seq_num) {
                EMBARC_PRINTF("###### 300 Expect 0x%x , but got 0x%x, then next expect 0x%x \r\n",can_node_session_id300.expect_seq_num, curr_seq_num, curr_seq_num + 1);
                can_node_session_id300.expect_seq_num = curr_seq_num + 1;
                can_node_session_id300.unexpect_seq_counter++;
            } else {
                can_node_session_id300.expect_seq_num ++;
            }
            break;
        case 0x301:
            if (curr_seq_num != can_node_session_id301.expect_seq_num) {
                EMBARC_PRINTF("###### 301 Expect 0x%x , but got 0x%x, then next expect 0x%x \r\n",can_node_session_id301.expect_seq_num, curr_seq_num, curr_seq_num + 1);
                can_node_session_id301.expect_seq_num = curr_seq_num + 1;
                can_node_session_id301.unexpect_seq_counter++;
            } else {
                can_node_session_id301.expect_seq_num ++;
            }
            break;
        case 0x310:
            if (curr_seq_num != can_node_session_id310.expect_seq_num) {
                EMBARC_PRINTF("###### 310 Expect 0x%x , but got 0x%x, then next expect 0x%x \r\n",can_node_session_id310.expect_seq_num, curr_seq_num, curr_seq_num + 1);
                can_node_session_id310.expect_seq_num = curr_seq_num + 1;
                can_node_session_id310.unexpect_seq_counter++;
            } else {
                can_node_session_id310.expect_seq_num ++;
            }
            break;
        case 0x311:
            if (curr_seq_num != can_node_session_id311.expect_seq_num) {
                EMBARC_PRINTF("###### 311 Expect 0x%x , but got 0x%x, then next expect 0x%x \r\n",can_node_session_id311.expect_seq_num, curr_seq_num, curr_seq_num + 1);
                can_node_session_id311.expect_seq_num = curr_seq_num + 1;
                can_node_session_id311.unexpect_seq_counter++;
            } else {
                can_node_session_id311.expect_seq_num ++;
            }
            break;
        case 0x320:
            if (curr_seq_num != can_node_session_id320.expect_seq_num) {
                EMBARC_PRINTF("###### 320 Expect 0x%x , but got 0x%x, then next expect 0x%x \r\n",can_node_session_id320.expect_seq_num, curr_seq_num, curr_seq_num + 1);
                can_node_session_id320.expect_seq_num = curr_seq_num + 1;
                can_node_session_id320.unexpect_seq_counter++;
            } else {
                can_node_session_id320.expect_seq_num ++;
            }
            break;
        case 0x321:
            if (curr_seq_num != can_node_session_id321.expect_seq_num) {
                EMBARC_PRINTF("###### 321 Expect 0x%x , but got 0x%x, then next expect 0x%x \r\n",can_node_session_id321.expect_seq_num, curr_seq_num, curr_seq_num + 1);
                can_node_session_id321.expect_seq_num = curr_seq_num + 1;
                can_node_session_id321.unexpect_seq_counter++;
            } else {
                can_node_session_id321.expect_seq_num ++;
            }
            break;
        case 0x100:
            if (curr_seq_num != can_node_session_id100.expect_seq_num) {
                EMBARC_PRINTF("###### 100 Expect 0x%x , but got 0x%x, then next expect 0x%x \r\n",can_node_session_id100.expect_seq_num, curr_seq_num, curr_seq_num + 1);
                can_node_session_id100.expect_seq_num = curr_seq_num + 1;
                can_node_session_id100.unexpect_seq_counter++;
            } else {
                can_node_session_id100.expect_seq_num ++;
            }
            break;
        default:
            if (curr_seq_num != can_node_session_idother.expect_seq_num) {
                EMBARC_PRINTF("###### other Expect 0x%x , but got 0x%x, then next expect 0x%x \r\n",can_node_session_idother.expect_seq_num, curr_seq_num, curr_seq_num + 1);
                can_node_session_idother.expect_seq_num = curr_seq_num + 1;
                can_node_session_idother.unexpect_seq_counter++;
            } else {
                can_node_session_idother.expect_seq_num ++;
            }
    }
}

static uint32_t CHECKSUM_BYTES(uint8_t* data, uint32_t len)
{
    uint32_t sum = 0;

    //xil_printf("TX id %d : %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\r\n",msg->msg_id,msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7],msg->data[8], msg->data[9], msg->data[10], msg->data[11], msg->data[12], msg->data[13], msg->data[14], msg->data[15] );
    for (uint32_t i = 0; i<len; i++) {
        sum += data[i];
    }

    return sum;
}

void can_msg_checksum(uint32_t id, uint8_t* data, uint32_t len, uint32_t is_rx)
{
    //taskENTER_CRITICAL();
    switch (id) {
        case 0x300:
            if (is_rx) {
                can_node_session_id300.rx_cnt++;
                can_node_session_id300.rx_sum = (can_node_session_id300.rx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            } else {
                can_node_session_id300.tx_cnt++;
                can_node_session_id300.tx_sum = (can_node_session_id300.tx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            }
            break;
        case 0x301:
            if (is_rx) {
                can_node_session_id301.rx_cnt++;
                can_node_session_id301.rx_sum = (can_node_session_id301.rx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            } else {
                can_node_session_id301.tx_cnt++;
                can_node_session_id301.tx_sum = (can_node_session_id301.tx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            }
            break;
        case 0x310:
            if (is_rx) {
                can_node_session_id310.rx_cnt++;
                can_node_session_id310.rx_sum = (can_node_session_id310.rx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            } else {
                can_node_session_id310.tx_cnt++;
                can_node_session_id310.tx_sum = (can_node_session_id310.tx_sum + CHECKSUM_BYTES(data, len))  & 0xFF;
            }
            break;
        case 0x311:
            if (is_rx) {
                can_node_session_id311.rx_cnt++;
                can_node_session_id311.rx_sum = (can_node_session_id311.rx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            } else {
                can_node_session_id311.tx_cnt++;
                can_node_session_id311.tx_sum = (can_node_session_id311.tx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            }
            break;
        case 0x320:
            if (is_rx) {
                can_node_session_id320.rx_cnt++;
                can_node_session_id320.rx_sum = (can_node_session_id320.rx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            } else {
                can_node_session_id320.tx_cnt++;
                can_node_session_id320.tx_sum = (can_node_session_id320.tx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            }
            break;
        case 0x321:
            if (is_rx) {
                can_node_session_id321.rx_cnt++;
                can_node_session_id321.rx_sum = (can_node_session_id321.rx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            } else {
                can_node_session_id321.tx_cnt++;
                can_node_session_id321.tx_sum = (can_node_session_id321.tx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            }
            break;
        case 0x100:
            if (is_rx) {
                can_node_session_id100.rx_cnt++;
                can_node_session_id100.rx_sum = (can_node_session_id100.rx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            } else {
                can_node_session_id100.tx_cnt++;
                can_node_session_id100.tx_sum = (can_node_session_id100.tx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            }
            break;
        default:
            if (is_rx) {
                can_node_session_idother.rx_cnt ++;
                can_node_session_idother.rx_sum = (can_node_session_idother.rx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            } else {
                can_node_session_idother.tx_cnt ++;
                can_node_session_idother.tx_sum = (can_node_session_idother.tx_sum + CHECKSUM_BYTES(data, len)) & 0xFF;
            }
            break;
    }

    //xil_printf("300 tx_cnt:0x%x, tx_sum:0x%x \r\n", can_node_session_id300.tx_cnt, can_node_session_id300.tx_sum);
    //xil_printf("301 tx_cnt:0x%x, tx_sum:0x%x \r\n", can_node_session_id301.tx_cnt, can_node_session_id301.tx_sum);
    //taskEXIT_CRITICAL();
}

void can_reset_checksum(can_checksum_session_t *node_id)
{
    node_id->rx_cnt = 0;
    node_id->tx_cnt = 0;
    node_id->rx_sum = 0;
    node_id->tx_sum = 0;
    node_id->expect_seq_num = 0;
    node_id->unexpect_seq_counter = 0;
}
void can_reset_checksum_all()
{
    can_reset_checksum(&can_node_session_id100);
    can_reset_checksum(&can_node_session_id300);
    can_reset_checksum(&can_node_session_id301);
    can_reset_checksum(&can_node_session_id310);
    can_reset_checksum(&can_node_session_id311);
    can_reset_checksum(&can_node_session_id320);
    can_reset_checksum(&can_node_session_id321);
    can_reset_checksum(&can_node_session_idother);
}

//#define ID300 1
//#define ID310 1
#define ID320 1

#ifdef ID300
#define TX_TASK_ID              (0x300)
#define TEST_CAN_BURST_NUM      (2)  //(1439999)           // ID300 10 hour  2375999
#define TX_TASK_INTERVAL        (DELAY_15_MS)
#define TX_TASK_BURST_CNT       (5)
#define TX_TASK_DATA_IDX        (0)
#endif
#ifdef ID310
#define TX_TASK_ID              (0x310)
#define TEST_CAN_BURST_NUM      (3)  //(1799999)           // ID310 10 hour
#define TX_TASK_INTERVAL        (DELAY_20_MS)
#define TX_TASK_BURST_CNT       (8)
#define TX_TASK_DATA_IDX        (6)
#endif
#ifdef ID320
#define TX_TASK_ID              (0x310)
#define TEST_CAN_BURST_NUM      (200000)//200000 - 10 hour
#define TX_TASK_INTERVAL        (DELAY_15_MS)
#define TX_TASK_BURST_CNT       (12)
#define TX_TASK_DATA_IDX        (6)
#endif

volatile uint32_t can_frame_num = 0;
volatile uint32_t can_burst_cnt = 0;

static uint32_t can_send_test(void)
{
    /* variable */
    uint32_t track_data[128] = {0};
    uint32_t i = 0;
    uint32_t data_len = 0;
    uint32_t send_cnt = 0;
    track_data[0] = 0x00000000;
    track_data[1] = 0x11111111;
    track_data[2] = 0x93874624;
    track_data[3] = 0x33333333;

    track_data[4] = 0x44444444;
    track_data[5] = 0x55555555;
    track_data[6] = 0xF7DC7B29;
    track_data[7] = 0xA767D7F7;

    track_data[8] = 0x88888888;
    track_data[9] = 0x90399469;
    track_data[10] = 0xA7729EBF;
    track_data[11] = 0x12345678;

    track_data[12] = 0x01213149;
    track_data[13] = 0x91314151;
    track_data[14] = 0x41526374;
    track_data[15] = 0x01515157;

    track_data[16] = 0x16D6F636;
    track_data[17] = 0x717E7379;
    track_data[18] = 0x28A8C858;
    track_data[19] = 0x919D979F;

    if (can_burst_cnt < TEST_CAN_BURST_NUM) {

        #if (USE_CAN_FD == 1)
        data_len = eDATA_LEN_64;
        #else
        data_len = eDATA_LEN_8;
        #endif

        for(i=0;i<TX_TASK_BURST_CNT;i++){
            track_data[TX_TASK_DATA_IDX] = can_frame_num;
            can_frame_num++;

            while(can_send_data(0, TX_TASK_ID, &track_data[TX_TASK_DATA_IDX], data_len) == E_DBUSY){
                send_cnt++;
                if (send_cnt >= 5){
                    break;
                }
           }
            send_cnt = 0;
            can_msg_checksum(TX_TASK_ID, (unsigned char*)(&track_data[TX_TASK_DATA_IDX]), data_len, 0);
        }

        can_burst_cnt++;
        return 0;
    } else {
        EMBARC_PRINTF("[can_send_test] All transaction done: id= %d, burst=%d, burstCnt= %d!\r\n", TX_TASK_ID, can_burst_cnt, TX_TASK_BURST_CNT);
        return 1;
    }
}

#define TIMER_ID                (1)
#define TEST_CAN_CASE_TIME      (300000)

#define DELAY_10_SECONDS        (10000UL)
#define DELAY_1_SECOND          (1000UL)
#define DELAY_2_SECOND          (2000UL)
#define DELAY_30_MS             (30UL)
#define DELAY_25_MS             (25UL)
#define DELAY_20_MS             (20UL)
#define DELAY_15_MS             (15UL)
#define DELAY_10_MS             (10UL)

#define PRINT_INTERVAL          (DELAY_1_SECOND)

static TaskHandle_t can_test_handle = NULL;
static TimerHandle_t xTimer = NULL;
void can_test_print_rx_info(can_checksum_session_t* chksum)
{
    if(chksum->rx_cnt && chksum->rx_cnt != chksum->rx_cnt_shadow) {
        chksum->rx_cnt_shadow = chksum->rx_cnt;
        EMBARC_PRINTF("%X rx_cnt:0x%x, rx_sum:0x%x, seqerr_cnt: 0x%x\r\n", chksum->rx_cnt, chksum->rx_sum, chksum->unexpect_seq_counter);
    }
}
static void vTimerCallback( TimerHandle_t pxTimer )
{
    static uint32_t taskCntr = 0;

    long lTimerId;
    configASSERT( pxTimer );

    lTimerId = ( long ) pvTimerGetTimerID( pxTimer );

    if (lTimerId != TIMER_ID) {
        EMBARC_PRINTF("FreeRTOS Hello World Example FAILED");
    } else {
        if (taskCntr & 1) {
            // RX print
            //EMBARC_PRINTF("RX: rx_cnt:%x, rx_cnt_shadow:0x%x \r\n", can_node_session_id301.rx_cnt, can_node_session_id301.rx_cnt_shadow);

            if(can_node_session_id100.rx_cnt && can_node_session_id100.rx_cnt != can_node_session_id100.rx_cnt_shadow) {
                can_node_session_id100.rx_cnt_shadow = can_node_session_id100.rx_cnt;
                EMBARC_PRINTF("100 rx_cnt:0x%x, rx_sum:0x%x, seqerr_cnt: 0x%x\r\n", can_node_session_id100.rx_cnt, can_node_session_id100.rx_sum, can_node_session_id100.unexpect_seq_counter);
            }
            if(can_node_session_id300.rx_cnt && can_node_session_id300.rx_cnt != can_node_session_id300.rx_cnt_shadow) {
                can_node_session_id300.rx_cnt_shadow = can_node_session_id300.rx_cnt;
                EMBARC_PRINTF("300 rx_cnt:0x%x, rx_sum:0x%x, seqerr_cnt: 0x%x, %X %X\r\n", can_node_session_id300.rx_cnt, can_node_session_id300.rx_sum, can_node_session_id300.unexpect_seq_counter, raw_readl(0xa1100054), raw_readl(0xa1100058));
            }
            if(can_node_session_id301.rx_cnt && can_node_session_id301.rx_cnt != can_node_session_id301.rx_cnt_shadow){
                can_node_session_id301.rx_cnt_shadow = can_node_session_id301.rx_cnt;
                EMBARC_PRINTF("301 rx_cnt:0x%x, rx_sum:0x%x, seqerr_cnt: 0x%x\r\n", can_node_session_id301.rx_cnt, can_node_session_id301.rx_sum, can_node_session_id301.unexpect_seq_counter);
            }
            if(can_node_session_id310.rx_cnt && can_node_session_id310.rx_cnt != can_node_session_id310.rx_cnt_shadow){
                can_node_session_id310.rx_cnt_shadow = can_node_session_id310.rx_cnt;
                EMBARC_PRINTF("310 rx_cnt:0x%x, rx_sum:0x%x, seqerr_cnt: 0x%x\r\n", can_node_session_id310.rx_cnt, can_node_session_id310.rx_sum, can_node_session_id310.unexpect_seq_counter);
            }
            if(can_node_session_id311.rx_cnt && can_node_session_id311.rx_cnt != can_node_session_id311.rx_cnt_shadow){
                can_node_session_id311.rx_cnt_shadow = can_node_session_id311.rx_cnt;
                EMBARC_PRINTF("311 rx_cnt:0x%x, rx_sum:0x%x, seqerr_cnt: 0x%x\r\n", can_node_session_id311.rx_cnt, can_node_session_id311.rx_sum, can_node_session_id311.unexpect_seq_counter);
            }
            if(can_node_session_id320.rx_cnt && can_node_session_id320.rx_cnt != can_node_session_id320.rx_cnt_shadow){
                can_node_session_id320.rx_cnt_shadow = can_node_session_id320.rx_cnt;
                EMBARC_PRINTF("320 rx_cnt:0x%x, rx_sum:0x%x, seqerr_cnt: 0x%x\r\n", can_node_session_id320.rx_cnt, can_node_session_id320.rx_sum, can_node_session_id320.unexpect_seq_counter);
            }
            if(can_node_session_id321.rx_cnt && can_node_session_id321.rx_cnt != can_node_session_id321.rx_cnt_shadow){
                can_node_session_id321.rx_cnt_shadow = can_node_session_id321.rx_cnt;
                EMBARC_PRINTF("321 rx_cnt:0x%x, rx_sum:0x%x, seqerr_cnt: 0x%x\r\n", can_node_session_id321.rx_cnt, can_node_session_id321.rx_sum, can_node_session_id321.unexpect_seq_counter);
            }
            if(can_node_session_idother.rx_cnt && can_node_session_idother.rx_cnt != can_node_session_idother.rx_cnt_shadow){
                can_node_session_idother.rx_cnt_shadow = can_node_session_idother.rx_cnt;
                EMBARC_PRINTF("other rx_cnt:0x%x, rx_sum:0x%x, seqerr_cnt: 0x%x\r\n", can_node_session_idother.rx_cnt, can_node_session_idother.rx_sum, can_node_session_idother.unexpect_seq_counter);
            }
        } else {
            // TX print
            if(can_node_session_id300.tx_cnt && can_node_session_id300.tx_cnt != can_node_session_id300.tx_cnt_shadow){
                can_node_session_id300.tx_cnt_shadow = can_node_session_id300.tx_cnt;
                EMBARC_PRINTF("300 tx_cnt:0x%x, tx_sum:0x%x \r\n", can_node_session_id300.tx_cnt, can_node_session_id300.tx_sum);
            }
            if(can_node_session_id301.tx_cnt && can_node_session_id301.tx_cnt != can_node_session_id301.tx_cnt_shadow){
                can_node_session_id301.tx_cnt_shadow = can_node_session_id301.tx_cnt;
                EMBARC_PRINTF("301 tx_cnt:0x%x, tx_sum:0x%x \r\n", can_node_session_id301.tx_cnt, can_node_session_id301.tx_sum);
            }

            if(can_node_session_id310.tx_cnt && can_node_session_id310.tx_cnt != can_node_session_id310.tx_cnt_shadow){
                can_node_session_id310.tx_cnt_shadow = can_node_session_id310.tx_cnt;
                EMBARC_PRINTF("310 tx_cnt:0x%x, tx_sum:0x%x \r\n", can_node_session_id310.tx_cnt, can_node_session_id310.tx_sum);
            }
            if(can_node_session_id311.tx_cnt && can_node_session_id311.tx_cnt != can_node_session_id311.tx_cnt_shadow){
                can_node_session_id311.tx_cnt_shadow = can_node_session_id311.tx_cnt;
                EMBARC_PRINTF("311 tx_cnt:0x%x, tx_sum:0x%x \r\n", can_node_session_id311.tx_cnt, can_node_session_id311.tx_sum);
            }

            if(can_node_session_id320.tx_cnt && can_node_session_id320.tx_cnt != can_node_session_id320.tx_cnt_shadow){
                can_node_session_id320.tx_cnt_shadow = can_node_session_id320.tx_cnt;
                EMBARC_PRINTF("320 tx_cnt:0x%x, tx_sum:0x%x \r\n", can_node_session_id320.tx_cnt, can_node_session_id320.tx_sum);
            }
            if(can_node_session_id321.tx_cnt && can_node_session_id321.tx_cnt != can_node_session_id321.tx_cnt_shadow){
                can_node_session_id321.tx_cnt_shadow = can_node_session_id321.tx_cnt;
                EMBARC_PRINTF("321 tx_cnt:0x%x, tx_sum:0x%x \r\n", can_node_session_id321.tx_cnt, can_node_session_id321.tx_sum);
            }
            if(can_node_session_idother.tx_cnt && can_node_session_idother.tx_cnt != can_node_session_idother.tx_cnt_shadow){
                can_node_session_idother.tx_cnt_shadow = can_node_session_idother.tx_cnt;
                EMBARC_PRINTF("other tx_cnt:0x%x, tx_sum:0x%x \r\n", can_node_session_idother.tx_cnt, can_node_session_idother.tx_sum);
            }

            //xil_printf("TX Ret Cnt: lost 0x%x, id mismatch 0x%x, add pending: 0x%x, transmit_cnt: 0x%x, \r\n", read_pending_busy_cnt, tx_lost_retrans_cnt,write_add_pending_cnt,transmit_cnt);

            // xil_printf("***** ISR Cnt   : RX 0x%x, TX 0x%x, ERR 0x%x, 0x%x\r\n", can0_rx_isr_cnt, can0_tx_isr_cnt, can0_err_isr_cnt, can0_unused_isr_cnt);
            // xil_printf("***** HW ISR Cnt: RX 0x%x 0x%x, TX 0x%x, bus Err 0x%x, prot Err 0x%x, RX cli drop 0x%x\r\n", raw_readl(0x43c10000), raw_readl(0x43c10010),
            //                                                                               raw_readl(0x43c10004), bus_off_cnt,
            //                                                                               protocol_err_cnt, rx_cli_discard_cnt);
        }
    }

    xTimerStart( xTimer, 0 );

    if (taskCntr >= TEST_CAN_CASE_TIME) {
        EMBARC_PRINTF("test is done");
        vTaskDelete( can_test_handle );
    }

    taskCntr++;
}

void can_test_task(void *params)
{
    static uint32_t can_send_flag = 1;
    uint32_t result = 0;
    const TickType_t time = pdMS_TO_TICKS( TX_TASK_INTERVAL );

    chip_hw_mdelay(5000);

    for( ;; )
    {
        vTaskDelay( time );
        if (can_send_flag) {
            result = can_send_test();
            if (result) {
                // Done
                can_send_flag = 0;
                EMBARC_PRINTF("[can_test_task] Job done!\r\n");
            }
        }
    }
}

#define CAN_SCAN_SIGNAL_1_ID 0x300
#define CAN_SCAN_SIGNAL_2_ID 0x320

uint32_t loopback_data[20] = {0};

// callback - Check sequence number and checksum
int32_t can_scan_signal_1(uint8_t *data, uint32_t len)
{
    can_check_seqence_checksum(CAN_SCAN_SIGNAL_1_ID, data, len);
    can_msg_checksum(CAN_SCAN_SIGNAL_1_ID, data, len, 1);
    return 0;
}

// callback - Check sequence number and checksum
int32_t can_scan_signal_2(uint8_t *data, uint32_t len)
{
    can_check_seqence_checksum(CAN_SCAN_SIGNAL_2_ID, data, len);
    can_msg_checksum(CAN_SCAN_SIGNAL_2_ID, data, len, 1);
    return 0;
}

// callback - Loopback
int32_t can_scan_signal_3(uint8_t *data, uint32_t len)
{
    can_check_seqence_checksum(CAN_SCAN_SIGNAL_1_ID, data, len);
    can_msg_checksum(CAN_SCAN_SIGNAL_1_ID, data, len, 1);
    int n = 0;
    n = len / 4;
    if((len % 4) > 0){
        n++;
    }
    for (int i=0;i<n;i++) {
        loopback_data[i] = (data[i*4+3] << 24) | (data[i*4+2] << 16) | (data[i*4+1] << 8) | data[i*4];
    }

    can_send_data(0, CAN_SCAN_SIGNAL_1_ID + 1, loopback_data, len);
    return 0;
}

// callback - Loopback 
int32_t can_scan_signal_back(unsigned int back_id, uint8_t *data, uint32_t len)
{
    if (len%4 != 0) {
        EMBARC_PRINTF("[%s] : Incorrect input param len= %d! Quit!\r\n", __func__, len);
        return -1;
    }

    for (int i=0;i<len/4;i++) {
        loopback_data[i] = (data[i*4+3] << 24) | (data[i*4+2] << 16) | (data[i*4+1] << 8) | data[i*4];
    }

    can_send_data(0, back_id, loopback_data, len);
    return 0;
}

int32_t can_scan_signal_0x199(uint8_t *data, uint32_t len)
{
    //can_scan_signal_back(0x199+1, data, len);
    EMBARC_PRINTF("can_scan_199");
    return 0;
}
int32_t can_scan_signal_0x200(uint8_t *data, uint32_t len)
{
    //can_scan_signal_back(0x200+1, data, len);
    EMBARC_PRINTF("can_scan_200");

    return 0;
}
int32_t can_scan_signal_0x201(uint8_t *data, uint32_t len)
{
    //can_scan_signal_back(0x201+1, data, len);
    EMBARC_PRINTF("can_scan_201");

    return 0;
}
int32_t can_scan_signal_0x300(uint8_t *data, uint32_t len)
{
    //can_scan_signal_back(0x300+1, data, len);
    EMBARC_PRINTF("can_scan_300");

    return 0;
}
int32_t can_scan_signal_0x400(uint8_t *data, uint32_t len)
{
    //can_scan_signal_back(0x400+1, data, len);
    EMBARC_PRINTF("can_scan_400");
    return 0;
}
int32_t can_scan_signal_0x401(uint8_t *data, uint32_t len)
{
    //can_scan_signal_back(0x401+1, data, len);
    EMBARC_PRINTF("can_scan_401");

    return 0;
}
int32_t can_scan_signal_0x500(uint8_t *data, uint32_t len)
{
    //can_scan_signal_back(0x500+1, data, len);
    EMBARC_PRINTF("can_scan_500");
    return 0;
}
int32_t can_scan_signal_0x600(uint8_t *data, uint32_t len)
{
    can_scan_signal_back(0x600+1, data, len);
    return 0;
}
int32_t can_scan_signal_0x650(uint8_t *data, uint32_t len)
{
    can_scan_signal_back(0x650+1, data, len);
    return 0;
}
int32_t can_scan_signal_0x700(uint8_t *data, uint32_t len)
{
    can_scan_signal_back(0x700+1, data, len);
    return 0;
}

int32_t can_scan_signal_0x7ffffff(uint8_t *data, uint32_t len)
{
    EMBARC_PRINTF("can_scan_signal_0X7ffffff : %X %d\r\n", data[0], len);
    can_check_seqence_checksum(0X7ffffff, data, len);
    can_msg_checksum(0X7ffffff, data, len, 1);

    if (len%4 != 0) {
        EMBARC_PRINTF("[%s] : Incorrect input param len= %d! Quit!\r\n", __func__, len);
        return -1;
    }

    for (int i=0;i<len/4;i++) {
        loopback_data[i] = (data[i*4+3] << 24) | (data[i*4+2] << 16) | (data[i*4+1] << 8) | data[i*4];
    }

    can_send_data(0, 0X7ffffff+1, loopback_data, len);

	return 0;
}


int32_t can_scan_signal_0X1fffffff(uint8_t *data, uint32_t len)
{
    EMBARC_PRINTF("can_scan_signal_0X1fffffff : %X %d\r\n", data[0], len);
    can_check_seqence_checksum(0X1fffffff, data, len);
    can_msg_checksum(0X1fffffff, data, len, 1);

    if (len%4 != 0) {
        EMBARC_PRINTF("[%s] : Incorrect input param len= %d! Quit!\r\n", __func__, len);
        return -1;
    }

    for (int i=0;i<len/4;i++) {
        loopback_data[i] = (data[i*4+3] << 24) | (data[i*4+2] << 16) | (data[i*4+1] << 8) | data[i*4];
    }

    can_send_data(0, 0X1fffffff+1, loopback_data, len);

	return 0;
}
int32_t can_scan_signal_0X1(uint8_t *data, uint32_t len)
{
    EMBARC_PRINTF("can_scan_signal_0X1 : %X %d\r\n", data[0], len);
    can_check_seqence_checksum(0X1, data, len);
    can_msg_checksum(0X1, data, len, 1);

    if (len%4 != 0) {
        EMBARC_PRINTF("[%s] : Incorrect input param len= %d! Quit!\r\n", __func__, len);
        return -1;
    }

    for (int i=0;i<len/4;i++) {
        loopback_data[i] = (data[i*4+3] << 24) | (data[i*4+2] << 16) | (data[i*4+1] << 8) | data[i*4];
    }

    can_send_data(0, 0X2, loopback_data, len);

    return 0;
}

/*
 * CAN Parameter format:
 *     oper(2 charactors) + nomi_baud(8 charactors) + data_baud(8 charactors)
 *         oper: 00 - init, 01 - deinit
 */
void can_analyze_init (void* params, uint32_t len, uint32_t *oper, uint32_t *nomi_baud, uint32_t *data_baud)
{
    if (len < 2) {
        EMBARC_PRINTF("can param len should >= 2 (2byte oper)\r\n");
        return;
    }

    *oper = get_byte_from_param(&params);
    *nomi_baud = get_word_from_param(&params);
    *data_baud = get_word_from_param(&params);
    EMBARC_PRINTF("    Oper(is_deinit) : %d\r\n", *oper);
    EMBARC_PRINTF("    nomi_baud       : %d\r\n", *nomi_baud);
    EMBARC_PRINTF("    data_baud       : %d\r\n", *data_baud);
}

/*
 * CAN Parameter format:
 *     Type(2 charactors) + Payload(x charactors, depends on type)
 *         Payload:
 *             CAN_CONF_CAN_MODE   : 8 charactors. 00000000 - CAN, 00000001 - CAN FD.
 *             CAN_CONF_BAUDRATE   : 8 charactors. 4 charactors(nominal baudrate) + 4 charactors(data baudrate).
 *             CAN_CONF_STD_FILTER : 2 charactors(filter size) + 2 charactors(reject_no_match) + 20 charactors(filter_element) * filter size
 *                                   20 charactors(filter_element) : 2(type) + 2(config) + 8(filter id0) + 8(fileter id1)
 *             CAN_CONF_EXT_FILTER : Same with CAN_CONF_STD_FILTER.
 *             CAN_CONF_FRAME      : 4 charactors. 2(is_extend_frame) + 2(is_remote_frame)
 */
void can_analyze_config (void* params, uint32_t len, can_config_t *config)
{
    can_config_type_e type;
    if (len < 4) {
        EMBARC_PRINTF("can param len should >= 4 (1byte id, 1byte type)\r\n");
        return;
    }

    type = get_byte_from_param(&params);
    switch (type) {
        case CAN_CONF_WORK_MODE:
            config->can_mode = get_word_from_param(&params);
            break;
        case CAN_CONF_BAUDRATE:
            config->nomi_baud = get_word_from_param(&params);
            config->data_baud = get_word_from_param(&params);
            break;
        case CAN_CONF_STD_FILTER:
            config->std_id_filter.fiter_config.filter_size     = get_byte_from_param(&params);
            config->std_id_filter.fiter_config.reject_no_match = get_byte_from_param(&params);
            config->std_id_filter.filter_elem[0].filter_type = get_byte_from_param(&params);
            config->std_id_filter.filter_elem[0].filter_id0  = get_word_from_param(&params);
            config->std_id_filter.filter_elem[0].filter_id1  = get_word_from_param(&params);
            break;
        case CAN_CONF_EXT_FILTER:
            config->ext_id_filter.fiter_config.filter_size     = get_byte_from_param(&params);
            config->ext_id_filter.fiter_config.reject_no_match = get_byte_from_param(&params);
            config->ext_id_filter.filter_elem[0].filter_type = get_byte_from_param(&params);
            config->ext_id_filter.filter_elem[0].filter_id0  = get_word_from_param(&params);
            config->ext_id_filter.filter_elem[0].filter_id1  = get_word_from_param(&params);
            break;
        case CAN_CONF_FRAME:
            config->eframe_format = get_byte_from_param(&params);
            config->eframe_type = get_byte_from_param(&params);
            break;
#if 0
        case CAN_CONF_DATA_TX_MODE:
            config->data_tx_mode =
#endif
        default:
            EMBARC_PRINTF("[%s] unkonw type: %d\r\n", __func__, type);
            break;
    }
}

void can_config_print(can_config_t *config)
{
        EMBARC_PRINTF("    config->can_mode = %d\r\n", config->can_mode);

        EMBARC_PRINTF("    config->nomi_baud = %d\r\n", config->nomi_baud);
        EMBARC_PRINTF("    config->data_baud = %d\r\n", config->data_baud);

        EMBARC_PRINTF("    config->std_id_filter.fiter_config.filter_size     = %d\r\n", config->std_id_filter.fiter_config.filter_size);
        EMBARC_PRINTF("    config->std_id_filter.fiter_config.reject_no_match = %d\r\n", config->std_id_filter.fiter_config.reject_no_match);
        EMBARC_PRINTF("    config->std_id_filter.filter_elem[0].filter_type   = %d\r\n", config->std_id_filter.filter_elem[0].filter_type);
        EMBARC_PRINTF("    config->std_id_filter.filter_elem[0].filter_id0    = 0x%X\r\n", config->std_id_filter.filter_elem[0].filter_id0);
        EMBARC_PRINTF("    config->std_id_filter.filter_elem[0].filter_id1    = 0x%X\r\n", config->std_id_filter.filter_elem[0].filter_id1);

        EMBARC_PRINTF("    config->ext_id_filter.fiter_config.filter_size     = %d\r\n", config->ext_id_filter.fiter_config.filter_size);
        EMBARC_PRINTF("    config->ext_id_filter.fiter_config.reject_no_match = %d\r\n", config->ext_id_filter.fiter_config.reject_no_match);
        EMBARC_PRINTF("    config->ext_id_filter.filter_elem[0].filter_type   = %d\r\n", config->ext_id_filter.filter_elem[0].filter_type);
        EMBARC_PRINTF("    config->ext_id_filter.filter_elem[0].filter_id0    = 0x%X\r\n", config->ext_id_filter.filter_elem[0].filter_id0);
        EMBARC_PRINTF("    config->ext_id_filter.filter_elem[0].filter_id1    = 0x%X\r\n", config->ext_id_filter.filter_elem[0].filter_id1);

        EMBARC_PRINTF("    config->eframe_format = %d\r\n", config->eframe_format);
        EMBARC_PRINTF("    config->config->eframe_type = %d\r\n", config->eframe_type);
}

/*
 * CAN Parameter format:
 *     type(2 charactors) + enable(2 charactors)
 *         type   : 00 - rx interrupt, 01 - tx interrupt, 03 - error interrupt
 *         enable : 00 - disable, 01 - enable
 */
void can_analyze_int_enable (void* params, uint32_t len, uint32_t *type, uint32_t *enable)
{
    if (len < 4) {
        EMBARC_PRINTF("can param len should >= 4 (2byte type, 2byte enable)\r\n");
        return;
    }

    *type = get_byte_from_param(&params);
    *enable = get_byte_from_param(&params);
    EMBARC_PRINTF("    Int type : %d (0-rx, 1-tx, 3-err)\r\n", *type);
    EMBARC_PRINTF("    enable   : %d (0-disable, 1-enable)\r\n", *enable);
}

/*
 * CAN Parameter format:
 *     type(2 charactors) + enable(2 charactors)
 *         CB type : 00 - rx indication, 01 - tx indication, 03 - error indication
 *         CB func : 00 - loopback in rx ind
 */
void can_analyze_register_callback (void* params, uint32_t len, uint32_t *type, uint32_t *func)
{
    if (len < 4) {
        EMBARC_PRINTF("can param len should >= 4 (2byte type, 2byte func)\r\n");
        return;
    }

    *type = get_byte_from_param(&params);
    *func = get_byte_from_param(&params);
    EMBARC_PRINTF("    CB type : %d (0-rx ind, 1-tx ind, 3-err ind)\r\n", *type);
    EMBARC_PRINTF("    CB func : %d (0-loopback in rx ind)\r\n", *func);
}

static void tm08_can_case0_loopback_rx_ind_can0(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    uint32_t id = 0;
    can_config_t can_param;
    uint8_t case64_buff[64+1];
    uint32_t ext_id = 0;

    can_param.eframe_type = 0;
    if (ide == eEXTENDED_FRAME) {
        ext_id = msg_id;
        can_param.eframe_format = 1;
    } else {
        ext_id = msg_id >> 18;
        can_param.eframe_format = 0;
    }
    can_set_config(id, (void*)(&can_param));

    EMBARC_PRINTF("[%s] : id= %X, rawid= %X, len=%d, %X %X\r\n", __func__, msg_id, ext_id, len, data[0], data[1]);
    transfer_bytes_stream(data, case64_buff, len);
    EMBARC_PRINTF("[%s] : id= %X, len=%X, %X %X %X %X\r\n", __func__, msg_id, len, case64_buff[0], case64_buff[1], case64_buff[2], case64_buff[3]);

    can_send_data(id, ext_id+1, data, len);
}
static void tm08_can_case0_loopback_rx_ind_can1(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    uint32_t id = 1;
    can_config_t can_param;
    uint8_t case64_buff[64+1];
    uint32_t ext_id = 0;

    can_param.eframe_type = 0;
    if (ide == eEXTENDED_FRAME) {
        ext_id = msg_id;
        can_param.eframe_format = 1;
    } else {
        ext_id = msg_id >> 18;
        can_param.eframe_format = 0;
    }
    can_set_config(id, (void*)(&can_param));

    EMBARC_PRINTF("[%s] : id= %X, rawid= %X, len=%d, %X %X\r\n", __func__, msg_id, ext_id, len, data[0], data[1]);
    transfer_bytes_stream(data, case64_buff, len);
    EMBARC_PRINTF("[%s] : id= %X, len=%X, %X %X %X %X\r\n", __func__, msg_id, len, case64_buff[0], case64_buff[1], case64_buff[2], case64_buff[3]);

    can_send_data(id, ext_id+1, data, len);
}

/*
 * CASE 0
 * [CAN/CANFD]  API Calling
 *
 *     api(2char) + id(2char) + param(depends on api)
 *
 *         params:
 *             api 0 (init/deinit) : oper(2char) + nomi_baud(8char) + data_baud(8char)
 *             api 1 (set config)  : Type(2 charactors) + Payload(x charactors, depends on type)
 *             api 2 (int enable)  : int type(2char) + enable(2char)
 *             api 3 (callback reg): cb type(2char) + cb func(2char)
 * tm start 8 0 0000nnnnnnnndddddddd
*/
int32_t tm08_can_case0_loopback(void *self, void *params, uint32_t len)
{
    uint32_t can_id;
    uint32_t api_id;
    uint32_t oper;
    uint32_t nomi_baud;
    uint32_t data_baud;
    can_config_t can_param;
    uint32_t type;
    uint32_t enable;
    uint32_t func;

    api_id = get_byte_from_param(&params);
    EMBARC_PRINTF("api_id = %d\r\n", api_id);

    can_id = get_byte_from_param(&params);
    EMBARC_PRINTF("can_id = %d\r\n", can_id);

    switch (api_id) {
        case 0:
            EMBARC_PRINTF("CAN: can_init()!\r\n");
            can_analyze_init(params, len, &oper, &nomi_baud, &data_baud);
            if (oper == 0) {
                can_init(can_id, nomi_baud, data_baud);
            } else {
                can_deinit(can_id);
            }
            break;
        case 1:
            EMBARC_PRINTF("CAN: can_set_config()!\r\n");
            can_analyze_config (params, len, &can_param);
            can_config_print (&can_param);
            can_set_config (can_id, (void*)(&can_param));
            break;
        case 2:
            EMBARC_PRINTF("CAN: can_interrupt_enable ()!\r\n");
            can_analyze_int_enable(params, len, &type, &enable);
            can_interrupt_enable(can_id, type, enable);
            break;
        case 3:
            EMBARC_PRINTF("CAN: can_xxxx_register()!\r\n");
            can_analyze_register_callback(params, len, &type, &func);
            if (type == 0) {
                if (func == 0) {
                    if (can_id == 0) {
                        can_indication_register(can_id, tm08_can_case0_loopback_rx_ind_can0);
                    } else {
                        can_indication_register(can_id, tm08_can_case0_loopback_rx_ind_can1);
                    }
                } else {

                }
            } else if (type == 1) {

            } else {

            }
            break;
        default:
            EMBARC_PRINTF("CAN: unknow!\r\n");
            break;
    }

    return 0;
}

/*
 * CASE 1 , CASE10
 *
 * [CAN/CANFD]  Loopback
 *     Receive ID300, send back to ID301
*/
int32_t tm08_can_case1_loopback(void *self, void *params, uint32_t len)
{
    can_init(0, 500000, 0);
    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(0, 3, 1);
    can_cli_register(0, CAN_SCAN_SIGNAL_1_ID, can_scan_signal_3);
    //can_cli_register(0, 0X7ffffff, can_scan_signal_0x7ffffff);
    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}


int32_t tm08_can_case14_mixframe(void *self, void *params, uint32_t len)
{
    can_cli_register(0, 0x1, can_scan_signal_0X1);
    can_cli_register(0, CAN_SCAN_SIGNAL_1_ID, can_scan_signal_3);
    can_cli_register(0, 0x1fffffff, can_scan_signal_0X1fffffff);

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);

    return 0;
}


/*
 * CASE 4 ,
 *
 * [CAN/CANFD]  Fileter
 *     Receive ID x, send back to ID x+1
 *     x - 0x199, 0x200, 0x201, 0x300, 0x400, 0x401, 0x500
*/
int32_t tm08_can_case4_filter(void *self, void *params, uint32_t len)
{
    can_cli_register(0, 0x199, can_scan_signal_0x199);
    can_cli_register(0, 0x200, can_scan_signal_0x200);
    can_cli_register(0, 0x201, can_scan_signal_0x201);
    can_cli_register(0, 0x300, can_scan_signal_0x300);
    can_cli_register(0, 0x400, can_scan_signal_0x400);
    can_cli_register(0, 0x401, can_scan_signal_0x401);
    can_cli_register(0, 0x500, can_scan_signal_0x500);

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

int32_t tm08_can_case8_case17_long_time_high_load(void *self, void *params, uint32_t len)
{
    can_cli_register(0, CAN_SCAN_SIGNAL_1_ID, can_scan_signal_1);
    can_cli_register(0, CAN_SCAN_SIGNAL_2_ID, can_scan_signal_2);

    if (xTaskCreate(can_test_task, "can_test", 1048, (void *)0, configMAX_PRIORITIES-1, &can_test_handle) != pdPASS) {
        EMBARC_PRINTF("create can_result error\r\n");
        return -1;
    }

    const TickType_t x1second = pdMS_TO_TICKS( PRINT_INTERVAL );
    xTimer = xTimerCreate( (const char *) "Timer",
                            x1second,
                            pdFALSE,
                            (void *) TIMER_ID,
                            vTimerCallback);
    configASSERT( xTimer );
    xTimerStart( xTimer, 0 );

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

static void can_0_rx_ind(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    // uint8_t buff_can0_rx_ind[64];
    EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X\r\n", __func__, msg_id, len, data[0], data[1]);
    // transfer_bytes_stream(data, buff_can0_rx_ind, len);
    // EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X\r\n", __func__, msg_id, len, data[0], data[1]);
    // can_check_seqence_checksum(CAN_SCAN_SIGNAL_2_ID, buff_can0_rx_ind, len);
    // can_msg_checksum(CAN_SCAN_SIGNAL_2_ID, buff_can0_rx_ind, len*4, 1);

    // can_send_data(0, CAN_SCAN_SIGNAL_2_ID, data, len);
}

static void can_1_rx_ind(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    // uint8_t buff_can1_rx_ind[64];
    EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X \r\n", __func__, msg_id, len, data[0], data[1]);
    // transfer_bytes_stream(data, buff_can1_rx_ind, len);
    // can_check_seqence_checksum(CAN_SCAN_SIGNAL_1_ID, buff_can1_rx_ind, len);
    // can_msg_checksum(CAN_SCAN_SIGNAL_1_ID, buff_can1_rx_ind, len*4, 1);

    // can_send_data(1, CAN_SCAN_SIGNAL_1_ID, data, len);
}

int32_t tm08_can_case9_case18_2can(void *self, void *params, uint32_t len)
{
    can_init(0, CAN_BAUDRATE_1MBPS, 0);
    can_init(1, CAN_BAUDRATE_1MBPS, 0);

    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(1, 0, 1);

    can_interrupt_enable(0, 3, 1);
    can_interrupt_enable(1, 3, 1);

    can_indication_register(0, can_0_rx_ind);
    can_indication_register(1, can_1_rx_ind);

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}


static void can_0_case22(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{

    can_send_data(0, 0x301, data, len);
}

static void canfd_1_case22(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{

    can_send_data(1, 0x301, data, len);
}


/*case22*/
int32_t tm08_can_case22_can0_canfd1_loopback(void *self, void *params, uint32_t len)
{
    can_init(0, CAN_BAUDRATE_500KBPS, 0);
    can_init(1, CAN_BAUDRATE_500KBPS, CAN_BAUDRATE_1MBPS);


    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(1, 0, 1);

    can_interrupt_enable(0, 3, 1);
    can_interrupt_enable(1, 3, 1);

    can_indication_register(0, can_0_case22);
    can_indication_register(1, canfd_1_case22);

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}


static void can_0_rx_ind_loopback(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    //EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X\r\n", __func__, msg_id, len, data[0], data[1]);
    can_send_data(0, 0x260, data, len);
}

static void can_1_rx_ind_loopback(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    //EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X \r\n", __func__, msg_id, len, data[0], data[1]);
    can_send_data(1, 0x201, data, len);
}

int32_t tm09_can_case9_2can_loopback(void *self, void *params, uint32_t len)
{
    can_init(0, CAN_BAUDRATE_500KBPS, 0);
    can_init(1, CAN_BAUDRATE_500KBPS, 0);

    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(1, 0, 1);

    can_interrupt_enable(0, 3, 1);
    can_interrupt_enable(1, 3, 1);

    can_indication_register(0, can_0_rx_ind_loopback);
    can_indication_register(1, can_1_rx_ind_loopback);

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

static void canfd_0_rx_ind_loopback(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X\r\n", __func__, msg_id, len, data[0], data[1]);
    can_send_data(0, 0x660, data, len);
}

static void canfd_1_rx_ind_loopback(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X \r\n", __func__, msg_id, len, data[0], data[1]);
    can_send_data(1, 0x601, data, len);
}
int32_t tm10_can_case20_2canfd_loopback(void *self, void *params, uint32_t len)
{
    can_init(0, CAN_BAUDRATE_500KBPS, CAN_BAUDRATE_1MBPS);
    can_init(1, CAN_BAUDRATE_500KBPS, CAN_BAUDRATE_1MBPS);

    // can_transceiver_init();

    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(1, 0, 1);

    can_interrupt_enable(0, 3, 1);
    can_interrupt_enable(1, 3, 1);

    can_indication_register(0, canfd_0_rx_ind_loopback);
    can_indication_register(1, canfd_1_rx_ind_loopback);

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

#if 0
static uint32_t can0_dma_rx_buf[4];
static uint32_t can1_dma_rx_buf[4];
static uint32_t can0_dma_tx_buf[4] = {0x0c040000, 0x00080000, 0x33221100, 0x77665544};
static uint32_t can1_dma_tx_buf[4] = {0x0c040000, 0x00080000, 0x33221100, 0x77665544};
static uint32_t can0_rx_dma_done_count = 0;
static uint32_t can0_tx_dma_done_count = 0;
static uint32_t can1_rx_dma_done_count = 0;
static uint32_t can1_tx_dma_done_count = 0;


static void can0_rx_dma_callback(void *params)
{
        can0_rx_dma_done_count++;
        EMBARC_PRINTF("can0_dma rx done %d!\r\n", can0_rx_dma_done_count);

        EMBARC_PRINTF("%s: 0x%x, 0x%x, 0x%x, 0x%x\r\n", __func__, can0_dma_rx_buf[0], can0_dma_rx_buf[1], can0_dma_rx_buf[2], can0_dma_rx_buf[3]);

        can_dma_read(can_get_dev(0), can0_dma_rx_buf, 4, can0_rx_dma_callback);

        int32_t result = dma_release_channel((uint32_t)params);
        if (E_OK != result) {
                EMBARC_PRINTF("DMA channel release failed.\r\n");
        }
}


static void can0_tx_dma_callback(void *params)
{
        can0_tx_dma_done_count++;
        EMBARC_PRINTF("can0_dma tx done %d!\r\n", can0_tx_dma_done_count);

        int32_t result = dma_release_channel((uint32_t)params);
        if (E_OK != result) {
                EMBARC_PRINTF("DMA channel release failed.\r\n");
        }
}


static void can1_rx_dma_callback(void *params)
{
        can1_rx_dma_done_count++;
        EMBARC_PRINTF("can1_dma rx done %d!\r\n", can1_rx_dma_done_count);

        EMBARC_PRINTF("%s: 0x%x, 0x%x, 0x%x, 0x%x\r\n", __func__, can1_dma_rx_buf[0], can1_dma_rx_buf[1], can1_dma_rx_buf[2], can1_dma_rx_buf[3]);

        can_dma_read(can_get_dev(1), can1_dma_rx_buf, 4, can1_rx_dma_callback);

        int32_t result = dma_release_channel((uint32_t)params);
        if (E_OK != result) {
                EMBARC_PRINTF("DMA channel release failed.\r\n");
        }
}


static void can1_tx_dma_callback(void *params)
{
        can1_tx_dma_done_count++;
        EMBARC_PRINTF("can1_dma tx done %d!\r\n", can1_tx_dma_done_count);

        int32_t result = dma_release_channel((uint32_t)params);
        if (E_OK != result) {
                EMBARC_PRINTF("DMA channel release failed.\r\n");
        }
}


void can_dma_rx_handshake(uint32_t can_bus_id)
{
        if (can_bus_id == 0) {
               /* frame size = 4 */
               can_dma_read(can_get_dev(0), can0_dma_rx_buf, 4, can0_rx_dma_callback);
        } else {
               can_dma_read(can_get_dev(1), can1_dma_rx_buf, 4, can1_rx_dma_callback);
        }
}


void can_dma_tx_handshake(uint32_t can_bus_id)
{
        uint32_t loop_count = 10000;

        for (uint i = 0; i < loop_count; i++) {
                if (can_bus_id == 0) {
                        /* frame size = 4 */
                        can_dma_write(can_get_dev(0), can0_dma_tx_buf, 4, can0_tx_dma_callback);
                        chip_hw_mdelay(1);
                } else {
                        /* frame size = 4 */
                        can_dma_write(can_get_dev(1), can1_dma_tx_buf, 4, can1_tx_dma_callback);
                        chip_hw_mdelay(1);
                }
        }
}
#endif

/* verify CAN IO MUX function */
void can0_iomux_sel(uint32_t cfg)
{
    /*switch (cfg) {
		case 1:
			iomux_cfg_16(0);
			iomux_cfg_17(0);
			break;
		case 2:
			iomux_cfg_16(0);
			iomux_cfg_21(3);
			break;
		case 3:
			iomux_cfg_16(0);
			iomux_cfg_23(1);
			break;
		case 4:
			iomux_cfg_20(3);
			iomux_cfg_17(0);
			break;
		case 5:
			iomux_cfg_20(3);
			iomux_cfg_21(3);
			break;
		case 6:
			iomux_cfg_20(3);
			iomux_cfg_23(1);
			break;
		case 7:
			iomux_cfg_22(1);
			iomux_cfg_17(0);
			break;
		case 8:
			iomux_cfg_22(1);
			iomux_cfg_21(3);
			break;
		case 9:
			iomux_cfg_22(1);
			iomux_cfg_23(1);
			break;
	}*/
}


void can1_iomux_sel(uint32_t cfg)
{
	/*switch (cfg) {
		case 1:
			iomux_cfg_13(3);
			iomux_cfg_14(3);
			break;
		case 2:
			iomux_cfg_13(3);
			iomux_cfg_17(1);
			break;
		case 3:
			iomux_cfg_13(3);
			iomux_cfg_19(0);
			break;
		case 4:
			iomux_cfg_13(3);
			iomux_cfg_23(2);
			break;
		case 5:
			iomux_cfg_16(1);
			iomux_cfg_14(3);
			break;
		case 6:
			iomux_cfg_16(1);
			iomux_cfg_17(1);
			break;
		case 7:
			iomux_cfg_16(1);
			iomux_cfg_19(0);
			break;
		case 8:
			iomux_cfg_16(1);
			iomux_cfg_23(2);
			break;
		case 9:
			iomux_cfg_18(2);
			iomux_cfg_14(3);
			break;
		case 10:
			iomux_cfg_18(2);
			iomux_cfg_17(1);
			break;
		case 11:
			iomux_cfg_18(2);
			iomux_cfg_19(0);
			break;
		case 12:
			iomux_cfg_18(2);
			iomux_cfg_23(2);
			break;
		case 13:
			iomux_cfg_22(2);
			iomux_cfg_14(3);
			break;
		case 14:
			iomux_cfg_22(2);
			iomux_cfg_17(1);
			break;
		case 15:
			iomux_cfg_22(2);
			iomux_cfg_19(0);
			break;
		case 16:
			iomux_cfg_22(2);
			iomux_cfg_23(2);
			break;
		case 17:
			iomux_cfg_14(3);
			iomux_cfg_15(3);
			break;
		default:
			break;
	}*/
}

/*case3,can1 pin mux*/
void tm08_can_case3_pinmux(void *self, void *params, uint32_t len)
{
	// clock_enable(IO_MUX_CLOCK,1);// enable io mux clock.
	// iomux_cfg_18(4);//can1_tx->gpio
	// iomux_cfg_19(4);//can1_rx->gpio
	can1_iomux_sel(17);//spi_s0->can1
	can_cli_register(0, 0x300, can_scan_signal_0x300);
}

void tm08_can_case24_timestamp(uint32_t id, uint32_t prescale, uint32_t mode)
{
	//CAN_LOCK(can_sema[id]);

	can_t *dev_can_timestamp[2] = { NULL, NULL};
	dev_can_timestamp[id] = can_get_dev(id);
	can_ops_t *can_ops = NULL;
	can_ops = (can_ops_t *)dev_can_timestamp[id]->ops;
	can_ops->timestamp_config(dev_can_timestamp[id], prescale, mode);

	//CAN_UNLOCK(can_sema[id]);
}

#if 0

static void can_tx_dma_loopback_callback(void *params)
{
        can0_tx_dma_done_count++;
        EMBARC_PRINTF("can_tx_dma_loopback_callback done %d!\r\n", can0_tx_dma_done_count);

        int32_t result = dma_release_channel((uint32_t)params);
        if (E_OK != result) {
                EMBARC_PRINTF("DMA channel release failed.\r\n");
        }
}


static void can0_rx_dma_loopback_callback(void *params)
{
        can0_rx_dma_done_count++;
        EMBARC_PRINTF("can0_rx_dma_loopback_callback done %d!\r\n", can0_rx_dma_done_count);

        EMBARC_PRINTF("%s: 0x%x, 0x%x, 0x%x, 0x%x\r\n", __func__, can0_dma_rx_buf[0], can0_dma_rx_buf[1], can0_dma_rx_buf[2], can0_dma_rx_buf[3]);

        can_dma_read(can_get_dev(0), can0_dma_rx_buf, 4, can0_rx_dma_loopback_callback);

        can0_dma_rx_buf[0] = 0x0c040000;//id 301

        /* send id 0x301 */
        can_dma_write(can_get_dev(0), can0_dma_rx_buf, 4, can_tx_dma_loopback_callback);
        int32_t result = dma_release_channel((uint32_t)params);
        if (E_OK != result) {
                EMBARC_PRINTF("DMA channel release failed.\r\n");
        }
}


static void can1_rx_dma_loopback_callback(void *params)
{
        can1_rx_dma_done_count++;
        EMBARC_PRINTF("can1_rx_dma_loopback_callback done %d!\r\n", can1_rx_dma_done_count);

        EMBARC_PRINTF("%s: 0x%x, 0x%x, 0x%x, 0x%x\r\n", __func__, can1_dma_rx_buf[0], can1_dma_rx_buf[1], can1_dma_rx_buf[2], can1_dma_rx_buf[3]);

        can_dma_read(can_get_dev(1), can1_dma_rx_buf, 4, can1_rx_dma_loopback_callback);

        can1_dma_rx_buf[0] = 0x0c040000;//id 301

        /* send id 0x301 */
        can_dma_write(can_get_dev(1), can1_dma_rx_buf, 4, can_tx_dma_loopback_callback);
        int32_t result = dma_release_channel((uint32_t)params);
        if (E_OK != result) {
                EMBARC_PRINTF("DMA channel release failed.\r\n");
        }
}


/* case4,can_dma loop back */
void can_dma_loop_back(uint32_t can_bus_id)
{
        can_init(can_bus_id, 500000, 0);

        if (can_bus_id == 0) {
               /* frame size = 4, waiting for id 0x300, then send id 0x301 */
               can_dma_read(can_get_dev(0), can0_dma_rx_buf, 4, can0_rx_dma_loopback_callback);
        } else {
               can_dma_read(can_get_dev(1), can1_dma_rx_buf, 4, can1_rx_dma_loopback_callback);
        }
}
#endif

/* specific filter buffer 2: filter in buffer 2, msg in buffer 2 */

/*
 * CASE 50
 *
 * [CAN/CANFD]  Loopback
*/
int32_t tm08_can_case50_init_deinit(void *self, void *params, uint32_t len)
{
    can_config_t can_param;

    EMBARC_PRINTF("[%s]: ======== CAN_CONF_CAN_MODE. ========\r\n", __func__);
    can_init(0, 500000, 0);
    can_param.can_mode = 0;
    can_get_config(0, (void*)(&can_param));
    can_set_config(0, (void*)(&can_param));
    can_deinit(0);

    EMBARC_PRINTF("[%s]: ======== CAN_CONF_BAUDRATE. ========\r\n", __func__);
    can_init(0, 500000, 0);
    can_get_config(0, (void*)(&can_param));
    can_param.nomi_baud = 1000000;
    can_param.data_baud = 2000000;
    can_set_config(0, (void*)(&can_param));
    can_deinit(0);

    EMBARC_PRINTF("[%s]: ======== CAN_CONF_STD_FILTER. ========\r\n", __func__);
    can_init(0, 500000, 0);
    can_get_config(0, (void*)(&can_param));
    can_param.std_id_filter.fiter_config.filter_size     = 1;
    can_param.std_id_filter.fiter_config.reject_no_match = 0;
    can_param.std_id_filter.filter_elem[0].filter_type = 2;
    can_param.std_id_filter.filter_elem[0].filter_id0  = 0x221;
    can_param.std_id_filter.filter_elem[0].filter_id1  = 0x331;
    can_set_config(0, (void*)(&can_param));
    can_deinit(0);

    EMBARC_PRINTF("[%s]: ======== CAN_CONF_EXT_FILTER. ========\r\n", __func__);
    can_init(0, 500000, 0);
    can_get_config(0, (void*)(&can_param));
    can_param.ext_id_filter.fiter_config.filter_size     = 1;
    can_param.ext_id_filter.fiter_config.reject_no_match = 1;
    can_param.ext_id_filter.filter_elem[0].filter_type = 3;
    can_param.ext_id_filter.filter_elem[0].filter_id0  = 0x441;
    can_param.ext_id_filter.filter_elem[0].filter_id1  = 0x551;
    can_set_config(0, (void*)(&can_param));
    can_deinit(0);

    EMBARC_PRINTF("[%s]: ======== CAN_CONF_FRAME. ========\r\n", __func__);
    can_init(0, 500000, 0);
    can_get_config(0, (void*)(&can_param));
    can_param.eframe_format     = 1;
    can_param.eframe_type       = 1;
    can_set_config(0, (void*)(&can_param));
    can_deinit(0);

    EMBARC_PRINTF("[%s]: ======== Done. ========\r\n", __func__);
    return 0;
}


static void tm08_can_case51_rx_ind(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    uint8_t case51_buff[64];
    EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X\r\n", __func__, msg_id, len, data[0], data[1]);
    transfer_bytes_stream(data, case51_buff, len);
    EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X %X %X\r\n", __func__, msg_id, len, case51_buff[0], case51_buff[1], case51_buff[2], case51_buff[3]);
    can_check_seqence_checksum(CAN_SCAN_SIGNAL_1_ID, case51_buff, len);
    can_msg_checksum(CAN_SCAN_SIGNAL_1_ID, case51_buff, len, 1);

    can_send_data_isr(0, CAN_SCAN_SIGNAL_1_ID+1, data, len);
}
int32_t tm08_can_case51_hal_loopback(void *self, void *params, uint32_t len)
{
    can_config_t can_param;

    can_init(0, CAN_BAUDRATE_500KBPS, 0);
    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(0, 3, 1);
    can_indication_register(0, tm08_can_case51_rx_ind);

    can_get_config(0, (void*)(&can_param));
    can_param.data_tx_mode = DEV_XFER_INTERRUPT;
    can_param.std_id_filter.fiter_config.filter_size     = 1;
    can_param.std_id_filter.fiter_config.reject_no_match = 1;
    can_param.std_id_filter.filter_elem[0].filter_type = eDUAL_ID;
    can_param.std_id_filter.filter_elem[0].filter_id0 = CAN_SCAN_SIGNAL_1_ID;
    can_param.std_id_filter.filter_elem[0].filter_id1 = CAN_SCAN_SIGNAL_1_ID + 1;
    can_set_config(0, (void*)(&can_param));

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

int32_t tm08_can_case52_hal_loopback_1M(void *self, void *params, uint32_t len)
{
    can_config_t can_param;

    can_init(0, CAN_BAUDRATE_500KBPS, 0);
    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(0, 3, 1);
    can_indication_register(0, tm08_can_case51_rx_ind);

    can_get_config(0, (void*)(&can_param));
    can_param.nomi_baud = 1000000;
    can_param.data_baud = 0;
    can_param.data_tx_mode = DEV_XFER_INTERRUPT;
    can_param.std_id_filter.fiter_config.filter_size     = 1;
    can_param.std_id_filter.fiter_config.reject_no_match = 1;
    can_param.std_id_filter.filter_elem[0].filter_type = eDUAL_ID;
    can_param.std_id_filter.filter_elem[0].filter_id0 = CAN_SCAN_SIGNAL_1_ID;
    can_param.std_id_filter.filter_elem[0].filter_id1 = CAN_SCAN_SIGNAL_1_ID + 1;
    can_set_config(0, (void*)(&can_param));

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

#define CAN_SCAN_SIGNAL_1_EXT_ID 0x7FFFFF0
static void tm08_can_case53_rx_ind(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    uint8_t case51_buff[64];
    uint32_t ext_id = 0;

    if (ide == eEXTENDED_FRAME) {
        ext_id = msg_id;
    } else {
        ext_id = ext_id >> 18;
    }

    EMBARC_PRINTF("[%s] : id= %X, rawid= %d, len=%d, %X %X\r\n", __func__, msg_id, ext_id, len, data[0], data[1]);
    transfer_bytes_stream(data, case51_buff, len);
    EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X %X %X\r\n", __func__, msg_id, len, case51_buff[0], case51_buff[1], case51_buff[2], case51_buff[3]);
    can_check_seqence_checksum(CAN_SCAN_SIGNAL_1_ID, case51_buff, len);
    can_msg_checksum(CAN_SCAN_SIGNAL_1_ID, case51_buff, len, 1);

    can_send_data_isr(0, ext_id+1, data, len);
}
int32_t tm08_can_case53_hal_loopback_extend_id(void *self, void *params, uint32_t len)
{
    can_config_t can_param;

    can_init(0, CAN_BAUDRATE_500KBPS, 0);
    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(0, 3, 1);
    can_indication_register(0, tm08_can_case53_rx_ind);

    can_get_config(0, (void*)(&can_param));
    can_param.eframe_format = 1;
    can_param.eframe_type = 0;
    can_param.data_tx_mode = DEV_XFER_INTERRUPT;
    can_param.ext_id_filter.fiter_config.filter_size     = 1;
    can_param.ext_id_filter.fiter_config.reject_no_match = 1;
    can_param.ext_id_filter.filter_elem[0].filter_type = eDUAL_ID;
    can_param.ext_id_filter.filter_elem[0].filter_id0 = CAN_SCAN_SIGNAL_1_ID;
    can_param.ext_id_filter.filter_elem[0].filter_id1 = CAN_SCAN_SIGNAL_1_ID + 1;
    can_set_config(0, (void*)(&can_param));

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

/*
 * CASE 60
 *
 * [CAN/CANFD]  Loopback
*/
int32_t tm08_can_case60_hal_FD_init_deinit(void *self, void *params, uint32_t len)
{
    can_config_t can_param;

    EMBARC_PRINTF("[%s]: ======== CAN_CONF_CAN_MODE. ========\r\n", __func__);
    can_init(1, 500000, 1000000);
    can_get_config(1, (void*)(&can_param));
    can_param.can_mode= 1;
    can_set_config(1, (void*)(&can_param));
    can_deinit(1);

    EMBARC_PRINTF("[%s]: ======== CAN_CONF_BAUDRATE. ========\r\n", __func__);
    can_init(1, 500000, 1000000);
    can_get_config(1, (void*)(&can_param));
    can_param.nomi_baud = 1000000;
    can_param.data_baud = 2000000;
    can_set_config(1, (void*)(&can_param));
    can_deinit(1);

    EMBARC_PRINTF("[%s]: ======== CAN_CONF_STD_FILTER. ========\r\n", __func__);
    can_init(1, 500000, 1000000);
    can_get_config(1, (void*)(&can_param));
    can_param.std_id_filter.fiter_config.filter_size     = 1;
    can_param.std_id_filter.fiter_config.reject_no_match = 0;
    can_param.std_id_filter.filter_elem[0].filter_type = 2;
    can_param.std_id_filter.filter_elem[0].filter_id0  = 0x221;
    can_param.std_id_filter.filter_elem[0].filter_id1  = 0x331;
    can_set_config(1, (void*)(&can_param));
    can_deinit(1);

    EMBARC_PRINTF("[%s]: ======== CAN_CONF_EXT_FILTER. ========\r\n", __func__);
    can_init(1, 500000, 1000000);
    can_get_config(1, (void*)(&can_param));
    can_param.ext_id_filter.fiter_config.filter_size     = 1;
    can_param.ext_id_filter.fiter_config.reject_no_match = 1;
    can_param.ext_id_filter.filter_elem[0].filter_type = 3;
    can_param.ext_id_filter.filter_elem[0].filter_id0  = 0x441;
    can_param.ext_id_filter.filter_elem[0].filter_id1  = 0x551;
    can_set_config(1, (void*)(&can_param));
    can_deinit(1);

    EMBARC_PRINTF("[%s]: ======== CAN_CONF_FRAME. ========\r\n", __func__);
    can_init(1, 500000, 1000000);
    can_get_config(1, (void*)(&can_param));
    can_param.eframe_format     = 1;
    can_param.eframe_type     = 1;
    can_set_config(1, (void*)(&can_param));
    can_deinit(1);

    EMBARC_PRINTF("[%s]: ======== Done. ========\r\n", __func__);
    return 0;
}


static void tm08_can_case61_rx_ind(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    uint8_t case51_buff[64+1];
    EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X\r\n", __func__, msg_id, len, data[0], data[1]);
    transfer_bytes_stream(data, case51_buff, len);
    EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X %X %X\r\n", __func__, msg_id, len, case51_buff[0], case51_buff[1], case51_buff[2], case51_buff[3]);
    can_check_seqence_checksum(CAN_SCAN_SIGNAL_1_ID, case51_buff, len);
    can_msg_checksum(CAN_SCAN_SIGNAL_1_ID, case51_buff, len, 1);

    can_send_data_isr(1, CAN_SCAN_SIGNAL_1_ID+1, data, len);
}
int32_t tm08_can_case61_hal_loopback_FD_500K_1M(void *self, void *params, uint32_t len)
{
    can_reset_checksum_all();
    can_config_t can_param;

    can_init(1, CAN_BAUDRATE_500KBPS, CAN_BAUDRATE_1MBPS);
    can_interrupt_enable(1, 0, 1);
    can_interrupt_enable(1, 3, 1);
    can_indication_register(1, tm08_can_case61_rx_ind);

    can_get_config(1, (void*)(&can_param));
    can_param.data_tx_mode = DEV_XFER_INTERRUPT;
    can_set_config(1, (void*)(&can_param));

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

int32_t tm08_can_case62_hal_loopback_FD_1M_2M(void *self, void *params, uint32_t len)
{
    can_config_t can_param;

    can_reset_checksum_all();

    can_init(1, CAN_BAUDRATE_500KBPS, CAN_BAUDRATE_1MBPS);
    can_interrupt_enable(1, 0, 1);
    can_interrupt_enable(1, 3, 1);
    can_indication_register(1, tm08_can_case61_rx_ind);

    can_get_config(1, (void*)(&can_param));
    can_param.nomi_baud = 1000000;
    can_param.data_baud = 2000000;
    can_param.data_tx_mode = DEV_XFER_INTERRUPT;
    can_set_config(1, (void*)(&can_param));


    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

#define CAN_SCAN_SIGNAL_1_EXT_ID 0x7FFFFF0
static void tm08_can_case63_rx_ind(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    uint8_t case51_buff[64+1];
    uint32_t ext_id = 0;

    if (ide == eEXTENDED_FRAME) {
        ext_id = msg_id;
    } else {
        ext_id = ext_id >> 18;
    }

    EMBARC_PRINTF("[%s] : id= %X, rawid= %d, len=%d, %X %X\r\n", __func__, msg_id, ext_id, len, data[0], data[1]);
    transfer_bytes_stream(data, case51_buff, len);
    EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X %X %X\r\n", __func__, msg_id, len, case51_buff[0], case51_buff[1], case51_buff[2], case51_buff[3]);
    can_check_seqence_checksum(CAN_SCAN_SIGNAL_1_ID, case51_buff, len);
    can_msg_checksum(CAN_SCAN_SIGNAL_1_ID, case51_buff, len, 1);

    can_send_data_isr(1, ext_id+1, data, len);
}
int32_t tm08_can_case63_hal_loopback_FD_extend_id(void *self, void *params, uint32_t len)
{
    can_config_t can_param;

    can_reset_checksum_all();

    can_init(1, CAN_BAUDRATE_500KBPS, CAN_BAUDRATE_1MBPS);
    can_interrupt_enable(1, 0, 1);
    can_interrupt_enable(1, 3, 1);
    can_indication_register(1, tm08_can_case63_rx_ind);

    can_get_config(1, (void*)(&can_param));
    can_param.eframe_format = 1;
    can_param.eframe_type = 0;

    can_param.ext_id_filter.fiter_config.filter_size     = 2;
    can_param.ext_id_filter.fiter_config.reject_no_match = 1;
    can_param.ext_id_filter.filter_elem[0].filter_type = eDUAL_ID;
    can_param.ext_id_filter.filter_elem[0].filter_id0  = 0x7FFFFF0;
    can_param.ext_id_filter.filter_elem[0].filter_id1  = 0x7FFFFF1;
    can_param.ext_id_filter.filter_elem[1].filter_type = eDUAL_ID;
    can_param.ext_id_filter.filter_elem[1].filter_id0  = 0x700;
    can_param.ext_id_filter.filter_elem[1].filter_id1  = 0x800;
    can_param.data_tx_mode = DEV_XFER_INTERRUPT;
    can_set_config(1, (void*)(&can_param));


    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

static void tm08_can_case64_rx_ind(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    uint32_t id = 1;
    can_config_t can_param;
    uint8_t case64_buff[64+1];
    uint32_t ext_id = 0;

    can_get_config(id, (void*)(&can_param));
    can_param.eframe_type = 0;
    if (ide == eEXTENDED_FRAME) {
        ext_id = msg_id;
        can_param.eframe_format = 1;
    } else {
        ext_id = msg_id >> 18;
        can_param.eframe_format = 0;
    }
    can_set_config(id, (void*)(&can_param));

    EMBARC_PRINTF("[%s] : id= %X, rawid= %X, len=%d, %X %X\r\n", __func__, msg_id, ext_id, len, data[0], data[1]);
    transfer_bytes_stream(data, case64_buff, len);
    EMBARC_PRINTF("[%s] : id= %X, len=%X, %X %X %X %X\r\n", __func__, msg_id, len, case64_buff[0], case64_buff[1], case64_buff[2], case64_buff[3]);

    can_send_data_isr(id, ext_id+1, data, len);
}
int32_t tm08_can_case64_hal_filter_dual_specific(void *self, void *params, uint32_t len)
{
    can_config_t can_param;
    uint32_t id = 1;

    can_reset_checksum_all();

    can_init(id, CAN_BAUDRATE_500KBPS, CAN_BAUDRATE_1MBPS);
    can_interrupt_enable(id, 0, 1);
    can_interrupt_enable(id, 3, 1);
    can_indication_register(id, tm08_can_case64_rx_ind);
    can_get_config(id, (void*)(&can_param));

    can_param.eframe_format = 1;
    can_param.eframe_type = 0;

    can_param.std_id_filter.fiter_config.filter_size     = 2;
    can_param.std_id_filter.fiter_config.reject_no_match = 1;
    can_param.std_id_filter.filter_elem[0].filter_type = eDUAL_ID;
    can_param.std_id_filter.filter_elem[0].filter_id0  = 0x333;
    can_param.std_id_filter.filter_elem[0].filter_id1  = 0x444;
    can_param.std_id_filter.filter_elem[1].filter_type = eRANGE;
    can_param.std_id_filter.filter_elem[1].filter_id0  = 0x555;
    can_param.std_id_filter.filter_elem[1].filter_id1  = 0x666;

    can_param.ext_id_filter.fiter_config.filter_size     = 2;
    can_param.ext_id_filter.fiter_config.reject_no_match = 1;
    can_param.ext_id_filter.filter_elem[0].filter_type = eDUAL_ID;
    can_param.ext_id_filter.filter_elem[0].filter_id0  = 0x7FFFFF0;
    can_param.ext_id_filter.filter_elem[0].filter_id1  = 0x7FFFFF1;
    can_param.ext_id_filter.filter_elem[1].filter_type = eRANGE;
    can_param.ext_id_filter.filter_elem[1].filter_id0  = 0x777;
    can_param.ext_id_filter.filter_elem[1].filter_id1  = 0x888;

    can_param.data_tx_mode = DEV_XFER_INTERRUPT;
    can_set_config(id, (void*)(&can_param));


    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

void can0_busoff_timeout(void *can_param)
{
    uint32_t *bus_id = (uint32_t*)can_param;
    can_bus_off_recover(*bus_id);
    dw_timer_stop(0);
}

void can0_bus_off_handler(uint32_t error_stat)
{
    static uint32_t bus_id = 0;

    if (error_stat & CAN_INT_BUS_OFF) {

        can_config_t can_busoff_param;

        can_get_config(bus_id, &can_busoff_param);

        if (can_busoff_param.can_busoff_recover_mode == MANUAL){
            dw_timer_start(0, 8000000, can0_busoff_timeout, (void *)(&bus_id));// count 80ms,then can module recover
        }
    }
}

int32_t tm08_can_case70_hal_busoff_time_control(void *self, void *params, uint32_t len)
{
    int32_t result = E_OK;
    can_config_t can_param;
    uint32_t test_data[8] = {0,1};
    int i = 700000;

    can_init(0, CAN_BAUDRATE_500KBPS, 0);
    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(0, 3, 1);

    can_get_config(0, (void*)(&can_param));
    can_param.can_busoff_recover_mode = MANUAL;
    can_set_config(0, (void*)(&can_param));

    result = can_error_event_register(0, can0_bus_off_handler);
    if (E_OK != result) {
        EMBARC_PRINTF("bus_off_handler register failed\r\n");
    }

    while(i--){
        result = can_send_data(0,0x300,test_data,8);
        vTaskDelay(5/portTICK_RATE_MS);
        if(result == E_OK){
            test_data[1]++;
        }
    }

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

int32_t tm08_can_case71_hal_busoff_time_control(void *self, void *params, uint32_t len)
{
    int32_t result = E_OK;
    can_config_t can_param;
    uint32_t test_data[2] = {0,1};
    int i = 700000;

    can_init(0, CAN_BAUDRATE_500KBPS, 0);
    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(0, 3, 1);

    can_get_config(0, (void*)(&can_param));
    can_param.can_busoff_recover_mode = AUTO;
    can_set_config(0, (void*)(&can_param));

    while(i--){
        result = can_send_data(0,0x300,test_data,8);
        vTaskDelay(5/portTICK_RATE_MS);
        if(result == E_OK){
            test_data[1]++;
        }
    }

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

int32_t tm08_can_case100_hal_switch_to_canfd_mode(void *self, void *params, uint32_t len)
{
    int32_t result = E_OK;
    can_config_t can_param;
    uint32_t test_data[4] = {0x11112222,0x33334444,0x55556666,0x77778888};
    int i = 10;

    can_init(0, CAN_BAUDRATE_500KBPS, 0);
    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(0, 3, 1);

    can_get_config(0, (void*)(&can_param));
    can_param.can_mode = CAN_FD_MODE;
    can_param.data_baud = CAN_BAUDRATE_1MBPS;
    can_set_config(0, (void*)(&can_param));

    while(i--){
        result = can_send_data(0,0x300,test_data,16);
        vTaskDelay(5/portTICK_RATE_MS);
        if(result == E_OK){
            test_data[1]++;
        }
    }

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

static void tm08_can_case101_rx_ind(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len)
{
    EMBARC_PRINTF("[%s] : id= %X, len=%d, %X %X\r\n", __func__, msg_id, len, data[0], data[1]);
}

int32_t tm08_can_case101_hal_64byte_highload_no_frame_miss(void *self, void *params, uint32_t len)
{
    int32_t result = E_OK;
    can_config_t can_param;
    int frame_num = 0;
    int msg_id = 0;
    int frame_cnt = 0;
    uint32_t test_data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    msg_id = get_word_from_param(&params);
    frame_num = get_word_from_param(&params);
    EMBARC_PRINTF("msg_id: %d,frame_num: %d\r\n",msg_id,frame_num);

    can_init(0, CAN_BAUDRATE_500KBPS, 0);
    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(0, 3, 1);

    can_get_config(0, (void*)(&can_param));
    can_param.can_mode = CAN_FD_MODE;
    can_param.data_baud = CAN_BAUDRATE_500KBPS;
    /* all std frame store in rx buffer, ignore these data*/
    can_param.std_id_filter.fiter_config.filter_size     = 0;
    can_param.std_id_filter.fiter_config.reject_no_match = 1;
    can_set_config(0, (void*)(&can_param));

    while(frame_cnt < frame_num){
        EMBARC_PRINTF("frame_cnt: %d,data[0]:0x%x \r\n",frame_cnt,test_data[0]);
        result = can_send_data(0,msg_id,test_data,64);
        vTaskDelay(50/portTICK_RATE_MS);
        if(result == E_OK){
            test_data[0]++;
        }
        frame_cnt++;
    }

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

int32_t tm08_can_case102_hal_64byte_highload_no_frame_miss(void *self, void *params, uint32_t len)
{
    int32_t result = E_OK;
    can_config_t can_param;
    int frame_num = 0;
    int msg_id = 0;
    int frame_cnt = 0;
    uint32_t test_data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    msg_id = get_word_from_param(&params);
    frame_num = get_word_from_param(&params);
    EMBARC_PRINTF("msg_id: %d,frame_num: %d\r\n",msg_id,frame_num);

    can_init(0, CAN_BAUDRATE_500KBPS, 0);
    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(0, 3, 1);
    can_indication_register(0, tm08_can_case101_rx_ind);

    can_get_config(0, (void*)(&can_param));
    can_param.can_mode = CAN_FD_MODE;
    can_param.std_id_filter.fiter_config.filter_size     = 1;
    can_param.std_id_filter.fiter_config.reject_no_match = 1;
    can_param.std_id_filter.filter_elem[0].filter_type = eDUAL_ID;
    can_param.std_id_filter.filter_elem[0].filter_id0 = 0x2;
    can_param.std_id_filter.filter_elem[0].filter_id1 = 0x2;
    can_param.data_baud = CAN_BAUDRATE_500KBPS;
    can_set_config(0, (void*)(&can_param));

    while(frame_cnt < frame_num){
        EMBARC_PRINTF("frame_cnt: %d,data[0]:0x%x \r\n",frame_cnt,test_data[0]);
        result = can_send_data(0,msg_id,test_data,64);
        vTaskDelay(50/portTICK_RATE_MS);
        if(result == E_OK){
            test_data[0]++;
        }
        frame_cnt++;
    }

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;
}

static int32_t can_cli0_loop_back(uint8_t *data, uint32_t len)
{
    uint32_t test_data[2] = {0,0};
    EMBARC_PRINTF("[%s]\r\n", __func__);

    transfer_word_stream(data, test_data, len);

    can_send_data(0, 0x401, test_data, len);
    return 0;
}
static int32_t can_cli1_loop_back(uint8_t *data, uint32_t len)
{
    uint32_t test_data[2] = {0,0};
    EMBARC_PRINTF("[%s]\r\n", __func__);

    transfer_word_stream(data, test_data, len);

    can_send_data(1, 0x402, test_data, len);
    return 0;
}

int32_t tm08_can_case103_can_cli_0_1(void *self, void *params, uint32_t len)
{
    int32_t result = E_OK;
    can_config_t can_param;

    can_cli_pre_init();
    can_cli_init(0);
    can_cli_init(1);

    can_get_config(0, (void*)(&can_param));
    can_param.can_mode = CAN_FD_MODE;
    can_param.data_baud = 1000000;
    can_param.std_id_filter.fiter_config.filter_size     = 1;
    can_param.std_id_filter.fiter_config.reject_no_match = 1;
    can_param.std_id_filter.filter_elem[0].filter_type = eDUAL_ID;
    can_param.std_id_filter.filter_elem[0].filter_id0  = 0x400;
    can_param.std_id_filter.filter_elem[0].filter_id1  = 0x400;
    can_set_config(0, (void*)(&can_param));

    can_get_config(1, (void*)(&can_param));
    can_param.std_id_filter.fiter_config.filter_size     = 1;
    can_param.std_id_filter.fiter_config.reject_no_match = 1;
    can_param.std_id_filter.filter_elem[0].filter_type = eDUAL_ID;
    can_param.std_id_filter.filter_elem[0].filter_id0  = 0x400;
    can_param.std_id_filter.filter_elem[0].filter_id1  = 0x400;
    can_set_config(1, (void*)(&can_param));

    can_cli_register(0, 0x400, can_cli0_loop_back);
    can_cli_register(1, 0x400, can_cli1_loop_back);
    return result;
}

TaskHandle_t can_case104_handle = NULL;
TimerHandle_t xTimer_case104 = NULL;
volatile bool case104_flag = true;
int case104_frame_num = 0;
uint32_t tx_msg_id[35] = {0x37,0x38,0x39,0x44,0x45,0x48,0x49,0x50,0x51,0x13f,0x131,
						 0x135,0x210,0x211,0x212,0x213,0x214,0x215,0x217,0x218,
						 0x219,0x21a,0x21b,0x21c,0x220,0x221,0x222,0x223,0x224,
						 0x225,0x226,0x227,0x228,0x229,0x230};
uint32_t test_data_len[35] = {20,8,12,8,8,8,8,8,
							  64,64,16,8,8,64,24,8,
							  12,12,8,8,12,12,12,12,
							  64,64,64,64,64,64,64,64,
							  64,64,64};
uint32_t test_data[35][16];

void can_case104_task(void *params)
{
    int32_t result = E_OK;
    int frame_cnt = 0;

    while(1){
        while(frame_cnt < case104_frame_num){

            if(case104_flag == true){
                case104_flag = false;

                for(int i=0;i<35;i++){
                    EMBARC_PRINTF("i:%d,frame_cnt: %d, msg_id: 0x%x, data[0]: 0x%x \r\n",i,frame_cnt,tx_msg_id[i],test_data[i][0]);
                    result = can_send_data(0,tx_msg_id[i],&test_data[i][0],test_data_len[i]);
                    if(result == E_OK){
                        test_data[i][0]++;
                    }else{
                        EMBARC_PRINTF("send fail, result:%d, frame_cnt: %d, msg_id: 0x%x, data[0]: 0x%x \r\n",result,frame_cnt,tx_msg_id[i],test_data[i][0]);
                    }
                }

                frame_cnt++;
            }

        }
    }

}
static void case104_vTimerCallback( TimerHandle_t pxTimer )
{
    case104_flag = true;
    xTimerStart( xTimer_case104, 0 );
}

int32_t tm08_can_case104_hal_64byte_typical_case(void *self, void *params, uint32_t len)
{
    case104_frame_num = get_word_from_param(&params);
    EMBARC_PRINTF("frame_num: %d\r\n",case104_frame_num);

	for(int t=0;t<35;t++){
		for(int j=0;j<16;j++){
			test_data[t][j] = 0;
		}
	}
    can_config_t can_param;
    can_init(0, CAN_BAUDRATE_500KBPS, 0);
    can_interrupt_enable(0, 0, 1);
    can_interrupt_enable(0, 3, 1);
    can_indication_register(0, tm08_can_case101_rx_ind);

    can_get_config(0, (void*)(&can_param));
    can_param.can_mode = CAN_FD_MODE;
    can_param.std_id_filter.fiter_config.filter_size     = 1;
    can_param.std_id_filter.fiter_config.reject_no_match = 1;
    can_param.std_id_filter.filter_elem[0].filter_type = eRANGE;
    can_param.std_id_filter.filter_elem[0].filter_id0 = 0x2;
    can_param.std_id_filter.filter_elem[0].filter_id1 = 0x200;
    can_param.data_baud = CAN_BAUDRATE_500KBPS;
    can_set_config(0, (void*)(&can_param));

    if (xTaskCreate(can_case104_task, "can_case104_task", 1048, (void *)0, configMAX_PRIORITIES-1, &can_case104_handle) != pdPASS) {
        EMBARC_PRINTF("create can_case104_task error\r\n");
        return -1;
    }

    xTimer_case104 = xTimerCreate( (const char *) "Timer_case104",
                            pdMS_TO_TICKS(50),
                            pdFALSE,
                            (void *) 1,
                            case104_vTimerCallback);

    xTimerStart( xTimer_case104, 0 );

    EMBARC_PRINTF("[%s] : Done!\r\n", __func__);
    return 0;

}

#endif


CAN_VALIDATION_SYS_CASE(CAN_CID_0, tm08_can_case0_loopback);
CAN_VALIDATION_SYS_CASE(CAN_CID_1, tm08_can_case1_loopback);

// CAN_VALIDATION_SYS_CASE(CAN_CID_0, tm08_can_case4_filter);
// CAN_VALIDATION_SYS_CASE(CAN_CID_0, tm08_can_case8_case17_long_time_high_load);
// CAN_VALIDATION_SYS_CASE(CAN_CID_0, tm08_can_case9_case18_2can);
// CAN_VALIDATION_SYS_CASE(CAN_CID_0, tm09_can_case9_2can_loopback);
// CAN_VALIDATION_SYS_CASE(CAN_CID_0, tm10_can_case20_2canfd_loopback);

CAN_VALIDATION_SYS_CASE(CAN_CID_50, tm08_can_case50_init_deinit);
CAN_VALIDATION_SYS_CASE(CAN_CID_51, tm08_can_case51_hal_loopback);
CAN_VALIDATION_SYS_CASE(CAN_CID_52, tm08_can_case52_hal_loopback_1M);
CAN_VALIDATION_SYS_CASE(CAN_CID_53, tm08_can_case53_hal_loopback_extend_id);

CAN_VALIDATION_SYS_CASE(CAN_CID_60, tm08_can_case60_hal_FD_init_deinit);
CAN_VALIDATION_SYS_CASE(CAN_CID_61, tm08_can_case61_hal_loopback_FD_500K_1M);
CAN_VALIDATION_SYS_CASE(CAN_CID_62, tm08_can_case62_hal_loopback_FD_1M_2M);
CAN_VALIDATION_SYS_CASE(CAN_CID_63, tm08_can_case63_hal_loopback_FD_extend_id);
CAN_VALIDATION_SYS_CASE(CAN_CID_64, tm08_can_case64_hal_filter_dual_specific);

CAN_VALIDATION_SYS_CASE(CAN_CID_70, tm08_can_case70_hal_busoff_time_control);
CAN_VALIDATION_SYS_CASE(CAN_CID_71, tm08_can_case71_hal_busoff_time_control);

CAN_VALIDATION_SYS_CASE(CAN_CID_100, tm08_can_case100_hal_switch_to_canfd_mode);
CAN_VALIDATION_SYS_CASE(CAN_CID_101, tm08_can_case101_hal_64byte_highload_no_frame_miss);
CAN_VALIDATION_SYS_CASE(CAN_CID_102, tm08_can_case102_hal_64byte_highload_no_frame_miss);
CAN_VALIDATION_SYS_CASE(CAN_CID_103, tm08_can_case103_can_cli_0_1);
CAN_VALIDATION_SYS_CASE(CAN_CID_104, tm08_can_case104_hal_64byte_typical_case);

