#ifndef CAN_HAL_H
#define CAN_HAL_H

#include "dev_common.h"

#include "can.h"

/* the number of CAN channel*/
#define CAN_CHANNEL_NUM     (2)

/* the number of ID ﬁlter elements */
#define CAN_ID_FILTER_ELEMENT_NUM          10

/* For CAN */
#define CAN_FRAM_BUF_DATA_SIZE    (8)
#define CAN_FRAM_MAX_BUF_NUM      (64)


/* For CAN FD */

// Possible Data Field Length of FD Frame: 8, 12, 16, 20, 24, 32, 48, 64. In Bytes.
#define CAN_FD_FRAM_BUF_DATA_SIZE (64)

#if   CAN_FD_FRAM_BUF_DATA_SIZE == 8
#define CAN_FD_FRAM_MAX_BUF_NUM  (64)
#elif CAN_FD_FRAM_BUF_DATA_SIZE == 12
#define CAN_FD_FRAM_MAX_BUF_NUM  (48)
#elif CAN_FD_FRAM_BUF_DATA_SIZE == 16
#define CAN_FD_FRAM_MAX_BUF_NUM  (40)
#elif CAN_FD_FRAM_BUF_DATA_SIZE == 20
#define CAN_FD_FRAM_MAX_BUF_NUM  (32)
#elif CAN_FD_FRAM_BUF_DATA_SIZE == 24
#define CAN_FD_FRAM_MAX_BUF_NUM  (32)
#elif CAN_FD_FRAM_BUF_DATA_SIZE == 32
#define CAN_FD_FRAM_MAX_BUF_NUM  (24)
#elif CAN_FD_FRAM_BUF_DATA_SIZE == 48
#define CAN_FD_FRAM_MAX_BUF_NUM  (16)
#elif CAN_FD_FRAM_BUF_DATA_SIZE == 64
#define CAN_FD_FRAM_MAX_BUF_NUM  (14)
#else
#error "Incorrect FD Frame Size!"
#endif

typedef enum{
        eDATA_LEN_8  = 8,
        eDATA_LEN_12 = 12,
        eDATA_LEN_16 = 16,
        eDATA_LEN_20 = 20,
        eDATA_LEN_24 = 24,
        eDATA_LEN_32 = 32,
        eDATA_LEN_48 = 48,
        eDATA_LEN_64 = 64
} eCAN_DATA_LEN;

typedef struct {
        uint32_t msg_id;					/* message identifier */
        eCAN_FRAME_FORMAT eframe_format;	/* frame format: 0 is standard frame, 1 is extended frame */
        eCAN_FRAME_TYPE eframe_type;		/* frame type:   0 is data frame, 1 is remote frame */
        uint16_t len;						/* Length of Message Data Field , in Bytes. */
} can_frame_params_t;

typedef struct can_data_message {
        can_frame_params_t *frame_params;
        uint8_t  data[eDATA_LEN_64];
        uint16_t can_ch;
        uint16_t lock;
} can_data_message_t;


typedef enum{
        eDATA_DLC_8  = 8,
        eDATA_DLC_9,
        eDATA_DLC_10,
        eDATA_DLC_11,
        eDATA_DLC_12,
        eDATA_DLC_13,
        eDATA_DLC_14,
        eDATA_DLC_15
} eCAN_DLC;

typedef enum{
        CAN_MODE,
        CAN_FD_MODE
} eCAN_MODE;

typedef enum{
        AUTO,
        MANUAL
} eCAN_BUSOFF_RECOVER_MODE;

#ifdef OS_FREERTOS
#define CAN_ISR_QUEUE_LENGTH 64
#endif

#define CAN_TX_COMPLETE_MONITOR

typedef void (*rx_indication_callback)(uint32_t msg_id, uint32_t ide, uint32_t *data, uint32_t len);

#ifdef CAN_TX_COMPLETE_MONITOR
typedef void (*tx_confirmation_callback)(uint32_t msg_id, uint32_t txFailed);
#else
typedef void (*tx_confirmation_callback)(uint32_t msg_id);
#endif

/* Below is bit mask to input parameter of error_indication_callback
 *     1 means corresponding error occured.
 *     0 means corresponding error not occured.
 **/
#define CAN_ERR_STAT_BIT_MASK_PROTOCOL_ERR            (1 << 23)
#define CAN_ERR_STAT_BIT_MASK_BUS_OFF                 (1 << 22)
#define CAN_ERR_STAT_BIT_MASK_ERR_PASSIVE             (1 << 20)
typedef void (*error_indication_callback)(uint32_t error_stat);

int32_t can_init(uint32_t id, uint32_t nomi_baud, uint32_t data_baud);

int32_t can_read(uint32_t id, uint32_t *buf, uint32_t len);
int32_t can_write(uint32_t id, uint32_t frame_id, uint32_t *buf, uint32_t len, uint32_t flag);
int32_t can_write_isr(uint32_t id, uint32_t frame_id, uint32_t *buf, uint32_t len, uint32_t flag);

int32_t can_interrupt_enable(uint32_t id, uint32_t type, uint32_t enable);

int32_t can_xfer_callback_install(uint32_t id, void (*func)(void *));

#ifdef OS_FREERTOS
int32_t can_queue_install(uint32_t id, QueueHandle_t queue, uint32_t rx_or_tx);
#endif
void *can_xfer_buffer(uint32_t id, uint32_t rx_or_tx);

int32_t can_indication_register(uint32_t dev_id, rx_indication_callback func);
int32_t can_confirmation_register(uint32_t dev_id, tx_confirmation_callback func);
int32_t can_error_event_register(uint32_t dev_id, error_indication_callback func);

int32_t can_receive_data(uint32_t dev_id, can_data_message_t *msg);

int32_t can_frame_params_setup(uint32_t id, can_frame_params_t *params);
/* Only used in Task call,support Sync/ASync mode*/
int32_t can_send_data(uint32_t bus_id, uint32_t frame_id, uint32_t *data, uint32_t len);
/* Only used in ISR call, ONLY support ASync mode */
int32_t can_send_data_isr(uint32_t bus_id, uint32_t frame_id, uint32_t *data, uint32_t len);

void can_deinit(uint32_t id);
int32_t can_get_config(uint32_t id, void *param);
int32_t can_set_config(uint32_t id, void *param);

void can_bus_off_recover(uint32_t bus_id);

/* CAN Filter related */
typedef struct {
    uint32_t filter_size;
    bool reject_no_match;
} can_cfg_filter_param_t;

typedef struct {
    uint32_t filter_type;
    uint32_t filter_id0;
    uint32_t filter_id1;
} can_cfg_filter_element_t;

typedef struct {
    can_cfg_filter_param_t   fiter_config;
    can_cfg_filter_element_t filter_elem[CAN_ID_FILTER_ELEMENT_NUM];
} can_cfg_filter_t;

typedef struct {
        /* CAN Bus Mode: CAN , CAN FD */
        eCAN_MODE can_mode;                     /* 0 - CAN_MODE, 1 - CAN_FD_MODE */

        /* CAN Busoff Recover Mode: AUTO, MANUAL */
        eCAN_BUSOFF_RECOVER_MODE can_busoff_recover_mode;/* 0 - AUTO, 1 - MANUAL */

        /* CAN Tx_mode: polling, int */
        uint8_t data_tx_mode;                   /* sync mode: 0 - DEV_XFER_POLLING , async mode: 1 - DEV_XFER_INTERRUPT */

        /* CAN Baudrate */
        uint32_t nomi_baud;
        uint32_t data_baud;

        /* CAN RX Filter Configure */
        can_cfg_filter_t std_id_filter;         /* CAN_STD_FILTER */
        can_cfg_filter_t ext_id_filter;         /* CAN_EXT_FILTER */

        /* CAN TX frame format */
        eCAN_FRAME_FORMAT eframe_format;        /* frame format: 0 - eSTANDARD_FRAME, 1 - eEXTENDED_FRAME */
        eCAN_FRAME_TYPE eframe_type;            /* frame type:   0 - eDATA_FRAME, 1 - eREMOTE_FRAME */

} can_config_t;


/*************************Function Begin*************************
* Function Name: swap_hlbyte
*
* Description: Firstly, the uint32_t data will be exchanged high and low byte,
*              then it will be stored as uint8_t data.
*
* Inputs: Parameter 1 is a pointer to the data type uint32_t
*         Parameter 3 is the number of data in parameter 1
*         That is how many uint32_t data needs to be stored as uint8_t data.
* Outputs: Parameter 2 is a pointer to the data type uint8_t
*
*************************Function End***************************/
static inline void swap_hlbyte(uint32_t *data, uint8_t *msg_data, uint32_t len)
{
        int32_t i = 0, j = 0, n = (len / 4);
        int32_t temp = 0;

        for (i = 0; i < n / 2; i++)
        {
                temp = data[i];
                data[i] = data[n - i - 1];
                data[n - i - 1] = temp;
        }

        while (n--) {
                for (i = 3; i >= 0; i--) {
                        msg_data[j] = (uint8_t)(*data >> (i << 3));
                        j++;
                }
                data++;
        }
}


/*************************Function Begin*************************
* Function Name: transfer_bytes_stream
*
* Description: The uint32_t data will be stored as uint8_t data.
*
* Inputs: Parameter 1 is a pointer to the data type uint32_t
*         Parameter 3 is the number of data in parameter 2
*         That is how many uint32_t data needs to be stored as uint8_t data.
* Outputs: Parameter 2 is a pointer to the data type uint8_t
*
*************************Function End***************************/
static inline void transfer_bytes_stream(uint32_t *src, uint8_t *dst, uint32_t len)
{
        int32_t i = 0, j = 0, n = (len / sizeof(uint32_t));

		if((len % sizeof(uint32_t)) > 0){
			n++;
		}

        while (n--) {
                for (i = 0; i < sizeof(uint32_t); i++) {
                        dst[j] = (uint8_t)(*src >> (i << 3));
                        j++;
                }
                src++;
        }
}


/*************************Function Begin*************************
* Function Name: transfer_word_stream
*
* Description: The uint8_t data will be stored as uint32_t data.
*
* Inputs: Parameter 1 is a pointer to the data type uint8_t
*         Parameter 3 is the number of data in parameter 1
*         That is how many uint8_t data needs to be stored as uint32_t data.
* Outputs: Parameter 2 is a pointer to the data type uint32_t
*
*************************Function End***************************/

static inline void transfer_word_stream(uint8_t *src, uint32_t *dst, uint32_t len)
{
        int32_t i = 0, n = (len / sizeof(uint32_t));

        while (n--) {
                *dst = 0;
                for (i = 0; i < sizeof(uint32_t); i++) {
                        *dst |= (uint32_t)(*src++ << (i << 3));
                }
                dst++;
        }
}
#endif
