#include "embARC_toolchain.h"
#include "embARC_error.h"
#include "embARC.h"
#include "embARC_debug.h"
#include "arc_exception.h"
#include <string.h>
#include "can_hal.h"
#include "dev_can.h"
#include "otp_hal.h"
#include "timer_hal.h"
#ifdef FUNC_SAFETY
#include "func_safety.h"
#endif



static int32_t can_get_buf(uint32_t id, void *param);
static int32_t can_set_buf(uint32_t id, void *param);
static void timeout_handler(void *can_param);

#define CAN_REF_CLOCK_DEFAULT       (40000000)
#define CAN_DFS_2_FRAMESIZE(dfs) \
        ( ((dfs) < 5) ? ((dfs) + 4) : ((((dfs) - 5) << 2) + 10) )

#define CAN_DATASIZE_2_DFS(data_size) \
        ( ((data_size) < 32) ? ((data_size >> 2) - 2) : (((((data_size) >> 2) - 8) >> 2) + 5) )

#ifdef OS_FREERTOS
#include "FreeRTOS.h"
#include "task.h"

#define CAN_LOCK(lock)    \
        while (xSemaphoreTake(lock, portMAX_DELAY) != pdTRUE) { \
                taskYIELD(); \
        }
#define CAN_UNLOCK(lock)    xSemaphoreGive(lock)

static xSemaphoreHandle can_sema[2] = {NULL, NULL};
#else
#define CAN_LOCK(lock)
#define CAN_UNLOCK(lock)
#endif

typedef enum{
        eCAN_INT_TYPE_RX = 0,
        eCAN_INT_TYPE_TX = 1,
        eCAN_INT_TYPE_ERROR = 3
} eCAN_INT_TYPE;

typedef enum{
        eCAN_INT_DISABLE = 0,
        eCAN_INT_ENABLE  = 1
} eCAN_INT_OP;

typedef enum{
        eCAN_BUFFER_MODE = 0,
        eCAN_FIFO_MODE   = 1
} eCAN_OPRATION_MODE;

#ifndef CHIP_ALPS_A

typedef struct can_xfer_descriptor {
        /* @mode: 0->buffer mode, 1->fifo mode. */
        uint8_t mode;

        /* @xfer_mode: 0->polling, 1->int, 2->dma, 3->task agency. */
        uint8_t data_rx_mode;
        uint8_t data_tx_mode;

        void *tx_buf;
        uint32_t tx_len;
        uint32_t tx_ofs;

        void *rx_buf;
        uint32_t rx_len;
        uint32_t rx_ofs;

        /* 000 - 8Bytes, 001 - 12Bytes, 010 - 16Bytes, 011 - 20Bytes,
         * 100 - 24Bytes, 101 - 32Bytes, 110 - 48Bytes, 111 - 64Byte.
         */
        uint8_t rx_dfs;
        uint8_t tx_dfs;
        uint8_t rx_last_id;
        uint8_t tx_last_id;
        uint8_t rx_max_id;
        uint8_t tx_max_id;

#ifdef OS_FREERTOS
        QueueHandle_t tx_queue;
        QueueHandle_t rx_queue;
#endif

        /* CAN Bus Mode: CAN , CAN FD */
        eCAN_MODE can_mode;

        /* CAN Busoff Recover Mode: AUTO, MANUAL */
        eCAN_BUSOFF_RECOVER_MODE can_busoff_recover_mode;

        /* CAN Baudrate */
        uint32_t nomi_baud;
        uint32_t data_baud;

        /* CAN RX Standard Filter Configure */
        can_id_filter_param_t   filter;
        can_id_filter_element_t filter_element[CAN_ID_FILTER_ELEMENT_NUM];

        /* CAN RX Extended Filter Configure */
        can_id_filter_param_t   ext_filter;
        can_id_filter_element_t ext_filter_element[CAN_ID_FILTER_ELEMENT_NUM];

        /* CAN Protocol Configure */
        can_protocol_cfg_t      protocol;

        /* CAN Frame Parameters */
        can_frame_params_t      frame_parameters;

#ifdef CAN_TX_COMPLETE_MONITOR
                uint32_t rx_isr_free_counter;
#endif

        rx_indication_callback rx_indication;
        tx_confirmation_callback tx_confirmation;
        error_indication_callback err_indication;
        void (*xfer_callback)(void *);
} can_xfer_t;

typedef struct can_struct {
        can_t *dev_can_s;
        can_xfer_t *can_buf_s;
} can_ts;

#ifdef OS_FREERTOS
/* Task Communication */
extern QueueHandle_t queue_can_isr;
#endif

static can_t *dev_can[2] = { NULL, NULL};
static can_xfer_t can_buf[2];
static can_ts can_exa;
static can_ts can_exa_1;
extern volatile uint8_t can_tx_done_flag[2];
extern volatile uint8_t can_node_rec_flag[2];
volatile uint32_t assert_reset_flag[2] = {0,0};
__attribute__((aligned(4))) static uint8_t can_sw_buf[eDATA_LEN_64];


static can_xfer_t can_mode_default = {
            /* buffer mode. */
    #ifdef OS_FREERTOS
            .tx_queue = NULL,
            .rx_queue = NULL,
    #endif
            .mode = eCAN_FIFO_MODE,
            .data_rx_mode = 1,
        #ifdef OS_FREERTOS
            .data_tx_mode = DEV_XFER_INTERRUPT,
        #else
            .data_tx_mode = DEV_XFER_POLLING,
        #endif
            .rx_dfs = CAN_DATASIZE_2_DFS(CAN_FRAM_BUF_DATA_SIZE),
            .tx_dfs = CAN_DATASIZE_2_DFS(CAN_FRAM_BUF_DATA_SIZE),
            .rx_last_id = 0,
            .tx_last_id = 0,
            .rx_max_id  = (CAN_FRAM_MAX_BUF_NUM - 1),
            .tx_max_id  = 0,

            .can_mode  = CAN_MODE,
            .can_busoff_recover_mode = MANUAL,//MANUAL,//AUTO,
            .nomi_baud = CAN_BAUDRATE_500KBPS,
            .data_baud = 0,

            .filter.frame_format    = eSTANDARD_FRAME,
            .filter.reject_no_match = false,
            .filter.filter_size     = CAN_ID_FILTER_ELEMENT_NUM,
            .filter.mask            = 0x1FFFFFFF,

            .filter_element = {
                    {
                            /* element 0 */
                            .frame_format    = eSTANDARD_FRAME,
                            .filter_type = eDUAL_ID,

                            /* Possible filter_cfg: eRX_FIFO, eRX_BUF, eSPEC_RX_BUF, eREJ_ID */
                            .filter_cfg = eRX_FIFO,
                            .filter_id0 = 0x111,
#ifdef FUNC_SAFETY
                            .filter_id1 = FUNC_SAFETY_FRAME_ID
#else
                            .filter_id1 = 0x200
#endif
                    },
                    {
                            /* element 1 */
                            .frame_format = eSTANDARD_FRAME,
                            .filter_type  = eDUAL_ID,

                            /* Possible filter_cfg: eRX_FIFO, eRX_BUF, eSPEC_RX_BUF, eREJ_ID */
                            .filter_cfg = eRX_FIFO,
                            .filter_id0 = 0x222,
                            .filter_id1 = 0x200

                    },
                    {
                            /* element 2 */
                            .frame_format = eSTANDARD_FRAME,
                            .filter_type = eDUAL_ID,
                            .filter_cfg = eRX_FIFO,
                            .filter_id0 = 0x203, /* the command msg id of CAN HIL */
                            .filter_id1 = 0x204  /* the command msg id of CAN SENSOR CONFIG */
                    },
                    {
                            /* element 3,frames that do not need to be received are placed in the buffer */
                            .frame_format = eSTANDARD_FRAME,
                            .filter_type = eRANGE,
                            .filter_cfg = eRX_FIFO, //eRX_BUF,
                            .filter_id0 = 0x0,
                            .filter_id1 = 0x7ff
                    }
            },

            .ext_filter.frame_format    = eEXTENDED_FRAME,
            .ext_filter.reject_no_match = false,
            .ext_filter.filter_size     = CAN_ID_FILTER_ELEMENT_NUM,
            .ext_filter.mask            = 0x1FFFFFFF,

            .ext_filter_element = {
                    {
                            /* element 0,frames that do not need to be received are placed in the buffer */
                            .frame_format = eEXTENDED_FRAME,
                            .filter_type = eRANGE,
                            .filter_cfg = eRX_BUF,
                            .filter_id0 = 0x0,
                            .filter_id1 = 0x1fffffff
                    }
            },

            .protocol.fd_operation = 0,
            .protocol.bit_rate_switch = 0,
            .protocol.tx_delay_compensate = 0,
            .protocol.auto_retransmission = 1,

            .frame_parameters.msg_id = 0,
            .frame_parameters.eframe_format = eSTANDARD_FRAME,
            .frame_parameters.eframe_type = eDATA_FRAME,
            .frame_parameters.len = CAN_FRAM_BUF_DATA_SIZE,
};

static can_xfer_t can_fd_mode_default_1 = {

            /* buffer mode. */
    #ifdef OS_FREERTOS
            .tx_queue = NULL,
            .rx_queue = NULL,
    #endif
            .mode = eCAN_FIFO_MODE,
            .data_rx_mode = 1,
#ifdef OS_FREERTOS
            .data_tx_mode = DEV_XFER_INTERRUPT,
#else
            .data_tx_mode = DEV_XFER_POLLING,
#endif
            .rx_dfs = CAN_DATASIZE_2_DFS(CAN_FD_FRAM_BUF_DATA_SIZE),
            .tx_dfs = CAN_DATASIZE_2_DFS(CAN_FD_FRAM_BUF_DATA_SIZE),
            .rx_last_id = 0,
            .tx_last_id = 0,
            .rx_max_id  = (CAN_FD_FRAM_MAX_BUF_NUM-1),
            .tx_max_id  = 0,

            .can_mode  = CAN_FD_MODE,
            .can_busoff_recover_mode = MANUAL,//MANUAL,//AUTO,
            .nomi_baud = CAN_BAUDRATE_500KBPS,
            .data_baud = CAN_BAUDRATE_1MBPS,

            .filter.frame_format    = eSTANDARD_FRAME,
            .filter.reject_no_match = true,
            .filter.filter_size     = CAN_ID_FILTER_ELEMENT_NUM,
            .filter.mask            = 0x1FFFFFFF,

            .filter_element = {
                    {
                            /* element 0 */
                            .frame_format = eSTANDARD_FRAME,
                            .filter_type = eDUAL_ID,
                            .filter_cfg = eRX_FIFO,
                            .filter_id0 = 0x200,
                            .filter_id1 = 0x201
                    },
                    {
                            /* element 1 */
                            .frame_format = eSTANDARD_FRAME,
                            .filter_type = eDUAL_ID,
                            .filter_cfg = eRX_FIFO,
                            .filter_id0 = 0x300,
                            .filter_id1 = 0x301
                    },
                    {
                            /* element 2,frames that do not need to be received are placed in the buffer */
                            .frame_format = eSTANDARD_FRAME,
                            .filter_type = eRANGE,
                            .filter_cfg = eRX_BUF,
                            .filter_id0 = 0x0,
                            .filter_id1 = 0x7ff
                    }

            },

            .ext_filter.frame_format    = eEXTENDED_FRAME,
            .ext_filter.reject_no_match = true,
            .ext_filter.filter_size     = CAN_ID_FILTER_ELEMENT_NUM,
            .ext_filter.mask            = 0x1FFFFFFF,

            .ext_filter_element = {
                    {
                            /* element 0,frames that do not need to be received are placed in the buffer */
                            .frame_format = eEXTENDED_FRAME,
                            .filter_type = eRANGE,
                            .filter_cfg = eRX_BUF,
                            .filter_id0 = 0x0,
                            .filter_id1 = 0x1fffffff
                    }
            },

            .protocol.fd_operation = 1,
            .protocol.bit_rate_switch = 1,
            .protocol.tx_delay_compensate = 0,
            .protocol.auto_retransmission = 1,

            .frame_parameters.msg_id = 0,
            .frame_parameters.eframe_format = eSTANDARD_FRAME,
            .frame_parameters.eframe_type = eDATA_FRAME,
            .frame_parameters.len = CAN_FD_FRAM_BUF_DATA_SIZE,
};


can_fifo_param_t rx_fifo_param = {
        /* We set watermark to FIFO element size.
         * Means that the RX FIFO is full when watermark reached.
         * */
        .watermark = 1,

        /* We set RX FIFO size to FIFO element size(FIFO depth).
         * Means that RX direction use all the FIFO element.
         * */
        .size = 64,

        /* Possible Mode: 0 - FIFO blocking mode, 1 - FIFO overwrite mode.*/
        .mode = 0,
};


can_fifo_param_t tx_fifo_param = {
        .watermark = 1,

        .size = 64,

        /* Possible Mode: 0 - FIFO blocking mode, 1 - FIFO overwrite mode.*/
        .mode = 0,
};

static inline int32_t _can_interrupt_enable(can_t *can_ptr, uint32_t type, uint32_t enable, uint32_t mode);
static int32_t can_int_install(uint32_t id);
static int32_t can_install_data_buffer(uint32_t id, uint32_t *buf, uint32_t len, uint32_t rx_or_tx);
static void can_reset_data_buffer(can_xfer_t *buf);
static inline int32_t can_get_dlc(eCAN_MODE mode, uint32_t len);
static inline int32_t can_get_datalen(eCAN_MODE mode, uint32_t dlc);

static inline clock_source_t can_clock_source(uint32_t id)
{
        if (id > 0) {
                return CAN1_CLOCK;
        } else {
                return CAN0_CLOCK;
        }
}

/*
 * Default: standard frame, buffer mode with len 8bytes.
 * */
int32_t can_init(uint32_t id, uint32_t nomi_baud, uint32_t data_baud)
{
        int32_t result = E_OK;
        can_ops_t *can_ops = NULL;
        can_baud_t *baud_param = NULL;

        uint32_t index = 0;

        dw_timer_init();

        if ((id >= 2) || (0 == nomi_baud)) {
                result = E_PAR;
        } else {
                dev_can[id] = can_get_dev(id);
                if (NULL == dev_can[id]) {
                        result = E_NOEXS;
                } else {
                        can_ops = (can_ops_t *)dev_can[id]->ops;
                        if (NULL == can_ops) {
                            result = E_NOOPS;
                        } else {
                                /* Save User Configures */
                                if (/*id == 0 ||*/ data_baud == 0) {
                                        memcpy(&can_buf[id], &can_mode_default,  sizeof(can_xfer_t));
                                } else {
                                        memcpy(&can_buf[id], &can_fd_mode_default_1, sizeof(can_xfer_t));
                                }
                                can_buf[id].nomi_baud = nomi_baud;
                                can_buf[id].data_baud = data_baud;
                                /* As long as the firmware starts to run, the CAN controller may encounter an uncertain state.
                                 * So need to do a software reset before initializing CAN controller.
                                 * */
                                result = can_ops->reset(dev_can[id], 1);
                                if (result != E_OK) {
                                        EMBARC_PRINTF("CAN rest failed!\r\n");
                                }
                                /* Can reset tx done flag */
                                can_tx_done_flag[id] = 1;
                                if (0 == id) {
                                        can0_enable(1);
                                } else {
                                        can1_enable(1);
                                }

                                if (can_buf[id].mode == eCAN_FIFO_MODE) {
                                        /* The current FIFO mode is only RX FIFO mode */
                                        /* Initialization of TX FIFO to be determined */

                                        /* Configuration CAN RX FIFO */
                                        can_ops->fifo_config(dev_can[id], CAN_RX_FIFO, &rx_fifo_param);

                                        /* Possible Timestamp Counter Mode(Parameter 3):
                                         * 0x1 - increment according to TSCP(Timestamp Counter Prescale).
                                         * 0x2 - external timestamp counter value.
                                         * */
                                        can_ops->timestamp_config(dev_can[id], 0, 0x1);

                                        /* Init timeout counter
                                         * Parameter 3: Timeout Period.
                                         * Start value of Timeout Counter value.
                                         * 10000 here means that the time duration to send 10000bits.
                                         * */
                                        can_ops->timeout_config(dev_can[id], eCAN_TIMEOUT_ENABLE, 10000, eCAN_TIMEOUT_RX_FIFO);
                                }

                                /* Enable the automatic retransmission */
                                /* The maximum value of the retransferring times is 16 */
                                can_buf[id].protocol.auto_retransmission = 1;
                                can_ops->protocol_config(dev_can[id], &can_buf[id].protocol);

                                /* Configurate ID Filter Control Register */
                                can_ops->id_filter_config(dev_can[id], &can_buf[id].filter);

#ifdef FUNC_SAFETY
                                /* enable can ecc before the configuraton of ID filter element */
                                func_safety_enable_can_ecc(id);
#endif

                                /* Setting the configuraton of ID filter element */
                                for (index = 0; (E_OK == result) && (index < CAN_ID_FILTER_ELEMENT_NUM); index++) {
                                        result = can_ops->id_filter_element(dev_can[id], index, &can_buf[id].filter_element[index]);
                                }

                                clock_source_t clk_src = can_clock_source(id);
                                baud_param = can_get_baud(clock_frequency(clk_src), can_buf[id].nomi_baud);
                                if (NULL == baud_param) {
                                        result = E_NOEXS;
                                        EMBARC_PRINTF("[%s] unsupported nomi bitrate %d!\r\n",__func__,nomi_baud);
                                } else {
                                        can_baud_t Can_baud_param;
					if( CLOCK_100MHZ == clock_frequency(clk_src) )
					{
						// if( id == VCAN )
						// {
						// 	//¡À¨¨??¦Ì?¨°a?¨®VCAN2¨¦?¨´¦Ì??a70%
						// 	//???¡ã2¨¦?¨´¦Ì?¨¦¨¨??72%
						// 	Can_baud_param.sync_jump_width = 4;
						// 	Can_baud_param.bit_rate_prescale = clock_frequency(clk_src) / nomi_baud / 25 - 1 ;
						// 	Can_baud_param.segment1 = 16;
						// 	Can_baud_param.segment2 = 6;
						// }
						// else
						{
							//¡À¨¨??¦Ì?¨°a?¨®PCAN2¨¦?¨´¦Ì??a81.25%
							//???¡ã2¨¦?¨´¦Ì?¨¦¨¨??80%
							Can_baud_param.sync_jump_width = 4;
							Can_baud_param.bit_rate_prescale = clock_frequency(clk_src) / nomi_baud / 25 - 1 ;
							Can_baud_param.segment1 = 18;
							Can_baud_param.segment2 = 4;
						}
						baud_param = &Can_baud_param;
					}

                                        result = can_ops->baud(dev_can[id], 0, baud_param);
                                }

                                if(can_buf[id].can_mode == CAN_FD_MODE){
                                        /* Initial CAN fd*/
                                        can_ops->protocol_config(dev_can[id], &can_buf[id].protocol);

                                        baud_param = can_get_baud(clock_frequency(clk_src), data_baud);
                                        if (NULL == baud_param) {
                                                result = E_NOEXS;
                                                EMBARC_PRINTF("[%s] unsupported data bitrate %d!\r\n",__func__,data_baud);
                                        } else {
                                                can_baud_t Can_baud_param;
						if( CLOCK_100MHZ == clock_frequency(clk_src) )
						{
							// if( id == VCAN )
							{
								//¡À¨¨??¦Ì?¨°a?¨®VCAN2¨¦?¨´¦Ì??a70%
								//???¡ã2¨¦?¨´¦Ì?¨¦¨¨??72%
								Can_baud_param.sync_jump_width = 4;
								Can_baud_param.bit_rate_prescale = clock_frequency(clk_src) / data_baud / 25 - 1 ;
								Can_baud_param.segment1 = 16;
								Can_baud_param.segment2 = 6;
							}
							// else
							// {
							// 	//¡À¨¨??¦Ì?¨°a?¨®PCAN2¨¦?¨´¦Ì??a81.25%
							// 	//???¡ã2¨¦?¨´¦Ì?¨¦¨¨??80%
							// 	Can_baud_param.sync_jump_width = 4;
							// 	Can_baud_param.bit_rate_prescale = clock_frequency(clk_src) / data_baud / 25 - 1 ;
							// 	Can_baud_param.segment1 = 18;
							// 	Can_baud_param.segment2 = 4;
							// }
							baud_param = &Can_baud_param;
						}

                                                /* Setting the baud rate parameters of DATA bit */
                                                result = can_ops->baud(dev_can[id], 1, baud_param);
                                        }
                                }

                                if (E_OK == result) {
                                        result = can_int_install(id);
                                }
                        }
                }
        }

        if (E_OK == result) {
            result = can_ops->data_field_size(dev_can[id], 0, \
                    can_buf[id].rx_dfs, can_buf[id].rx_dfs);
            if (E_OK == result) {
                    result = can_ops->data_field_size(dev_can[id], 1, \
                            can_buf[id].tx_dfs, can_buf[id].tx_dfs);
            }
        }

#ifdef OS_FREERTOS
        if ((E_OK == result)) {
                if (DEV_XFER_INTERRUPT <= can_buf[id].data_tx_mode) {
                        if (can_buf[id].tx_queue == NULL) {
                                can_buf[id].tx_queue = xQueueCreate(5, 12);
                                if (NULL == can_buf[id].tx_queue) {
                                        result = E_SYS;
                                }
                        }
                }
        }

        if ((E_OK == result)) {
                if (DEV_XFER_INTERRUPT <= can_buf[id].data_rx_mode) {
                        if (can_buf[id].rx_queue == NULL) {
                                can_buf[id].rx_queue = xQueueCreate(5, 12);
                                if (NULL == can_buf[id].rx_queue) {
                                        result = E_SYS;
                                }
                        }
                }
        }

        if (E_OK == result) {
                if(can_sema[id] == NULL)
                {
                        can_sema[id] = xSemaphoreCreateBinary();
                }
                xSemaphoreGive(can_sema[id]);
        }
#else
        /* TODO: if need, add self-define queue scheme. */
#endif

        return result;
}

void can_deinit(uint32_t id)
{
#ifdef OS_FREERTOS
        if (can_buf[id].tx_queue != NULL) {
            vQueueDelete(can_buf[id].tx_queue);
            can_buf[id].tx_queue = NULL;
        }

        if (can_buf[id].rx_queue != NULL) {
            vQueueDelete(can_buf[id].rx_queue);
            can_buf[id].rx_queue = NULL;
        }
#endif

        /* Reset HW */
        if (id == 0) {
            CAN_0_reset(1);
            CAN_0_reset(0);
        } else {
            CAN_1_reset(1);
            CAN_1_reset(0);
        }
}


/* is_data_baud: 0 - nomi baud, 1 - data baud */
uint32_t _can_update_baud(uint32_t id, uint32_t is_data_baud)
{
    int32_t result = E_OK;
    clock_source_t clk_src;
    can_baud_t *baud_param = NULL;
    can_ops_t *can_ops = NULL;

    if (dev_can[id] == NULL || dev_can[id]->ops == NULL) {
        return E_PAR;
    }

    can_ops = (can_ops_t *)dev_can[id]->ops;

    clk_src = can_clock_source(id);
    EMBARC_PRINTF("[%s]: id(%d) is_data_baud(%d) nomi(%d) data(%d)\r\n", __func__, id, is_data_baud, can_buf[id].nomi_baud, can_buf[id].data_baud);
    if (is_data_baud) {
        baud_param = can_get_baud(clock_frequency(clk_src), can_buf[id].data_baud);
        can_baud_t Can_baud_param;
        if( CLOCK_100MHZ == clock_frequency(clk_src) )
        {
                // if( id == VCAN )
                {
                        //¡À¨¨??¦Ì?¨°a?¨®VCAN2¨¦?¨´¦Ì??a70%
                        //???¡ã2¨¦?¨´¦Ì?¨¦¨¨??72%
                        Can_baud_param.sync_jump_width = 4;
                        Can_baud_param.bit_rate_prescale = clock_frequency(clk_src) / can_buf[id].data_baud / 25 - 1 ;
                        Can_baud_param.segment1 = 16;
                        Can_baud_param.segment2 = 6;
                }
                // else
                // {
                //         //¡À¨¨??¦Ì?¨°a?¨®PCAN2¨¦?¨´¦Ì??a81.25%
                //         //???¡ã2¨¦?¨´¦Ì?¨¦¨¨??80%
                //         Can_baud_param.sync_jump_width = 4;
                //         Can_baud_param.bit_rate_prescale = clock_frequency(clk_src) / can_buf[id].data_baud / 25 - 1 ;
                //         Can_baud_param.segment1 = 18;
                //         Can_baud_param.segment2 = 4;
                // }
                baud_param = &Can_baud_param;
        }
    } else {
        can_baud_t Can_baud_param;
        if( CLOCK_100MHZ == clock_frequency(clk_src) )
        {
                // if( id == VCAN )
                // {
                //         //¡À¨¨??¦Ì?¨°a?¨®VCAN2¨¦?¨´¦Ì??a70%
                //         //???¡ã2¨¦?¨´¦Ì?¨¦¨¨??72%
                //         Can_baud_param.sync_jump_width = 4;
                //         Can_baud_param.bit_rate_prescale = clock_frequency(clk_src) / can_buf[id].nomi_baud / 25 - 1 ;
                //         Can_baud_param.segment1 = 16;
                //         Can_baud_param.segment2 = 6;
                // }
                // else
                {
                        //¡À¨¨??¦Ì?¨°a?¨®PCAN2¨¦?¨´¦Ì??a81.25%
                        //???¡ã2¨¦?¨´¦Ì?¨¦¨¨??80%
                        Can_baud_param.sync_jump_width = 4;
                        Can_baud_param.bit_rate_prescale = clock_frequency(clk_src) / can_buf[id].nomi_baud / 25 - 1 ;
                        Can_baud_param.segment1 = 18;
                        Can_baud_param.segment2 = 4;
                }
                baud_param = &Can_baud_param;
        }
        baud_param = can_get_baud(clock_frequency(clk_src), can_buf[id].nomi_baud);
    }
    if (NULL == baud_param) {
        EMBARC_PRINTF("[%s]: can_get_baud failed.\r\n", __func__);
        return E_NOEXS;
    }

    result = can_ops->baud(dev_can[id], is_data_baud, baud_param);

    return result;
}

int32_t can_get_config(uint32_t id, void *param)
{
        int32_t result = E_OK;
        uint32_t index = 0;
        can_config_t* can_conf = (can_config_t*)param;

        if (can_conf == NULL || dev_can[id] == NULL) {
            return E_PAR;
        }

        can_conf->can_mode                = can_buf[id].can_mode;
        can_conf->data_tx_mode            = can_buf[id].data_tx_mode;
        can_conf->nomi_baud               = can_buf[id].nomi_baud;
        can_conf->data_baud               = can_buf[id].data_baud;
        can_conf->can_busoff_recover_mode = can_buf[id].can_busoff_recover_mode;

        can_conf->std_id_filter.fiter_config.filter_size     = can_buf[id].filter.filter_size - 1;
        can_conf->std_id_filter.fiter_config.reject_no_match = can_buf[id].filter.reject_no_match;
        for (index = 0; index < can_buf[id].filter.filter_size; index++) {
            if(can_buf[id].filter_element[index].filter_cfg != eRX_BUF){
                can_conf->std_id_filter.filter_elem[index].filter_type = can_buf[id].filter_element[index].filter_type;
                can_conf->std_id_filter.filter_elem[index].filter_id0  = can_buf[id].filter_element[index].filter_id0;
                can_conf->std_id_filter.filter_elem[index].filter_id1  = can_buf[id].filter_element[index].filter_id1;
            }
        }

        can_conf->ext_id_filter.fiter_config.filter_size     = can_buf[id].ext_filter.filter_size - 1;
        can_conf->ext_id_filter.fiter_config.reject_no_match = can_buf[id].ext_filter.reject_no_match;
        for (index = 0; index < can_buf[id].ext_filter.filter_size; index++) {
            if(can_buf[id].ext_filter_element[index].filter_cfg != eRX_BUF){
                can_conf->ext_id_filter.filter_elem[index].filter_type = can_buf[id].ext_filter_element[index].filter_type;
                can_conf->ext_id_filter.filter_elem[index].filter_id0  = can_buf[id].ext_filter_element[index].filter_id0;
                can_conf->ext_id_filter.filter_elem[index].filter_id1  = can_buf[id].ext_filter_element[index].filter_id1;
            }
        }

        can_conf->eframe_format = can_buf[id].frame_parameters.eframe_format;
        can_conf->eframe_type   = can_buf[id].frame_parameters.eframe_type;

        return result;
}

/* mainly ID filter,can be used after can_init and can_get_config! */
int32_t can_set_config(uint32_t id, void *param)
{
        uint32_t index = 0;
        int32_t result = E_OK;
        can_config_t* can_conf = (can_config_t*)param;
        can_ops_t *can_ops = NULL;

        if (can_conf == NULL || dev_can[id] == NULL) {
            return E_PAR;
        }

        can_ops = (can_ops_t *)dev_can[id]->ops;
        if (NULL == can_ops) {
            return E_PAR;
        }

        CAN_LOCK(can_sema[id]);
        if (result == E_OK){
            /* data tx mode cfg*/
            can_buf[id].data_tx_mode = can_conf->data_tx_mode;

            /* baudrate cfg */
            can_buf[id].nomi_baud = can_conf->nomi_baud;
            can_buf[id].data_baud = can_conf->data_baud;
        }

        if (result == E_OK){
            /* can busoff recover mode cfg*/
            can_buf[id].can_busoff_recover_mode = can_conf->can_busoff_recover_mode;
        }

        /* can mode cfg */
        if (can_conf->can_mode) {
            if (result == E_OK) {
                can_buf[id].can_mode    = CAN_FD_MODE;
                can_buf[id].rx_dfs      = CAN_DATASIZE_2_DFS(CAN_FD_FRAM_BUF_DATA_SIZE);
                can_buf[id].tx_dfs      = CAN_DATASIZE_2_DFS(CAN_FD_FRAM_BUF_DATA_SIZE);

                can_buf[id].rx_max_id   = CAN_FD_FRAM_MAX_BUF_NUM-1;
                can_buf[id].tx_max_id   = 0;

                can_buf[id].protocol.fd_operation    = 1;
                can_buf[id].protocol.bit_rate_switch = 1;

                if (E_OK == result) {
                    result = can_ops->data_field_size(dev_can[id], 0, can_buf[id].rx_dfs, can_buf[id].rx_dfs);
                    if(result != E_OK){
                        EMBARC_PRINTF("[%s] can_fd rx_dfs config failed\r\n",__func__);
                    }
                }

                if (E_OK == result) {
                        result = can_ops->data_field_size(dev_can[id], 1, can_buf[id].tx_dfs, can_buf[id].tx_dfs);
                        if(result != E_OK){
                            EMBARC_PRINTF("[%s] can_fd tx_dfs config failed\r\n",__func__);
                        }
                }

                if (E_OK == result) {
                    result = can_ops->protocol_config(dev_can[id], &can_buf[id].protocol);
                    if(result != E_OK){
                        EMBARC_PRINTF("[%s] can_fd protocol config failed\r\n",__func__);
                    }
                }

            }

            if (result == E_OK) {
                can_buf[id].frame_parameters.len = CAN_FD_FRAM_BUF_DATA_SIZE;
                result = _can_update_baud(id, 0);
                result = _can_update_baud(id, 1);
                if(result != E_OK){
                    EMBARC_PRINTF("[%s] can_fd baudrate config failed\r\n",__func__);
                }
            }
        } else {
            if (result == E_OK) {
                can_buf[id].can_mode    = CAN_MODE;
                can_buf[id].rx_dfs      = CAN_DATASIZE_2_DFS(CAN_FRAM_BUF_DATA_SIZE);
                can_buf[id].tx_dfs      = CAN_DATASIZE_2_DFS(CAN_FRAM_BUF_DATA_SIZE);

                can_buf[id].rx_max_id   = CAN_FRAM_MAX_BUF_NUM-1;
                can_buf[id].tx_max_id   = 0;

                can_buf[id].protocol.fd_operation    = 0;
                can_buf[id].protocol.bit_rate_switch = 0;

                if (E_OK == result) {
                    result = can_ops->data_field_size(dev_can[id], 0, can_buf[id].rx_dfs, can_buf[id].rx_dfs);
                    if(result != E_OK){
                        EMBARC_PRINTF("[%s] can_fd rx_dfs config failed\r\n",__func__);
                    }
                }

                if (E_OK == result) {
                        result = can_ops->data_field_size(dev_can[id], 1, can_buf[id].tx_dfs, can_buf[id].tx_dfs);
                        if(result != E_OK){
                            EMBARC_PRINTF("[%s] can_fd tx_dfs config failed\r\n",__func__);
                        }
                }

                if (E_OK == result) {
                    result = can_ops->protocol_config(dev_can[id], &can_buf[id].protocol);
                    if(result != E_OK){
                        EMBARC_PRINTF("[%s] can protocol config failed\r\n",__func__);
                    }
                }

            }

            if (result == E_OK) {
                can_buf[id].frame_parameters.len = CAN_FRAM_BUF_DATA_SIZE;
                result = _can_update_baud(id, 0);
                if(result != E_OK){
                    EMBARC_PRINTF("[%s] can baudrate config failed\r\n",__func__);
                }
            }
        }

        /* standard filter cfg */
        if (result == E_OK) {
            if(can_conf->std_id_filter.fiter_config.filter_size < CAN_ID_FILTER_ELEMENT_NUM){
                can_buf[id].filter.frame_format    = eSTANDARD_FRAME;
                can_buf[id].filter.mask = 0x1ffffff;
                can_buf[id].filter.filter_size     = can_conf->std_id_filter.fiter_config.filter_size + 1;
                can_buf[id].filter.reject_no_match = can_conf->std_id_filter.fiter_config.reject_no_match;
                result = can_ops->id_filter_config(dev_can[id], &can_buf[id].filter);
                if(result != E_OK){
                    EMBARC_PRINTF("[%s] std filter config failed\r\n",__func__);
                }
            }else{
                result = E_PAR;
                EMBARC_PRINTF("[%s] the number of std_filters exceeds limit : %d\r\n",__func__, result);
            }
        }

        for (index = 0; index < can_conf->std_id_filter.fiter_config.filter_size; index++) {
            if (result == E_OK) {
                can_buf[id].filter_element[index].frame_format = eSTANDARD_FRAME;
                can_buf[id].filter_element[index].filter_type  = can_conf->std_id_filter.filter_elem[index].filter_type;
                can_buf[id].filter_element[index].filter_cfg   = eRX_FIFO;
                can_buf[id].filter_element[index].filter_id0   = can_conf->std_id_filter.filter_elem[index].filter_id0;
                can_buf[id].filter_element[index].filter_id1   = can_conf->std_id_filter.filter_elem[index].filter_id1;
                result = can_ops->id_filter_element(dev_can[id], index, &can_buf[id].filter_element[index]);
                if(result != E_OK){
                    EMBARC_PRINTF("[%s] std filter element config failed\r\n",__func__);
                }
            }
        }
        /* frames that do not need receive stroed in rx_buffer */
        if (result == E_OK) {
            can_buf[id].filter_element[index].frame_format = eSTANDARD_FRAME;
            can_buf[id].filter_element[index].filter_type  = eRANGE;
            can_buf[id].filter_element[index].filter_cfg   = eRX_BUF;
            can_buf[id].filter_element[index].filter_id0   = 0x0;
            can_buf[id].filter_element[index].filter_id1   = 0x7ff;
            result = can_ops->id_filter_element(dev_can[id], index, &can_buf[id].filter_element[index]);
            if(result != E_OK){
                EMBARC_PRINTF("[%s] std buf filter element config failed\r\n",__func__);
            }
        }

        /* extended filter cfg */
        if (result == E_OK) {
            if (can_conf->ext_id_filter.fiter_config.filter_size < CAN_ID_FILTER_ELEMENT_NUM) {
                can_buf[id].ext_filter.frame_format    = eEXTENDED_FRAME;
                can_buf[id].ext_filter.mask = 0x1fffffff;
                can_buf[id].ext_filter.filter_size     = can_conf->ext_id_filter.fiter_config.filter_size + 1;
                can_buf[id].ext_filter.reject_no_match = can_conf->ext_id_filter.fiter_config.reject_no_match;
                result = can_ops->id_filter_config(dev_can[id], &can_buf[id].ext_filter);
                if(result != E_OK){
                    EMBARC_PRINTF("[%s] ext filter config failed\r\n",__func__);
                }
            } else {
                result = E_PAR;
                EMBARC_PRINTF("[%s] the number of ext_filters exceeds limit : %d\r\n",__func__, result);
            }
        }

        for (index = 0; index < can_conf->ext_id_filter.fiter_config.filter_size; index++) {
            if (result == E_OK) {
                can_buf[id].ext_filter_element[index].frame_format = eEXTENDED_FRAME;
                can_buf[id].ext_filter_element[index].filter_type  = can_conf->ext_id_filter.filter_elem[index].filter_type;
                can_buf[id].ext_filter_element[index].filter_cfg   = eRX_FIFO;
                can_buf[id].ext_filter_element[index].filter_id0   = can_conf->ext_id_filter.filter_elem[index].filter_id0;
                can_buf[id].ext_filter_element[index].filter_id1   = can_conf->ext_id_filter.filter_elem[index].filter_id1;
                result = can_ops->id_filter_element(dev_can[id], index, &can_buf[id].ext_filter_element[index]);
                if(result != E_OK){
                    EMBARC_PRINTF("[%s] ext filter element config failed\r\n",__func__);
                }
            }
        }
        /* frames that do not need receive stroed in rx_buffer */
        if (result == E_OK) {
            can_buf[id].ext_filter_element[index].frame_format = eEXTENDED_FRAME;
            can_buf[id].ext_filter_element[index].filter_type  = eRANGE;
            can_buf[id].ext_filter_element[index].filter_cfg   = eRX_BUF;
            can_buf[id].ext_filter_element[index].filter_id0   = 0x0;
            can_buf[id].ext_filter_element[index].filter_id1   = 0x1fffffff;
            result = can_ops->id_filter_element(dev_can[id], index, &can_buf[id].ext_filter_element[index]);
            if(result != E_OK){
                EMBARC_PRINTF("[%s] ext buf filter element config failed\r\n",__func__);
            }
        }

        /* tx frame format cfg */
        if (result == E_OK){
            can_buf[id].frame_parameters.eframe_format = can_conf->eframe_format;
            can_buf[id].frame_parameters.eframe_type   = can_conf->eframe_type;
        }

        CAN_UNLOCK(can_sema[id]);

        return result;
}

static int32_t can_get_buf(uint32_t id, void *param)
{
        int32_t result = E_OK;

        if((id >= 2) ||(NULL == param)){
                result = E_PAR;
        }else{
                memcpy(param, &can_buf[id], sizeof(can_xfer_t));
        }

        return result;
}

static int32_t can_set_buf(uint32_t id, void *param)
{
        int32_t result = E_OK;

        if((id >= 2) ||(NULL == param)){
                result = E_PAR;
        }else{
                if(((can_xfer_t *)param)->data_baud == 0){
                    memcpy(&can_mode_default, (can_xfer_t *)param, sizeof(can_xfer_t));
                }else{
                    memcpy(&can_fd_mode_default_1, (can_xfer_t *)param, sizeof(can_xfer_t));
                }
        }

        return result;
}

static int32_t can_internal_reset(uint32_t id)
{
        int32_t result = E_OK;
        can_xfer_t can_config;

        if (id > 1) {
                result = E_PAR;
        }

        if(E_OK == result){

                /*can_buf copy to can_mode_default*/
                can_get_buf(id, &can_config);
                can_set_buf(id, &can_config);

                result = can_init(id, can_buf[id].nomi_baud, can_buf[id].data_baud);

                /* Enable the RX interrupt */
                can_interrupt_enable(id, 0, 1);
                /* Enable the tX interrupt */
                can_interrupt_enable(id, 1, 1);
                /* Enable the error interrupt */
                can_interrupt_enable(id, 3, 1);

                EMBARC_PRINTF("[%s] bus id : %d, can reset!\r\n", __func__, id);

                can_buf[id].rx_isr_free_counter = 0;
                can_node_rec_flag[id] = 0;
                can_tx_done_flag[id] = 1;

        }

        return result;
}

int32_t can_indication_register(uint32_t dev_id, rx_indication_callback func)
{
    int32_t result = E_OK;

    if ((dev_id >= 2) || (NULL == func)) {
            result = E_PAR;
    } else {
            uint32_t cpu_status = arc_lock_save();
            can_buf[dev_id].rx_indication = func;
            arc_unlock_restore(cpu_status);
    }

    return result;
}

int32_t can_confirmation_register(uint32_t dev_id, tx_confirmation_callback func)
{
    int32_t result = E_OK;

    if ((dev_id >= 2) || (NULL == func)) {
            result = E_PAR;
    } else {
            uint32_t cpu_status = arc_lock_save();
            can_buf[dev_id].tx_confirmation = func;
            arc_unlock_restore(cpu_status);
    }

    return result;
}

int32_t can_error_event_register(uint32_t dev_id, error_indication_callback func)
{
    int32_t result = E_OK;

    if ((dev_id >= 2) || (NULL == func)) {
            result = E_PAR;
    } else {
            uint32_t cpu_status = arc_lock_save();
            can_buf[dev_id].err_indication = func;
            arc_unlock_restore(cpu_status);
    }

    return result;
}

#ifdef OS_FREERTOS
int32_t can_queue_install(uint32_t id, QueueHandle_t queue, uint32_t rx_or_tx)
{
        int32_t result = E_OK;

        if ((id >= 2) || (NULL == queue)) {
                result = E_PAR;
        } else {
                if (rx_or_tx) {
                        can_buf[id].tx_queue = queue;
                } else {
                        can_buf[id].rx_queue = queue;
                }
        }

        return result;
}
#endif

void *can_xfer_buffer(uint32_t id, uint32_t rx_or_tx)
{
        void *buf = NULL;

        if (id < 2) {
                if (rx_or_tx) {
                        buf = (void *)((uint32_t)&can_buf[id].tx_buf);
                } else {
                        buf = (void *)((uint32_t)&can_buf[id].rx_buf);
                }
        }

        return buf;
}

int32_t can_read(uint32_t id, uint32_t *buf, uint32_t len)
{
        int32_t result = E_OK;

        uint32_t cpu_sts = 0;

        uint32_t frame_size = 0;
        dev_xfer_mode_t xfer_mode = DEV_XFER_POLLING;

        can_ops_t *can_ops = (can_ops_t *)dev_can[id]->ops;

        CAN_LOCK(can_sema[id]);
        if ((id >= 2) || (NULL == dev_can[id]) || (NULL == can_ops) || \
                (NULL == buf) || (0 == len)) {
                result = E_PAR;
        } else {
                frame_size = CAN_DFS_2_FRAMESIZE(can_buf[id].rx_dfs);
                if (len % frame_size) {
                        result = E_PAR;
                } else {
                        xfer_mode = can_buf[id].data_rx_mode;
                }
        }

        if (E_OK == result) {
            uint32_t buf_id = can_buf[id].rx_last_id;

            switch (xfer_mode) {
            case DEV_XFER_POLLING:
                    while (len) {
                            result = can_ops->read_frame_blocked(dev_can[id], \
                                    buf_id, buf, frame_size);
                            if (E_OK != result) {
                                    break;
                            } else {
                                    buf_id += 1;
                                    if (buf_id > 63) {
                                            buf_id = 0;
                                    }

                                    buf += frame_size;
                                    len -= frame_size << 2;
                            }
                    }
                    can_buf[id].rx_last_id = buf_id;
                    break;

           case DEV_XFER_INTERRUPT:
                    cpu_sts = arc_lock_save();
                    if (can_buf[id].rx_len > can_buf[id].rx_ofs) {
#ifdef OS_FREERTOS
                            //result = io_queue_send(can_buf[id].rx_queue, (void *)buf, len, 0);
#else
                            /* TODO: if need, add self-define queue scheme here! */
#endif
                    } else {
                            can_buf[id].rx_buf = buf;
                            can_buf[id].rx_len = len;
                            can_buf[id].rx_ofs = 0;
                            /* re-enable rx interrupt. */
                            result = _can_interrupt_enable(dev_can[id], eCAN_INT_TYPE_RX, eCAN_INT_ENABLE, can_buf[id].mode);
                    }
                    arc_unlock_restore(cpu_sts);
                    break;

           case DEV_XFER_DMA:
                    /* TODO: don't support in current platform! */
                    break;
           case DEV_XFER_TASK_AGENCY:
                    /* nothing to do, taks agency will push data. */
                    break;

           default:
                    result = E_SYS;
                    break;
            }
        }
        CAN_UNLOCK(can_sema[id]);

        return result;
}

int32_t can_frame_read(uint32_t dev_id, uint32_t *msg_id, uint32_t **data)
{
        int32_t result = E_OK;

        if ((dev_id >= 2) || (NULL == msg_id) || (NULL == data)) {
                result = E_PAR;
        } else {
#ifdef OS_FREERTOS
                DEV_BUFFER message;
                if(pdTRUE != xQueueReceive(can_buf[dev_id].rx_queue,\
                            (void *)&message, 0)) {
                        result = E_SYS;
                } else {
                        result = message.len;
                        *data = (uint32_t *)message.buf;
                        *msg_id = message.ofs;
                }
#endif
        }
        return result;
}


int32_t can_frame_params_setup(uint32_t id, can_frame_params_t *params)
{
        int32_t result = E_OK;

        uint32_t cpu_sts = arc_lock_save();
        if ((id >= 2) || (NULL == params)) {
                result = E_PAR;
        } else {
                can_buf[id].frame_parameters.msg_id = params->msg_id;
                can_buf[id].frame_parameters.eframe_format = params->eframe_format;
                can_buf[id].frame_parameters.eframe_type = params->eframe_type;
                can_buf[id].frame_parameters.len = params->len;
        }
        arc_unlock_restore(cpu_sts);

        return result;
}


uint32_t can_is_busy(uint32_t id)
{
    uint32_t ret = 0;
    can_ops_t *can_ops = (can_ops_t *)dev_can[id]->ops;

    ret = can_ops->tx_buf_add_request_status(dev_can[id], can_buf[id].tx_last_id);
    if (ret) {
        return E_DBUSY;
    }
    return E_OK;
}

int32_t can_send_data(uint32_t bus_id, uint32_t frame_id, uint32_t *data, uint32_t len)
{
        int32_t result = E_OK;
        uint32_t data_len = 0;

        if(assert_reset_flag[bus_id] == 1){
            result = E_CLSED;
            return result;
        }

        if(exc_sense()){
                result = E_ILUSE;
                EMBARC_PRINTF("[%s] warning! this function should be used in task!\r\n", __func__);
                return result;
        }

        if ((NULL == data)) {
                return E_NODATA;
        }

        if (can_buf[bus_id].can_mode == CAN_MODE) {
                data_len = CAN_FRAM_BUF_DATA_SIZE;
        }else{
                data_len = CAN_FD_FRAM_BUF_DATA_SIZE;
        }

        if ((len <= 8)  || (len == 16) || (len == 32) || (len == 64) || (len % data_len == 0) || \
            (len == 12) || (len == 20) || (len == 24) || (len == 48)){
                /* Acceptable Length.*/
        } else {
                /* The parameter len must be equal to possible DLC or an integer multiple of CAN_FRAM_BUF_DATA_SIZE.*/
                EMBARC_PRINTF("[%s] length is incorrect!\r\n", __func__);
                return E_QOVR;
        }

        /* Asynchronous mode, do not wait for tx_done signal.
         * judge whether the previous transmission is completed before transmission */
        if(can_buf[bus_id].data_tx_mode == DEV_XFER_INTERRUPT){
                result = can_is_busy(bus_id);
                if ((E_DBUSY == result) || (0 == can_tx_done_flag[bus_id])) {
                        //EMBARC_PRINTF("[%s] can is busy!\r\n", __func__);
                        return E_DBUSY;
                }
                result = can_write(bus_id, frame_id, data, len, can_buf[bus_id].frame_parameters.eframe_format);
        }
        /* Synchronous mode, waiting for tx_done signal */
        else{
                result = can_write(bus_id, frame_id, data, len, can_buf[bus_id].frame_parameters.eframe_format);
        }

        return result;

}

/* used for tx_int mode only */
int32_t can_send_data_isr(uint32_t bus_id, uint32_t frame_id, uint32_t *data, uint32_t len)
{
        int32_t result = E_OK;
        uint32_t data_len = 0;

        if(assert_reset_flag[bus_id] == 1){
            result = E_CLSED;
            return result;
        }

        if(exc_sense() == 0){
                result = E_ILUSE;
                EMBARC_PRINTF("[%s] warning! this function should be used in interrupt!\r\n", __func__);
                return result;
        }

        if ((NULL == data)) {
                return E_NODATA;
        }

        if (can_buf[bus_id].can_mode == CAN_MODE) {
                data_len = CAN_FRAM_BUF_DATA_SIZE;
        }else{
                data_len = CAN_FD_FRAM_BUF_DATA_SIZE;
        }

        if ((len <= 8)  || (len == 16) || (len == 32) || (len == 64) || (len % data_len == 0) || \
            (len == 12) || (len == 20) || (len == 24) || (len == 48)){
                /* Acceptable Length.*/
        } else {
                /* The parameter len must be equal to possible DLC or an integer multiple of CAN_FRAM_BUF_DATA_SIZE.*/
                EMBARC_PRINTF("[%s] length is incorrect!\r\n", __func__);
                return E_QOVR;
        }

        result = can_is_busy(bus_id);
        if (E_DBUSY == result) {
                EMBARC_PRINTF("[%s] can is busy!\r\n", __func__);
                return E_DBUSY;
        }

        result = can_write_isr(bus_id, frame_id, data, len, can_buf[bus_id].frame_parameters.eframe_format);

        return result;

}


static int32_t can_send_frame(can_t *dev_can, can_ops_t *dev_ops, can_xfer_t *xfer)
{
        int32_t result = E_OK;
        uint32_t header0 = 0, header1 = 0;
        uint32_t idx, cnt = 0,cnt_1 = 0, tx_cnt = 0, remain_size = 0;
        uint32_t *data_ptr = NULL;
        uint32_t frame_buf[CAN_DFS_2_FRAMESIZE(eDATA_LEN_64)];
        int32_t dlc = 0;            /* dlc: data length code */
        int32_t dfw_cnt = 0;        /* dfw: data fileld by word */
        uint32_t buf_element_size = 0; /* Unit: Word */
        can_ops_t *can_ops = NULL;
        uint32_t buf_id = 0;
        uint32_t data_len = 0;

        if(exc_sense()){
                result = E_ILUSE;
                EMBARC_PRINTF("[%s] warning! this function should be used in task!\r\n", __func__);
                return result;
        }

        can_ops = (can_ops_t *)dev_can->ops;

        header0 = xfer->frame_parameters.msg_id & 0x1FFFFFFF;
        if (eSTANDARD_FRAME == xfer->frame_parameters.eframe_format) {
                /* standard frame. */
                header0 = (xfer->frame_parameters.msg_id << 18);
        } else {
                /* extended frame. */
                header0 |= (1 << 30);
        }

        if (eREMOTE_FRAME == xfer->frame_parameters.eframe_type) {
                /* remote frame. */
                header0 |= (1 << 29);
        }

        /*When tx length < CAN_FRAM_BUF_DATA_SIZE, DLC = tx length;
         *when tx length >= CAN_FRAM_BUF_DATA_SIZE, DLC = CAN_FRAM_BUF_DATA_SIZE.
         */
        if(xfer->can_mode == CAN_MODE){
                data_len = CAN_FRAM_BUF_DATA_SIZE;
        }else{
                data_len = CAN_FD_FRAM_BUF_DATA_SIZE;
        }

        if (xfer->tx_len < data_len) {
                dlc = can_get_dlc(xfer->can_mode, xfer->tx_len);
        }else{
                dlc = can_get_dlc(xfer->can_mode, xfer->frame_parameters.len);
        }

        if (dlc < 0) {
                EMBARC_PRINTF("can_send_frame length error!\r\n");
                return E_PAR;
        }

        header1 |= (((dlc & 0xF) << 16) | (xfer->protocol.bit_rate_switch << 20) | \
                     (xfer->protocol.fd_operation << 21));

        /* Fill frame_buf data with header0 and header1 */
        frame_buf[0] = header0;
        frame_buf[1] = header1;
        buf_element_size = CAN_DFS_2_FRAMESIZE(xfer->tx_dfs);

        /* send one frame at a time */
        /* tx_len is the total length that need to be transferred */
        /* tx_ofs is the offset number to indicate the length that already transferred */
        /* remain_size is the reamining length that still need to be transferred */
        remain_size = xfer->tx_len - xfer->tx_ofs;

        do {
                if (eDATA_FRAME == xfer->frame_parameters.eframe_type) {
                        if (remain_size > xfer->frame_parameters.len) {
                                tx_cnt = xfer->frame_parameters.len;
                        } else {
                                tx_cnt = remain_size;
                        }

                        data_ptr = (uint32_t *)xfer->tx_buf;
                        /* Change the unit length of tx_ofs from uint32_t to uint_8 by ">>2"  */
                        /* data_ptr should be incremented by BYTE unit */
                        data_ptr += (xfer->tx_ofs >> 2);

                        /* Fill frame_buf data after header0 and header1 */
                        /* idx start from 2 because frame_buf[0] and frame_buf[1] are header0 and header1 */
                        /* Change the unit length of tx_cnt from uint32_t to uint_8 by ">>2"  */
                        /* idx should be incremented by BYTE unit */
                        if (tx_cnt < eDATA_LEN_8) {
                                dfw_cnt = 2;
                        } else {
                                dfw_cnt = (tx_cnt >> 2);
                        }

                        for (idx = 2; idx < (dfw_cnt + 2); idx++) {
                                frame_buf[idx] = *data_ptr++;
                        }
                }

                if (can_buf[dev_can->id].mode == eCAN_BUFFER_MODE) {
                        buf_id = xfer->tx_last_id;
                }

#ifdef CAN_TX_COMPLETE_MONITOR
                xfer->rx_isr_free_counter = 0;
#endif
                /* Asynchronous mode */
                if(xfer->data_tx_mode == DEV_XFER_INTERRUPT){
                        if (dev_can->id == 0){
                                if (dw_timer_start(3, 50000000, timeout_handler, (void *)(&can_exa)) != E_OK){
                                    EMBARC_PRINTF("[%s] start timer3 failed \r\n",__func__);
                                }
                        }else{
                                if(dw_timer_start(2, 50000000, timeout_handler, (void *)(&can_exa_1)) != E_OK){
                                    EMBARC_PRINTF("[%s] start timer2 failed \r\n",__func__);
                                }
                        }

                        uint32_t cpu_status = arc_lock_save();
                        result = can_ops->write_frame(dev_can, buf_id, frame_buf, buf_element_size);
                        if (E_OK == result) {
                                /* Increment tx_ofs with tx_cnt */
                                xfer->tx_ofs += tx_cnt;
                                if (can_buf[dev_can->id].mode == eCAN_BUFFER_MODE) {
                                        buf_id += 1;
                                        if (buf_id > xfer->tx_max_id) {
                                                buf_id = 0;
                                        }
                                        xfer->tx_last_id = buf_id;
                                }
                        }
                        arc_unlock_restore(cpu_status);

                        if (E_OK != result) {
                            return result;
                        }
                        /* For Int mode, only need to send the first frame to trigger can tx interrupt */
                        /* The remaining data send will be implement in can Tx interrupt handler : _can_tx_handler */
                        break;
                }
                /* Synchronous mode*/
                else{
                        uint32_t cpu_status = arc_lock_save();
                        /* Send check */
                        for (cnt = 0; cnt < 200; cnt++) {
                                result = can_ops->write_frame(dev_can, buf_id, frame_buf, buf_element_size);
                                if (E_DBUSY == result) {
                                        chip_hw_udelay(100);
                                } else if (E_OK == result) {
                                        break;
                                }
                        }
                        arc_unlock_restore(cpu_status);
                        if (E_OK != result) {
                                // EMBARC_PRINTF("[%s] send request failed \r\n",__func__);
                                return result;
                        }
                        

                        /* Send finish check */
                        for (cnt_1 = 0; cnt_1 < 800; cnt_1++) {
                                if(assert_reset_flag[dev_can->id] == 1){
                                    result = E_CLSED;
                                    break;
                                }else if(can_tx_done_flag[dev_can->id] == 0){
                                    chip_hw_udelay(100);
                                    result = E_DBUSY;
                                }else{
                                    result = E_OK;
                                    /* Increment tx_ofs with tx_cnt */
                                    xfer->tx_ofs += tx_cnt;
                                    if (can_buf[dev_can->id].mode == eCAN_BUFFER_MODE) {
                                            buf_id += 1;
                                            if (buf_id > xfer->tx_max_id) {
                                                    buf_id = 0;
                                            }
                                            xfer->tx_last_id = buf_id;
                                    }
                                    /* For Polling mode, data send will be implemented frame by frame until all the data have been sent : remain_size == 0 */
                                    remain_size = xfer->tx_len - xfer->tx_ofs;
                                    break;
                                }
                        }
                        if (E_OK != result) {
                                // EMBARC_PRINTF("[%s] send finish failed \r\n",__func__);
                                return result;
                        }
                }
        }while(remain_size);

    return result;
}

static int32_t can_send_frame_isr(can_t *dev_can, can_ops_t *dev_ops, can_xfer_t *xfer)
{
        int32_t result = E_OK;
        uint32_t header0 = 0, header1 = 0;
        uint32_t idx, tx_cnt = 0, remain_size = 0;
        uint32_t *data_ptr = NULL;
        uint32_t frame_buf[CAN_DFS_2_FRAMESIZE(eDATA_LEN_64)];
        int32_t dlc = 0;            /* dlc: data length code */
        int32_t dfw_cnt = 0;        /* dfw: data fileld by word */
        uint32_t buf_element_size = 0; /* Unit: Word */
        can_ops_t *can_ops = NULL;
        uint32_t buf_id = 0;
        uint32_t data_len = 0;

        if(exc_sense() == 0){
                result = E_ILUSE;
                EMBARC_PRINTF("[%s] warning! this function should be used in interrupt!\r\n", __func__);
                return result;
        }

        can_ops = (can_ops_t *)dev_can->ops;

        header0 = xfer->frame_parameters.msg_id & 0x1FFFFFFF;
        if (eSTANDARD_FRAME == xfer->frame_parameters.eframe_format) {
                /* standard frame. */
                header0 = (xfer->frame_parameters.msg_id << 18);
        } else {
                /* extended frame. */
                header0 |= (1 << 30);
        }

        if (eREMOTE_FRAME == xfer->frame_parameters.eframe_type) {
                /* remote frame. */
                header0 |= (1 << 29);
        }

        /*When tx length < CAN_FRAM_BUF_DATA_SIZE, DLC = tx length;
         *when tx length >= CAN_FRAM_BUF_DATA_SIZE, DLC = CAN_FRAM_BUF_DATA_SIZE.
         */
        if(xfer->can_mode == CAN_MODE){
                data_len = CAN_FRAM_BUF_DATA_SIZE;
        }else{
                data_len = CAN_FD_FRAM_BUF_DATA_SIZE;
        }

        if (xfer->tx_len < data_len) {
                dlc = can_get_dlc(xfer->can_mode, xfer->tx_len);
        }else{
                dlc = can_get_dlc(xfer->can_mode, xfer->frame_parameters.len);
        }

        if (dlc < 0) {
                EMBARC_PRINTF("can_send_frame length error!\r\n");
                return E_PAR;
        }

        header1 |= (((dlc & 0xF) << 16) | (xfer->protocol.bit_rate_switch << 20) | \
                     (xfer->protocol.fd_operation << 21));

        /* Fill frame_buf data with header0 and header1 */
        frame_buf[0] = header0;
        frame_buf[1] = header1;
        buf_element_size = CAN_DFS_2_FRAMESIZE(xfer->tx_dfs);

        /* send one frame at a time */
        /* tx_len is the total length that need to be transferred */
        /* tx_ofs is the offset number to indicate the length that already transferred */
        /* remain_size is the reamining length that still need to be transferred */
        remain_size = xfer->tx_len - xfer->tx_ofs;

        do {
                if (eDATA_FRAME == xfer->frame_parameters.eframe_type) {
                        if (remain_size > xfer->frame_parameters.len) {
                                tx_cnt = xfer->frame_parameters.len;
                        } else {
                                tx_cnt = remain_size;
                        }

                        data_ptr = (uint32_t *)xfer->tx_buf;
                        /* Change the unit length of tx_ofs from uint32_t to uint_8 by ">>2"  */
                        /* data_ptr should be incremented by BYTE unit */
                        data_ptr += (xfer->tx_ofs >> 2);


                        /* Fill frame_buf data after header0 and header1 */
                        /* idx start from 2 because frame_buf[0] and frame_buf[1] are header0 and header1 */
                        /* Change the unit length of tx_cnt from uint32_t to uint_8 by ">>2"  */
                        /* idx should be incremented by BYTE unit */
                        if (tx_cnt < eDATA_LEN_8) {
                                dfw_cnt = 2;
                        } else {
                                dfw_cnt = (tx_cnt >> 2);
                        }

                        for (idx = 2; idx < (dfw_cnt + 2); idx++) {
                                frame_buf[idx] = *data_ptr++;
                        }
                }

                if (can_buf[dev_can->id].mode == eCAN_BUFFER_MODE) {
                        buf_id = xfer->tx_last_id;
                }

#ifdef CAN_TX_COMPLETE_MONITOR
                xfer->rx_isr_free_counter = 0;
#endif
                if (dev_can->id == 0){
                        can_exa.dev_can_s = dev_can;
                        can_exa.can_buf_s = xfer;
                        if (dw_timer_start(3, 50000000, timeout_handler, (void *)(&can_exa)) != E_OK){
                            EMBARC_PRINTF("[%s] start timer3 failed \r\n",__func__);
                        }
                }else{
                        can_exa_1.dev_can_s = dev_can;
                        can_exa_1.can_buf_s = xfer;
                        if(dw_timer_start(2, 50000000, timeout_handler, (void *)(&can_exa_1)) != E_OK){
                            EMBARC_PRINTF("[%s] start timer2 failed \r\n",__func__);
                        }
                }

                 result = can_ops->write_frame(dev_can, buf_id, frame_buf, buf_element_size);
                 if (E_OK == result) {
                         /* Increment tx_ofs with tx_cnt */
                         xfer->tx_ofs += tx_cnt;
                         if (can_buf[dev_can->id].mode == eCAN_BUFFER_MODE) {
                                 buf_id += 1;
                                 if (buf_id > xfer->tx_max_id) {
                                         buf_id = 0;
                                 }
                                 xfer->tx_last_id = buf_id;
                         }
                 }

                if (E_OK != result) {
                    break;
                }
                if(xfer->data_tx_mode == DEV_XFER_INTERRUPT){
                    /* For Int mode, only need to send the first frame to trigger can tx interrupt */
                    /* The remaining data send will be implement in can Tx interrupt handler : _can_tx_handler */
                    break;
                }
                /* For Polling mode, data send will be implemented frame by frame until all the data have been sent : remain_size == 0 */
                remain_size = xfer->tx_len - xfer->tx_ofs;

        }while(remain_size);


    return result;
}


int32_t can_write(uint32_t id, uint32_t frame_id, uint32_t *buf, uint32_t len, uint32_t flag)
{
        int32_t result = E_OK;
        dev_xfer_mode_t xfer_mode = can_buf[id].data_tx_mode;
        can_ops_t *can_ops = NULL;

        can_ops = (can_ops_t *)dev_can[id]->ops;
        if ((id >= 2) || (NULL == dev_can[id]) || (NULL == can_ops) || \
            (NULL == buf) || (0 == len)) {
                result = E_PAR;
        }

        /* CAN_LOCK here to avoid multi-task access identical can port  */
        CAN_LOCK(can_sema[id]);

        /* save frame params to global varible */
        can_buf[id].frame_parameters.msg_id = frame_id;

        if (E_OK == result) {
                switch (xfer_mode) {
                /* sync mode */
                case DEV_XFER_POLLING:
                        result = can_install_data_buffer(id, buf, len, 1);
                        result = _can_interrupt_enable(dev_can[id], eCAN_INT_TYPE_TX, eCAN_INT_ENABLE, can_buf[id].mode);
                        result = can_send_frame(dev_can[id], can_ops, &can_buf[id]);
                        if (E_OK != result) {
                                EMBARC_PRINTF("[%s] CAN Polling Mode Transfer failed! result = %d\r\n", __func__, result);
                               _can_interrupt_enable(dev_can[id], eCAN_INT_TYPE_TX, eCAN_INT_DISABLE, can_buf[id].mode);
                               if(E_DBUSY == result){
                                      /* Reset can */
                                      can_internal_reset(id);
                                      /* can resend frame */
                                      result = can_send_frame(dev_can[id], can_ops, &can_buf[id]);
                               }
                        }
                        break;
                /* Asynchronous mode */
                case DEV_XFER_INTERRUPT:
                        if (can_buf[id].tx_len > can_buf[id].tx_ofs) {
                                /* The previous data send has NOT finished    */
                                /* Theoretically, by the increment of "tx_ofs", finally, "tx_ofs" should equle to "tx_len"
                                   which means the previous data send is finished, then a new round of data send can be started */
                                EMBARC_PRINTF("[%s] CAN Int Mode Bus Busy! result = %d\r\n", __func__,result);
                                result = E_DBUSY;
                        } else {
                                result = can_install_data_buffer(id, buf, len, 1);
                                result = _can_interrupt_enable(dev_can[id], eCAN_INT_TYPE_TX, eCAN_INT_ENABLE, can_buf[id].mode);

                                result = can_send_frame(dev_can[id], can_ops, &can_buf[id]);
                                if (E_DBUSY == result) {
                                        /* the sending status of the first frame is busy, need to disable interrupt */
                                        EMBARC_PRINTF("[%s] CAN Int Mode first frame Data Send Busy! result= %d\r\n", __func__,result);
                                        _can_interrupt_enable(dev_can[id], eCAN_INT_TYPE_TX, eCAN_INT_DISABLE, can_buf[id].mode);
                                        can_reset_data_buffer(&can_buf[id]);
                                }
                        }
                        break;

                case DEV_XFER_DMA:
                        /* NOT support in current platform! */
                        result = E_SYS;
                        break;

                case DEV_XFER_TASK_AGENCY:
                        /* TODO: if need, add self-define queue scheme at here. */
                        break;

                default:
                        result = E_SYS;
                        break;
            }
        }

        CAN_UNLOCK(can_sema[id]);

        return result;
}
int32_t can_write_isr(uint32_t id, uint32_t frame_id, uint32_t *buf, uint32_t len, uint32_t flag)
{
        int32_t result = E_OK;
        dev_xfer_mode_t xfer_mode = can_buf[id].data_tx_mode;
        can_ops_t *can_ops = NULL;

        can_ops = (can_ops_t *)dev_can[id]->ops;

        if ((id >= 2) || (NULL == dev_can[id]) || (NULL == can_ops) || \
            (NULL == buf) || (0 == len)) {
                result = E_PAR;
                EMBARC_PRINTF("[%s] E_PAR !\r\n", __func__);
        }

        /* save frame params to global varible */
        can_buf[id].frame_parameters.msg_id = frame_id;

        if (E_OK == result) {
                switch (xfer_mode) {
                case DEV_XFER_POLLING:
                        /* NOT support in current platform! */
                        EMBARC_PRINTF("[%s] ISR Mode not support Polling Sync mode!\r\n",__func__);
                        result = E_SYS;
                        break;
                case DEV_XFER_INTERRUPT:
                        if (can_buf[id].tx_len > can_buf[id].tx_ofs) {
                                /* The previous data send has NOT finished    */
                                /* Theoretically, by the increment of "tx_ofs", finally, "tx_ofs" should equle to "tx_len"
                                   which means the previous data send is finished, then a new round of data send can be started */
                                EMBARC_PRINTF("[%s] ISR CAN Int Mode Bus Busy!\r\n",__func__);
                                result = E_DBUSY;
                        } else {
                                result = can_install_data_buffer(id, buf, len, 1);
                                result = _can_interrupt_enable(dev_can[id], eCAN_INT_TYPE_TX, eCAN_INT_ENABLE, can_buf[id].mode);
                                result = can_send_frame_isr(dev_can[id], can_ops, &can_buf[id]);
                                if (E_DBUSY == result) {
                                        /* the sending status of the first frame is busy, need to disable interrupt */
                                        EMBARC_PRINTF("[%s] CAN Int Mode ISR first frame Data Send Busy!\r\n",__func__);
                                        _can_interrupt_enable(dev_can[id], eCAN_INT_TYPE_TX, eCAN_INT_DISABLE, can_buf[id].mode);
                                        can_reset_data_buffer(&can_buf[id]);
                                }
                        }
                        break;

                case DEV_XFER_DMA:
                        /* NOT support in current platform! */
                        result = E_SYS;
                        break;

                case DEV_XFER_TASK_AGENCY:
                        /* TODO: if need, add self-define queue scheme at here. */
                        break;
                default:
                        result = E_SYS;
                        break;
            }
        }

        return result;
}

/* This function will be called only once at the beginning of the CAN data send */
static int32_t can_install_data_buffer(uint32_t id, uint32_t *buf, uint32_t len, uint32_t rx_or_tx)
{
        int32_t result = E_OK;
        if ((id >= 2) || (NULL == dev_can[id]) || (NULL == buf) || (0 == len)) {
                result = E_PAR;
        }

        /* make data address 4 byte align, or it will cause system crash*/
        if((len <= eDATA_LEN_64) && ((uint32_t)buf % sizeof(uint32_t) != 0)){
            int buf_idx = 0;
            uint8_t *temp_buf = (uint8_t *)buf;
            for(buf_idx = 0; buf_idx < len; buf_idx++){
                can_sw_buf[buf_idx] = temp_buf[buf_idx];
            }

            /* lock cou here since can_buf is a global parameter */
            uint32_t cpu_status = arc_lock_save();

            if (rx_or_tx) {
                    /* CAN TX Data Install */
                    can_buf[id].tx_buf = can_sw_buf;
                    can_buf[id].tx_len = len;
                    can_buf[id].tx_ofs = 0;
            } else {
                    /* CAN RX Data Install */
                    can_buf[id].rx_buf = can_sw_buf;
                    can_buf[id].rx_len = len;
                    can_buf[id].rx_ofs = 0;
            }

            arc_unlock_restore(cpu_status);
        }else{
            /* lock cou here since can_buf is a global parameter */
            uint32_t cpu_status = arc_lock_save();

            if (rx_or_tx) {
                    /* CAN TX Data Install */
                    can_buf[id].tx_buf = buf;
                    can_buf[id].tx_len = len;
                    can_buf[id].tx_ofs = 0;
            } else {
                    /* CAN RX Data Install */
                    can_buf[id].rx_buf = buf;
                    can_buf[id].rx_len = len;
                    can_buf[id].rx_ofs = 0;
            }

            arc_unlock_restore(cpu_status);
        }

        return result;
}

static void can_reset_data_buffer(can_xfer_t *buf)
{
        /* lock cou here since can_buf is a global parameter */
        uint32_t cpu_status = arc_lock_save();

        buf->tx_buf = NULL;
        buf->tx_len = 0;
        buf->tx_ofs = 0;

        arc_unlock_restore(cpu_status);
}

int32_t can_interrupt_enable(uint32_t id, uint32_t type, uint32_t enable)
{
        int32_t result = E_OK;

        CAN_LOCK(can_sema[id]);
        if ((id >= 2) || (NULL == dev_can[id])) {
                result = E_PAR;
        } else {
                result = _can_interrupt_enable(dev_can[id], type, enable, can_buf[id].mode);
        }
        CAN_UNLOCK(can_sema[id]);

        return result;
}

static inline int32_t _can_interrupt_enable(can_t *can_ptr, uint32_t type, uint32_t enable, uint32_t mode)
{
        int32_t result = E_OK;

        uint32_t int_bitmap = 0;
        can_ops_t *can_ops = (can_ops_t *)can_ptr->ops;

        if ((NULL == can_ptr) || (NULL == can_ops)) {
                result = E_PAR;
        } else {
                if (eCAN_BUFFER_MODE == mode) {
                        /* Buffer Mode */
                        if (type == eCAN_INT_TYPE_TX) {
                                int_bitmap = CAN_INT_TX_COMPLISHED;
                        } else if (type == eCAN_INT_TYPE_ERROR){
                                int_bitmap = CAN_INT_BUS_OFF | CAN_INT_PROTOCOL_ERR;
                                can_ptr->int_line_bitmap[3] = int_bitmap;
                        } else {
                                int_bitmap = CAN_INT_RX_NEW_MESSAGE;
                                can_ptr->int_line_bitmap[0] = int_bitmap;
                        }
                } else {
                        /* FIFO Mode */
                        if (type == eCAN_INT_TYPE_RX) {
                                int_bitmap = CAN_INT_RX_FIFO_FILLED;
                                can_ptr->int_line_bitmap[0] = int_bitmap;
                        } else if (type == eCAN_INT_TYPE_ERROR){
                                int_bitmap = CAN_INT_BUS_OFF | CAN_INT_PROTOCOL_ERR;
                                can_ptr->int_line_bitmap[3] = int_bitmap;
                        } else {
                                int_bitmap = CAN_INT_TX_COMPLISHED;
                        }
                }

                result = can_ops->int_enable(can_ptr, enable, int_bitmap);
        }

        return result;
}

/* Note: called during boot. */
int32_t can_xfer_callback_install(uint32_t id, void (*func)(void *))
{
    int32_t result = E_OK;

    CAN_LOCK(can_sema[id]);
    if ((id >= 2) || (NULL == dev_can[id]) || (NULL == func)) {
            result = E_PAR;
    } else {
            can_buf[id].xfer_callback = func;
    }
    CAN_UNLOCK(can_sema[id]);

    return result;
}

static inline int32_t can_get_dlc(eCAN_MODE mode, uint32_t len)
{
        int32_t dlc = 0;

        if (len <= eDATA_LEN_8) {
            return len;
        }

        if (mode == CAN_MODE) {
                return E_PAR;
        }

        switch (len) {
        case eDATA_LEN_64:
        case eDATA_LEN_48:
        case eDATA_LEN_32:
                dlc = 11 + (len >> 4);
                break;
        case eDATA_LEN_24:
        case eDATA_LEN_20:
        case eDATA_LEN_16:
        case eDATA_LEN_12:
                dlc = 6 + (len >> 2);
                break;
        default:
                return E_PAR;
        }

        return dlc;
}
/*
 *    DLC    <-->     Data Field Len of frame(In bytes.)
 *    0...8             0...8
 *    eDATA_DLC_9       eDATA_LEN_12
 *    eDATA_DLC_10      eDATA_LEN_16
 *    eDATA_DLC_11      eDATA_LEN_20
 *    eDATA_DLC_12      eDATA_LEN_24
 *    eDATA_DLC_13      eDATA_LEN_32
 *    eDATA_DLC_14      eDATA_LEN_48
 *    eDATA_DLC_15      eDATA_LEN_64
 */
static inline int32_t can_get_datalen(eCAN_MODE mode, uint32_t dlc)
{
        int32_t len = 0;

        if (dlc <= eDATA_DLC_8) {
                return dlc;
        }

        if (mode == CAN_MODE) {
                return eDATA_LEN_8;
        }

        if (dlc > eDATA_DLC_15) {
            dlc = eDATA_DLC_15;
        }


        switch (dlc) {
        case eDATA_DLC_15:
        case eDATA_DLC_14:
        case eDATA_DLC_13:
                len = (dlc - 11) << 4;
                break;
        case eDATA_DLC_12:
        case eDATA_DLC_11:
        case eDATA_DLC_10:
        case eDATA_DLC_9:
                len = (dlc - 6) << 2;
                break;
        default:
                EMBARC_PRINTF("can_get_datalen: Invalid input dlc %x\r\n", dlc);
                //return E_PAR;
                len = E_PAR;
                break;
        }

        return len;
}

int32_t can_receive_data(uint32_t dev_id, can_data_message_t *msg)
{
        int32_t result = E_OK;
        uint32_t buf_id = 0, dlc = 0, len = 0;
        can_ops_t *can_ops = (can_ops_t *)dev_can[dev_id]->ops;
        uint32_t frame_size = CAN_DFS_2_FRAMESIZE(can_buf[dev_id].rx_dfs);
        uint32_t rx_buf[18];

        can_frame_params_t *params = msg->frame_params;

        if ((dev_id >= 2) || (NULL == dev_can[dev_id]) || (NULL == can_ops)) {
                result = E_PAR;
        }

        if (can_buf[dev_id].rx_last_id > can_buf[dev_id].rx_max_id) {
                can_buf[dev_id].rx_last_id = 0;
        }
        buf_id = can_buf[dev_id].rx_last_id;

        result = can_ops->read_frame(dev_can[dev_id], buf_id, rx_buf, frame_size);
        if (E_OK == result) {
                params->msg_id = rx_buf[0] & 0x1FFFFFFF;
                params->eframe_format = (rx_buf[0] & (1 << 30)) ? (1) : (0);
                dlc = (rx_buf[1] >> 16) & 0xF;
                len = can_get_datalen(can_buf[dev_id].can_mode, dlc);
                if (len > 0) {
                        params->len = len;

                        if (params->eframe_format == eEXTENDED_FRAME) {
                                params->msg_id = rx_buf[0] & 0x1FFFFFFF;
                        } else {
                                params->msg_id = (rx_buf[0] & 0x1FFFFFFF) >> 18;
                        }
                        transfer_bytes_stream(&rx_buf[2], msg->data, params->len);
                        can_buf[dev_id].rx_last_id += 1;
                }
        }

        return result;
}

static int32_t _can_rx_buffer_full(can_t *can_ptr, can_xfer_t *buf)
{
        int32_t result = E_OK;

        if ((can_ptr) && (buf)) {
                buf->rx_buf = NULL;
                buf->rx_len = 0;
                buf->rx_ofs = 0;

                result = _can_interrupt_enable(can_ptr, eCAN_INT_TYPE_RX, eCAN_INT_DISABLE, buf->mode);
                if (E_OK == result) {
                    if (buf->xfer_callback) {
                            buf->xfer_callback(NULL);
                    } else {
                            result = E_SYS;
                    }
                }
        } else {
                result = E_PAR;
        }

        return result;
}

static int32_t _can_adjust_buf_id(can_t *can_ptr, can_ops_t *can_ops, can_xfer_t *buf)
{
	uint32_t start_id = buf->rx_last_id;

	while (0 == can_ops->rx_buf_status(can_ptr, buf->rx_last_id)) {
		buf->rx_last_id ++;
		if (buf->rx_last_id > buf->rx_max_id) {
			buf->rx_last_id = 0;
		}
		if (start_id == buf->rx_last_id) {
			break;
		}
	}

	return buf->rx_last_id;
}

static int32_t _can_frame_read(can_t *can_ptr, can_ops_t *can_ops, can_xfer_t *buf)
{
	int32_t result = E_OK;
	uint32_t i;
	uint32_t buf_id = 0, len = 0;
	uint32_t frame_size = CAN_DFS_2_FRAMESIZE(buf->rx_dfs);
	uint32_t rx_buf[18];

	if (can_buf[can_ptr->id].mode == eCAN_BUFFER_MODE) {
		/* Adjust FW RX buffer id. */
		_can_adjust_buf_id(can_ptr, can_ops, buf);
	}

	/* Here, we read out all received frame from HW buffer,
	 * start from FW RX buffer id.
	 */
	for (i=0;i<CAN_FRAM_MAX_BUF_NUM;i++){
		if (can_buf[can_ptr->id].mode == eCAN_FIFO_MODE) {
			result = can_ops->fifo_read(can_ptr, rx_buf, frame_size);
		} else {
			if (buf->rx_last_id > buf->rx_max_id) {
				buf->rx_last_id = 0;
			}
			buf_id = buf->rx_last_id;

			result = can_ops->read_frame(can_ptr, buf_id, rx_buf, frame_size);
		}

		if (E_OK == result) {
			uint32_t msg_id, ide, dlc;
			msg_id = rx_buf[0] & 0x1FFFFFFF;
			ide = (rx_buf[0] & (1 << 30)) ? (1) : (0);
			dlc = (rx_buf[1] >> 16) & 0xF;
			len = can_get_datalen(buf->can_mode, dlc);
			if (len > 0) {
				if (buf->rx_indication) {
					buf->rx_indication(msg_id, ide, &rx_buf[2], len);
				}
			}

			if (can_buf[can_ptr->id].mode == eCAN_BUFFER_MODE) {
				buf->rx_last_id += 1;
			}
		} else if (E_NODATA == result) {
			result = E_OK;
			break;
		} else {
			EMBARC_PRINTF("[%s] can_ops->read_frame ret err: %d\r\n", __func__, result);
			break;
		}
	}

	return result;
}

/*
 * rx queue is in can driver layer!
 * */
static int32_t _can_rx_buffer_read(can_t *can_ptr, can_ops_t *can_ops, can_xfer_t *buf)
{
        int32_t result = E_OK;

        uint32_t buf_id = 0;
        uint32_t frame_size = CAN_DFS_2_FRAMESIZE(buf->rx_dfs);

        uint32_t *rx_buf = NULL;

        if (buf->rx_last_id > buf->rx_max_id) {
                buf->rx_last_id = 0;
        }
        buf_id = buf->rx_last_id;

        while (buf->rx_len > buf->rx_ofs) {
            if (buf->rx_len - buf->rx_ofs < (frame_size << 2)) {
                    /* TODO: rx buffer unaligned with frame size, Panic! */
                    result = E_SYS;
                    break;
            } else {
                    /* received data. if need, restall rx buffer! */
                    rx_buf = (uint32_t *)((uint32_t)buf->rx_buf + buf->rx_ofs);
                    result = can_ops->read_frame(can_ptr, buf_id, rx_buf, frame_size);
                    if (E_OK == result) {
                            buf_id += 1;
                            if (buf_id > buf->rx_max_id) {
                                    buf_id = 0;
                            }

                            buf->rx_ofs += frame_size << 2;
                            if (buf->rx_ofs >= buf->rx_len) {
                                    result = _can_rx_buffer_full(can_ptr, buf);
                                    break;
                            }
                    } else if (E_NODATA == result) {
                            /* hw rx buffer no data! */
                            result = E_OK;
                            break;
                    } else {
                            /* device operation error, Panic! */
                            break;
                    }
                }
        }

        /* record rx buffer id. */
        buf->rx_last_id = buf_id;

        return result;
}

/* rx fifo reaches to watermark. or tx buffer/fifo new message. */
static void _can_rx_handler(can_t *can_ptr, can_xfer_t *buf, uint32_t line, void *param)
{
        int32_t result = E_OK;
        uint32_t int_status = 0;
        uint32_t int_bitmap = 0;

        can_ops_t *can_ops = NULL;

        if ((NULL == can_ptr) || (line >= 4) || \
                (NULL == buf)) {
                result = E_PAR;
        } else {
                can_ops = (can_ops_t *)can_ptr->ops;
                if (NULL == can_ops) {
                        result = E_NOOPS;
                } else {
                        int_bitmap = can_ptr->int_line_bitmap[line];

                        result = can_ops->int_status(can_ptr, 0xFFFFFFFFU);
                        if (result > 0) {
                                int_status = result & int_bitmap;

                                /* We firstly clear the interrupts! */
                                result = can_ops->int_clear(can_ptr, int_status);
                        } else {
                                /* TODO: exception! no status, but interrupt! */
                                result = E_SYS;
                                EMBARC_PRINTF("[%s] exception! no status, but interrupt!\r\n", __func__);
                                return;
                        }
                }
        }

        if (E_OK == result) {
#ifdef CAN_TX_COMPLETE_MONITOR
                if (int_status & CAN_INT_RX_FIFO_FILLED) {
                        buf->rx_isr_free_counter ++;
                }

#endif
                if ((int_status & CAN_INT_RX_NEW_MESSAGE) || (int_status & CAN_INT_RX_FIFO_FILLED)) {
                        /* received frame. */
                        if (buf->rx_indication) {
                                result = _can_frame_read(can_ptr, can_ops, buf);
                        } else {
                                if ((NULL == buf->rx_buf) || \
                                        (0 == buf->rx_len) || \
                                        (buf->rx_ofs >= buf->rx_len)) {
                                        result = E_PAR;
                                } else {
                                        result = _can_rx_buffer_read(can_ptr, can_ops, buf);
                                }
                        }
                } else {
                        result = E_SYS;
                }
        }

        if (E_OK != result) {
                /* TODO: Error happended in interrupt context, system crash! */
        }
}

/* tx fifo is empty. or tx finished. */
static void _can_tx_handler(can_t *can_ptr, can_xfer_t *buf, uint32_t line, void *param)
{
        int32_t result = E_OK;
        uint32_t int_status = 0;
        uint32_t bus_id;
        int32_t buf_num = 0;

        can_ops_t *can_ops = NULL;
        bus_id = can_ptr->id;

        if ((NULL == can_ptr) || (line >= 4) || (NULL == buf) ) {
                result = E_PAR;
        } else {
                can_ops = (can_ops_t *)can_ptr->ops;
                if (NULL == can_ops) {
                        result = E_NOOPS;
                } else {
                        /* Check CAN Tx interrupt status */
                        result = can_ops->int_status(can_ptr, 0xFFFFFFFFU);
                        if (result > 0) {
                                int_status = result;
                                result = E_OK;
                        } else {
                                /* TODO: exception! no status, but interrupt! */
                                result = E_SYS;
                        }
                }
        }

        /* Clear TX complete interrupt flag */
        can_ops->int_clear(can_ptr, CAN_INT_TX_COMPLISHED | CAN_INT_TX_FIFO_READY);
        /* disable rx lost int */
        can_ops->int_enable(can_ptr, 0, CAN_INT_RAM_ACCESS_FAIL);

        if (E_OK == result) {
                    if (int_status & CAN_INT_TX_COMPLISHED) {
#ifdef CAN_TX_COMPLETE_MONITOR
                            result = E_OK;
                            /*bug1549:since fifo water mark is 1 and read one frame a time,fifo_frame_num = rx_isr_free_counter*/
                            buf_num = can_ops->rx_buf_frame_num_get(can_ptr);
                            buf->rx_isr_free_counter += buf_num;
                            if (buf->rx_isr_free_counter == 32 + can_node_rec_flag[can_ptr->id]) {
                                    /* Bug1252: Senario #2, Work around.
                                     *     Before TX Complete, there are 32 times rx isr,
                                     *     think 32 hw retransmitions fail.
                                     **/
                                    EMBARC_PRINTF("[%s] fake tx completec isr, due to 32 arbitrate fail!\r\n", __func__);
                                    result = E_SYS;
                            } else if (buf->rx_isr_free_counter > 32 + can_node_rec_flag[can_ptr->id]) {
                                    EMBARC_PRINTF("[%s] bus id : %d ,Unexpected rx isr cnt %d before tx complete!\r\n", __func__, bus_id, buf->rx_isr_free_counter);
                                    result = E_SYS;
                            }

                            if (result == E_SYS) {
                                    /* Since HW send 32 times fail, we keep retry by re-triggering. */
                                    EMBARC_PRINTF("[%s] bus id : %d, retry send data!\r\n", __func__, bus_id);

                                    buf->rx_isr_free_counter = 0;
                                    can_node_rec_flag[can_ptr->id] = 0;

                                    if(can_ptr->id == 0){
                                            if(dw_timer_stop(3) != E_OK){
                                                EMBARC_PRINTF("[%s] stop timer 3 failed \r\n",__func__);
                                            }
                                            can_exa.dev_can_s = can_ptr;
                                            can_exa.can_buf_s = buf;
                                            if (dw_timer_start(3, 50000000, timeout_handler, (void *)(&can_exa)) != E_OK){
                                                EMBARC_PRINTF("[%s] start timer3 failed \r\n",__func__);
                                            }
                                    }else{
                                            if(dw_timer_stop(2) != E_OK){
                                                EMBARC_PRINTF("[%s] stop timer 2 failed \r\n",__func__);
                                            }
                                            can_exa_1.dev_can_s = can_ptr;
                                            can_exa_1.can_buf_s = buf;
                                            if(dw_timer_start(2, 50000000, timeout_handler, (void *)(&can_exa_1)) != E_OK){
                                                EMBARC_PRINTF("[%s] start timer2 failed \r\n",__func__);
                                            }
                                    }

                                    can_ops->write_frame_try(can_ptr, 0);
                                    return;
                            }else{
                                    /* can send data suceess */
                                    can_tx_done_flag[can_ptr->id] = 1;
                            }
#endif

                            if(can_ptr->id == 0){
                                    if(dw_timer_stop(3) != E_OK){
                                        EMBARC_PRINTF("[%s] stop timer 3 failed \r\n",__func__);
                                    }
                            }else{
                                    if(dw_timer_stop(2) != E_OK){
                                        EMBARC_PRINTF("[%s] stop timer 2 failed \r\n",__func__);
                                    }
                            }

                            /* Check whether still have data need to be sent */
                            if((buf->data_tx_mode == DEV_XFER_INTERRUPT) && (buf->tx_len > buf->tx_ofs)) {
                                    buf->rx_isr_free_counter = 0;
                                    can_node_rec_flag[can_ptr->id] = 0;
                                    if(can_ptr->id == 0){
                                            can_exa.dev_can_s = can_ptr;
                                            can_exa.can_buf_s = buf;
                                            if (dw_timer_start(3, 50000000, timeout_handler, (void *)(&can_exa)) != E_OK){
                                                EMBARC_PRINTF("[%s] start timer3 failed \r\n",__func__);
                                            }
                                    }else{
                                            can_exa_1.dev_can_s = can_ptr;
                                            can_exa_1.can_buf_s = buf;
                                            if(dw_timer_start(2, 50000000, timeout_handler, (void *)(&can_exa_1)) != E_OK){
                                                EMBARC_PRINTF("[%s] start timer2 failed \r\n",__func__);
                                            }
                                    }
                                    can_send_frame_isr(can_ptr, can_ops, buf);
                            } else {
                                    /* All the data have been sent, reset can data buffer */
                                    //can_reset_data_buffer(buf);

                                    buf->rx_isr_free_counter = 0;

                                    if (buf->tx_confirmation) {

#ifdef CAN_TX_COMPLETE_MONITOR
                                            buf->tx_confirmation(buf->frame_parameters.msg_id, 0);
#else
                                            buf->tx_confirmation(buf->frame_parameters.msg_id);
#endif
                                    }
                            }

                    }else if (int_status & CAN_INT_TX_FIFO_READY) {
                            EMBARC_PRINTF("[%s] int_status & CAN_INT_TX_FIFO_READY!\r\n", __func__);
                    } else {
                            result = E_SYS;
                            EMBARC_PRINTF("[%s] result = E_SYS!\r\n", __func__);
                    }
        } else {
                /* TODO: Error happended in interrupt context, system crash! */
                EMBARC_PRINTF("[%s] Error happended in interrupt ,int_status=%d,result =%d\r\n", __func__,int_status,result);
        }
}

static void can_assert_reset(uint32_t id)
{
    /* disable HW */
    if (id == 0) {
        CAN_0_reset(1);
    } else {
        CAN_1_reset(1);
    }

    assert_reset_flag[id] = 1;
}

static void can_deassert_reset(uint32_t id)
{
    /* enable HW */
    if (id == 0) {
        CAN_0_reset(0);
    } else {
        CAN_1_reset(0);
    }

    assert_reset_flag[id] = 0;
}

void can_bus_off_recover(uint32_t bus_id)
{
    if(assert_reset_flag[bus_id] == 1){
        //EMBARC_PRINTF("[%s] can %d deassert reset,recover\r\n", __func__, bus_id);
        can_deassert_reset(bus_id);
        can_internal_reset(bus_id);
    }else{
        //EMBARC_PRINTF("[%s] can %d already deassert reset,should not go to here\r\n", __func__, bus_id);
    }

}

static void _can_error_handler(can_t *can_ptr, can_xfer_t *buf, uint32_t line, void *param)
{
        int32_t result = E_OK;

        uint32_t int_bitmap = 0;
        uint32_t int_status = 0;

        can_ops_t *can_ops = NULL;
        uint32_t bus_id = 0;

        bus_id = can_ptr->id;

        if ((NULL == can_ptr) || (line >= 4)) {
                result = E_PAR;
        } else {
            int_bitmap = can_ptr->int_line_bitmap[line];

            can_ops = (can_ops_t *)can_ptr->ops;
            if (NULL == can_ops) {
                    result = E_NOOPS;
            } else {
                    result = can_ops->int_status(can_ptr, 0xFFFFFFFFU);
                    if (result > 0) {
                            int_status = result;
                            result = E_OK;
                    } else {
                            /* TODO: exception! no status, but interrupt! */
                            result = E_SYS;
                    }
            }
        }

        if (E_OK == result) {
            int_status = int_status & int_bitmap;
            while (int_status) {
                /* Clear the error interrupts */
                can_ops->int_clear(can_ptr, int_status);

                if ((int_status & CAN_INT_ERR_PASSIVE) || (int_status & CAN_INT_PROTOCOL_ERR)) {
                        //EMBARC_PRINTF("[%s] protocol err: %x \r\n", __func__, int_status);
                }

                //TODO:debug info
                if (buf->err_indication)
                {
                        /* If Upper layer set error_indication_callback,
                                        * Then the process to error event is fully handled by upper layer. */
                        buf->err_indication(int_status);
                }
                else
                {
                        /* If Upper layer did not set error_indication_callback,
                                        * can will automatically recover. */
                }

                if (int_status & CAN_INT_BUS_OFF) {

                        if (buf->can_busoff_recover_mode == MANUAL){
                            //EMBARC_PRINTF("[%s] bus off: can %d manual recovery, int status: %x \r\n", __func__, bus_id, int_status);
                            can_assert_reset(bus_id);

                            if(buf->data_tx_mode == DEV_XFER_INTERRUPT){
                                if (bus_id == 0){
                                    dw_timer_stop(3);
                                }else{
                                    dw_timer_stop(2);
                                }
                            }

                        }else{
                            EMBARC_PRINTF("[%s] bus off: can %d  automatic recovery, int status: %x \r\n", __func__, bus_id, int_status);
                        }
                }

                // if (buf->err_indication) {
                //         /* If Upper layer set error_indication_callback,
                //          * Then the process to error event is fully handled by upper layer. */
                //         buf->err_indication(int_status);
                // } else {
                //         /* If Upper layer did not set error_indication_callback,
                //          * can will automatically recover. */
                // }

                result = can_ops->int_status(can_ptr, 0xFFFFFFFFU);
                int_status = result & int_bitmap;
            }
        }else{
            EMBARC_PRINTF("[%s] E_PAR \r\n",__func__);
        }
}

static void _can_unused_handler(can_t *can_ptr, uint32_t line, void *param)
{
        /*
         * TODO: this kind of interrupt weren't enabled.
         * but their status are valid? system crash!
         * */
}


static void timeout_handler(void *can_param)
{
        can_ts *can_to = NULL;
        can_t *can_ptr = NULL;
        can_ops_t *can_ops = NULL;
        can_xfer_t *buf = NULL;
        uint32_t bus_id;
        int32_t result = E_OK;

        can_to = (can_ts*)can_param;
        buf = can_to->can_buf_s;
        can_ptr = can_to->dev_can_s;
        can_ops = (can_ops_t *)can_ptr->ops;
        bus_id = can_ptr->id;

        if ((NULL == can_to) || (NULL == can_ptr) || (NULL == buf)) {
                result = E_PAR;
        }

        if(result == E_OK){
                if(bus_id == 0){
                        if(dw_timer_stop(3) != E_OK){
                            EMBARC_PRINTF("[%s] stop timer 3 failed \r\n",__func__);
                        }
                }else{
                        if(dw_timer_stop(2) != E_OK){
                            EMBARC_PRINTF("[%s] stop timer 2 failed \r\n",__func__);
                        }
                }

                /* Reset can */
                can_internal_reset(bus_id);

                if(bus_id == 0){
                        can_exa.dev_can_s = can_ptr;
                        can_exa.can_buf_s = buf;
                        if (dw_timer_start(3, 50000000, timeout_handler, (void *)(&can_exa)) != E_OK){
                            EMBARC_PRINTF("[%s] start timer3 failed \r\n",__func__);
                        }
                }else{
                        can_exa_1.dev_can_s = can_ptr;
                        can_exa_1.can_buf_s = buf;
                        if(dw_timer_start(2, 50000000, timeout_handler, (void *)(&can_exa_1)) != E_OK){
                            EMBARC_PRINTF("[%s] start timer2 failed \r\n",__func__);
                        }
                }

                can_ops->write_frame_try(can_ptr, 0);

        }else{
                        /* TODO: Error happended in interrupt context, system crash! */
                        EMBARC_PRINTF("[%s] bus id : %d,error happened in timeout!\r\n",__func__,bus_id);
        }

}



/* CAN0 interrupt service routine. */
static void can0_line0_isr(void *param)
{
        _can_rx_handler(dev_can[0], &can_buf[0], 0, param);
}

static void can0_line1_isr(void *param)
{
        _can_tx_handler(dev_can[0], &can_buf[0], 1, param);
}

static void can0_line2_isr(void *param)
{
        _can_unused_handler(dev_can[0], 2, param);
}

static void can0_line3_isr(void *param)
{
        _can_error_handler(dev_can[0], &can_buf[0], 3, param);
}


/* CAN1 interrupt service routine. */
static void can1_line0_isr(void *param)
{
        _can_rx_handler(dev_can[1], &can_buf[1], 0, param);
}

static void can1_line1_isr(void *param)
{
        _can_tx_handler(dev_can[1], &can_buf[1], 1, param);
}

static void can1_line2_isr(void *param)
{
        _can_unused_handler(dev_can[1], 2, param);
}

static void can1_line3_isr(void *param)
{
        _can_error_handler(dev_can[1], &can_buf[1], 3, param);
}


static int32_t can_int_install(uint32_t id)
{
        int32_t result = E_OK;

        INT_HANDLER int_info[4];
        uint32_t int_idx;

        can_ops_t *can_ops = (can_ops_t *)dev_can[id]->ops;

        if (0 == id) {
                int_info[0] = can0_line0_isr;
                int_info[1] = can0_line1_isr;
                int_info[2] = can0_line2_isr;
                int_info[3] = can0_line3_isr;
        } else if (1 == id) {
                int_info[0] = can1_line0_isr;
                int_info[1] = can1_line1_isr;
                int_info[2] = can1_line2_isr;
                int_info[3] = can1_line3_isr;
        } else {
                result = E_PAR;
        }

        if (E_OK == result) {
                if (can_buf[id].mode == eCAN_FIFO_MODE) {
                        /* RX FIFO Full. select INT0 */
                        result = can_ops->int_line_select(dev_can[id], CAN_INT_RX_FIFO_REAL_FULL, 0);
                        if (E_OK != result) {
                                return result;
                        }

                        /* RX FIFO Watermark Reached. select INT0 */
                        result = can_ops->int_line_select(dev_can[id], CAN_INT_RX_FIFO_FULL, 0);
                        if (E_OK != result) {
                                return result;
                        }

                        /* Timeout Occurred. select INT0 */
                        result = can_ops->int_line_select(dev_can[id], CAN_INT_TIMEOUT_OCCURED, 0);
                        if (E_OK != result) {
                                return result;
                        }

                        /* Tx interrupt line select INT1, but it isn't enabled in code now*/
                        result = can_ops->int_line_select(dev_can[id], CAN_INT_TX_COMPLISHED, 1);
                        if (E_OK != result) {
                                return result;
                        }
                } else {
                        /* Tx interrupt line select INT1, but it isn't enabled in code now*/
                        result = can_ops->int_line_select(dev_can[id], CAN_INT_TX_COMPLISHED, 1);
                        if (E_OK != result) {
                                return result;
                        }
                }

                /* Error Passive. select INT3 */
                result = can_ops->int_line_select(dev_can[id], CAN_INT_ERR_PASSIVE, 3);
                if (E_OK != result) {
                        return result;
                }

                /* Bus_Off Status. select INT3 */
                result = can_ops->int_line_select(dev_can[id], CAN_INT_BUS_OFF, 3);
                if (E_OK != result) {
                        return result;
                }

                /* Protocol Error. select INT3 */
                result = can_ops->int_line_select(dev_can[id], CAN_INT_PROTOCOL_ERR, 3);
                if (E_OK != result) {
                        return result;
                }
        }

        for (int_idx = 0; (E_OK == result) && (int_idx < 4); int_idx++) {
                result = int_handler_install(dev_can[id]->int_line[int_idx], \
                        int_info[int_idx]);
                if (E_OK == result) {
                        result = int_enable(dev_can[id]->int_line[int_idx]);
                        if (E_OK == result) {
                                result = can_ops->int_line_enable(dev_can[id], int_idx, 1);
                        }
                        dmu_irq_enable(dev_can[id]->int_line[int_idx], 1);
                }
        }

        return result;
}
#endif
