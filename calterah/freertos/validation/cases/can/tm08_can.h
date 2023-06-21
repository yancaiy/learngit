#ifndef TM08_CAN_H
#define TM08_CAN_H

#include <string.h>
#include "embARC_debug.h"
#include "embARC.h"

/**
 * \name CAN validation test ID names
 * @{
 */
typedef enum {
    CAN_CONF_WORK_MODE,
    CAN_CONF_BAUDRATE,
    CAN_CONF_STD_FILTER,
    CAN_CONF_EXT_FILTER,
    CAN_CONF_FRAME,
    CAN_CONF_DATA_TX_MODE,

    /* More configure type add here */
    CAN_CONF_MAX
} can_config_type_e;

#define CAN_CID_0           0
#define CAN_CID_1           1
#define CAN_CID_2           2
#define CAN_CID_3           3
#define CAN_CID_4           4
#define CAN_CID_5           5

#define CAN_CID_50          50
#define CAN_CID_51          51
#define CAN_CID_52          52
#define CAN_CID_53          53
#define CAN_CID_54          54
#define CAN_CID_55          55
#define CAN_CID_56          56

#define CAN_CID_60          60
#define CAN_CID_61          61
#define CAN_CID_62          62
#define CAN_CID_63          63
#define CAN_CID_64          64

#define CAN_CID_70          70
#define CAN_CID_71          71

#define CAN_CID_100          100
#define CAN_CID_101          101
#define CAN_CID_102          102
#define CAN_CID_103          103
#define CAN_CID_104          104


#endif
