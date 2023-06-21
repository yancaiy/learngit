#ifndef CAN_OBJ_H
#define CAN_OBJ_H

#define CAN_0_ID	0	/*!< can 0 id macro */
#define CAN_1_ID	1	/*!< can 1 id macro */

//CAN通道与实际应用CAN的对应关系
#define PCAN        CAN_0_ID   //NOTE!!!!
#define VCAN        CAN_0_ID
#define DUTCAN      PCAN

void *can_get_dev(uint32_t id);

#endif
