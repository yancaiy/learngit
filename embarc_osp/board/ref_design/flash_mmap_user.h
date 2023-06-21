/*
 * @Author: fang yongjun
 * @Date: 2021-11-03 17:10:47
 * @LastEditTime: 2021-11-03 18:36:48
 * @LastEditors: fang yongjun
 * @Description: 
 * @FilePath: flash_mmap_user.h
 * Copyright (C) 2021 Chengtech Ltd.
 */
#ifndef _FLASH_MMAP_USER_H_
#define _FLASH_MMAP_USER_H_

// /* the memory mapping address for flash */
// #define FLASH_MMAP_FIRMWARE_BASE    (0x300000)

// /* flash address */
// /* If you want change the start address of "FLASH_HEADER_BASE" or "FLASH_BOOT_BASE" or "FLASH_FIRMWARE_BASE",
//    Don't forget to change the value in "settings" file under "C:\Calterah_downloader" accordingly */
#define FLASH_SIZE (0x400000)

#define FLASH_SWT_INFO_BASE	(0x001000)  //存储 重启次数以及看门狗触发次数 --- YOu , 长度 0x1000
#define FLASH_SERVICE_CALI_INFO_BASE	(0x002000)  //?? ???????? --- YOu , ?? 0x1000

#define FLASH_BOOT_FLAG        (0x009000)
#define CONFIG_ADDR	(0x010000)
#define CONFIG_ADDR_SZIE (0x4000)   //目前配置值小于16k，如果有更新，这里需要修改

#define FLASH_FIRMWARE_BASE_BK	(0x100000)

#define FLASH_DID_BASE (0x200000)
#define FLASH_DID_SIZE (4 << 10) //4k

#define FLASH_DTC_BASE (0x201000)
#define FLASH_DTC_SIZE (4 << 10) //4K

#define FLASH_MISS_CALIB_FLAG_BASE (0X203000)
#define FLASH_MISS_CALIB_FLAG_SIZE (4 << 10) //4K

#define FLASH_UPDATE_CNT_REC_BASE (0X204000)
#define FLASH_UPDATE_CNT_REC_SIZE (4 << 10) //4K

#define FLASH_SNAPSHOT_BASE (0x207000)
#define FLASH_SNAPSHOT_SIZE (12 << 10) //12K
#define BUFF_SNAPSHOT_SIZE (4 << 10)

#define FLASH_SYS_CAN0_SNAPSHOT_BASE (0x205000)
#define FLASH_SYS_CAN0_SNAPSHOT_SIZE (4 << 10) //4K

#define FLASH_SYS_CAN1_SNAPSHOT_BASE (0x206000)
#define FLASH_SYS_CAN1_SNAPSHOT_SIZE (4 << 10) //4K

#define FLASH_SYS_CAN_TXBUFF_SIZE (255) 
#define FLASH_SYS_CAN_RXBUFF_SIZE (255)
#define FLASH_SYS_CAN_TXBUFF_FIFO_SIZE (17) 
#define FLASH_SYS_CAN_RXBUFF_FIFO_SIZE (17)
#define FLASH_SYS_CAN_STFILTER_SIZE (127) 
#define FLASH_SYS_CAN_EXFILTER_SIZE (127) 

#endif
