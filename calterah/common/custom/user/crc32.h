#ifndef _CRC32_H_
#define _CRC32_H_
#ifdef __cplusplus
extern "C" {
#endif

void gen_crc_table(void);
unsigned int update_crc(unsigned int crc_accum, char *data_blk_ptr, int data_blk_size);

#ifdef __cplusplus
}
#endif
#endif
