#ifndef SPIS_SERVER_H
#define SPIS_SERVER_H

void spis_server_init(void);
int32_t spis_server_write(uint32_t *data, uint32_t len);
int32_t spis_server_transmit_done(void);

#endif
