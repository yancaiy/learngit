#ifndef CALTERAH_TRACK_CLI_H
#define CALTERAH_TRACK_CLI_H

/* Tacking data output type */
enum OBJECT_OUTPUT {
    UART_STRING = 0,     /* original string by uart interface */
    UART_HEX,            /* hex output by uart interface */
    CAN_POLLING,         /* polling output by CAN interface */
    CAN_INT              /* INT output mode by CAN interface */
};

/*--- DECLARAION ---------------------*/
void track_cmd_register(void);
int8_t get_track_cfg(void);
void set_track_cfg(int8_t value);
#endif
