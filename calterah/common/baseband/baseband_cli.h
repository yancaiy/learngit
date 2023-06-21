#ifndef BASEBAND_CLI_H
#define BASEBAND_CLI_H

/* This Marco is used to control all the Baseband related command-line,
   comment out this Marco can size RAM size but make all Baseband command-line can no longer be used */
#define BASEBAND_CLI

void baseband_cli_commands( void );
bool baseband_stream_on_dmp_mid();
bool baseband_stream_on_dmp_fnl();
bool baseband_stream_on_fft1d();
void set_baseband_stream_on_dmp_mid(bool value);
void set_baseband_stream_on_dmp_fnl(bool value);
void set_baseband_stream_on_fft1d(bool value);
bool baseband_scan_stop_req();
bool baseband_stream_off_req();
void set_scan_stop_flag(bool value);
void set_stream_on_en(bool value);
bool get_stream_on_en();
//avoid DC interference and elevate precision
#define RNG_START_SEARCH_IDX                (4)

#endif
