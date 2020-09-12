#ifndef _AHP_XC_H
#define _AHP_XC_H

#ifdef  __cplusplus
extern "C" {
#endif
#ifdef _WIN32
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT extern
#endif

#include <stdio.h>
#include <string.h>

#define XC_BASE_RATE 57600

typedef enum {
    R_57600 = 0,
    R_115200 = 1,
    R_230400 = 2,
    R_460800 = 3,
} baud_rate;

typedef enum {
    CLEAR = 0,
    SET_INDEX = 1,
    SET_LEDS = 2,
    SET_BAUD_RATE = 3,
    SET_DELAY_LO = 4,
    SET_DELAY_HI = 5,
    SET_LINE_LO = 6,
    SET_LINE_HI = 7,
    SET_FREQ_DIV = 8,
    ENABLE_CAPTURE = 13
} it_cmd;

DLL_EXPORT unsigned long xc_get_bps();
DLL_EXPORT unsigned long xc_get_nlines();
DLL_EXPORT unsigned long xc_get_nbaselines();
DLL_EXPORT unsigned long xc_get_delaysize();
DLL_EXPORT unsigned long xc_get_frequency();
DLL_EXPORT unsigned long xc_get_packetsize();
DLL_EXPORT void xc_connect(char *port);
DLL_EXPORT void xc_disconnect();
DLL_EXPORT void xc_get_properties();
DLL_EXPORT void xc_enable_capture(int enable);
DLL_EXPORT void xc_select_input(int index);
DLL_EXPORT void xc_set_rate(baud_rate rate);
DLL_EXPORT void xc_set_5v(int index, int value);
DLL_EXPORT void xc_set_ht(int index, int value);
DLL_EXPORT void xc_set_delay(int index, unsigned char value);
DLL_EXPORT void xc_set_line(int index, unsigned char value);
DLL_EXPORT void xc_set_frequency_divider(unsigned char value);
DLL_EXPORT void xc_get_packet(unsigned long *counts, unsigned long *autocorrelations, unsigned long *correlations);
DLL_EXPORT int xc_send_command(it_cmd c, unsigned char value);

#ifdef __cplusplus
} // extern "C"
#endif

#endif //_AHP_XC_H


