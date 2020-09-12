#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rs232.h"
#include "ahp_xc.h"

static unsigned int xc_bps;
static unsigned int xc_nlines;
static unsigned int xc_nbaselines;
static unsigned int xc_delaysize;
static unsigned int xc_frequency;
static unsigned int xc_packetsize;
static unsigned long *xc_counts;
static unsigned long *xc_autocorrelations;
static unsigned long *xc_correlations;
static int comport_number;

unsigned long xc_get_bps()
{
    return xc_bps;
}

unsigned long xc_get_nlines()
{
    return xc_nlines;
}

unsigned long xc_get_nbaselines()
{
    return xc_nbaselines;
}

unsigned long xc_get_delaysize()
{
    return xc_delaysize;
}

unsigned long xc_get_frequency()
{
    return xc_frequency;
}

unsigned long xc_get_packetsize()
{
    return xc_packetsize;
}

void xc_connect(char *port)
{
    comport_number = RS232_GetPortnr(port);
    RS232_OpenComport(comport_number, XC_BASE_RATE, "8N2", 0);
}

void xc_disconnect()
{
    xc_set_rate(R_57600);
    RS232_CloseComport(comport_number);
}

static int send_char(unsigned char c)
{
    RS232_flushTX(comport_number);
    return RS232_SendByte(comport_number, c);
}

void xc_get_packet(unsigned long *counts, unsigned long *autocorrelations, unsigned long *correlations)
{
    unsigned int x = 0;
    char *buf = (char*)malloc(xc_packetsize);
    RS232_PollComport(comport_number, (unsigned char*)buf, (int)xc_packetsize);
    buf += 16;
    for(x = 0; x < xc_nlines; x++) {
        unsigned int n = xc_bps/4;
        char *sample = (char*)malloc(xc_bps/4);
        strncpy(sample, buf+x*n, n);
        sscanf(sample, "%lX", &xc_counts[x]);
    }
    for(x = 0; x < xc_nlines; x++) {
        unsigned int n = xc_bps/4;
        char *sample = (char*)malloc(xc_bps/4);
        strncpy(sample, buf+x*n, n);
        sscanf(sample, "%lX", &xc_autocorrelations[x]);
    }
    for(x = 0; x < xc_nbaselines; x++) {
        unsigned int n = xc_bps/4;
        char *sample = (char*)malloc(xc_bps/4);
        strncpy(sample, buf+x*n, n);
        sscanf(sample, "%lX", &xc_correlations[x]);
    }
    counts = xc_counts;
    autocorrelations = xc_autocorrelations;
    correlations = xc_correlations;
}

void xc_get_properties()
{
    unsigned char buf[16];
    RS232_flushRX(comport_number);
    xc_enable_capture(1);
    RS232_PollComport(comport_number, buf, 16);
    xc_enable_capture(0);
    RS232_flushRX(comport_number);
    unsigned int _bps, _nlines, _jittersize, _delaysize, _frequency;
    sscanf((char*)buf, "%02X%01X%02X%03X%08X", &_bps, &_nlines, &_jittersize, &_delaysize, &_frequency);
    xc_bps = _bps;
    xc_nlines = _nlines;
    xc_nbaselines = xc_nlines*(xc_nlines-1)/2;
    xc_packetsize = (xc_nlines*2+xc_nbaselines)*xc_bps/4+16+1;
    xc_delaysize = _delaysize;
    xc_frequency = _frequency;
    xc_counts = (unsigned long*)malloc(sizeof(long)*xc_nlines);
    xc_autocorrelations = (unsigned long*)malloc(sizeof(long)*xc_nlines);
    xc_correlations = (unsigned long*)malloc(sizeof(long)*xc_nbaselines);
}

void xc_enable_capture(int enable)
{
    xc_send_command(ENABLE_CAPTURE, (unsigned char)enable);
}

void xc_select_input(int index)
{
    xc_send_command(SET_INDEX, (unsigned char)index);
}

void xc_set_rate(baud_rate rate)
{
    xc_send_command(SET_BAUD_RATE, (unsigned char)rate);
    RS232_CloseComport(comport_number);
    RS232_OpenComport(comport_number, XC_BASE_RATE<<rate, "8N2", 0);
}

void xc_set_5v(int index, int value)
{
    xc_select_input(index);
    xc_send_command(SET_LEDS, value&0x1);
}

void xc_set_ht(int index, int value)
{
    xc_select_input(index);
    xc_send_command(SET_LEDS, (value>>1)&0x1);
}

void xc_set_delay(int index, unsigned char value)
{
    xc_select_input(index);
    xc_send_command(SET_DELAY_LO, value&0xf);
    value >>= 4;
    xc_send_command(SET_DELAY_HI, value&0xf);
}

void xc_set_line(int index, unsigned char value)
{
    xc_select_input(index);
    xc_send_command(SET_LINE_LO, value&0xf);
    value >>= 4;
    xc_send_command(SET_LINE_HI, value&0xf);
}

void xc_set_frequency_divider(unsigned char value)
{
    xc_send_command(SET_FREQ_DIV, value&0x1f);
}

int xc_send_command(it_cmd c, unsigned char value)
{
    return send_char((unsigned char)((unsigned char)c|(value<<4)));
}




