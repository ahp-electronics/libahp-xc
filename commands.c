#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "rs232.h"
#include "ahp_xc.h"
static int xc_bps = -1;
static int xc_nlines = -1;
static int xc_nbaselines = -1;
static int xc_delaysize = -1;
static int xc_frequency = -1;
static int xc_frequency_divider = 0;
static int xc_packetsize = 16;
static unsigned char xc_flags = 0;
static baud_rate xc_rate = R_57600;
static unsigned long *xc_counts = NULL;
static unsigned long *xc_autocorrelations = NULL;
static unsigned long *xc_crosscorrelations = NULL;
static char xc_comport[128];

int xc_get_flags()
{
    return xc_flags;
}

int xc_get_baudrate()
{
    return XC_BASE_RATE << xc_rate;
}

int xc_get_bps()
{
    return xc_bps;
}

int xc_get_nlines()
{
    return xc_nlines;
}

int xc_get_nbaselines()
{
    return xc_nbaselines;
}

int xc_get_delaysize()
{
    return xc_delaysize;
}

int xc_get_frequency()
{
    return xc_frequency;
}

unsigned int xc_get_packettime()
{
    return (unsigned int)10000000 * (unsigned int)xc_get_packetsize() / (unsigned int)xc_get_baudrate();
}

int xc_get_packetsize()
{
    return xc_packetsize;
}

int xc_connect(const char *port)
{
    xc_bps = -1;
    xc_nlines = -1;
    xc_nbaselines = -1;
    xc_delaysize = -1;
    xc_frequency = -1;
    xc_packetsize = 16;
    xc_rate = R_57600;
    xc_counts = NULL;
    xc_autocorrelations = NULL;
    xc_crosscorrelations = NULL;
    strncpy(xc_comport, port, strlen(port));
    return RS232_OpenComport(xc_comport, XC_BASE_RATE, "8N2", 0);
}

void xc_disconnect()
{
    xc_set_rate(R_57600);
    RS232_CloseComport();
    if(xc_counts != NULL)
        free(xc_counts);
    if(xc_autocorrelations != NULL)
        free(xc_autocorrelations);
    if(xc_crosscorrelations != NULL)
        free(xc_crosscorrelations);
}

static ssize_t send_char(unsigned char c)
{
    return RS232_SendByte(c);
}

void xc_scan_crosscorrelations(unsigned long *crosscorrelations, double *percent)
{
    int i = 0;
    int index1, index2;
    char *buf = (char*)malloc((unsigned int)xc_packetsize);
    memset(crosscorrelations, 0, sizeof(unsigned long)*(unsigned int)(xc_get_delaysize()*2+1)*(unsigned int)xc_get_nbaselines());
    int n = (int)xc_get_bps()/4;
    char *sample = (char*)malloc((unsigned int)n);
    for(index1 = 0; index1 < xc_get_nlines(); index1++)
        xc_set_delay(index1, 0);
    for(index1 = 0; index1 < xc_get_nlines(); index1++) {
        for(i = xc_get_delaysize()-1; i >= 0; i --) {
            xc_set_delay(index1, i);
            for(index2 = 0; index2 < xc_get_nlines(); index2++) {
                if(index2 == index1)
                    continue;
                int idx1 = (index1<index2?index1:index2);
                int idx2 = (index1>index2?index1:index2);
                int idx = (idx1*(xc_get_nlines()+xc_get_nlines()-idx1-1)/2+idx2-idx1-1);
                xc_enable_capture(1);
                xc_align_frame();
                ssize_t n_read = RS232_PollComport((unsigned char*)buf, (int)xc_packetsize);
                xc_enable_capture(0);
                if(n_read == xc_packetsize) {
                    int offset = (int)xc_get_nlines()*2+((int)xc_get_nbaselines()-1-idx);
                    char* packet = buf+16+offset*n;
                    strncpy(sample, packet, (unsigned int)n);
                    crosscorrelations[(index1>index2?-i:i)+xc_get_delaysize()+(xc_get_delaysize()*2+1)*idx] = (unsigned long)strtol(sample, NULL, 16);
                    (*percent) += 50.0 / (xc_get_delaysize()*xc_get_nbaselines());
                } else index2--;
            }
        }
    }
    free(buf);
    free(sample);
}

void xc_scan_spectrum(unsigned long *spectrum, double *percent)
{
    unsigned char i = 0;
    char *buf = (char*)malloc((unsigned int)xc_packetsize);
    memset(spectrum, 0, sizeof(unsigned long)*(unsigned int)xc_get_delaysize()*(unsigned int)xc_get_nlines());
    int n = (int)xc_get_bps()/4;
    int index = 0;
    char *sample = (char*)malloc((unsigned int)n);
    for(index = 0; index < xc_get_nlines(); index++) {
        xc_set_line(index, 0);
    }
    for(i = 0; i < xc_get_delaysize(); i ++) {
        xc_enable_capture(1);
        xc_align_frame();
        ssize_t n_read = RS232_PollComport((unsigned char*)buf, (int)xc_packetsize);
        xc_enable_capture(0);
        if(n_read == xc_packetsize) {
            for(index = 0; index < xc_get_nlines(); index++) {
                int offset = (int)xc_get_nlines() + ((int)xc_get_nlines()-1-index);
                char* packet = buf + 16 + offset * n;
                strncpy(sample, packet, (unsigned int)n);
                spectrum[i+xc_get_delaysize()*index] = (unsigned long)strtol(sample, NULL, 16);
                xc_set_line(index, i+1);
            }
            (*percent) += 100.0 / (xc_get_delaysize()*xc_get_nlines());
        } else i--;
    }
    free(buf);
    free(sample);
}

int xc_align_frame()
{
    unsigned char c = 0;
    int ret = 0;
    while (c != '\r') {
        if((ret = RS232_PollComport(&c, 1))<0)
            break;
    }
    return ret;
}

void xc_get_packet(unsigned long *counts, unsigned long *autocorrelations, unsigned long *crosscorrelations)
{
    int x = 0;
    char *buf = (char*)malloc((unsigned int)xc_packetsize);
    memset(buf, 0, (unsigned int)xc_packetsize);
    memset(counts, 0, sizeof(unsigned long)*(unsigned int)xc_get_nlines());
    memset(autocorrelations, 0, sizeof(unsigned long)*(unsigned int)xc_get_nlines());
    memset(crosscorrelations, 0, sizeof(unsigned long)*(unsigned int)xc_get_nbaselines());
    xc_align_frame();
    ssize_t n_read = RS232_PollComport((unsigned char*)buf, xc_get_packetsize());
    if(n_read != xc_get_packetsize())
        goto err_end;
    int offset = 16;
    int n = xc_bps/4;
    char *sample = (char*)malloc((unsigned int)n);
    for(x = 0; x < xc_nlines; x++) {
        strncpy(sample, buf+offset+x*n, (unsigned int)n);
        counts[xc_get_nlines()-1-x] = (unsigned long)strtoul(sample, NULL, 16);
    }
    offset += xc_nlines*n;
    for(x = 0; x < xc_nlines; x++) {
        strncpy(sample, buf+offset+x*n, (unsigned int)n);
        autocorrelations[xc_get_nlines()-1-x] = (unsigned long)strtoul(sample, NULL, 16);
    }
    offset += xc_nlines*n;
    for(x = 0; x < xc_nbaselines; x++) {
        strncpy(sample, buf+offset+x*n, (unsigned int)n);
        crosscorrelations[xc_get_nbaselines()-1-x] = (unsigned long)strtoul(sample, NULL, 16);
    }
    free(sample);
err_end:
    free(buf);
}

int xc_get_properties()
{
    unsigned char header[16];
    ssize_t n_read;
    xc_enable_capture(1);
    n_read = RS232_PollComport(header, 16);
    xc_enable_capture(0);
    xc_enable_capture(1);
    n_read = RS232_PollComport(header, 16);
    xc_enable_capture(0);
    if(n_read != 16)
        return -2;
    int _bps, _nlines, _flags, _delaysize, _frequency;
    n_read = sscanf((char*)header, "%02X%01X%02X%03X%08X", &_bps, &_nlines, &_flags, &_delaysize, &_frequency);
    if(n_read != 5)
        return -1;
    xc_bps = _bps;
    xc_nlines = _nlines+1;
    xc_nbaselines = xc_nlines*(xc_nlines-1)/2;
    xc_packetsize = (xc_nlines*2+xc_nbaselines)*xc_bps/4+16;
    xc_flags = (unsigned char)_flags;
    xc_delaysize = _delaysize;
    xc_frequency = _frequency;
    xc_counts = (unsigned long*)malloc(sizeof(long)*(unsigned int)xc_nlines);
    xc_autocorrelations = (unsigned long*)malloc(sizeof(long)*(unsigned int)xc_nlines);
    xc_crosscorrelations = (unsigned long*)malloc(sizeof(long)*(unsigned int)xc_nbaselines);
    return 0;
}

void xc_enable_capture(int enable)
{
    xc_send_command(ENABLE_CAPTURE, (unsigned char)enable);
    if(enable)
        xc_align_frame();
}

void xc_select_input(int index)
{
    xc_send_command(SET_INDEX, (unsigned char)index);
}

void xc_set_rate(baud_rate rate)
{
    xc_rate = rate;
    xc_send_command(SET_BAUD_RATE, (unsigned char)rate);
    RS232_CloseComport();
    RS232_OpenComport(xc_comport, XC_BASE_RATE<<xc_rate, "8N2", 0);
}

void xc_set_power(int index, int lv, int hv)
{
    xc_select_input(index);
    xc_send_command(SET_LEDS, (lv&1)|((hv<<1)&2));
}

void xc_set_delay(int index, int value)
{
    xc_select_input(index);
    int idx = 0;
    xc_send_command((it_cmd)(SET_DELAY|idx++), value&0x7);
    value >>= 3;
    xc_send_command((it_cmd)(SET_DELAY|idx++), value&0x7);
    value >>= 3;
    xc_send_command((it_cmd)(SET_DELAY|idx++), value&0x7);
    value >>= 3;
    xc_send_command((it_cmd)(SET_DELAY|idx++), value&0x7);
}

void xc_set_line(int index, int value)
{
    xc_select_input(index);
    int idx = 0;
    xc_send_command((it_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
    value >>= 3;
    xc_send_command((it_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
    value >>= 3;
    xc_send_command((it_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
    value >>= 3;
    xc_send_command((it_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
}

void xc_set_frequency_divider(unsigned char value)
{
    value = min(value, 0x3f);
    xc_send_command(SET_FREQ_DIV, value);
    xc_get_properties();
    xc_frequency_divider = value;
}

ssize_t xc_send_command(it_cmd c, unsigned char value)
{
    return send_char((unsigned char)((unsigned char)c|(((unsigned char)(value<<4)|(unsigned char)(value>>4))&~c)));
}




