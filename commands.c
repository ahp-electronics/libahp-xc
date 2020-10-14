/*
    libahp_xc library to drive the AHP XC correlators
    Copyright (C) 2020  Ilia Platone

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

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
static int xc_packetsize = 4096;
static unsigned char xc_flags = 0;
static baud_rate xc_rate = R_57600;
static unsigned long *xc_counts = NULL;
static unsigned long *xc_autocorrelations = NULL;
static unsigned long *xc_crosscorrelations = NULL;
static char xc_comport[128];

static void xc_select_input(int index)
{
    xc_send_command(SET_INDEX, (unsigned char)index);
}

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
    return xc_frequency >> xc_frequency_divider;
}

unsigned int xc_get_packettime()
{
    return (unsigned int)10000000 * (unsigned int)xc_get_packetsize() / (unsigned int)xc_get_baudrate();
}

int xc_get_packetsize()
{
    return xc_packetsize;
}

void xc_connect_fd(int fd)
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
    RS232_SetFD(fd);
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
    if(!RS232_OpenComport(xc_comport))
        return  RS232_SetupPort(XC_BASE_RATE, "8N2", 0);
    return -1;
}

void xc_disconnect()
{
    xc_set_baudrate(R_57600, 1);
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

void xc_scan_crosscorrelations(correlation *crosscorrelations, double *percent, int *interrupt)
{
    int i = 0;
    int index1, index2;
    unsigned long *data1 = (unsigned long*)malloc((unsigned int)xc_get_nlines()*sizeof(unsigned long));
    unsigned long *data2 = (unsigned long*)malloc((unsigned int)xc_get_nbaselines()*sizeof(unsigned long));
    memset(crosscorrelations, 0, sizeof(correlation)*(unsigned int)(xc_get_delaysize()*2+1)*(unsigned int)xc_get_nbaselines());
    for(index1 = 0; index1 < xc_get_nlines(); index1++)
        xc_set_delay(index1, 0);
    xc_enable_capture(1);
    for(index1 = 0; index1 < xc_get_nlines(); index1++) {
        for(i = xc_get_delaysize()-1; i >= 0; i --) {
            if(*interrupt)
                goto stop;
            xc_get_packet(data1, NULL, data2);
            xc_set_delay(index1, i+1);
            for(index2 = 0; index2 < xc_get_nlines(); index2++) {
                if(index2 == index1)
                    continue;
                int idx1 = (index1<index2?index1:index2);
                int idx2 = (index1>index2?index1:index2);
                int idx = (idx1*(xc_get_nlines()+xc_get_nlines()-idx1-1)/2+idx2-idx1-1);
                int index = ((index1>index2?-i:i)+xc_get_delaysize()+(xc_get_delaysize()*2+1)*idx);
                crosscorrelations[index].counts = (data1[index1]+data1[index2])/2;
                crosscorrelations[index].correlations = data2[idx];
                crosscorrelations[index].coherence = (double)crosscorrelations[index].correlations/(double)crosscorrelations[index].counts;
            }
            (*percent) += 100.0 / (xc_get_delaysize()*xc_get_nlines());
        }
    }
stop:
    xc_enable_capture(0);
    free(data1);
    free(data2);
}

void xc_scan_autocorrelations(correlation *autocorrelations, double *percent, int *interrupt)
{
    int i = 0;
    unsigned long *data1 = (unsigned long*)malloc((unsigned int)xc_get_nlines()*sizeof(unsigned long));
    unsigned long *data2 = (unsigned long*)malloc((unsigned int)xc_get_nlines()*sizeof(unsigned long));
    memset(autocorrelations, 0, sizeof(correlation)*(unsigned int)xc_get_delaysize()*(unsigned int)xc_get_nlines());
    int index = 0;
    xc_enable_capture(1);
    for(i = 0; i < xc_get_delaysize(); i ++) {
        if(*interrupt)
            goto stop;
        xc_get_packet(data1, data2, NULL);
        for(index = 0; index < xc_get_nlines(); index++) {
            int idx = i+xc_get_delaysize()*index;
            autocorrelations[idx].counts = data1[index];
            autocorrelations[idx].correlations = data2[index];
            autocorrelations[idx].coherence = (double)autocorrelations[idx].correlations/(double)autocorrelations[idx].counts;
            xc_set_line(index, i+1);
        }
        (*percent) += 100.0 / xc_get_delaysize();
    }
stop:
    xc_enable_capture(0);
    free(data1);
    free(data2);
}

void xc_get_packet(unsigned long *counts, unsigned long *autocorrelations, unsigned long *crosscorrelations)
{
    int x = 0;
    char *buf = (char*)malloc((unsigned int)xc_packetsize);
    memset(buf, '0', (unsigned int)xc_packetsize);
    ssize_t n_read = RS232_PollComport((unsigned char*)buf, xc_get_packetsize());
    if(n_read != xc_get_packetsize())
        goto err_end;
    int offset = 16;
    int n = xc_bps/4;
    char *sample = (char*)malloc((unsigned int)n);
    if(counts != NULL) {
        memset(counts, 0, sizeof(unsigned long)*(unsigned int)xc_get_nlines());
        for(x = 0; x < xc_nlines; x++) {
            strncpy(sample, buf+offset+x*n, (unsigned int)n);
            counts[xc_get_nlines()-1-x] = (unsigned long)strtoul(sample, NULL, 16);
        }
    }
    offset += xc_nlines*n;
    if(autocorrelations != NULL) {
        memset(autocorrelations, 0, sizeof(unsigned long)*(unsigned int)xc_get_nlines());
        for(x = 0; x < xc_nlines; x++) {
            strncpy(sample, buf+offset+x*n, (unsigned int)n);
            autocorrelations[xc_get_nlines()-1-x] = (unsigned long)strtoul(sample, NULL, 16);
        }
    }
    offset += xc_nlines*n;
    if(crosscorrelations != NULL) {
        memset(crosscorrelations, 0, sizeof(unsigned long)*(unsigned int)xc_get_nbaselines());
        for(x = 0; x < xc_nbaselines; x++) {
            strncpy(sample, buf+offset+x*n, (unsigned int)n);
            crosscorrelations[xc_get_nbaselines()-1-x] = (unsigned long)strtoul(sample, NULL, 16);
        }
    }
    free(sample);
err_end:
    free(buf);
}

int xc_get_properties()
{
    unsigned char header[16];
    ssize_t n_read;
    retry:
    xc_enable_capture(1);
    usleep(xc_get_packettime());
    n_read = RS232_PollComport(header, 16);
    xc_enable_capture(0);
    if(n_read < 0)
        return -2;
    if(n_read < 16)
        goto retry;
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
}

void xc_set_baudrate(baud_rate rate, int setterm)
{
    xc_rate = rate;
    xc_send_command(SET_BAUD_RATE, (unsigned char)rate);
    if(setterm)
        RS232_SetupPort(XC_BASE_RATE<<xc_rate, "8N2", 0);
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
    xc_frequency_divider = value;
}

ssize_t xc_send_command(it_cmd c, unsigned char value)
{
    return send_char((unsigned char)((unsigned char)c|(((unsigned char)(value<<4))|((unsigned char)(value>>4)&~c))));
}




