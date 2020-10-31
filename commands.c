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
static int ahp_xc_bps = -1;
static int ahp_xc_nlines = -1;
static int ahp_xc_nbaselines = -1;
static int ahp_xc_delaysize = -1;
static int ahp_xc_frequency = -1;
static int ahp_xc_frequency_divider = 0;
static int ahp_xc_packetsize = 4096;
static unsigned char ahp_xc_flags = 0;
static baud_rate ahp_xc_rate = R_57600;
static unsigned long *ahp_xc_counts = NULL;
static unsigned long *ahp_xc_autocorrelations = NULL;
static unsigned long *ahp_xc_crosscorrelations = NULL;
static char ahp_xc_comport[128];

static void ahp_xc_select_input(int index)
{
    ahp_xc_send_command(SET_INDEX, (unsigned char)index);
}

static int ahp_xc_align_frame()
{
    unsigned char c = 0;
    int ret = 0;
    while (c != '\r') {
        if((ret = RS232_PollComport(&c, 1))<0)
            break;
    }
    return ret;
}

int ahp_xc_get_flags()
{
    return ahp_xc_flags;
}

int ahp_xc_get_baudrate()
{
    return XC_BASE_RATE << ahp_xc_rate;
}

int ahp_xc_get_bps()
{
    return ahp_xc_bps;
}

int ahp_xc_get_nlines()
{
    return ahp_xc_nlines;
}

int ahp_xc_get_nbaselines()
{
    return ahp_xc_nbaselines;
}

int ahp_xc_get_delaysize()
{
    return ahp_xc_delaysize;
}

int ahp_xc_get_frequency()
{
    return ahp_xc_frequency;
}

int ahp_xc_get_frequency_divider()
{
    return  ahp_xc_frequency_divider;
}

unsigned int ahp_xc_get_packettime()
{
    return (unsigned int)10000000 * (unsigned int)ahp_xc_get_packetsize() / (unsigned int)ahp_xc_get_baudrate();
}

int ahp_xc_get_packetsize()
{
    return ahp_xc_packetsize;
}

void ahp_xc_connect_fd(int fd)
{
    ahp_xc_bps = -1;
    ahp_xc_nlines = -1;
    ahp_xc_nbaselines = -1;
    ahp_xc_delaysize = -1;
    ahp_xc_frequency = -1;
    ahp_xc_packetsize = 16;
    ahp_xc_rate = R_57600;
    ahp_xc_counts = NULL;
    ahp_xc_autocorrelations = NULL;
    ahp_xc_crosscorrelations = NULL;
    RS232_SetFD(fd);
}

int ahp_xc_connect(const char *port)
{
    ahp_xc_bps = -1;
    ahp_xc_nlines = -1;
    ahp_xc_nbaselines = -1;
    ahp_xc_delaysize = -1;
    ahp_xc_frequency = -1;
    ahp_xc_packetsize = 16;
    ahp_xc_rate = R_57600;
    ahp_xc_counts = NULL;
    ahp_xc_autocorrelations = NULL;
    ahp_xc_crosscorrelations = NULL;
    strncpy(ahp_xc_comport, port, strlen(port));
    if(!RS232_OpenComport(ahp_xc_comport))
        return  RS232_SetupPort(XC_BASE_RATE, "8N2", 0);
    return -1;
}

void ahp_xc_disconnect()
{
    ahp_xc_set_baudrate(R_57600, 1);
    RS232_CloseComport();
    if(ahp_xc_counts != NULL)
        free(ahp_xc_counts);
    if(ahp_xc_autocorrelations != NULL)
        free(ahp_xc_autocorrelations);
    if(ahp_xc_crosscorrelations != NULL)
        free(ahp_xc_crosscorrelations);
}

static ssize_t send_char(unsigned char c)
{
    return RS232_SendByte(c);
}

void ahp_xc_scan_crosscorrelations(correlation *crosscorrelations, int stacksize, double *percent, int *interrupt)
{
    int i = 0, x;
    int index1, index2;
    unsigned long *data1 = (unsigned long*)malloc((unsigned int)ahp_xc_get_nlines()*sizeof(unsigned long));
    unsigned long *data2 = (unsigned long*)malloc((unsigned int)ahp_xc_get_nbaselines()*sizeof(unsigned long));
    memset(crosscorrelations, 0, sizeof(correlation)*(unsigned int)(ahp_xc_get_delaysize()*(ahp_xc_get_frequency_divider()+2)+1)*(unsigned int)ahp_xc_get_nbaselines());
    for(index1 = 0; index1 < ahp_xc_get_nlines(); index1++)
        ahp_xc_set_delay(index1, 0);
    for(x = ahp_xc_get_frequency_divider(); x >= 0; x--) {
        ahp_xc_send_command(SET_FREQ_DIV, x);
        for(index1 = 0; index1 < ahp_xc_get_nlines(); ) {
            for(i = ahp_xc_get_delaysize()-1; i >= (x > 0 ? ahp_xc_get_delaysize() / 2 : 0); i --) {
                if(*interrupt)
                    goto stop;
                for(index1 = 0; index1 < ahp_xc_get_nlines(); index1++)
                    ahp_xc_set_line(index1, i);
                int stack = stacksize;
                while (stack-- > 0) {
                    int ntries = 5;
                    while(ahp_xc_get_packet(data1, NULL, data2) && ntries-- > 0)
                        usleep(ahp_xc_get_packettime());
                    for(index2 = 0; index2 < ahp_xc_get_nlines(); index2++) {
                        if(index2 == index1)
                            continue;
                        int idx1 = (index1<index2?index1:index2);
                        int idx2 = (index1>index2?index1:index2);
                        int idx = (idx1*(ahp_xc_get_nlines()+ahp_xc_get_nlines()-idx1-1)/2+idx2-idx1-1);
                        int index = (i+ahp_xc_get_delaysize()*x/2);
                        index = ((index1>index2?-index:index)+(ahp_xc_get_delaysize()*(ahp_xc_get_frequency_divider()+2)/2)+(ahp_xc_get_delaysize()*(ahp_xc_get_frequency_divider()+2)+1)*idx);
                        crosscorrelations[index].counts += (data1[index1]+data1[index2])/2;
                        crosscorrelations[index].correlations += data2[idx];
                    }
                }
                for(index2 = 0; index2 < ahp_xc_get_nlines(); index2++) {
                    if(index2 == index1)
                        continue;
                    int idx1 = (index1<index2?index1:index2);
                    int idx2 = (index1>index2?index1:index2);
                    int idx = (idx1*(ahp_xc_get_nlines()+ahp_xc_get_nlines()-idx1-1)/2+idx2-idx1-1);
                    int index = (i+ahp_xc_get_delaysize()*x/2);
                    index = ((index1>index2?-index:index)+(ahp_xc_get_delaysize()*(ahp_xc_get_frequency_divider()+2)/2)+(ahp_xc_get_delaysize()*(ahp_xc_get_frequency_divider()+2)+1)*idx);
                    if(crosscorrelations[idx].counts > 0)
                        crosscorrelations[index].coherence = (double)crosscorrelations[index].correlations/(double)crosscorrelations[index].counts;
                }

                (*percent) += 100.0 / (ahp_xc_get_delaysize()*ahp_xc_get_nlines()*(ahp_xc_get_frequency_divider() + 2)/2);
                index1++;
            }
        }
    }
stop:
    free(data1);
    free(data2);
}

void ahp_xc_scan_autocorrelations(correlation *autocorrelations, int stacksize, double *percent, int *interrupt)
{
    int i = 0, x;
    unsigned long *data1 = (unsigned long*)malloc((unsigned int)ahp_xc_get_nlines()*sizeof(unsigned long));
    unsigned long *data2 = (unsigned long*)malloc((unsigned int)ahp_xc_get_nlines()*sizeof(unsigned long));
    memset(autocorrelations, 0, sizeof(correlation)*(unsigned int)ahp_xc_get_delaysize()*(unsigned int)ahp_xc_get_nlines()*(ahp_xc_get_frequency_divider()+2)/2);
    int index = 0;
    for(x = 0; x <= ahp_xc_get_frequency_divider(); x++) {
        ahp_xc_send_command(SET_FREQ_DIV, x);
        for(i = 0; i < ahp_xc_get_delaysize(); ) {
            if(*interrupt)
                goto stop;
            if(x > 0 && i < ahp_xc_get_delaysize() / 2) {
                i = ahp_xc_get_delaysize() / 2;
            }
            for(index = 0; index < ahp_xc_get_nlines(); index++)
                ahp_xc_set_line(index, i);
            int stack = stacksize;
            while (stack-- > 0) {
                int ntries = 5;
                while(ahp_xc_get_packet(data1, data2, NULL) && ntries-- > 0)
                    usleep(ahp_xc_get_packettime());
                for(index = 0; index < ahp_xc_get_nlines(); index++) {
                    int idx = index * ahp_xc_get_delaysize() * (ahp_xc_get_frequency_divider() + 2) / 2;
                    idx += i;
                    idx += ahp_xc_get_delaysize() * x / 2;
                    autocorrelations[idx].counts += data1[index];
                    autocorrelations[idx].correlations += data2[index];
                }
            }
            for(index = 0; index < ahp_xc_get_nlines(); index++) {
                int idx = index * ahp_xc_get_delaysize() * (ahp_xc_get_frequency_divider() + 2) / 2;
                idx += i;
                idx += ahp_xc_get_delaysize() * x / 2;
                if(autocorrelations[idx].counts > 0)
                    autocorrelations[idx].coherence = (double)autocorrelations[idx].correlations/(double)autocorrelations[idx].counts;
            }
            (*percent) += 100.0 / (ahp_xc_get_delaysize() * (ahp_xc_get_frequency_divider() + 2) / 2);
            i ++;
        }
    }
stop:
    free(data1);
    free(data2);
}

int ahp_xc_get_packet(unsigned long *counts, unsigned long *autocorrelations, unsigned long *crosscorrelations)
{
    int x = 0;
    char *buf = (char*)malloc((unsigned int)ahp_xc_packetsize);
    memset(buf, '0', (unsigned int)ahp_xc_packetsize);
    ahp_xc_align_frame();
    ssize_t n_read = RS232_PollComport((unsigned char*)buf, ahp_xc_get_packetsize());
    if(n_read < ahp_xc_get_packetsize())
        goto err_end;
    int offset = 16;
    int n = ahp_xc_bps/4;
    char *sample = (char*)malloc((unsigned int)n);
    if(counts != NULL) {
        memset(counts, 0, sizeof(unsigned long)*(unsigned int)ahp_xc_get_nlines());
        for(x = 0; x < ahp_xc_nlines; x++) {
            strncpy(sample, buf+offset+x*n, (unsigned int)n);
            counts[ahp_xc_get_nlines()-1-x] = (unsigned long)strtoul(sample, NULL, 16);
        }
    }
    offset += ahp_xc_nlines*n;
    if(autocorrelations != NULL) {
        memset(autocorrelations, 0, sizeof(unsigned long)*(unsigned int)ahp_xc_get_nlines());
        for(x = 0; x < ahp_xc_nlines; x++) {
            strncpy(sample, buf+offset+x*n, (unsigned int)n);
            autocorrelations[ahp_xc_get_nlines()-1-x] = (unsigned long)strtoul(sample, NULL, 16);
        }
    }
    offset += ahp_xc_nlines*n;
    if(crosscorrelations != NULL) {
        memset(crosscorrelations, 0, sizeof(unsigned long)*(unsigned int)ahp_xc_get_nbaselines());
        for(x = 0; x < ahp_xc_nbaselines; x++) {
            strncpy(sample, buf+offset+x*n, (unsigned int)n);
            crosscorrelations[ahp_xc_get_nbaselines()-1-x] = (unsigned long)strtoul(sample, NULL, 16);
        }
    }
    free(sample);
    free(buf);
    return 0;
err_end:
    free(buf);
    return 1;
}

int ahp_xc_get_properties()
{
    unsigned char header[16];
    ssize_t n_read;
    int ntries = 5;
    retry:
    ahp_xc_enable_capture(1);
    usleep(ahp_xc_get_packettime());
    n_read = RS232_PollComport(header, 16);
    ahp_xc_enable_capture(0);
    if(n_read < 0)
        return -2;
    if(n_read < 16 && ntries-- > 0)
        goto retry;
    if(ntries == 0)
        return -1;
    int _bps, _nlines, _flags, _delaysize, _frequency;
    n_read = sscanf((char*)header, "%02X%01X%02X%03X%08X", &_bps, &_nlines, &_flags, &_delaysize, &_frequency);
    if(n_read != 5)
        return -1;
    ahp_xc_bps = _bps;
    ahp_xc_nlines = _nlines+1;
    ahp_xc_nbaselines = ahp_xc_nlines*(ahp_xc_nlines-1)/2;
    ahp_xc_packetsize = (ahp_xc_nlines*2+ahp_xc_nbaselines)*ahp_xc_bps/4+16+1;
    ahp_xc_flags = (unsigned char)_flags;
    ahp_xc_delaysize = _delaysize;
    ahp_xc_frequency = _frequency;
    ahp_xc_counts = (unsigned long*)malloc(sizeof(long)*(unsigned int)ahp_xc_nlines);
    ahp_xc_autocorrelations = (unsigned long*)malloc(sizeof(long)*(unsigned int)ahp_xc_nlines);
    ahp_xc_crosscorrelations = (unsigned long*)malloc(sizeof(long)*(unsigned int)ahp_xc_nbaselines);
    return 0;
}

void ahp_xc_enable_capture(int enable)
{
    ahp_xc_send_command(ENABLE_CAPTURE, (unsigned char)enable);
    if(enable)
        ahp_xc_align_frame();
}

void ahp_xc_set_baudrate(baud_rate rate, int setterm)
{
    ahp_xc_rate = rate;
    ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)rate);
    if(setterm)
        RS232_SetupPort(XC_BASE_RATE<<ahp_xc_rate, "8N2", 0);
}

void ahp_xc_set_power(int index, int lv, int hv)
{
    ahp_xc_select_input(index);
    ahp_xc_send_command(SET_LEDS, (lv&1)|((hv<<1)&2));
}

void ahp_xc_set_delay(int index, int value)
{
    ahp_xc_select_input(index);
    int idx = 0;
    ahp_xc_send_command((it_cmd)(SET_DELAY|idx++), value&0x7);
    value >>= 3;
    ahp_xc_send_command((it_cmd)(SET_DELAY|idx++), value&0x7);
    value >>= 3;
    ahp_xc_send_command((it_cmd)(SET_DELAY|idx++), value&0x7);
    value >>= 3;
    ahp_xc_send_command((it_cmd)(SET_DELAY|idx++), value&0x7);
}

void ahp_xc_set_line(int index, int value)
{
    ahp_xc_select_input(index);
    int idx = 0;
    ahp_xc_send_command((it_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
    value >>= 3;
    ahp_xc_send_command((it_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
    value >>= 3;
    ahp_xc_send_command((it_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
    value >>= 3;
    ahp_xc_send_command((it_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
}

void ahp_xc_set_frequency_divider(unsigned char value)
{
    value = min(value, 0x3f);
    ahp_xc_send_command(SET_FREQ_DIV, value);
    ahp_xc_frequency_divider = value;
}

ssize_t ahp_xc_send_command(it_cmd c, unsigned char value)
{
    return send_char((unsigned char)((unsigned char)c|(((unsigned char)(value<<4))|((unsigned char)(value>>4)&~c))));
}




