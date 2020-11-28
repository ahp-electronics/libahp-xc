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
static int ahp_xc_jittersize = -1;
static int ahp_xc_delaysize = -1;
static int ahp_xc_flags = 0;
static int ahp_xc_frequency = -1;
static int ahp_xc_frequency_divider = 0;
static int ahp_xc_packetsize = 4096;
static baud_rate ahp_xc_rate = R_57600;
static char ahp_xc_comport[128];


static void ahp_xc_align_packet()
{
    unsigned char c = 0;
    int tries = ahp_xc_get_packetsize();
    while (c != '\r' && tries-- > 0)
        RS232_PollComport(&c, 1);
}
static void ahp_xc_select_input(int index)
{
    int idx = 0;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
    index >>= 2;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
    index >>= 2;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
    index >>= 2;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
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

int ahp_xc_get_autocorrelator_jittersize()
{
    return (ahp_xc_flags & AHP_XC_LIVE_AUTOCORRELATOR) ? ahp_xc_jittersize : 1;
}

int ahp_xc_get_crosscorrelator_jittersize()
{
    return (ahp_xc_flags & AHP_XC_LIVE_CROSSCORRELATOR) ? ahp_xc_jittersize : 1;
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
    strncpy(ahp_xc_comport, port, strlen(port));
    if(!RS232_OpenComport(ahp_xc_comport))
        return  RS232_SetupPort(XC_BASE_RATE, "8N1", 0);
    return -1;
}

void ahp_xc_disconnect()
{
    ahp_xc_set_baudrate(R_57600);
    RS232_CloseComport();
}

static ssize_t send_char(unsigned char c)
{
    return RS232_SendByte(c);
}

void ahp_xc_scan_crosscorrelations(correlation *crosscorrelations, int stacksize, double *percent, int *interrupt)
{
    int i = 0, x;
    int index1, index2;
    correlation *data2 = (correlation*)malloc((unsigned int)ahp_xc_get_nbaselines()*(unsigned int)(ahp_xc_get_crosscorrelator_jittersize()*2-1)*sizeof(correlation));
    memset(crosscorrelations, 0, sizeof(correlation)*(unsigned int)(ahp_xc_get_delaysize()*(ahp_xc_get_frequency_divider()+2)+1)*(unsigned int)ahp_xc_get_nbaselines());
    for(index1 = 0; index1 < ahp_xc_get_nlines(); index1++)
        ahp_xc_set_delay(index1, 0);
    for(x = ahp_xc_get_frequency_divider(); x >= 0; x--) {
        ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)x);
        for(index1 = 0; index1 < ahp_xc_get_nlines(); ) {
            for(i = ahp_xc_get_delaysize()-1; i >= (x > 0 ? ahp_xc_get_delaysize() / 2 : 0); i --) {
                if(*interrupt)
                    goto stop;
                for(index1 = 0; index1 < ahp_xc_get_nlines(); index1++)
                    ahp_xc_set_line(index1, i);
                int stack = stacksize;
                while (stack-- > 0) {
                    int ntries = 5;
                    while(ahp_xc_get_packet(NULL, data2) && ntries-- > 0)
                        usleep(ahp_xc_get_packettime());
                    for(index2 = 0; index2 < ahp_xc_get_nlines(); index2++) {
                        if(index2 == index1)
                            continue;
                        int idx1 = (index1<index2?index1:index2);
                        int idx2 = (index1>index2?index1:index2);
                        int idx = (idx1*(ahp_xc_get_nlines()+ahp_xc_get_nlines()-idx1-1)/2+idx2-idx1-1);
                        int index = (i+ahp_xc_get_delaysize()*x/2);
                        index = ((index1>index2?-index:index)+(ahp_xc_get_delaysize()*(ahp_xc_get_frequency_divider()+2)/2)+(ahp_xc_get_delaysize()*(ahp_xc_get_frequency_divider()+2)+1)*idx);
                        crosscorrelations[index].counts += data2[idx*(ahp_xc_get_crosscorrelator_jittersize()*2-1)+ahp_xc_get_crosscorrelator_jittersize()-1].counts;
                        crosscorrelations[index].correlations += data2[idx*(ahp_xc_get_crosscorrelator_jittersize()*2-1)+ahp_xc_get_crosscorrelator_jittersize()-1].correlations;
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
    free(data2);
}

void ahp_xc_scan_autocorrelations(correlation *autocorrelations, int stacksize, double *percent, int *interrupt)
{
    int i = 0, x;
    correlation *data2 = (correlation*)malloc((unsigned int)ahp_xc_get_nlines()*(unsigned int)ahp_xc_get_autocorrelator_jittersize()*sizeof(correlation));
    memset(autocorrelations, 0, sizeof(correlation)*(unsigned int)(ahp_xc_get_delaysize()+ahp_xc_get_autocorrelator_jittersize()-1)*(unsigned int)ahp_xc_get_nlines()*(unsigned int)(ahp_xc_get_frequency_divider()+2)/2);
    int index = 0;
    for(x = 0; x <= ahp_xc_get_frequency_divider(); x++) {
        ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)x);
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
                while(ahp_xc_get_packet(data2, NULL) && ntries-- > 0)
                    usleep(ahp_xc_get_packettime());
                for(index = 0; index < ahp_xc_get_nlines(); index++) {
                    int idx = index * ahp_xc_get_delaysize() * (ahp_xc_get_frequency_divider() + 2) / 2;
                    idx += i;
                    idx += ahp_xc_get_delaysize() * x / 2;
                    autocorrelations[idx].counts += data2[index*ahp_xc_get_autocorrelator_jittersize()].counts;
                    autocorrelations[idx].correlations += data2[index*ahp_xc_get_autocorrelator_jittersize()].correlations;
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
    free(data2);
}

int ahp_xc_get_packet(correlation *autocorrelations, correlation *crosscorrelations)
{
    int x = 0, y = 0, z = 0;
    char *buf = (char*)malloc((unsigned int)ahp_xc_get_packetsize());
    int n = ahp_xc_get_bps()/4;
    char *sample = (char*)malloc((unsigned int)n);
    memset(buf, '0', (unsigned int)ahp_xc_get_packetsize());
    ahp_xc_enable_capture(1);
    ahp_xc_align_packet();
    ssize_t n_read = RS232_PollComport((unsigned char*)buf, ahp_xc_get_packetsize());
    if(n_read < ahp_xc_get_packetsize())
        goto err_end;
    int offset = 16+ahp_xc_get_nlines()*ahp_xc_get_autocorrelator_jittersize()*n-n;
    if(autocorrelations != NULL) {
        memset(autocorrelations, 0, sizeof(correlation)*(unsigned int)ahp_xc_get_nlines()*(unsigned int)ahp_xc_get_autocorrelator_jittersize());
        int idx = 0;
        for(x = 0; x < ahp_xc_get_nlines(); x++) {
            for(y = 0; y < ahp_xc_get_autocorrelator_jittersize(); y++) {
                strncpy(sample, buf+offset, (unsigned int)n);
                autocorrelations[idx].correlations = strtoul(sample, NULL, 16);
                autocorrelations[idx].counts = autocorrelations[x*ahp_xc_get_autocorrelator_jittersize()].correlations;
                autocorrelations[idx].coherence = (double)autocorrelations[idx].correlations/(double)autocorrelations[idx].counts;
                offset -= n;
                idx ++;
            }
        }
    }
    offset += ahp_xc_get_nlines()*ahp_xc_get_autocorrelator_jittersize()*n;
    offset += ahp_xc_get_nbaselines()*(ahp_xc_get_crosscorrelator_jittersize()*2-1)*n-n;
    if(crosscorrelations != NULL) {
        memset(crosscorrelations, 0, sizeof(correlation)*(unsigned int)ahp_xc_get_nbaselines()*((unsigned int)ahp_xc_get_crosscorrelator_jittersize()*2-1));
        int idx = 0;
        for(x = 0; x < ahp_xc_get_nlines(); x++) {
            for(y = x+1; y < ahp_xc_get_nlines(); y++) {
                for(z = 0; z < ahp_xc_get_crosscorrelator_jittersize()*2-1; z++) {
                    strncpy(sample, buf+offset, (unsigned int)n);
                    crosscorrelations[idx].correlations = strtoul(sample, NULL, 16);
                    crosscorrelations[idx].counts = (autocorrelations[x*ahp_xc_get_autocorrelator_jittersize()].counts+autocorrelations[y*ahp_xc_get_autocorrelator_jittersize()].counts)/2;
                    crosscorrelations[idx].coherence = (double)crosscorrelations[idx].correlations/(double)crosscorrelations[idx].counts;
                    offset -= n;
                    idx ++;
                }
            }
        }
    }
    free(sample);
    free(buf);
    return 0;
err_end:
    ahp_xc_enable_capture(0);
    free(sample);
    free(buf);
    return 1;
}

int ahp_xc_get_properties()
{
    unsigned char header[16];
    ssize_t n_read;
    int ntries = 50;
    retry:
    ahp_xc_enable_capture(1);
    ahp_xc_align_packet();
    n_read = RS232_PollComport(header, 16);
    ahp_xc_enable_capture(0);
    if(n_read < 0)
        return -EIO;
    if(n_read < 16 && ntries-- > 0)
        goto retry;
    if(ntries == 1)
        return -EBUSY;
    int _bps, _nlines, _delaysize, _jittersize, _flags, _tau;
    n_read = sscanf((char*)header, "%02X%02X%03X%04X%01X%04X", &_bps, &_nlines, &_delaysize, &_jittersize, &_flags, &_tau);
    if(n_read != 6)
        return -EINVAL;
    ahp_xc_bps = _bps;
    ahp_xc_nlines = _nlines+1;
    ahp_xc_nbaselines = ahp_xc_nlines*(ahp_xc_nlines-1)/2;
    ahp_xc_delaysize = _delaysize;
    ahp_xc_jittersize = _jittersize;
    ahp_xc_flags = _flags;
    ahp_xc_packetsize = (ahp_xc_get_autocorrelator_jittersize()*ahp_xc_nlines+(ahp_xc_get_crosscorrelator_jittersize()*2-1)*ahp_xc_nbaselines)*ahp_xc_bps/4+16;
    ahp_xc_frequency = (int)((long)1000000000000/(long)_tau);
    return 0;
}

void ahp_xc_enable_capture(int enable)
{
    ahp_xc_send_command(ENABLE_CAPTURE, (unsigned char)enable);
    usleep(100000);
}

void ahp_xc_set_baudrate(baud_rate rate)
{
    ahp_xc_rate = rate;
    ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)rate);
    RS232_CloseComport();
    RS232_OpenComport(ahp_xc_comport);
    RS232_SetupPort(XC_BASE_RATE<<((int)ahp_xc_rate), "8N2", 0);
}

void ahp_xc_set_leds(int index, int leds)
{
    ahp_xc_select_input(index);
    ahp_xc_send_command(SET_LEDS, (unsigned char)(leds&0xf));
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
    value = (unsigned char)(value < 0x3f ? value : 0x3f);
    ahp_xc_send_command(SET_FREQ_DIV, value);
    ahp_xc_frequency_divider = value;
}

ssize_t ahp_xc_send_command(it_cmd c, unsigned char value)
{
    return send_char((unsigned char)((unsigned char)c|(((unsigned char)(value<<4))|((unsigned char)(value>>4)&~c))));
}




