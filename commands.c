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

static void ahp_xc_align_packet()
{
    unsigned char c = 0;
    int tries = ahp_xc_get_packetsize();
    while (c != '\r' && tries-- > 0)
        RS232_PollComport(&c, 1);
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
        return  RS232_SetupPort(XC_BASE_RATE, "8N2", 0);
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

ahp_xc_sample *ahp_xc_alloc_samples(unsigned long nlines, unsigned long size)
{
    unsigned long x;
    ahp_xc_sample* samples = (ahp_xc_sample*)malloc(sizeof(ahp_xc_sample)*nlines);
    memset(samples, 0, sizeof(ahp_xc_sample)*nlines);
    for(x = 0; x < nlines; x++) {
        samples[x].jitter_size = size;
        samples[x].correlations = (ahp_xc_correlation*)malloc(sizeof(ahp_xc_correlation)*size);
        memset(samples[x].correlations, 0, sizeof(ahp_xc_correlation)*size);
    }
    return samples;
}

void ahp_xc_free_samples(unsigned long nlines, ahp_xc_sample *samples)
{
    unsigned long x;
    if(samples != NULL) {
        for(x = 0; x < nlines; x++) {
            if(samples[x].correlations != NULL)
                free(samples[x].correlations);
        }
        free(samples);
    }
}

ahp_xc_packet *ahp_xc_alloc_packet()
{
    ahp_xc_packet *packet = (ahp_xc_packet*)malloc(sizeof(ahp_xc_packet));
    packet->bps = (unsigned long)ahp_xc_get_bps();
    packet->tau = (unsigned long)((1000000000000<<ahp_xc_get_frequency_divider())/ahp_xc_get_frequency());
    packet->n_lines = (unsigned long)ahp_xc_get_nlines();
    packet->n_baselines = (unsigned long)ahp_xc_get_nbaselines();
    packet->counts = (unsigned long*)malloc(packet->n_lines*sizeof(unsigned long));
    packet->autocorrelations = ahp_xc_alloc_samples((unsigned long)ahp_xc_get_nlines(), (unsigned long)ahp_xc_get_autocorrelator_jittersize());
    packet->crosscorrelations = ahp_xc_alloc_samples((unsigned long)ahp_xc_get_nbaselines(), (unsigned long)ahp_xc_get_crosscorrelator_jittersize()*2-1);
    return packet;
}

void ahp_xc_free_packet(ahp_xc_packet *packet)
{
    if(packet != NULL) {
        if(packet->counts != NULL)
            free(packet->counts);
        ahp_xc_free_samples((unsigned long)ahp_xc_get_nlines(), packet->autocorrelations);
        ahp_xc_free_samples((unsigned long)ahp_xc_get_nbaselines(), packet->crosscorrelations);
        free(packet);
    }
}

void ahp_xc_scan_crosscorrelations(ahp_xc_sample *crosscorrelations, int stacksize, double *percent, int *interrupt)
{
    int i = 0, x;
    int index1, index2;
    ahp_xc_packet *packet = ahp_xc_alloc_packet();
    for(index1 = 0; index1 < ahp_xc_get_nlines(); index1++)
        ahp_xc_set_delay(index1, 0);
    for(x = ahp_xc_get_frequency_divider(); x >= 0; x--) {
        ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)x);
        for(index1 = 0; index1 < ahp_xc_get_nlines(); ) {
            for(i = ahp_xc_get_delaysize()-1; i >= (x > 0 ? ahp_xc_get_delaysize() / 2 : 0); i --) {
                if(*interrupt)
                    break;
                for(index1 = 0; index1 < ahp_xc_get_nlines(); index1++) {
                    ahp_xc_set_delay(index1, i);
                    ahp_xc_set_line(index1, 0);
                }
                int stack = stacksize;
                while (stack-- > 0) {
                    int ntries = 5;
                    while(ahp_xc_get_packet(packet) && ntries-- > 0)
                        usleep(ahp_xc_get_packettime());
                    for(index2 = 0; index2 < ahp_xc_get_nlines(); index2++) {
                        if(index2 == index1)
                            continue;
                        int idx1 = (index1<index2?index1:index2);
                        int idx2 = (index1>index2?index1:index2);
                        int idx = (idx1*(ahp_xc_get_nlines()+ahp_xc_get_nlines()-idx1-1)/2+idx2-idx1-1);
                        int index = (i+ahp_xc_get_delaysize()*x/2);
                        index = ((index1>index2?-index:index)+(ahp_xc_get_delaysize()*(ahp_xc_get_frequency_divider()+2)/2)+(ahp_xc_get_delaysize()*(ahp_xc_get_frequency_divider()+2)+1)*idx);
                        crosscorrelations[idx].correlations[i].counts += (packet->counts[idx1]+packet->counts[idx2])/2.0;
                        crosscorrelations[idx].correlations[i].correlations += packet->crosscorrelations[idx].correlations[packet->crosscorrelations[idx].jitter_size/2].correlations;
                    }
                }
                for(x = 0; x < ahp_xc_get_nbaselines(); x++)
                    crosscorrelations[x].correlations[i].coherence = (double)crosscorrelations[x].correlations[i].correlations/(double)crosscorrelations[x].correlations[i].counts;
                (*percent) += 100.0 / (ahp_xc_get_delaysize()*ahp_xc_get_nlines()*(ahp_xc_get_frequency_divider() + 2)/2);
                index1++;
            }
        }
    }
    ahp_xc_free_packet(packet);
}

void ahp_xc_scan_autocorrelations(ahp_xc_sample *autocorrelations, int stacksize, double *percent, int *interrupt)
{
    int i = 0, x, y;
    ahp_xc_packet *packet = ahp_xc_alloc_packet();
    int index = 0;
    for(x = 0; x < ahp_xc_get_frequency_divider(); x++) {
        ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)x);
        if(x > 0) {
            i = ahp_xc_get_delaysize()/2;
        } else {
            i = 0;
        }
        while(i < ahp_xc_get_delaysize()) {
            if(*interrupt)
                break;
            ahp_xc_set_line(index, i);
            y = i+x*ahp_xc_get_delaysize()/2;
            int stack = stacksize;
            while (stack-- > 0) {
                int ntries = 5;
                while(ahp_xc_get_packet(packet) && ntries-- > 0)
                    usleep(ahp_xc_get_packettime());
                for(index = 0; index < ahp_xc_get_nlines(); index++) {
                    autocorrelations[index].correlations[y].counts += packet->counts[index];
                    autocorrelations[index].correlations[y].correlations += packet->autocorrelations[index].correlations[0].correlations;
                }
            }
            for(index = 0; index < ahp_xc_get_nlines(); index++)
                autocorrelations[index].correlations[y].coherence = (double)autocorrelations[index].correlations[y].correlations / (double)autocorrelations[index].correlations[y].counts;
            (*percent) += 100.0 / (ahp_xc_get_delaysize() * (ahp_xc_get_frequency_divider() + 2) / 2);
            i ++;
        }
    }
    ahp_xc_free_packet(packet);
}

int ahp_xc_get_packet(ahp_xc_packet *packet)
{
    int ret = 1;
    int x = 0, y = 0, z = 0;
    char *data = (char*)malloc((unsigned int)ahp_xc_get_packetsize());
    int n = ahp_xc_get_bps()/4;
    char *sample = (char*)malloc((unsigned int)n+1);
    memset(data, '0', (unsigned int)ahp_xc_get_packetsize());
    ahp_xc_enable_capture(1);
    ahp_xc_align_packet();
    ssize_t n_read = RS232_PollComport((unsigned char*)data, ahp_xc_get_packetsize());
    if(n_read != ahp_xc_get_packetsize())
        goto end;
    char *buf = data;
    buf += 16;
    buf += n*((int)packet->n_lines-1);
    for(x = 0; x < (int)packet->n_lines; x++) {
        sample[n] = 0;
        memcpy(sample, buf, (unsigned int)n);
        if(1>sscanf(sample, "%lX",  &packet->counts[x]))
            goto err_end;
        packet->counts[x] = (packet->counts[x] == 0 ? 1 : packet->counts[x]);
        buf -= n;
    }
    buf += n*ahp_xc_get_nlines()+n;
    int idx = 0;
    buf += n*(ahp_xc_get_nlines()*ahp_xc_get_autocorrelator_jittersize()-1);
    for(x = 0; x < ahp_xc_get_nlines(); x++) {
        for(y = 0; y < ahp_xc_get_autocorrelator_jittersize(); y++) {
            sample[n] = 0;
            memcpy(sample, buf, (unsigned int)n);
            if(1>sscanf(sample, "%lX",  &packet->autocorrelations[x].correlations[y].correlations))
                goto err_end;
            packet->autocorrelations[x].correlations[y].counts = packet->counts[x];
            packet->autocorrelations[x].correlations[y].coherence = (double)packet->autocorrelations[x].correlations[y].correlations/(double)packet->autocorrelations[x].correlations[y].counts;
            buf -= n;
        }
    }
    buf += n*ahp_xc_get_nlines()*ahp_xc_get_autocorrelator_jittersize()+n;
    buf += n*(ahp_xc_get_nbaselines()*(ahp_xc_get_crosscorrelator_jittersize()*2-1)-1);
    idx = 0;
    for(x = 0; x < ahp_xc_get_nlines(); x++) {
        for(y = x+1; y < ahp_xc_get_nlines(); y++) {
            for(z = 0; z < (int)packet->crosscorrelations[idx].jitter_size; z++) {
                sample[n] = 0;
                memcpy(sample, buf, (unsigned int)n);
                if(1>sscanf(sample, "%lX",  &packet->crosscorrelations[x].correlations[z].correlations))
                    goto err_end;
                packet->crosscorrelations[idx].correlations[z].counts = (packet->counts[x]+packet->counts[y])/2;
                packet->crosscorrelations[idx].correlations[z].coherence = (double)packet->crosscorrelations[idx].correlations[z].correlations/(double)packet->crosscorrelations[idx].correlations[z].counts;
                idx ++;
                buf -= n;
            }
        }
    }
    ret = 0;
    goto end;
err_end:
    ahp_xc_enable_capture(0);
end:
    free(sample);
    free(data);
    return ret;
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
    ahp_xc_packetsize = (ahp_xc_get_nlines()+ahp_xc_get_autocorrelator_jittersize()*ahp_xc_get_nlines()+(ahp_xc_get_crosscorrelator_jittersize()*2-1)*ahp_xc_get_nbaselines())*ahp_xc_get_bps()/4+16;
    ahp_xc_frequency = (int)((long)1000000000000/(long)_tau);
    return 0;
}

void ahp_xc_enable_capture(int enable)
{
    ahp_xc_send_command(ENABLE_CAPTURE, (unsigned char)enable);
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




