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
static int ahp_xc_voltage = 0;
static int ahp_xc_test = 0;
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

void ahp_xc_scan_crosscorrelations(int index1, int index2, ahp_xc_sample **crosscorrelations, int *interrupt, double *percent)
{
    int i = 0, x, y;
    int n = ahp_xc_get_bps()/4;
    *crosscorrelations = NULL;
    if(index1==index2)
        return;
    ahp_xc_sample *correlations = ahp_xc_alloc_samples((unsigned int)(ahp_xc_get_delaysize()*2-1), (unsigned int)ahp_xc_get_crosscorrelator_jittersize());
    unsigned int size = (unsigned int)(ahp_xc_get_packetsize()*8);
    unsigned char* buf = (unsigned char*)malloc(size);
    ahp_xc_enable_capture(0);
    RS232_flushRX();
    ahp_xc_set_lag_cross(index1, 0);
    ahp_xc_set_lag_cross(index2, 0);
    ahp_xc_set_lag_auto(index1, 0);
    ahp_xc_set_lag_auto(index2, 0);
    ahp_xc_set_test(index1, SCAN_CROSS);
    ahp_xc_clear_test(index2, SCAN_CROSS);
    ahp_xc_enable_capture(1);
    i = ahp_xc_get_delaysize();
    char* sample = (char*)malloc((unsigned int)n);
    sample[n] = 0;
    (*percent) = 0;
    char *packet = (char*)buf;
    while(i-- > 0) {
        if(*interrupt)
            break;
        usleep(ahp_xc_get_packettime()*8);
        ssize_t nread = RS232_PollComport(buf, (int)size);
        if(nread < size)
            break;
        char *packet = (char*)buf;
        for(x = 0; x < 8; x++) {
            if(packet == NULL)
                continue;
            char *offset = strchr(packet, 0x0d);
            if(offset != NULL)
                offset++;
            if(offset == NULL)
                continue;
            offset += 16;
            for(y = 0; y < ahp_xc_get_crosscorrelator_jittersize()*2-1; y++) {
                memcpy(sample, &packet[n*index1], (unsigned int)n);
                correlations[i].correlations[y].counts = strtoul(sample, NULL, 16);
                memcpy(sample, &packet[n*index2], (unsigned int)n);
                correlations[i].correlations[y].counts += strtoul(sample, NULL, 16);
                packet += (ahp_xc_get_nlines()+ahp_xc_get_autocorrelator_jittersize()*ahp_xc_get_nlines())*n;
                int idx1 = (index1 < index2 ? index1 : index2);
                int idx2 = (index1 > index2 ? index1 : index2);
                packet += n*((idx1*ahp_xc_get_nlines()*2-idx1-1)/2+idx2-idx1-1);
                memcpy(sample, packet, (unsigned int)n);
                correlations[i].correlations[y].correlations = strtoul(sample, NULL, 16);
                correlations[i].correlations[y].coherence = (double)correlations[i].correlations[y].correlations / (double)correlations[i].correlations[y].counts;
                if(correlations[i].correlations[y].counts == 0 || correlations[i].correlations[y].correlations == 0 || correlations[i].correlations[y].coherence == 1.0) {
                    if(i>0) {
                        correlations[i].correlations[y].counts = correlations[i-1].correlations[y].counts;
                        correlations[i].correlations[y].correlations = correlations[i-1].correlations[y].counts;
                        correlations[i].correlations[y].coherence = correlations[i-1].correlations[y].coherence;
                    }
                }
                (*percent) += 50.0 / ahp_xc_get_delaysize() / ahp_xc_get_autocorrelator_jittersize();
            }
            packet = offset;
            i--;
        }
    }
    ahp_xc_enable_capture(0);
    ahp_xc_clear_test(index1, SCAN_CROSS);
    ahp_xc_enable_capture(0);
    RS232_flushRX();
    ahp_xc_set_lag_cross(index1, 0);
    ahp_xc_set_lag_cross(index2, 0);
    ahp_xc_set_lag_auto(index1, 0);
    ahp_xc_set_lag_auto(index2, 0);
    ahp_xc_set_test(index2, SCAN_CROSS);
    ahp_xc_clear_test(index1, SCAN_CROSS);
    ahp_xc_enable_capture(1);
    i = ahp_xc_get_delaysize();
    packet = (char*)buf;
    while(i < ahp_xc_get_delaysize()*2-1) {
        if(*interrupt)
            break;
        usleep(ahp_xc_get_packettime()*8);
        ssize_t nread = RS232_PollComport(buf, (int)size);
        if(nread < size)
            break;
        char *packet = (char*)buf;
        for(x = 0; x < 8; x++) {
            if(packet == NULL)
                continue;
            char *offset = strchr(packet, 0x0d);
            if(offset != NULL)
                offset++;
            if(offset == NULL)
                continue;
            offset += 16;
            for(y = 0; y < ahp_xc_get_crosscorrelator_jittersize()*2-1; y++) {
                memcpy(sample, &packet[n*index1], (unsigned int)n);
                correlations[i].correlations[y].counts = strtoul(sample, NULL, 16);
                memcpy(sample, &packet[n*index2], (unsigned int)n);
                correlations[i].correlations[y].counts += strtoul(sample, NULL, 16);
                packet += (ahp_xc_get_nlines()+ahp_xc_get_autocorrelator_jittersize()*ahp_xc_get_nlines())*n;
                int idx1 = (index1 < index2 ? index1 : index2);
                int idx2 = (index1 > index2 ? index1 : index2);
                packet += n*((idx1*ahp_xc_get_nlines()*2-idx1-1)/2+idx2-idx1-1);
                memcpy(sample, packet, (unsigned int)n);
                correlations[i].correlations[y].correlations = strtoul(sample, NULL, 16);
                correlations[i].correlations[y].coherence = (double)correlations[i].correlations[y].correlations / (double)correlations[i].correlations[y].counts;
                if(correlations[i].correlations[y].counts == 0 || correlations[i].correlations[y].correlations == 0 || correlations[i].correlations[y].coherence == 1.0) {
                    if(i>0) {
                        correlations[i].correlations[y].counts = correlations[i-1].correlations[y].counts;
                        correlations[i].correlations[y].correlations = correlations[i-1].correlations[y].counts;
                        correlations[i].correlations[y].coherence = correlations[i-1].correlations[y].coherence;
                    }
                }
                (*percent) += 50.0 / ahp_xc_get_delaysize() / ahp_xc_get_autocorrelator_jittersize();
            }
            packet = offset;
            i++;
        }
    }
    ahp_xc_enable_capture(0);
    ahp_xc_clear_test(index1, SCAN_CROSS);
    ahp_xc_clear_test(index2, SCAN_CROSS);
    free(buf);
    free(sample);
    *crosscorrelations = correlations;
}

void ahp_xc_scan_autocorrelations(int index, ahp_xc_sample **autocorrelations, int *interrupt, double *percent)
{
    int i = 0, x, y;
    int n = ahp_xc_get_bps()/4;
    *autocorrelations = NULL;
    ahp_xc_sample *correlations = ahp_xc_alloc_samples((unsigned int)ahp_xc_get_delaysize(), (unsigned int)ahp_xc_get_autocorrelator_jittersize());
    unsigned int size = (unsigned int)(ahp_xc_get_packetsize()*8);
    unsigned char* buf = (unsigned char*)malloc(size);
    ahp_xc_set_lag_auto(index, 0);
    ahp_xc_set_lag_cross(index, 0);
    RS232_flushRX();
    ahp_xc_set_test(index, SCAN_AUTO);
    ahp_xc_enable_capture(1);
    i = 0;
    char* sample = (char*)malloc((unsigned int)n+1);
    sample[n] = 0;
    (*percent) = 0;
    while(i < ahp_xc_get_delaysize()) {
        usleep(ahp_xc_get_packettime()*8);
        ssize_t nread = RS232_PollComport(buf, (int)size);
        if(nread < size)
            break;
        char *packet = (char*)buf;
        for(x = 0; x < 8; x++) {
            if(*interrupt || i >= ahp_xc_get_delaysize())
                break;
            if(packet == NULL)
                continue;
            char *offset = strchr(packet, 0x0d);
            if(offset != NULL)
                offset++;
            else
                continue;
            offset += 16;
            for(y = 0; y < ahp_xc_get_autocorrelator_jittersize(); y++) {
                memcpy(sample, &offset[n*index], (unsigned int)n);
                correlations[i].correlations[y].counts = strtoul(sample, NULL, 16);
                memcpy(sample, &offset[(ahp_xc_get_nlines()+ahp_xc_get_autocorrelator_jittersize()*index)*n], (unsigned int)n);
                correlations[i].correlations[y].correlations = strtoul(sample, NULL, 16);
                correlations[i].correlations[y].coherence = (double)correlations[i].correlations[y].correlations / (double)correlations[i].correlations[y].counts;
                if(correlations[i].correlations[y].counts == 0 || correlations[i].correlations[y].correlations == 0 || correlations[i].correlations[y].coherence == 1.0) {
                    if(i>0) {
                        correlations[i].correlations[y].counts = correlations[i-1].correlations[y].counts;
                        correlations[i].correlations[y].correlations = correlations[i-1].correlations[y].counts;
                        correlations[i].correlations[y].coherence = correlations[i-1].correlations[y].coherence;
                    }
                }
                (*percent) += 100.0 / ahp_xc_get_delaysize() / ahp_xc_get_autocorrelator_jittersize();
            }
            packet = offset;
            i++;
        }
    }
    free(buf);
    free(sample);
    ahp_xc_clear_test(index, SCAN_AUTO);
    ahp_xc_enable_capture(0);
    *autocorrelations = correlations;
}

int ahp_xc_get_packet(ahp_xc_packet *packet)
{
    int ret = 1;
    int x = 0, y = 0, z = 0;
    char *data = (char*)malloc((unsigned int)ahp_xc_get_packetsize()*3);
    int n = ahp_xc_get_bps()/4;
    char *sample = (char*)malloc((unsigned int)n+1);
    memset(data, '0', (unsigned int)ahp_xc_get_packetsize()*3);
    ahp_xc_enable_capture(1);
    int n_read = RS232_PollComport((unsigned char*)data, ahp_xc_get_packetsize()*3);
    if(n_read != ahp_xc_get_packetsize()*3) {
        ret = -EIO;
        goto err_end;
    }
    if(data == NULL) {
        ret = -ENODATA;
        goto err_end;
    }
    unsigned char *buf = strchr(data, 0x0d)+1;
    unsigned char *end = strchr(buf, 0x0d);
    *end = 0;
    if(strlen(buf) < ahp_xc_get_packetsize()-1) {
        ret = -EBUSY;
        goto err_end;
    }
    ahp_xc_enable_capture(0);
    if(packet == NULL) {
        ret = -EINVAL;
        goto end;
    }
    buf += 16;
    for(x = 0; x < ahp_xc_get_nlines(); x++) {
        sample[n] = 0;
        memcpy(sample, buf, (unsigned int)n);
        if(1>sscanf(sample, "%lX", &packet->counts[x])) {
            ret = -ENOENT;
            goto end;
        }
        packet->counts[x] = (packet->counts[x] == 0 ? 1 : packet->counts[x]);
        buf += n;
    }
    int idx = 0;
    for(x = 0; x < ahp_xc_get_nlines(); x++) {
        for(y = 0; y < ahp_xc_get_autocorrelator_jittersize(); y++) {
            sample[n] = 0;
            memcpy(sample, buf, (unsigned int)n);
            if(1>sscanf(sample, "%lX",  &packet->autocorrelations[x].correlations[y].correlations)) {
                ret = -ENOENT;
                goto end;
            }
            packet->autocorrelations[x].correlations[y].counts = packet->counts[x];
            packet->autocorrelations[x].correlations[y].coherence = (double)packet->autocorrelations[x].correlations[y].correlations/(double)packet->autocorrelations[x].correlations[y].counts;
            buf += n;
        }
    }
    idx = 0;
    for(x = 0; x < ahp_xc_get_nlines(); x++) {
        for(y = x+1; y < ahp_xc_get_nlines(); y++) {
            for(z = 0; z < (int)packet->crosscorrelations[idx].jitter_size; z++) {
                sample[n] = 0;
                memcpy(sample, buf, (unsigned int)n);
                if(1>sscanf(sample, "%lX",  &packet->crosscorrelations[x].correlations[z].correlations)) {
                    ret = -ENOENT;
                    goto end;
                }
                packet->crosscorrelations[idx].correlations[z].counts = (packet->counts[x]+packet->counts[y])/2;
                packet->crosscorrelations[idx].correlations[z].coherence = (double)packet->crosscorrelations[idx].correlations[z].correlations/(double)packet->crosscorrelations[idx].correlations[z].counts;
                idx ++;
                buf += n;
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
    unsigned char data[32];
    ssize_t n_read;
    int ntries = 3;
    unsigned char *buf;
    unsigned char *end;
retry:
    ahp_xc_enable_capture(1);
    n_read = RS232_PollComport((unsigned char*)data, 32);
    if(n_read <= 0)
        return -EIO;
    data[31] = 0;
    buf = strchr(data, 0x0d)+1;
    ahp_xc_enable_capture(0);
    if((int)(buf-data) >= 16 && ntries-- > 0)
        goto retry;
    if(ntries == 0)
        return -EBUSY;
    int _bps, _nlines, _delaysize, _jittersize, _flags, _tau;
    n_read = sscanf((char*)buf, "%02X%02X%03X%04X%01X%04X", &_bps, &_nlines, &_delaysize, &_jittersize, &_flags, &_tau);
    if(n_read != 6)
        return -EINVAL;
    ahp_xc_bps = _bps;
    ahp_xc_nlines = _nlines+1;
    ahp_xc_nbaselines = ahp_xc_nlines*(ahp_xc_nlines-1)/2;
    ahp_xc_delaysize = _delaysize;
    ahp_xc_jittersize = _jittersize;
    ahp_xc_flags = _flags;
    ahp_xc_packetsize = (ahp_xc_get_nlines()+ahp_xc_get_autocorrelator_jittersize()*ahp_xc_get_nlines()+(ahp_xc_get_crosscorrelator_jittersize()*2-1)*ahp_xc_get_nbaselines())*ahp_xc_get_bps()/4+16+1;
    ahp_xc_frequency = (int)((long)1000000000000/(long)(!_tau?1:_tau));
    return 0;
}

void ahp_xc_enable_capture(int enable)
{
    ahp_xc_send_command(ENABLE_CAPTURE, 0);
    usleep(ahp_xc_get_packettime()*4);
    RS232_flushRX();
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

void ahp_xc_set_lag_cross(int index, int value)
{
    ahp_xc_select_input(index);
    int idx = 0;
    ahp_xc_send_command((xc_cmd)(SET_DELAY|idx++), value&0x7);
    value >>= 3;
    ahp_xc_send_command((xc_cmd)(SET_DELAY|idx++), value&0x7);
    value >>= 3;
    ahp_xc_send_command((xc_cmd)(SET_DELAY|idx++), value&0x7);
    value >>= 3;
    ahp_xc_send_command((xc_cmd)(SET_DELAY|idx++), value&0x7);
    ahp_xc_get_packet(NULL);
}

void ahp_xc_set_lag_auto(int index, int value)
{
    ahp_xc_select_input(index);
    int idx = 0;
    ahp_xc_send_command((xc_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
    value >>= 3;
    ahp_xc_send_command((xc_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
    value >>= 3;
    ahp_xc_send_command((xc_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
    value >>= 3;
    ahp_xc_send_command((xc_cmd)(SET_DELAY|idx++), 0x8|(value&0x7));
    ahp_xc_get_packet(NULL);
}

void ahp_xc_set_frequency_divider(unsigned char value)
{
    value = (unsigned char)(value < 0xf ? value : 0xf);
    ahp_xc_send_command(SET_FREQ_DIV, value);
    ahp_xc_frequency_divider = value;
}

void ahp_xc_set_voltage(int index, unsigned char value)
{
    ahp_xc_select_input(index);
    value = (unsigned char)(value < 0xf ? value : 0xf);
    ahp_xc_send_command(SET_VOLTAGE, value);
    ahp_xc_voltage = value;
}

void ahp_xc_set_test(int index, xc_test value)
{
    ahp_xc_select_input(index);
    ahp_xc_test |= value;
    ahp_xc_send_command(ENABLE_TEST, ahp_xc_test);
}

void ahp_xc_clear_test(int index, xc_test value)
{
    ahp_xc_select_input(index);
    ahp_xc_test &= ~value;
    ahp_xc_send_command(ENABLE_TEST, ahp_xc_test);
}

 ssize_t ahp_xc_send_command(xc_cmd c, unsigned char value)
{
    usleep(100);
    return send_char((unsigned char)((unsigned char)c|(((unsigned char)(value<<4))|((unsigned char)(value>>4)&~c))));
}
