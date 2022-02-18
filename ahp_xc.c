/*
    MIT License

    libahp_xc library to drive the AHP XC correlators
    Copyright (C) 2020  Ilia Platone

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <sys/time.h>
#include "rs232.h"
#include "ahp_xc.h"
#ifndef AIRY
#define AIRY 1.21966
#endif
static int xc_current_input = 0;
static long sign = 1;
static long fill = 0;
static unsigned char *ahp_xc_test = NULL;
static unsigned char *ahp_xc_leds = NULL;
static unsigned int ahp_xc_bps = 0;
static unsigned int ahp_xc_nlines = 0;
static unsigned int ahp_xc_nbaselines = 0;
static unsigned int ahp_xc_auto_lagsize = 0;
static unsigned int ahp_xc_cross_lagsize = 0;
static unsigned int ahp_xc_delaysize = 0;
static unsigned int ahp_xc_flags = 0;
static unsigned int ahp_xc_frequency = 1;
static unsigned int ahp_xc_frequency_divider = 0;
static unsigned int ahp_xc_voltage = 0;
static unsigned int ahp_xc_connected = 0;
static unsigned int ahp_xc_detected = 0;
static unsigned int ahp_xc_packetsize = 4096;
static int ahp_xc_baserate = XC_BASE_RATE;
static baud_rate ahp_xc_rate = R_BASE;
static char ahp_xc_comport[128];
static char ahp_xc_header[17] = { 0 };
static unsigned char ahp_xc_capture_flags = 0;

static void complex_phase_magnitude(ahp_xc_correlation *sample)
{
    if(!ahp_xc_connected) return;
    sample->magnitude = (double)sqrt(pow((double)sample->real, 2)+pow((double)sample->imaginary, 2));
    double rad = 0.0;
    if(sample->magnitude > 0.0) {
        rad = acos ((double)sample->imaginary / sample->magnitude);
        if(sample->real < 0 && rad != 0.0)
            rad = M_PI*2-rad;
    }
    sample->phase = rad;
}

int calc_checksum(char *data)
{
    if(!ahp_xc_connected) return -ENOENT;
    int x;
    unsigned int checksum = 0x00;
    unsigned int calculated_checksum = 0;
    unsigned char v = data[ahp_xc_get_packetsize()-3];
    checksum = v < 'A' ? (v - '0') : (v - 'A' + 10);
    checksum *= 16;
    v = data[ahp_xc_get_packetsize()-2];
    checksum += v < 'A' ? (v - '0') : (v - 'A' + 10);
    for(x = 16; x < (int)ahp_xc_get_packetsize()-3; x++) {
        calculated_checksum += data[x] < 'A' ? (data[x] - '0') : (data[x] - 'A' + 10);
        calculated_checksum &= 0xff;
    }
    if(checksum != calculated_checksum) {
        return -EINVAL;
    }
    return 0;
}

static int grab_next_packet(unsigned char* buf)
{
    if(!ahp_xc_connected) return -ENOENT;
    int err = 0;
    unsigned int size = ahp_xc_get_packetsize();
    memset(buf, 0, (unsigned int)size);
    if(size == 16)
        err = RS232_AlignFrame('\r', 4096);
    if(err)
        return -ENODATA;
    int nread = RS232_RecvBuf(buf, (int)size);
    if(nread < 0) {
        err = -ETIMEDOUT;
    } else {
        if(size > 16) {
            off_t len = (off_t)(strchr((char*)buf, '\r')-(char*)buf);
            if(len < size-1) {
                if(strncmp(ahp_xc_get_header(), (char*)buf, 16)) {
                    err = -EINVAL;
                } else {
                    err = -EPIPE;
                }
                RS232_AlignFrame('\r', (int)size);
            }
        }
    }
    if(strlen((char*)buf) < size) {
        err = -ENODATA;
    }
    if(size > 16 && !err)
        return calc_checksum((char*)buf);
    return err;
}

static char* grab_packet()
{
    if(!ahp_xc_connected) return NULL;
    char *buf = (char*)malloc(ahp_xc_get_packetsize());
    int err = 0;
    int max_errored = 8;
    while (err != -ETIMEDOUT && max_errored-- > 0) {
        err = grab_next_packet((unsigned char*)buf);
        if(!err)
            break;
    }
    if(err)
        return NULL;
    return buf;
}

unsigned int ahp_xc_current_input()
{
    return xc_current_input;
}

void ahp_xc_select_input(unsigned int index)
{
    if(!ahp_xc_detected) return;
    int idx = 0;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
    index >>= 2;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
    index >>= 2;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
    index >>= 2;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
    xc_current_input = index;
}

int ahp_xc_has_crosscorrelator()
{
    if(!ahp_xc_detected) return 0;
    return (ahp_xc_flags & HAS_CROSSCORRELATOR ? 1 : 0);
}

int ahp_xc_has_psu()
{
    if(!ahp_xc_detected) return 0;
    return (ahp_xc_flags & HAS_PSU ? 1 : 0);
}

int ahp_xc_has_leds()
{
    if(!ahp_xc_detected) return 0;
    return (ahp_xc_flags & HAS_LEDS ? 1 : 0);
}

int ahp_xc_has_differential_only()
{
    if(!ahp_xc_detected) return 0;
    return (ahp_xc_flags & HAS_DIFFERENTIAL_ONLY ? 1 : 0);
}

char* ahp_xc_get_header()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_header;
}

int ahp_xc_get_baudrate()
{
    if(!ahp_xc_detected) return 0;
    return XC_BASE_RATE << ahp_xc_rate;
}

unsigned int ahp_xc_get_bps()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_bps;
}

unsigned int ahp_xc_get_nlines()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_nlines;
}

unsigned int ahp_xc_get_nbaselines()
{
    if(!ahp_xc_detected) return 0;
    if(!ahp_xc_has_crosscorrelator())
        return 0;
    return ahp_xc_nbaselines;
}

unsigned int ahp_xc_get_delaysize()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_delaysize;
}

unsigned int ahp_xc_get_autocorrelator_lagsize()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_auto_lagsize;
}

unsigned int ahp_xc_get_crosscorrelator_lagsize()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_cross_lagsize;
}

unsigned int ahp_xc_get_frequency()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_frequency;
}

unsigned int ahp_xc_get_frequency_divider()
{
    if(!ahp_xc_detected) return 0;
    return  ahp_xc_frequency_divider;
}

double ahp_xc_get_sampletime()
{
    return pow(2.0, (double)ahp_xc_get_frequency_divider())/(double)ahp_xc_get_frequency();
}

double ahp_xc_get_packettime()
{
    if(!ahp_xc_detected) return 0;
    return  10.0  * (double)ahp_xc_get_packetsize() / (double)ahp_xc_get_baudrate();
}

unsigned int ahp_xc_get_packetsize()
{
    return ahp_xc_packetsize;
}

int ahp_xc_get_fd()
{
    return RS232_GetFD();
}

int ahp_xc_connect_fd(int fd)
{
    ahp_xc_bps = 0;
    ahp_xc_nlines = 0;
    ahp_xc_nbaselines = 0;
    ahp_xc_delaysize = 0;
    ahp_xc_frequency = 0;
    ahp_xc_packetsize = 16;
    ahp_xc_rate = R_BASE;
    if(fd > -1) {
        ahp_xc_connected = 1;
        ahp_xc_detected = 0;
        RS232_SetFD(fd, XC_BASE_RATE);
        xc_current_input = 0;
        return 0;
    }
    return 1;
}

int ahp_xc_connect(const char *port, int high_rate)
{
    if(ahp_xc_connected)
        return 0;
    ahp_xc_header[0] = 0;
    ahp_xc_header[16] = 0;
    int ret = 1;
    ahp_xc_bps = 0;
    ahp_xc_nlines = 0;
    ahp_xc_nbaselines = 0;
    ahp_xc_delaysize = 0;
    ahp_xc_frequency = 0;
    ahp_xc_packetsize = 16;
    ahp_xc_baserate = (high_rate ? XC_HIGH_RATE : XC_BASE_RATE);
    ahp_xc_rate = R_BASE;
    strcpy(ahp_xc_comport, port);
    if(!RS232_OpenComport(ahp_xc_comport))
        ret = RS232_SetupPort(ahp_xc_baserate, "8N2", 0);
    if(!ret) {
        xc_current_input = 0;
        ahp_xc_connected = 1;
        ahp_xc_detected = 0;
    }
    return ret;
}
void ahp_xc_disconnect()
{
    if(ahp_xc_connected) {
        ahp_xc_connected = 0;
        ahp_xc_detected = 0;
        ahp_xc_bps = 0;
        ahp_xc_nlines = 0;
        ahp_xc_nbaselines = 0;
        ahp_xc_delaysize = 0;
        ahp_xc_frequency = 0;
        ahp_xc_packetsize = 16;
        ahp_xc_set_baudrate(ahp_xc_baserate);
        RS232_CloseComport();
    }
}

unsigned int ahp_xc_is_connected()
{
    return ahp_xc_connected;
}

unsigned int ahp_xc_is_detected()
{
    return ahp_xc_detected;
}

ahp_xc_sample *ahp_xc_alloc_samples(unsigned long nlines, unsigned long size)
{
    unsigned long x;
    ahp_xc_sample* samples = (ahp_xc_sample*)malloc(sizeof(ahp_xc_sample)*nlines);
    memset(samples, 0, sizeof(ahp_xc_sample)*nlines);
    for(x = 0; x < nlines; x++) {
        samples[x].lag_size = size;
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
    packet->autocorrelations = ahp_xc_alloc_samples((unsigned long)ahp_xc_get_nlines(), (unsigned long)ahp_xc_get_autocorrelator_lagsize());
    packet->crosscorrelations = ahp_xc_alloc_samples((unsigned long)ahp_xc_get_nbaselines(), (unsigned long)ahp_xc_get_crosscorrelator_lagsize()*2-1);
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

void ahp_xc_start_crosscorrelation_scan(unsigned int index, off_t start, size_t size)
{
    if(!ahp_xc_detected) return;
    ahp_xc_end_crosscorrelation_scan(index);
    ahp_xc_set_capture_flags((ahp_xc_get_capture_flags()|CAP_RESET_TIMESTAMP)&~CAP_ENABLE);
    ahp_xc_set_channel_cross(index, start, size);
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|SCAN_CROSS);
    ahp_xc_set_capture_flags((ahp_xc_get_capture_flags()|CAP_ENABLE)&~CAP_RESET_TIMESTAMP);
}

void ahp_xc_end_crosscorrelation_scan(unsigned int index)
{
    if(!ahp_xc_detected) return;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~SCAN_CROSS);
    ahp_xc_set_channel_cross(index, 0, 0);
    grab_packet();
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_ENABLE);
}

void ahp_xc_get_crosscorrelation(ahp_xc_sample *sample, int index1, int index2, const char *data, double lag)
{
    unsigned int y;
    int n = ahp_xc_get_bps() / 4;
    const char *packet = data;
    char *subpacket = (char*)malloc(n+1);
    sample->lag_size = (ahp_xc_get_crosscorrelator_lagsize()*2-1);
    sample->lag = lag;
    packet += 16;
    unsigned long counts = 0;
    memcpy(subpacket, &packet[index1*n], (unsigned int)n);
    counts += strtoul(subpacket, NULL, 16)|1;
    memcpy(subpacket, &packet[index2*n], (unsigned int)n);
    counts += strtoul(subpacket, NULL, 16)|1;
    packet += n*ahp_xc_get_nlines();
    packet += n*ahp_xc_get_autocorrelator_lagsize()*ahp_xc_get_nlines()*2;
    packet += n*(ahp_xc_get_crosscorrelator_lagsize()*2-1)*((index1*(ahp_xc_get_nlines()*2-index1-1))/2+index2-index1-1)*2;
    for(y = 0; y < sample->lag_size; y++) {
        sample->correlations[y].lag = sample->lag+y*ahp_xc_get_sampletime();
        sample->correlations[y].counts = counts;
        memcpy(subpacket, packet, (unsigned int)n);
        sscanf(subpacket, "%lX",  &sample->correlations[y].real);
        if(sample->correlations[y].real >= sign) {
            sample->correlations[y].real ^= fill;
            sample->correlations[y].real ++;
            sample->correlations[y].real = ~sample->correlations[y].real;
            sample->correlations[y].real ++;
        }
        packet += n;
        memcpy(subpacket, packet, (unsigned int)n);
        sscanf(subpacket, "%lX",  &sample->correlations[y].imaginary);
        if(sample->correlations[y].imaginary >= sign) {
            sample->correlations[y].imaginary ^= fill;
            sample->correlations[y].imaginary ++;
            sample->correlations[y].imaginary = ~sample->correlations[y].imaginary;
            sample->correlations[y].imaginary ++;
        }
        packet += n;
        complex_phase_magnitude(&sample->correlations[y]);
    }
    free(subpacket);
}

int ahp_xc_scan_crosscorrelations(unsigned int index1, unsigned int index2, ahp_xc_sample **crosscorrelations, off_t start1, size_t size1, off_t start2, size_t size2, int *interrupt, double *percent)
{
    if(!ahp_xc_detected) return 0;
    int r = -1;
    int f = -1;
    unsigned int n = ahp_xc_get_bps()/4;
    *crosscorrelations = NULL;
    unsigned int idx1 = (index1 < index2 ? index1 : index2);
    unsigned int idx2 = (index1 > index2 ? index1 : index2);
    if(index1 == index2)
        return r;
    r++;
    f++;
    start1 = (start1 < ahp_xc_get_delaysize()-2 ? start1 : (off_t)ahp_xc_get_delaysize()-2);
    start2 = (start2 < ahp_xc_get_delaysize()-2 ? start2 : (off_t)ahp_xc_get_delaysize()-2);
    char* data = (char*)malloc(ahp_xc_get_packetsize()*(size1+size2));
    ahp_xc_sample *correlations = ahp_xc_alloc_samples((unsigned int)(size1+size2), (unsigned int)ahp_xc_get_crosscorrelator_lagsize());
    char* sample = (char*)malloc((unsigned int)n+1);
    sample[n] = 0;
    (*percent) = 0;
    ahp_xc_set_channel_cross(index2, start2, 0);
    ahp_xc_start_crosscorrelation_scan(index1, start1, size1);
    int i = size1;
    while(r < (int)size1) {
        if(*interrupt)
            break;
        char* buf = grab_packet();
        if(!buf)
            continue;
        memcpy(data+i*ahp_xc_get_packetsize(), buf, ahp_xc_get_packetsize());
        i--;
        free(buf);
        (*percent) += 100.0 / (size1+size2);
        r++;
    }
    ahp_xc_end_crosscorrelation_scan(index1);
    ahp_xc_set_channel_cross(index1, start1, 0);
    ahp_xc_start_crosscorrelation_scan(index2, start2, size2);
    i = size1;
    while(f < (int)size2) {
        if(*interrupt)
            break;
        char* buf = grab_packet();
        if(!buf)
            continue;
        memcpy(data+i*ahp_xc_get_packetsize(), buf, ahp_xc_get_packetsize());
        i++;
        free(buf);
        (*percent) += 100.0 / (size1+size2);
        f++;
    }
    ahp_xc_end_crosscorrelation_scan(index2);
    char timestamp[16];
    double ts = 0.0;
    i = size1;
    while(i >= 0) {
        if(*interrupt)
            break;
        char *packet = (char*)data+i*ahp_xc_get_packetsize();
        strncpy(timestamp, &packet[ahp_xc_get_packetsize()-19], 16);
        ts = (double)strtoul(timestamp, NULL, 16) / 1000000000.0;
        ahp_xc_get_crosscorrelation(&correlations[i], idx1, idx2, packet, -ts);
        i--;
    }
    i = size1;
    while(i < (int)(size1+size2)) {
        if(*interrupt)
            break;
        char *packet = (char*)data+i*ahp_xc_get_packetsize();
        strncpy(timestamp, &packet[ahp_xc_get_packetsize()-19], 16);
        ts = (double)strtoul(timestamp, NULL, 16) / 1000000000.0;
        ahp_xc_get_crosscorrelation(&correlations[i], idx1, idx2, packet, ts);
        i++;
    }
    free(data);
    free(sample);
    *crosscorrelations = correlations;
    return i;
}

void ahp_xc_start_autocorrelation_scan(unsigned int index, off_t start, size_t size)
{
    if(!ahp_xc_detected) return;
    ahp_xc_set_capture_flags((ahp_xc_get_capture_flags()|CAP_RESET_TIMESTAMP)&~CAP_ENABLE);
    ahp_xc_set_channel_auto(index, start, size);
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|SCAN_AUTO);
    ahp_xc_set_capture_flags((ahp_xc_get_capture_flags()|CAP_ENABLE)&~CAP_RESET_TIMESTAMP);
}

void ahp_xc_end_autocorrelation_scan(unsigned int index)
{
    if(!ahp_xc_detected) return;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~SCAN_AUTO);
    ahp_xc_set_channel_auto(index, 0, 0);
    grab_packet();
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_ENABLE);
}

void ahp_xc_get_autocorrelation(ahp_xc_sample *sample, int index, const char *data, double lag)
{
    unsigned int y;
    int n = ahp_xc_get_bps() / 4;
    const char *packet = data;
    char *subpacket = (char*)malloc(n+1);
    sample->lag_size = ahp_xc_get_autocorrelator_lagsize();
    sample->lag = lag;
    packet += 16;
    memcpy(subpacket, &packet[index*n], (unsigned int)n);
    unsigned long counts = strtoul(subpacket, NULL, 16)|1;
    packet += n*ahp_xc_get_nlines();
    packet += n*index*ahp_xc_get_autocorrelator_lagsize()*2;
    for(y = 0; y < sample->lag_size; y++) {
        sample->correlations[y].lag = sample->lag+y*ahp_xc_get_sampletime();
        sample->correlations[y].counts = counts;
        memcpy(subpacket, packet, (unsigned int)n);
        sscanf(subpacket, "%lX",  &sample->correlations[y].real);
        if(sample->correlations[y].real >= sign) {
            sample->correlations[y].real ^= fill;
            sample->correlations[y].real ++;
            sample->correlations[y].real = ~sample->correlations[y].real;
            sample->correlations[y].real ++;
        }
        packet += n;
        memcpy(subpacket, packet, (unsigned int)n);
        sscanf(subpacket, "%lX",  &sample->correlations[y].imaginary);
        if(sample->correlations[y].imaginary >= sign) {
            sample->correlations[y].imaginary ^= fill;
            sample->correlations[y].imaginary ++;
            sample->correlations[y].imaginary = ~sample->correlations[y].imaginary;
            sample->correlations[y].imaginary ++;
        }
        packet += n;
        complex_phase_magnitude(&sample->correlations[y]);
    }
    free(subpacket);
}

int ahp_xc_scan_autocorrelations(unsigned int index, ahp_xc_sample **autocorrelations, off_t start, unsigned int len, int *interrupt, double *percent)
{
    if(!ahp_xc_detected) return 0;
    int r = -1;
    unsigned int n = ahp_xc_get_bps()/4;
    int i = 0;
    *autocorrelations = NULL;
    ahp_xc_sample *correlations = ahp_xc_alloc_samples(len, (unsigned int)ahp_xc_get_autocorrelator_lagsize());
    char* sample = (char*)malloc((unsigned int)n+1);
    sample[n] = 0;
    (*percent) = 0;
    r++;
    start = (start < ahp_xc_get_delaysize()-2 ? start : (off_t)ahp_xc_get_delaysize()-2);
    len = (start+(off_t)len < ahp_xc_get_delaysize() ? (off_t)len : (off_t)ahp_xc_get_delaysize()-1-start);
    char* data = (char*)malloc(ahp_xc_get_packetsize()*len);
    i = 0;
    ahp_xc_start_autocorrelation_scan(index, start, len);
    while(i < (int)len) {
        if(*interrupt)
            break;
        char* buf = grab_packet();
        if(!buf)
            continue;
        memcpy(data+i*ahp_xc_get_packetsize(), buf, ahp_xc_get_packetsize());
        i++;
        free(buf);
        (*percent) += 100.0 / len;
        r++;
    }
    ahp_xc_end_autocorrelation_scan(index);
    i = 0;
    char timestamp[16];
    double ts = 0.0;
    while(i < r) {
        if(*interrupt)
            break;
        char *packet = (char*)data+i*ahp_xc_get_packetsize();
        strncpy(timestamp, &packet[ahp_xc_get_packetsize()-19], 16);
        ts = (double)strtoul(timestamp, NULL, 16) / 1000000000.0;
        ahp_xc_get_autocorrelation(&correlations[i], index, packet, ts);
        i++;
    }
    free(data);
    free(sample);
    *autocorrelations = correlations;
    return i;
}

int ahp_xc_get_packet(ahp_xc_packet *packet)
{
    if(!ahp_xc_detected) return 0;
    int ret = 1;
    unsigned int x = 0, y = 0;
    int n = ahp_xc_get_bps()/4;
    if(packet == NULL) {
        return -EINVAL;
    }
    char *sample = (char*)malloc((unsigned int)n+1);
    char* data = grab_packet();
    if(!data){
        ret = -ENOENT;
        goto end;
    }
    packet->buf = data;
    const char *buf = packet->buf;
    buf += 16;
    for(x = 0; x < ahp_xc_get_nlines(); x++) {
        sample[n] = 0;
        memcpy(sample, buf, (unsigned int)n);
        if(1<sscanf(sample, "%lX", &packet->counts[x])) {
            ret = -ENOENT;
            goto end;
        }
        packet->counts[x] = (packet->counts[x] == 0 ? 1 : packet->counts[x]);
        buf += n;
    }
    int idx = 0;
    for(x = 0; x < ahp_xc_get_nlines(); x++) {
        ahp_xc_get_autocorrelation(&packet->autocorrelations[x], x, data, 0.0);
        if(ahp_xc_has_crosscorrelator()) {
            for(y = x+1; y < ahp_xc_get_nlines(); y++) {
                ahp_xc_get_crosscorrelation(&packet->crosscorrelations[idx++], x, y, data, 0.0);
            }
        }
    }
    char timestamp[16];
    strncpy(timestamp, &data[ahp_xc_get_packetsize()-19], 16);
    if(1<sscanf(timestamp, "%lX",  &packet->timestamp)) {
        ret = -ENOENT;
        goto end;
    }
    ret = 0;
    goto end;
err_end:
    fprintf(stderr, "%s: %s\n", __func__, strerror(-ret));
    free(data);
end:
    free(sample);
    return ret;
}

int ahp_xc_get_properties()
{
    if(!ahp_xc_connected) return -ENOENT;
    char *data = NULL;
    int n_read = 0;
    int ntries = 16;
    unsigned int _bps, _nlines, _delaysize, _auto_lagsize, _cross_lagsize, _flags, _tau;
    while(ntries-- > 0) {
        ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_ENABLE);
        ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_ENABLE);
        data = grab_packet();
        ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_ENABLE);
        if(data == NULL)
            continue;
        n_read = sscanf((char*)data, "%02X%02X%03X%02X%02X%01X%04X", &_bps, &_nlines, &_delaysize, &_auto_lagsize, &_cross_lagsize, &_flags, &_tau);
        if(n_read == 7) {
            strncpy(ahp_xc_header, (char*)data, 16);
            free(data);
            break;
        }
        free(data);
    }
    if(n_read != 7)
        return -ENODEV;
    ahp_xc_bps = _bps;
    ahp_xc_nlines = _nlines+1;
    ahp_xc_nbaselines = ahp_xc_nlines*(ahp_xc_nlines-1)/2;
    ahp_xc_delaysize = _delaysize;
    ahp_xc_auto_lagsize = _auto_lagsize+1;
    ahp_xc_cross_lagsize = _cross_lagsize+1;
    ahp_xc_flags = _flags;
    ahp_xc_packetsize = (ahp_xc_nlines+ahp_xc_auto_lagsize*ahp_xc_nlines*2+(ahp_xc_cross_lagsize*2-1)*ahp_xc_nbaselines*2)*ahp_xc_bps/4+16+16+2+1;
    ahp_xc_frequency = (unsigned int)((long)1000000000000/(long)(!_tau?1:_tau));
    sign = (1<<(ahp_xc_bps-1));
    fill = sign|(sign - 1);
    if(ahp_xc_test)
        free(ahp_xc_test);
    ahp_xc_test = (unsigned char*)malloc(ahp_xc_nlines);
    memset(ahp_xc_test, 0, ahp_xc_nlines);
    if(ahp_xc_leds)
        free(ahp_xc_leds);
    ahp_xc_leds = (unsigned char*)malloc(ahp_xc_nlines);
    memset(ahp_xc_leds, 0, ahp_xc_nlines);
    ahp_xc_detected = 1;
    return 0;
}

int ahp_xc_set_capture_flags(xc_capture_flags flags)
{
    if(!ahp_xc_connected) return -ENOENT;
    ahp_xc_capture_flags = flags;
    RS232_flushRX();
    return (int)ahp_xc_send_command(ENABLE_CAPTURE, (unsigned char)ahp_xc_capture_flags);
}

xc_capture_flags ahp_xc_get_capture_flags()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_capture_flags;
}

void ahp_xc_set_baudrate(baud_rate rate)
{
    if(!ahp_xc_detected) return;
    ahp_xc_rate = rate;
    ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)rate);
    RS232_CloseComport();
    RS232_OpenComport(ahp_xc_comport);
    RS232_SetupPort(ahp_xc_baserate<<((int)ahp_xc_rate), "8N2", 0);
}

unsigned char ahp_xc_get_test_flags(unsigned int index)
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_test[index];
}

unsigned char ahp_xc_get_leds(unsigned int index)
{
    if(!ahp_xc_detected) return 0;
    if(!ahp_xc_has_leds())
        return 0;
    return ahp_xc_leds[index];
}

void ahp_xc_set_leds(unsigned int index, int leds)
{
    if(!ahp_xc_detected) return;
    ahp_xc_leds[index] = (unsigned char)leds;
    ahp_xc_select_input(index);
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_EXTRA_CMD);
    ahp_xc_send_command(SET_LEDS, (unsigned char)((ahp_xc_leds[index]>>4)&0xf));
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
    ahp_xc_send_command(SET_LEDS, (unsigned char)(ahp_xc_leds[index]&0xf));
}

void ahp_xc_set_channel_cross(unsigned int index, off_t value, size_t size)
{
    if(!ahp_xc_detected) return;
    ahp_xc_select_input(index);
    int idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_EXTRA_CMD);
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
}

void ahp_xc_set_channel_auto(unsigned int index, off_t value, size_t size)
{
    if(!ahp_xc_detected) return;
    ahp_xc_select_input(index);
    int idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|0x8|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|0x8|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|0x8|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|0x8|(value&0x7)));
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_EXTRA_CMD);
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|0x8|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|0x8|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|0x8|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|0x8|(size&0x7)));
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
}

void ahp_xc_set_frequency_divider(unsigned char value)
{
    if(!ahp_xc_detected) return;
    value = (unsigned char)(value < 0xf ? value : 0xf);
    ahp_xc_send_command(SET_FREQ_DIV, value);
    ahp_xc_frequency_divider = value;
}

void ahp_xc_set_voltage(unsigned int index, unsigned char value)
{
    if(!ahp_xc_detected) return;
    ahp_xc_select_input(index);
    value = (unsigned char)(value < 0xff ? value : 0xff);
    int idx = 0;
    ahp_xc_send_command((xc_cmd)(SET_VOLTAGE), (unsigned char)((idx++<<2)|(value&0x3)));
    value >>= 2;
    ahp_xc_send_command((xc_cmd)(SET_VOLTAGE), (unsigned char)((idx++<<2)|(value&0x3)));
    value >>= 2;
    ahp_xc_send_command((xc_cmd)(SET_VOLTAGE), (unsigned char)((idx++<<2)|(value&0x3)));
    value >>= 2;
    ahp_xc_send_command((xc_cmd)(SET_VOLTAGE), (unsigned char)((idx++<<2)|(value&0x3)));
    value >>= 2;
    ahp_xc_voltage = value;
}

void ahp_xc_set_test_flags(unsigned int index, xc_test_flags value)
{
    if(!ahp_xc_detected) return;
    ahp_xc_select_input(index);
    ahp_xc_test[index] = value;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_EXTRA_CMD);
    ahp_xc_send_command(ENABLE_TEST, (unsigned char)((ahp_xc_test[index]>>4)&0xf));
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
    ahp_xc_send_command(ENABLE_TEST, (unsigned char)(ahp_xc_test[index]&0xf));
}

 int ahp_xc_send_command(xc_cmd c, unsigned char value)
{
    if(!ahp_xc_connected) return -ENOENT;
    return RS232_SendByte((unsigned char)(c|(((value<<4)|(value>>4))&0xf3)));
}

double* ahp_xc_get_2d_projection(double alt, double az, double *baseline)
{
    double* uv = (double*)malloc(sizeof(double)*3);
    memset(uv, 0, sizeof(double)*3);
    az *= M_PI / 180.0;
    alt *= M_PI / 180.0;
    uv[0] = (baseline[0] * sin(az) + baseline[1] * cos(az));
    uv[1] = (baseline[1] * sin(alt) * sin(az) - baseline[0] * sin(alt) * cos(az) + baseline[2] * cos(alt));
    uv[2] = cos(az) * baseline[1] * cos(alt) - baseline[0] * sin(az) * cos(alt) + sin(alt) * baseline[2];
    return uv;
}
