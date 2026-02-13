/*
*    XC Quantum correlators driver library
*    Copyright (C) 2015-2023  Ilia Platone <info@iliaplatone.com>
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>
#include <sys/time.h>
#include "ahp_xc.h"

#include "seral.h"

#ifndef AIRY
#define AIRY 1.21966
#endif
#ifndef EULER
#define EULER 2.71828182845904523536028747135266249775724709369995
#endif
static int32_t xc_current_input = 0;
static int64_t sign = 1;
static int64_t fill = 0;

typedef struct  {
    ahp_xc_sample *sample;
    int32_t index;
    int32_t *indexes;
    int32_t order;
    const char *data;
    double lag;
    double *lags;
} thread_argument;


typedef struct {
    unsigned char buffer[0x1000000];
    int32_t threads_running;
    int32_t size;
    int32_t capturing;
    pthread_mutex_t mutex;
} read_argument;

typedef struct {
    int32_t nthreads;
    thread_argument *autocorrelation_thread_args;
    thread_argument *crosscorrelation_thread_args;
    pthread_t *autocorrelation_threads;
    pthread_t *crosscorrelation_threads;
    pthread_mutex_t mutex;
    int32_t mutexes_initialized;

    int32_t max_threads;
    unsigned char *test;
    unsigned char *leds;
    ahp_xc_scan_request *auto_channel;
    ahp_xc_scan_request *cross_channel;
    uint32_t bps;
    uint32_t nlines;
    uint32_t nbaselines;
    uint32_t auto_lagsize;
    uint32_t cross_lagsize;
    uint32_t delaysize;
    uint32_t flags;
    uint32_t correlator_enabled;
    uint32_t intensity_correlator_enabled;
    double frequency;
    uint32_t voltage;
    uint32_t connected;
    uint32_t detected;
    uint32_t packetsize;
    int32_t baserate;
    baud_rate rate;
    uint32_t correlation_order;
    char comport[128];

    char *tmp_buf;
    char *buf;
    char *header;
    int32_t buf_allocd;
    int32_t header_allocd;
    int32_t buf_len;
    int32_t header_len;
    int32_t delaysize_len;
    unsigned char capture_flags;
    unsigned char max_lost_packets;
} ahp_xc_device;

ahp_xc_device ahp_xc;

static uint32_t get_npolytopes(int nlines, int32_t order)
{
    return nlines * (nlines - order + 1) / (order);
}

static int32_t get_line_index(int nlines, int32_t idx, int32_t order)
{
    return (idx + order * (idx / nlines + 1)) % nlines;
}

int32_t ahp_xc_get_line_index(int32_t idx, int32_t order)
{
    return get_line_index(ahp_xc_get_nlines(), idx, order);
}

int32_t ahp_xc_get_crosscorrelation_index(int32_t *lines, int32_t order)
{
    int32_t x, y, idx;
    int32_t npolytopes = ahp_xc_get_nbaselines();
    int* matches = (int*)malloc(sizeof(int)*npolytopes);
    memset(matches, 0, sizeof(int)*npolytopes);
    for(idx = 0; idx < npolytopes; idx ++) {
        for(x = 0; x < order; x ++) {
            for(y = 0; y < order; y ++) {
                if(lines[x] == ahp_xc_get_line_index(idx, y))
                    matches[idx]++;
            }
        }
    }
    int32_t index = 0;
    int32_t best_match = 0;
    for(idx = 0; idx < npolytopes; idx++) {
        if(matches[idx] > best_match) {
            best_match = matches[idx];
            index = idx;
        }
    }
    return index;
}

uint64_t ahp_xc_max_threads(uint64_t value)
{
    if(value>0) {
        ahp_xc.max_threads = value;
    }
    return ahp_xc.max_threads;
}

void wait_threads()
{
    while (ahp_xc.nthreads >= ahp_xc_max_threads(0))
        usleep(1);
}

void wait_no_threads()
{
    while (ahp_xc.nthreads > 0)
        usleep(1);
}

static void complex_phase_magnitude(ahp_xc_correlation *sample)
{
    if(!ahp_xc.detected) return;
    double cr = (double)sample->real / sample->counts;
    double ci = (double)sample->imaginary / sample->counts;
    double magnitude = (double)sqrt(pow(cr, 2)+pow((double)ci, 2));
    double phase = 0.0;
    if(magnitude > 0.0) {
        phase = asin ((double)cr / magnitude);
        if(ci < 0)
            phase = M_PI*2.0-phase;
    }
    sample->magnitude = magnitude;
    sample->phase = phase;
}

double get_timestamp(char *data)
{
    char timestamp[16] = { 0 };
    double ts = 0;
    uint32_t tmp = 0;
    strncpy(timestamp, &data[ahp_xc_get_packetsize()-19], 16);
    sscanf(timestamp, "%8X", &tmp);
    ts = (double)tmp * 4.294967296;
    sscanf(&timestamp[8], "%8X", &tmp);
    return (double)ts + tmp / 1000000000.0;
}

double ahp_xc_get_current_channel_auto(int n, char *data)
{
    char *current_channel = (char*)malloc(ahp_xc.delaysize_len);
    double channel = 0;
    uint32_t tmp = 0;
    char *message = &data[ahp_xc_get_packetsize()-19-ahp_xc.delaysize_len*ahp_xc_get_nlines()-ahp_xc.delaysize_len*(n+1)];
    strncpy(current_channel, message, ahp_xc.delaysize_len);
    sscanf(current_channel, "%X", &tmp);
    channel = (double)tmp;
    return channel;
}

double ahp_xc_get_current_channel_cross(int n, char *data)
{
    char *current_channel = (char*)malloc(ahp_xc.delaysize_len);
    double channel = 0;
    uint32_t tmp = 0;
    char *message = &data[ahp_xc_get_packetsize()-19-ahp_xc.delaysize_len*(n+1)];
    strncpy(current_channel, message, ahp_xc.delaysize_len);
    sscanf(current_channel, "%X", &tmp);
    channel = (double)tmp;
    return channel;
}

int32_t calc_checksum(char *data)
{
    if(!ahp_xc.connected) return -ENOENT;
    int32_t x;
    uint32_t checksum = 0x00;
    uint32_t calculated_checksum = 0;
    unsigned char v = data[ahp_xc_get_packetsize()-3];
    checksum = v < 'A' ? (v - '0') : (v - 'A' + 10);
    checksum *= 16;
    v = data[ahp_xc_get_packetsize()-2];
    checksum += v < 'A' ? (v - '0') : (v - 'A' + 10);
    for(x = ahp_xc.header_len; x < (int)ahp_xc_get_packetsize()-3; x++) {
        calculated_checksum += data[x] < 'A' ? (data[x] - '0') : (data[x] - 'A' + 10);
        calculated_checksum &= 0xff;
    }
    if(checksum != calculated_checksum) {
        return EINVAL;
    }
    return 0;
}

static int grab_packet(double *timestamp)
{
    errno = 0;
    uint32_t size = ahp_xc_get_packetsize();
    ahp_xc.buf_len = size;
    if(!ahp_xc.connected){
        errno = ENOENT;
        goto err_end;
    }
    int32_t nread = 0;
    char c = 0;
    while (c != '\r') {
        int n = ahp_serial_RecvBuf((unsigned char*)&c, 1);
        if(n > 0)
            ahp_xc.tmp_buf[nread++] = c;
    }
    if(nread < 2) {
        nread = 0;
        c = 0;
        while (c != '\r') {
            int n = ahp_serial_RecvBuf((unsigned char*)&c, 1);
            if(n > 0)
                ahp_xc.tmp_buf[nread++] = c;
        }
    }
    ahp_xc.tmp_buf[nread-1] = '\0';
    if(nread < 3) {
        errno = ENODATA;
    } else if(nread < 0) {
        errno = ETIMEDOUT;
    } else {
        if(ahp_xc.header_len > 0) {
            if(strncmp(ahp_xc_get_header(), ahp_xc.tmp_buf, ahp_xc.header_len))
                errno = EPERM;
            else {
                errno = calc_checksum((char*)ahp_xc.tmp_buf);
            }
        }
    }
    if(errno)
        goto err_end;
    if(timestamp != NULL)
        *timestamp = get_timestamp(ahp_xc.tmp_buf);
    memcpy(ahp_xc.buf, ahp_xc.tmp_buf, nread);
    return 0;
err_end:
    fprintf(stderr, "%s error: %s\n", __func__, strerror(errno));
    return -1;
}

uint32_t ahp_xc_current_input()
{
    return xc_current_input;
}

void ahp_xc_select_input(uint32_t index)
{
    if(!ahp_xc.detected) return;
    int32_t idx = 0;
    if(index >= ahp_xc_get_nlines())
        return;
    ahp_xc_send_command(CLEAR, SET_INDEX);
    int len = (((int)log2(ahp_xc_get_nlines()) & ~3) + 4) / 4;
    if(len < 0) len = 1;
    ahp_xc_send_command(SET_INDEX, (unsigned char)(len&0xf));
    for(idx = 0; idx < len; idx ++) {
        ahp_xc_send_command(SET_INDEX, (unsigned char)(index&0xf));
        index >>= 4;
    }
    xc_current_input = index;
}

void ahp_xc_enable_crosscorrelator(int32_t enable)
{
    if(!ahp_xc.detected) return;
    ahp_xc.correlator_enabled = enable;
}

void ahp_xc_enable_intensity_crosscorrelator(int32_t enable)
{
    if(!ahp_xc.detected) return;
    ahp_xc.intensity_correlator_enabled = enable;
}

int32_t ahp_xc_intensity_crosscorrelator_enabled()
{
    if(!ahp_xc.detected) return 0;
    return ahp_xc.intensity_correlator_enabled != 0 || !ahp_xc_has_crosscorrelator();
}

int32_t ahp_xc_has_crosscorrelator()
{
    if(!ahp_xc.detected) return 0;
    return (ahp_xc.flags & HAS_CROSSCORRELATOR ? ahp_xc.correlator_enabled : 0);
}

int32_t ahp_xc_has_psu()
{
    if(!ahp_xc.detected) return 0;
    return (ahp_xc.flags & HAS_PSU ? 1 : 0);
}

int32_t ahp_xc_has_leds()
{
    if(!ahp_xc.detected) return 0;
    return (ahp_xc.flags & HAS_LEDS ? 1 : 0);
}

int32_t ahp_xc_has_cumulative_only()
{
    if(!ahp_xc.detected) return 0;
    return (ahp_xc.flags & HAS_CUMULATIVE_ONLY ? 1 : 0);
}

char* ahp_xc_get_header()
{
    if(!ahp_xc.detected) return 0;
    return ahp_xc.header;
}

int32_t ahp_xc_get_baudrate()
{
    return ahp_xc.baserate << ahp_xc.rate;
}

uint32_t ahp_xc_get_bps()
{
    if(!ahp_xc.detected) return 0;
    return ahp_xc.bps;
}

uint32_t ahp_xc_get_nlines()
{
    if(!ahp_xc.detected) return 0;
    return ahp_xc.nlines;
}

uint32_t ahp_xc_get_nbaselines()
{
    if(!ahp_xc.detected) return 0;
    return ahp_xc_get_nlines() * (ahp_xc_get_nlines() - 1) / 2;
}

uint32_t ahp_xc_get_npolytopes(int32_t order)
{
    if(!ahp_xc.detected) return 0;
    return ahp_xc_get_nlines() * (ahp_xc_get_nlines() - order + 1) / order;
}

uint32_t ahp_xc_get_delaysize()
{
    if(!ahp_xc.detected) return 0;
    if(ahp_xc.delaysize == 0 || ahp_xc.delaysize == 4)
        return pow(2, 24);
    return ahp_xc.delaysize * 17;
}

uint32_t ahp_xc_get_autocorrelator_lagsize()
{
    if(!ahp_xc.detected) return 0;
    return ahp_xc.auto_lagsize;
}

uint32_t ahp_xc_get_crosscorrelator_lagsize()
{
    if(!ahp_xc.detected) return 0;
    return ahp_xc.cross_lagsize;
}

double ahp_xc_get_frequency()
{
    if(!ahp_xc.detected) return 0;
    return ahp_xc.frequency;
}

double ahp_xc_get_sampletime()
{
    return (double)1.0/ahp_xc_get_frequency();
}

double ahp_xc_get_packettime()
{
    return 9.0  * (double)ahp_xc_get_packetsize() / (double)ahp_xc_get_baudrate();
}

uint32_t ahp_xc_get_packetsize()
{
    return ahp_xc.packetsize;
}

int32_t ahp_xc_get_fd()
{
    return ahp_serial_GetFD();
}

int32_t ahp_xc_connect_fd(int32_t fd)
{
    if(ahp_xc.detected)
        return 0;
    ahp_xc.connected = 0;
    ahp_xc.detected = 0;
    ahp_xc.bps = 0;
    ahp_xc.nlines = 0;
    ahp_xc.nbaselines = 0;
    ahp_xc.delaysize = 0;
    ahp_xc.frequency = 0;
    ahp_xc.packetsize = 4096;
    ahp_xc.rate = R_BASE;
    if(fd > -1) {
        ahp_xc.connected = 1;
        ahp_xc.buf = (char*)malloc(ahp_xc.packetsize);
        ahp_xc.tmp_buf = (char*)malloc(ahp_xc.packetsize);
        ahp_xc.detected = 0;
        ahp_serial_SetFD(fd, XC_BASE_RATE);
        if(!ahp_xc.mutexes_initialized) {
            pthread_mutex_init(&ahp_xc.mutex, &ahp_serial_mutex_attr);
            ahp_xc.mutexes_initialized = 1;
        }
        ahp_xc.nthreads = 0;
        xc_current_input = 0;
        ahp_xc_get_properties();
    }
    if(!ahp_xc.detected)
        ahp_xc_disconnect();
    ahp_xc.connected = ahp_xc.detected;
    return !ahp_xc.detected;
}

int32_t ahp_xc_connect(const char *port)
{
    if(ahp_xc.detected)
        return 0;
    sleep(1);
    int32_t ret = 1;
    xc_current_input = 0;
    ahp_xc.nthreads = 0;
    ahp_xc.connected = 0;
    ahp_xc.detected = 0;
    ahp_xc.bps = 0;
    ahp_xc.nlines = 0;
    ahp_xc.nbaselines = 0;
    ahp_xc.delaysize = 0;
    ahp_xc.frequency = 1;
    ahp_xc.max_threads = 1;
    ahp_xc.max_lost_packets = 1;
    ahp_xc.packetsize = 4096;
    ahp_xc.baserate = XC_BASE_RATE;
    ahp_xc.rate = R_BASE;
    ahp_xc.correlator_enabled = 1;
    strcpy(ahp_xc.comport, port);
    if(!ahp_serial_OpenComport(port)) {
        ahp_xc.connected = 1;
        ahp_xc.buf = (char*)malloc(ahp_xc.packetsize);
        ahp_xc.tmp_buf = (char*)malloc(ahp_xc.packetsize);
        ahp_xc.buf_allocd = 1;
        ahp_xc.buf[0] = 0;
        ahp_xc.tmp_buf[0] = 0;
        ahp_xc.buf_len = 0;
        ahp_xc.header = (char*)malloc(1);
        ahp_xc.header_allocd = 1;
        ahp_xc.header[0] = 0;
        ahp_xc.header_len = 0;
        ret = ahp_serial_SetupPort(ahp_xc_get_baudrate(), "8N1", 0);
        if(!ret) {
            if(!ahp_xc.mutexes_initialized) {
                pthread_mutex_init(&ahp_xc.mutex, &ahp_serial_mutex_attr);
                ahp_xc.mutexes_initialized = 1;
            }
            ahp_xc_get_properties();
        }
    }
    return !ahp_xc.detected;
}
void ahp_xc_disconnect()
{
    if(ahp_xc.connected) {
        if(ahp_xc.detected) {
            ahp_xc_send_command(CLEAR, SET_INDEX);
            ahp_xc_send_command(CLEAR, SET_LEDS);
            ahp_xc_send_command(CLEAR, SET_BAUD_RATE);
            ahp_xc_send_command(CLEAR, SET_VOLTAGE);
            ahp_xc_send_command(CLEAR, SET_DELAY);
            ahp_xc_send_command(CLEAR, ENABLE_TEST);
            ahp_xc_send_command(CLEAR, CLEAR);
        }
        if(ahp_xc.mutexes_initialized) {
            pthread_mutex_unlock(&ahp_xc.mutex);
            pthread_mutex_destroy(&ahp_xc.mutex);
            ahp_xc.mutexes_initialized = 0;
        }
        free(ahp_xc.buf);
        free(ahp_xc.tmp_buf);
        free(ahp_xc.header);
        ahp_serial_CloseComport();
    }
}

uint32_t ahp_xc_is_connected()
{
    return ahp_xc.connected;
}

uint32_t ahp_xc_is_detected()
{
    return ahp_xc.detected;
}

ahp_xc_sample *ahp_xc_alloc_samples(uint64_t nlines, size_t size)
{
    uint64_t x, y;
    ahp_xc_sample* samples = (ahp_xc_sample*)malloc(sizeof(ahp_xc_sample)*nlines);
    memset(samples, 0, sizeof(ahp_xc_sample)*nlines);
    for(x = 0; x < nlines; x++) {
        samples[x].lag_size = size;
        samples[x].correlations = (ahp_xc_correlation*)malloc(sizeof(ahp_xc_correlation)*size);
        memset(samples[x].correlations, 0, sizeof(ahp_xc_correlation)*size);
    }
    return samples;
}

ahp_xc_sample *ahp_xc_copy_samples(ahp_xc_sample* src, uint64_t nlines, size_t size)
{
    uint64_t x;
    ahp_xc_sample* samples = ahp_xc_alloc_samples(nlines, size);
    for(x = 0; x < nlines; x++) {
        samples[x].lag = src[x].lag;
        memcpy(samples[x].correlations, src[x].correlations, sizeof(ahp_xc_correlation)*size);
    }
    return samples;
}

void ahp_xc_free_samples(uint64_t nlines, ahp_xc_sample *samples)
{
    uint64_t x, y;
    if(samples != NULL) {
        for(x = 0; x < nlines; x++) {
            if(samples[x].correlations != NULL) {
                free(samples[x].correlations);
            }
        }
        free(samples);
    }
}

ahp_xc_packet *ahp_xc_alloc_packet()
{
    ahp_xc_packet *packet = (ahp_xc_packet*)malloc(sizeof(ahp_xc_packet));
    packet->bps = (uint64_t)ahp_xc_get_bps();
    packet->tau = (uint64_t)(1.0/ahp_xc_get_frequency());
    packet->n_lines = (uint64_t)ahp_xc_get_nlines();
    packet->n_baselines = (uint64_t)ahp_xc_get_nbaselines();
    packet->auto_lag = ahp_xc_get_autocorrelator_lagsize();
    packet->cross_lag = ahp_xc_get_crosscorrelator_lagsize()*2-1;
    packet->counts = (uint64_t*)malloc((uint64_t)ahp_xc_get_nlines() * sizeof(uint64_t));
    packet->autocorrelations = ahp_xc_alloc_samples((uint64_t)ahp_xc_get_nlines(), (uint64_t)ahp_xc_get_autocorrelator_lagsize());
    packet->crosscorrelations = ahp_xc_alloc_samples((uint64_t)ahp_xc_get_nbaselines(), (uint64_t)ahp_xc_get_crosscorrelator_lagsize()*2-1);
    packet->lock = malloc(sizeof(pthread_mutex_t));
    pthread_mutex_init(((pthread_mutex_t*)packet->lock), &ahp_serial_mutex_attr);
    return packet;
}

ahp_xc_packet *ahp_xc_copy_packet(ahp_xc_packet *packet)
{
    ahp_xc_packet *copy = ahp_xc_alloc_packet();
    copy->timestamp = packet->timestamp;
    copy->bps = packet->bps;
    copy->tau = packet->tau;
    copy->n_lines = packet->n_lines;
    copy->n_baselines = packet->n_baselines;
    copy->auto_lag = packet->auto_lag;
    copy->cross_lag = packet->cross_lag;
    copy->counts = (uint64_t*)malloc((uint64_t)copy->n_lines * sizeof(uint64_t));
    memcpy(copy->counts, packet->counts, sizeof(uint64_t) * (uint64_t)copy->n_lines);
    ahp_xc_free_samples(copy->n_lines, copy->autocorrelations);
    ahp_xc_free_samples(copy->n_baselines, copy->crosscorrelations);
    copy->autocorrelations = ahp_xc_copy_samples(packet->autocorrelations, copy->n_lines, copy->auto_lag);
    copy->crosscorrelations = ahp_xc_copy_samples(packet->crosscorrelations, copy->n_baselines, copy->cross_lag);
    return copy;
}

void ahp_xc_free_packet(ahp_xc_packet *packet)
{
    if(packet != NULL) {
        if(packet->counts != NULL)
            free(packet->counts);
        pthread_mutex_destroy(((pthread_mutex_t*)packet->lock));
        free(packet->lock);
        ahp_xc_free_samples((uint64_t)packet->n_lines, packet->autocorrelations);
        ahp_xc_free_samples((uint64_t)packet->n_baselines, packet->crosscorrelations);
        free(packet);
    }
}

static void ahp_xc_start_autocorrelation_scan(uint32_t index)
{
    if(!ahp_xc.detected) return;
    int flags = ahp_xc_get_capture_flags();
    ahp_xc_set_capture_flags((flags|CAP_RESET_TIMESTAMP)&~(CAP_ENABLE|CAP_EXTRA_CMD));
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|SCAN_AUTO);
    ahp_xc_set_capture_flags(flags);
}

static void ahp_xc_end_autocorrelation_scan(uint32_t index)
{
    if(!ahp_xc.detected) return;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~SCAN_AUTO);
}

static void* _get_autocorrelation(void *o)
{
    thread_argument *arg = (thread_argument*)o;
    ahp_xc_sample *sample = arg->sample;
    int32_t index = arg->index;
    const char *data = arg->data;
    double lag = arg->lag;
    uint32_t y;
    int32_t n = ahp_xc_get_bps() / 4;
    const char *packet = data;
    char *subpacket = (char*)malloc(n+1);
    memset(subpacket, 0, n+1);
    sample->lag_size = ahp_xc_get_autocorrelator_lagsize();
    sample->lag = lag;
    packet += ahp_xc.header_len;
    memcpy(subpacket, &packet[index*n], (unsigned int)n);
    uint64_t counts = strtoul(subpacket, NULL, 16)|1;
    packet += n*ahp_xc_get_nlines();
    packet += n*index*ahp_xc_get_autocorrelator_lagsize()*2;
    for(y = 0; y < sample->lag_size; y++) {
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
        sample->correlations[y].lag = ahp_xc_get_current_channel_auto(index, data) * ahp_xc_get_sampletime();
    }
    packet += (ahp_xc_get_nbaselines() + ahp_xc_get_nlines()) * n;
    free(subpacket);
    if(ahp_xc.nthreads > 0)
        ahp_xc.nthreads--;
    return NULL;
}

void ahp_xc_get_autocorrelation(ahp_xc_sample *sample, int32_t index, const char *data, double lag)
{
    if(!ahp_xc.mutexes_initialized)
        return;
    ahp_xc.autocorrelation_thread_args[index].sample = sample;
    ahp_xc.autocorrelation_thread_args[index].index = index;
    ahp_xc.autocorrelation_thread_args[index].data = data;
    ahp_xc.autocorrelation_thread_args[index].lag = lag;
    _get_autocorrelation(&ahp_xc.autocorrelation_thread_args[index]);
}

static int32_t ahp_xc_scan_autocorrelations(ahp_xc_scan_request *lines, uint32_t nlines, ahp_xc_sample **autocorrelations, int32_t *interrupt, double *percent)
{
    if(!ahp_xc.detected) return 0;
    int32_t r = -1;
    uint32_t i = 0;
    uint32_t x = 0;
    uint32_t y = 0;
    double ts = 0.0;
    double ts0 = 0.0;
    int32_t s = 0;
    *autocorrelations = NULL;
    (*percent) = 0;
    r++;
    size_t len = 0;
    size_t size = 0;
    for(i = 0; i < nlines; i++) {
        lines[i].start = (lines[i].start < ahp_xc_get_delaysize()-2 ? lines[i].start : (off_t)ahp_xc_get_delaysize()-2);
        lines[i].len = (lines[i].start+(off_t)lines[i].len < ahp_xc_get_delaysize() ? (off_t)lines[i].len : (off_t)ahp_xc_get_delaysize()-1-lines[i].start);
        len = fmax(len, lines[i].len/lines[i].step);
        size += lines[i].len/lines[i].step;
    }
    ahp_xc_sample *correlations = ahp_xc_alloc_samples(size, (unsigned int)ahp_xc_get_autocorrelator_lagsize());
    char* data = (char*)malloc(ahp_xc_get_packetsize()*(len+1));
    for(i = 0; i < nlines; i++) {
        ahp_xc_select_input(lines[i].index);
        int capture_flags = ahp_xc_get_capture_flags();
        ahp_xc_set_capture_flags(capture_flags & ~CAP_EXTRA_CMD);
        ahp_xc_select_input(lines[i].index);
        ahp_xc_send_command(CLEAR, SET_DELAY);
        ahp_xc_set_capture_flags(capture_flags);
        ahp_xc_set_channel_auto(lines[i].index, lines[i].start, lines[i].len, lines[i].step);
        ahp_xc_start_autocorrelation_scan(lines[i].index);
    }
    char* buf = NULL;
    i = 0;
    ahp_xc_set_capture_flags((ahp_xc_get_capture_flags()|CAP_RESET_TIMESTAMP)&~CAP_ENABLE);
    ahp_serial_flushRX();
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_ENABLE);
    while(i < len) {
        if(*interrupt)
            break;
        unsigned char *buf = (unsigned char*)malloc(ahp_xc_get_packetsize());
        ahp_serial_RecvBuf((unsigned char*)buf, ahp_xc_get_packetsize());
        memcpy(data+i*ahp_xc_get_packetsize(), buf+1, ahp_xc_get_packetsize()-1);
        *((char*)data+i*ahp_xc_get_packetsize()+ahp_xc_get_packetsize()) = '\r';
        i++;
        free(buf);
        (*percent) += 100.0 / len;
        r++;
    }
    i = 0;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~(CAP_ENABLE|CAP_RESET_TIMESTAMP));
    for(i = 0; i < nlines; i++) {
        ahp_xc_select_input(lines[i].index);
        int capture_flags = ahp_xc_get_capture_flags();
        ahp_xc_set_capture_flags(capture_flags & ~CAP_EXTRA_CMD);
        ahp_xc_select_input(lines[i].index);
        ahp_xc_send_command(CLEAR, SET_DELAY);
        ahp_xc_set_capture_flags(capture_flags);
        ahp_xc_end_autocorrelation_scan(lines[i].index);
    }
    i = 0;
    while((int)i < r) {
        if(*interrupt)
            break;
        char *packet = (char*)data+i*ahp_xc_get_packetsize();
        ts = get_timestamp(packet);
        size_t off = 0;
        for(x = 0; x < nlines; x++) {
            if(i < lines[x].len/lines[x].step) {
                ahp_xc_get_autocorrelation(&correlations[i+off], lines[x].index, packet+y*ahp_xc_get_packetsize(), ahp_xc_get_current_channel_auto(lines[x].index, data));
                s++;
            }
            off += lines[x].len/lines[x].step;
        }
        wait_no_threads();
        i++;
    }
    free(data);
    *autocorrelations = correlations;
    return s;
}

static void ahp_xc_end_crosscorrelation_scan(uint32_t index)
{
    if(!ahp_xc.detected) return;
    if(!ahp_xc_intensity_crosscorrelator_enabled())
        ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~SCAN_CROSS);
    else
        ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~SCAN_AUTO);
}

static void ahp_xc_start_crosscorrelation_scan(uint32_t index)
{
    if(!ahp_xc.detected) return;
    ahp_xc_end_crosscorrelation_scan(index);
    int flags = ahp_xc_get_capture_flags();
    ahp_xc_set_capture_flags((flags|CAP_RESET_TIMESTAMP)&~(CAP_ENABLE|CAP_EXTRA_CMD));
    if(!ahp_xc_intensity_crosscorrelator_enabled())
        ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|SCAN_CROSS);
    else
        ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|SCAN_AUTO);
    ahp_xc_set_capture_flags(flags);
}

static void *_get_crosscorrelation(void *o)
{
    thread_argument *arg = (thread_argument*)o;
    ahp_xc_sample *sample = arg->sample;
    int32_t *indexes = arg->indexes;
    int32_t index = arg->index;
    uint32_t num_indexes = arg->order;
    const char *data = arg->data;
    uint32_t x, y;
    int32_t n = ahp_xc_get_bps() / 4;
    const char *packet = data;
    sample->lag_size = (ahp_xc_get_crosscorrelator_lagsize()*2-1);
    sample->lag = 0;
    if(ahp_xc_intensity_crosscorrelator_enabled()) {
        ahp_xc_sample **samples = (ahp_xc_sample**)malloc(sizeof(ahp_xc_sample*)*num_indexes);
        for(y = 0; y < num_indexes; y++) {
            samples[y] = ahp_xc_alloc_samples(1, ahp_xc_get_autocorrelator_lagsize());
            ahp_xc_get_autocorrelation(samples[y], indexes[y], packet, ahp_xc_get_current_channel_auto(indexes[y], data) * ahp_xc_get_sampletime());
        }
        wait_no_threads();
        for (y = 0; y < ahp_xc_get_autocorrelator_lagsize(); y++) {
            sample->correlations[y].num_indexes = num_indexes;
            sample->correlations[y].indexes = (int*)malloc(sizeof(int) * num_indexes);
            sample->correlations[y].lags = (double*)malloc(sizeof(double) * num_indexes);
            memcpy(sample->correlations[y].indexes, arg->indexes, sizeof(int)*num_indexes);
            memcpy(sample->correlations[y].lags, arg->lags, sizeof(double)*num_indexes);
            sample->correlations[y].lag = ahp_xc_get_current_channel_auto(indexes[y], data) * ahp_xc_get_sampletime();
            sample->correlations[y].counts = samples[0]->correlations[y].counts;
            sample->correlations[y].magnitude = samples[0]->correlations[y].magnitude;
            sample->correlations[y].phase = samples[0]->correlations[y].phase;
            sample->correlations[y].real = samples[0]->correlations[y].real;
            sample->correlations[y].imaginary = samples[0]->correlations[y].imaginary;
            ahp_xc_free_samples(1, samples[0]);
            for (x = 1; x < num_indexes; x++) {
                sample->correlations[y].lag = samples[0]->lag+y*ahp_xc_get_sampletime();
                sample->correlations[y].lags[x] = arg->lags[x];
                sample->correlations[y].indexes[x] = arg->indexes[x];
                sample->correlations[y].counts += samples[x]->correlations[y].counts;
                sample->correlations[y].magnitude *= samples[x]->correlations[y].magnitude;
                sample->correlations[y].phase += samples[x]->correlations[y].phase;
                ahp_xc_free_samples(1, samples[x]);
            }
            sample->correlations[y].counts /= num_indexes;
            sample->correlations[y].magnitude = pow(sample->correlations[y].magnitude, 1.0/num_indexes);
            sample->correlations[y].phase = fmod(sample->correlations[y].phase, M_PI*2.0);
            sample->correlations[y].real = (long)(sin(sample->correlations[y].phase) * sample->correlations[y].magnitude);
            sample->correlations[y].imaginary = (long)(cos(sample->correlations[y].phase) * sample->correlations[y].magnitude);
        }
        free(samples);
    } else {
        char *subpacket = (char*)malloc(n+1);
        memset(subpacket, 0, n+1);
        packet += ahp_xc.header_len;
        uint64_t counts = 0;
        for(y = 0; y < num_indexes; y++) {
            memcpy(subpacket, &packet[indexes[y]*n], (unsigned int)n);
            counts += strtoul(subpacket, NULL, 16)|1;
        }
        packet += n*ahp_xc_get_nlines();
        packet += n*ahp_xc_get_autocorrelator_lagsize()*ahp_xc_get_nlines()*2;
        packet += n*index*2;
        for(y = 0; y < sample->lag_size; y++) {
            sample->correlations[y].num_indexes = num_indexes;
            if(sample->correlations[y].indexes == NULL)
                sample->correlations[y].indexes = (int*)malloc(sizeof(int) * num_indexes);
            if(sample->correlations[y].lags == NULL)
                sample->correlations[y].lags = (double*)malloc(sizeof(double) * num_indexes);
            memcpy(sample->correlations[y].indexes, arg->indexes, sizeof(int)*num_indexes);
            memcpy(sample->correlations[y].lags, arg->lags, sizeof(double)*num_indexes);
            sample->correlations[y].lag = ahp_xc_get_current_channel_auto(indexes[y], data) * ahp_xc_get_sampletime();
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
    if(ahp_xc.nthreads > 0)
        ahp_xc.nthreads--;
    return NULL;
}

void ahp_xc_get_crosscorrelation(ahp_xc_sample *sample, int32_t *indexes, int32_t order, const char *data, double *lags)
{
    if(!ahp_xc.mutexes_initialized)
        return;
    int32_t index = ahp_xc_get_crosscorrelation_index(indexes, order);
    ahp_xc.crosscorrelation_thread_args[index].sample = sample;
    ahp_xc.crosscorrelation_thread_args[index].index = index;
    ahp_xc.crosscorrelation_thread_args[index].indexes = indexes;
    ahp_xc.crosscorrelation_thread_args[index].order = order;
    ahp_xc.crosscorrelation_thread_args[index].data = data;
    ahp_xc.crosscorrelation_thread_args[index].lags = lags;
    _get_crosscorrelation(&ahp_xc.crosscorrelation_thread_args[index]);
}

static int compare_scan_request_asc(const void *a, const  void *b)
{
    return ((ahp_xc_scan_request*)a)->len / ((ahp_xc_scan_request*)a)->step < ((ahp_xc_scan_request*)b)->len / ((ahp_xc_scan_request*)b)->step? 1 : -1;
}

static int32_t ahp_xc_scan_crosscorrelations(ahp_xc_scan_request *lines, uint32_t nlines, ahp_xc_sample **crosscorrelations, int32_t *interrupt, double *percent)
{
    if(!ahp_xc.detected) return 0;
    size_t k = 0;
    int i = 0;
    int o = 0;
    uint32_t x = 0;
    int y = 0;
    int z = 0;
    double ts = 0.0;
    double ts0 = 0.0;
    int order = ahp_xc_get_correlation_order();
    uint32_t n = ahp_xc_get_bps()/4;
    *crosscorrelations = NULL;
    qsort(lines, nlines, sizeof(ahp_xc_scan_request), &compare_scan_request_asc);
    int32_t size = 1;
    for(i = 0; i < order; i++) {
    }
    int32_t *inputs = (int*)malloc(sizeof(int)*ahp_xc_get_correlation_order());
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|(CAP_ENABLE|CAP_RESET_TIMESTAMP));
    for(x = 0; x < get_npolytopes(nlines, order); x++) {
        for(y = 0; y < order; y++) {
            int index = get_line_index(nlines, x, y);
            inputs[y] = lines[index].index;
            if(ahp_xc_intensity_crosscorrelator_enabled()) {
                ahp_xc_end_autocorrelation_scan(lines[index].index);
            } else {
                ahp_xc_end_crosscorrelation_scan(lines[index].index);
            }
            lines[index].cur_chan = lines[index].start;
            size *= (lines[index].len / lines[index].step);
        }
    }
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~(CAP_ENABLE|CAP_RESET_TIMESTAMP));
    char *buffer = (char*)malloc(ahp_xc_get_packetsize() * lines[0].len / lines[0].step);
    ahp_xc_sample *correlations = ahp_xc_alloc_samples((unsigned int)size, (unsigned int)ahp_xc_get_crosscorrelator_lagsize());
    char* sample = (char*)malloc((unsigned int)n+1);
    sample[n] = 0;
    (*percent) = 0;
    o = 0;
    while(o < size && !*interrupt) {
        for(x = 0; x < get_npolytopes(nlines, order) && !*interrupt; x++) {
            for(y = 1; y < order && !*interrupt; y++) {
                int index = get_line_index(nlines, x, y);
                if(lines[0].cur_chan >= lines[0].start + (off_t)lines[0].step * (off_t)lines[0].len)
                    continue;
                if(lines[index].cur_chan >= lines[index].start + (off_t)lines[index].step * (off_t)lines[index].len)
                    continue;
                int capture_flags = ahp_xc_get_capture_flags();
                ahp_xc_set_capture_flags(capture_flags | CAP_EXTRA_CMD);
                ahp_xc_select_input(index);
                ahp_xc_send_command(CLEAR, SET_DELAY);
                ahp_xc_set_capture_flags(capture_flags);
                if(ahp_xc_intensity_crosscorrelator_enabled())
                    ahp_xc_set_channel_auto(index, lines[0].start, lines[0].len, lines[0].step);
                else
                    ahp_xc_set_channel_cross(index, lines[0].start, lines[0].len, lines[0].step);

                ahp_xc_start_crosscorrelation_scan(lines[0].index);
                lines[index].cur_chan += lines[index].step;
                int i = 0;
                ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~(CAP_ENABLE|CAP_RESET_TIMESTAMP));
                ahp_serial_flushRX();
                ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_ENABLE);
                while(i < (int)(lines[0].len/lines[0].step)) {
                    if(*interrupt)
                        break;
                    unsigned char *buf = (unsigned char*)malloc(ahp_xc_get_packetsize());
                    ahp_serial_RecvBuf((unsigned char*)buf, ahp_xc_get_packetsize());
                    memcpy(buffer+i*ahp_xc_get_packetsize(), buf+1, ahp_xc_get_packetsize()-1);
                    *((char*)buffer+i*ahp_xc_get_packetsize()+ahp_xc_get_packetsize()) = '\r';
                    free(buf);
                    (*percent) += 100.0 / size;
                    i++;
                }
                ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~(CAP_ENABLE|CAP_RESET_TIMESTAMP));
                if(ahp_xc_intensity_crosscorrelator_enabled()) {
                    ahp_xc_end_autocorrelation_scan(lines[0].index);
                } else {
                    ahp_xc_end_crosscorrelation_scan(lines[0].index);
                }
                k = 0;
                ts0 = 0.0;
                while(k < lines[0].len/lines[0].step) {
                    if(*interrupt)
                        break;
                    char *packet = (char*)buffer+k*ahp_xc_get_packetsize();
                    double *lags = (double*)malloc(sizeof(double)*order);
                    ts = get_timestamp(packet);
                    for(z = 0; z < order && !*interrupt; z++)
                        lags[z] = ts;
                    ahp_xc_get_crosscorrelation(&correlations[o], inputs, order, packet, lags);
                    free(lags);
                    wait_no_threads();
                    o++;
                    k++;
                }
            }
        }
    }
    free(inputs);
    free(buffer);
    free(sample);
    *crosscorrelations = correlations;
    return o;
}

int32_t ahp_xc_scan_correlations(ahp_xc_scan_request *lines, uint32_t nlines, ahp_xc_sample **correlations, int32_t *interrupt, double *percent)
{
    if(ahp_xc_get_correlation_order() < 2)
        return ahp_xc_scan_autocorrelations(lines, nlines, correlations, interrupt, percent);
    else
        return ahp_xc_scan_crosscorrelations(lines, nlines, correlations, interrupt, percent);
}

int32_t ahp_xc_get_packet(ahp_xc_packet *packet)
{
    if(!ahp_xc.detected) return 0;
    char *sample = NULL;
    int32_t ret = 1;
    uint32_t x = 0, y = 0;
    int32_t n = ahp_xc_get_bps()/4;
    if(packet == NULL) {
        return -EINVAL;
    }
    if(pthread_mutex_trylock(((pthread_mutex_t*)packet->lock))) {
        ret = -EBUSY;
        goto end;
    }
    if(grab_packet(&packet->timestamp) < 0){
        ret = -ENOENT;
        goto end;
    }
    sample = (char*)malloc((unsigned int)n+1);
    packet->buf = ahp_xc.buf;
    const char *buf = packet->buf;
    buf += ahp_xc.header_len;
    for(x = 0; x < ahp_xc_get_nlines(); x++) {
        sample[n] = 0;
        memcpy(sample, buf, (unsigned int)n);
        if(1<sscanf(sample, "%lX", &packet->counts[x])) {
            ret = -ENOENT;
            goto err_end;
        }
        packet->counts[x] = (packet->counts[x] == 0 ? 1 : packet->counts[x]);
        buf += n;
    }
    int32_t *inputs = (int*)malloc(sizeof(int)*ahp_xc_get_correlation_order());
    double *lags = (double*)malloc(sizeof(double)*ahp_xc_get_correlation_order());
    for(x = 0; x < ahp_xc_get_nbaselines(); x++) {
        for(y = 0; y < (unsigned int)ahp_xc_get_correlation_order(); y++) {
            inputs[y] = ahp_xc_get_line_index(x, y);
            ahp_xc.cross_channel[inputs[y]].cur_chan = ahp_xc_get_current_channel_cross(inputs[y], ahp_xc.buf) * ahp_xc_get_packettime();
            lags[y] = (double)ahp_xc.cross_channel[inputs[y]].cur_chan;
        }
        ahp_xc_get_crosscorrelation(&packet->crosscorrelations[x], inputs, ahp_xc_get_correlation_order(), ahp_xc.buf, lags);
    }
    free(inputs);
    free(lags);
    wait_no_threads();
    for(x = 0; x < ahp_xc_get_nlines(); x++)
        ahp_xc_get_autocorrelation(&packet->autocorrelations[x], x, ahp_xc.buf, ahp_xc_get_current_channel_auto(x, ahp_xc.buf) * ahp_xc_get_packettime());
    wait_no_threads();
    ret = 0;
    goto free_end;
err_end:
    fprintf(stderr, "%s: %s\n", __func__, strerror(-ret));
free_end:
    free(sample);
end:
    pthread_mutex_unlock(((pthread_mutex_t*)packet->lock));
    return ret;
}

int32_t ahp_xc_get_properties()
{
    if(!ahp_xc.connected) return -ENOENT;
    if(ahp_xc.detected) return 0;
    int32_t ntries = 5;
    int32_t _bps = -1, _nlines = -1, _delaysize = -1, _auto_lagsize = -1, _cross_lagsize = -1, _flags = -1, _tau = -1;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_ENABLE);
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_ENABLE);
    while(ntries-- > 0) {
        if(grab_packet(NULL) < 0)
            continue;
        ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_ENABLE);
        int len = 0;
        char *n = (char*)malloc(2);
        char *buf = ahp_xc.buf;
        int xc_header_len = 0;
        n = (char*)realloc(n, 3);
        strncpy(n, buf, 2);
        n[2] = 0;
        int n_read = sscanf(n, "%X", &len);
        if(n_read < 1)
            return 1;
        n = (char*)realloc(n, len+1);
        buf += 2;
        strncpy(n, buf, len);
        n[len] = 0;
        n_read = sscanf(n, "%X", &_nlines);
        if(n_read < 1)
            return 1;
        xc_header_len += len + 2;
        _nlines++;
        buf += len;
        n = (char*)realloc(n, 3);
        strncpy(n, buf, 2);
        n[2] = 0;
        n_read = sscanf(n, "%X", &len);
        if(n_read < 1)
            return 1;
        n = (char*)realloc(n, len+1);
        buf += 2;
        strncpy(n, buf, len);
        n[len] = 0;
        n_read = sscanf(n, "%X", &_bps);
        if(n_read < 1)
            return 1;
        xc_header_len += len + 2;
        _bps++;
        buf += len;
        n = (char*)realloc(n, 3);
        strncpy(n, buf, 2);
        n[2] = 0;
        n_read = sscanf(n, "%X", &len);
        if(n_read < 1)
            return 1;
        n = (char*)realloc(n, len+1);
        buf += 2;
        strncpy(n, buf, len);
        n[len] = 0;
        n_read = sscanf(n, "%X", &_delaysize);
        if(n_read < 1)
            return 1;
        ahp_xc.delaysize_len = len;
        xc_header_len += len + 2;
        buf += len;
        n = (char*)realloc(n, 3);
        strncpy(n, buf, 2);
        n[2] = 0;
        n_read = sscanf(n, "%X", &len);
        if(n_read < 1)
            return 1;
        n = (char*)realloc(n, len+1);
        buf += 2;
        strncpy(n, buf, len);
        n[len] = 0;
        n_read = sscanf(n, "%X", &_auto_lagsize);
        if(n_read < 1)
            return 1;
        xc_header_len += len + 2;
        _auto_lagsize++;
        buf += len;
        n = (char*)realloc(n, 3);
        strncpy(n, buf, 2);
        n[2] = 0;
        n_read = sscanf(n, "%X", &len);
        if(n_read < 1)
            return 1;
        n = (char*)realloc(n, len+1);
        buf += 2;
        strncpy(n, buf, len);
        n[len] = 0;
        n_read = sscanf(n, "%X", &_cross_lagsize);
        if(n_read < 1)
            return 1;
        xc_header_len += len + 2;
        _cross_lagsize++;
        buf += len;
        n_read = sscanf(buf, "%02X%04X", &_flags, &_tau);
        if(n_read == 2) {
            xc_header_len += 6;
            ahp_xc.header_len = xc_header_len;
            ahp_xc.header = (char*)realloc(ahp_xc.header, ahp_xc.header_len+1);
            strncpy(ahp_xc.header, ahp_xc.buf, ahp_xc.header_len);
            ahp_xc.header[ahp_xc.header_len] = 0;
            ahp_xc.nlines = _nlines;
            ahp_xc.nbaselines = _nlines*(_nlines-1)/2;
            ahp_xc.bps = _bps;
            ahp_xc.delaysize = _delaysize;
            ahp_xc.auto_lagsize = _auto_lagsize;
            ahp_xc.cross_lagsize = _cross_lagsize;
            ahp_xc.flags = _flags;
            ahp_xc.frequency = 1000000000000.0/_tau;
            if(ahp_xc.flags & HAS_CROSSCORRELATOR)
                ahp_xc.packetsize = ahp_xc.nlines*(ahp_xc.nlines+2)*ahp_xc.bps/4+ahp_xc.header_len+16+2+1;
            else
                ahp_xc.packetsize = ahp_xc.nlines*3*ahp_xc.bps/4+ahp_xc.header_len+16+2+1;
            if(ahp_xc.leds == NULL)
                ahp_xc.leds = (unsigned char*)malloc(ahp_xc.nlines);
            if(ahp_xc.test == NULL)
                ahp_xc.test = (unsigned char*)malloc(ahp_xc.nlines);
            if(ahp_xc.autocorrelation_thread_args == NULL)
                ahp_xc.autocorrelation_thread_args = (thread_argument *)malloc(sizeof(thread_argument)*ahp_xc.nlines);
            if(ahp_xc.crosscorrelation_thread_args == NULL)
                ahp_xc.crosscorrelation_thread_args = (thread_argument *)malloc(sizeof(thread_argument)*ahp_xc.nlines);
            if(ahp_xc.autocorrelation_threads == NULL)
                ahp_xc.autocorrelation_threads = (pthread_t *)malloc(sizeof(pthread_t)*ahp_xc.nlines);
            if(ahp_xc.crosscorrelation_threads == NULL)
                ahp_xc.crosscorrelation_threads = (pthread_t *)malloc(sizeof(pthread_t)*ahp_xc.nlines);
            if(ahp_xc.auto_channel == NULL)
                ahp_xc.auto_channel = (ahp_xc_scan_request *)malloc(sizeof(ahp_xc_scan_request)*ahp_xc.nlines);
            if(ahp_xc.cross_channel == NULL)
                ahp_xc.cross_channel = (ahp_xc_scan_request *)malloc(sizeof(ahp_xc_scan_request)*ahp_xc.nbaselines);
            ahp_xc.detected = 1;
            break;
        }
        free(n);
    }
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_ENABLE);
    return 0;
}

int32_t ahp_xc_set_capture_flags(xc_capture_flags flags)
{
    if(!ahp_xc.connected) return -ENOENT;
    ahp_xc.max_lost_packets = 1;
    ahp_xc.capture_flags = flags;
    return (int)ahp_xc_send_command(ENABLE_CAPTURE, (unsigned char)ahp_xc.capture_flags);
}

xc_capture_flags ahp_xc_get_capture_flags()
{
    if(!ahp_xc.detected) return 0;
    return ahp_xc.capture_flags;
}

void ahp_xc_set_baudrate(baud_rate rate)
{
    if(!ahp_xc.detected) return;
    ahp_xc.rate = rate;
    int flags = ahp_xc_get_capture_flags();
    ahp_xc_set_capture_flags((xc_capture_flags)(flags&~CAP_EXTRA_CMD));
    ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)rate);
    ahp_xc_set_capture_flags((xc_capture_flags)(flags));
    ahp_serial_SetupPort(ahp_xc.baserate*pow(2, (int)ahp_xc.rate), "8N1", 0);
}

void ahp_xc_set_correlation_order(uint32_t order)
{
    if(!ahp_xc.detected) return;
    int32_t idx = 0;
    if(order >= ahp_xc_get_nlines())
        return;
    if(order < 1)
        return;
    ahp_xc.correlation_order = order;
    order --;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_EXTRA_CMD);
    int len = (((int)log2(order) & ~3) + 4) / 4;
    if(len < 0) len = 1;
    ahp_xc_send_command(CLEAR, SET_BAUD_RATE);
    ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)(len&0xf));
    for(idx = 0; idx < len; idx ++) {
        ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)(order&0xf));
        order >>= 4;
    }
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
}

int32_t ahp_xc_get_correlation_order()
{
    return ahp_xc.correlation_order;
}

unsigned char ahp_xc_get_test_flags(uint32_t index)
{
    if(!ahp_xc.detected) return 0;
    return ahp_xc.test[index];
}

unsigned char ahp_xc_get_leds(uint32_t index)
{
    if(!ahp_xc.detected) return 0;
    if(!ahp_xc_has_leds())
        return 0;
    return ahp_xc.leds[index];
}

void ahp_xc_set_leds(uint32_t index, int32_t leds)
{
    if(!ahp_xc.detected) return;
    ahp_xc.leds[index] = (unsigned char)leds;
    ahp_xc_select_input(index);
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
    ahp_xc_send_command(SET_LEDS, (unsigned char)((leds & (0xf & ~AHP_XC_LEDS_MASK)) | (ahp_xc_has_leds() ? leds & AHP_XC_LEDS_MASK : 0)));
    leds >>= 4;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_EXTRA_CMD);
    ahp_xc_send_command(SET_LEDS, (unsigned char)((leds & (0xf & ~AHP_XC_LEDS_MASK)) | (ahp_xc_has_leds() ? leds & AHP_XC_LEDS_MASK : 0)));
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
}

void ahp_xc_set_channel_cross(uint32_t index, off_t value, size_t size, size_t step)
{
    if(!ahp_xc.detected) return;
    int32_t idx = 0;
    if(value+size >= ahp_xc_get_delaysize())
        return;
    ahp_xc.cross_channel[index].start = value;
    ahp_xc.cross_channel[index].len = size;
    ahp_xc.cross_channel[index].step = step;
    int capture_flags = ahp_xc_get_capture_flags();
    int flags = ahp_xc_get_test_flags(index)&~TEST_STEP;
    int len = (((int)log2(step) & ~3) + 4) / 4;
    if(len < 0) len = 1;
    ahp_xc_set_test_flags(index, flags);
    ahp_xc_set_capture_flags(CAP_EXTRA_CMD);
    ahp_xc_select_input(index);
    ahp_xc_send_command(CLEAR, SET_DELAY);
    ahp_xc_send_command(CLEAR, CLEAR);
    ahp_xc_send_command(SET_DELAY, (unsigned char)(len&0xf));
    for(idx = 0; idx < len; idx ++) {
        ahp_xc_send_command(SET_DELAY, (unsigned char)(step&0xf));
        step >>= 4;
    }
    len = (((int)log2(size) & ~3) + 4) / 4;
    if(len < 0) len = 1;
    ahp_xc_select_input(index);
    ahp_xc_set_test_flags(index, flags|0x10);
    ahp_xc_send_command(CLEAR, CLEAR);
    ahp_xc_send_command(SET_DELAY, (unsigned char)(len&0xf));
    for(idx = 0; idx < len; idx ++) {
        ahp_xc_send_command(SET_DELAY, (unsigned char)(size&0xf));
        size >>= 4;
    }
    len = (((int)log2(value) & ~3) + 4) / 4;
    if(len < 0) len = 1;
    ahp_xc_select_input(index);
    ahp_xc_set_test_flags(index, flags|0x20);
    ahp_xc_send_command(CLEAR, CLEAR);
    ahp_xc_send_command(SET_DELAY, (unsigned char)(len&0xf));
    for(idx = 0; idx < len; idx ++) {
        ahp_xc_send_command(SET_DELAY, (unsigned char)(value&0xf));
        value >>= 4;
    }
    ahp_xc_set_test_flags(index, flags|TEST_STEP);
    ahp_xc_set_capture_flags(capture_flags);
}

void ahp_xc_set_channel_auto(uint32_t index, off_t value, size_t size, size_t step)
{
    if(!ahp_xc.detected) return;
    int32_t idx = 0;
    if(value+size >= ahp_xc_get_delaysize())
        return;
    ahp_xc.auto_channel[index].start = value;
    ahp_xc.auto_channel[index].len = size;
    ahp_xc.auto_channel[index].step = step;
    int capture_flags = ahp_xc_get_capture_flags();
    int flags = ahp_xc_get_test_flags(index)&~TEST_STEP;
    int len = (((int)log2(step) & ~3) + 4) / 4;
    if(len < 0) len = 1;
    ahp_xc_set_test_flags(index, flags);
    ahp_xc_set_capture_flags(0);
    ahp_xc_select_input(index);
    ahp_xc_send_command(CLEAR, SET_DELAY);
    ahp_xc_send_command(CLEAR, CLEAR);
    ahp_xc_send_command(SET_DELAY, (unsigned char)(len&0xf));
    for(idx = 0; idx < len; idx ++) {
        ahp_xc_send_command(SET_DELAY, (unsigned char)(step&0xf));
        step >>= 4;
    }
    len = (((int)log2(size) & ~3) + 4) / 4;
    if(len < 0) len = 1;
    ahp_xc_select_input(index);
    ahp_xc_set_test_flags(index, flags|0x10);
    ahp_xc_send_command(CLEAR, CLEAR);
    ahp_xc_send_command(SET_DELAY, (unsigned char)(len&0xf));
    for(idx = 0; idx < len; idx ++) {
        ahp_xc_send_command(SET_DELAY, (unsigned char)(size&0xf));
        size >>= 4;
    }
    len = (((int)log2(value) & ~3) + 4) / 4;
    if(len < 0) len = 1;
    ahp_xc_select_input(index);
    ahp_xc_set_test_flags(index, flags|0x20);
    ahp_xc_send_command(CLEAR, CLEAR);
    ahp_xc_send_command(SET_DELAY, (unsigned char)(len&0xf));
    for(idx = 0; idx < len; idx ++) {
        ahp_xc_send_command(SET_DELAY, (unsigned char)(value&0xf));
        value >>= 4;
    }
    ahp_xc_set_test_flags(index, flags|TEST_STEP);
    ahp_xc_set_capture_flags(capture_flags);
}

void ahp_xc_set_voltage(uint32_t index, unsigned char value)
{
    if(!ahp_xc.detected) return;
    ahp_xc_select_input(index);
    value = (unsigned char)(value < 0xff ? value : 0xff);
    ahp_xc.voltage = value;
    int flags = ahp_xc_get_capture_flags();
    ahp_xc_set_capture_flags(0);
    ahp_xc_send_command(SET_VOLTAGE, (unsigned char)(ahp_xc.voltage&0xf));
    ahp_xc_set_capture_flags(CAP_EXTRA_CMD);
    ahp_xc_send_command(SET_VOLTAGE, (unsigned char)((ahp_xc.voltage>>4)&0xf));
    ahp_xc_set_capture_flags(flags);
}

void ahp_xc_set_test_flags(uint32_t index, int32_t value)
{
    if(!ahp_xc.detected) return;
    ahp_xc_select_input(index);
    ahp_xc.test[index] = value;
    int flags = ahp_xc_get_capture_flags();
    ahp_xc_set_capture_flags(0);
    ahp_xc_send_command(ENABLE_TEST, (unsigned char)(ahp_xc.test[index]&0xf));
    ahp_xc_set_capture_flags(CAP_EXTRA_CMD);
    ahp_xc_send_command(ENABLE_TEST, (unsigned char)((ahp_xc.test[index]>>4)&0xf));
    ahp_xc_set_capture_flags(flags);
}

 int32_t ahp_xc_send_command(xc_cmd cmd, unsigned char value)
{
    if(!ahp_xc.connected) return -ENOENT;
    int32_t err = 0;
    unsigned char c = (unsigned char)(cmd|(value<<4));
    ahp_serial_flushTX();
    perr("%02X ", c);
    err |= ahp_serial_SendByte(c);
    return err;
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
