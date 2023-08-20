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
#include <pthread.h>
#include <sys/time.h>
#include "ahp_xc.h"

#include "rs232.c"

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
} thread_argument;

static uint64_t nthreads = 0;
thread_argument *autocorrelation_thread_args = NULL;
thread_argument *crosscorrelation_thread_args = NULL;
static pthread_t *autocorrelation_threads = NULL;
static pthread_t *crosscorrelation_threads = NULL;
static pthread_mutex_t ahp_xc_mutex;
static int32_t ahp_xc_mutexes_initialized = 0;

static int32_t AHP_XC_MAX_THREADS = 1;
static unsigned char *ahp_xc_test = NULL;
static unsigned char *ahp_xc_leds = NULL;
static uint32_t ahp_xc_bps = 0;
static uint32_t ahp_xc_nlines = 0;
static uint32_t ahp_xc_nbaselines = 0;
static uint32_t ahp_xc_auto_lagsize = 0;
static uint32_t ahp_xc_cross_lagsize = 0;
static uint32_t ahp_xc_delaysize = 0;
static uint32_t ahp_xc_flags = 0;
static uint32_t ahp_xc_correlator_enabled = 1;
static uint32_t ahp_xc_intensity_correlator_enabled = 0;
static double ahp_xc_frequency = 1;
static uint32_t ahp_xc_voltage = 0;
static uint32_t ahp_xc_connected = 0;
static uint32_t ahp_xc_detected = 0;
static uint32_t ahp_xc_packetsize = 17;
static int32_t ahp_xc_baserate = XC_BASE_RATE;
static baud_rate ahp_xc_rate = R_BASE;
static uint32_t ahp_xc_correlation_order = 0;
static char ahp_xc_comport[128];
static char ahp_xc_header[18] = { 0 };
static unsigned char ahp_xc_capture_flags = 0;
static unsigned char ahp_xc_max_lost_packets = 1;

static int32_t get_line_index(int32_t idx, int32_t order)
{
    return (idx + order * (idx / ahp_xc_get_nlines() + 1)) % ahp_xc_get_nlines();
}

static int32_t get_crosscorrelation_index(int32_t *lines, int32_t order)
{
    int32_t x, y, idx;
    int32_t nprisms = ahp_xc_get_nbaseprisms(order);
    int* matches = (int*)malloc(sizeof(int)*nprisms);
    memset(matches, 0, sizeof(int)*nprisms);
    for(x = 0; x < order; x ++) {
        for(idx = 0; idx < nprisms; idx++) {
            for(y = 0; y < order; y ++) {
                if(lines[y] == get_line_index(idx, x))
                    matches[idx]++;
            }
        }
    }
    int32_t index = 0;
    int32_t best_match = 0;
    for(idx = 0; idx < nprisms; idx++) {
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
        AHP_XC_MAX_THREADS = value;
    }
    return AHP_XC_MAX_THREADS;
}

void wait_threads()
{
    while (nthreads >= ahp_xc_max_threads(0))
        usleep(1);
}

void wait_no_threads()
{
    while (nthreads > 0)
        usleep(1);
}

static void complex_phase_magnitude(ahp_xc_correlation *sample)
{
    if(!ahp_xc_detected) return;
    double magnitude = (double)sqrt(pow((double)sample->real, 2)+pow((double)sample->imaginary, 2));
    double phase = 0.0;
    if(magnitude > 0.0) {
        phase = asin ((double)sample->real / magnitude);
        if(sample->imaginary < 0)
            phase = M_PI*2.0-phase;
    }
    phase += M_PI;
    sample->magnitude = magnitude;
    sample->phase = phase;
}

double get_timestamp(char *data)
{
    char timestamp[16];
    double ts = 0;
    uint32_t tmp = 0;
    strncpy(timestamp, &data[ahp_xc_get_packetsize()-19], 16);
    sscanf(timestamp, "%8X", &tmp);
    ts = (double)tmp * 4.294967296;
    sscanf(&timestamp[8], "%8X", &tmp);
    return (double)ts + tmp / 1000000000.0;
}

int32_t calc_checksum(char *data)
{
    if(!ahp_xc_connected) return -ENOENT;
    int32_t x;
    uint32_t checksum = 0x00;
    uint32_t calculated_checksum = 0;
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
        return EINVAL;
    }
    return 0;
}

int32_t check_sof(char *data)
{
    if(!ahp_xc_connected) return -ENOENT;
    if(!strncmp(data, "FFFFFFFFFFFFFFFF", 16))
        return 1;
    return 0;
}

static char * grab_packet(double *timestamp)
{
    errno = 0;
    uint32_t size = ahp_xc_get_packetsize();
    char *buf = (char*)malloc(ahp_xc_get_packetsize());
    memset(buf, 0, (unsigned int)size);
    if(!ahp_xc_connected){
        errno = ENOENT;
        goto err_end;
    }
    int32_t nread = 0;
    nread = ahp_serial_RecvBuf((unsigned char*)buf, size);
    if(buf[0] == '\r') {
        ahp_serial_AlignFrame('\r', (int)size);
        goto err_end;
    }
    buf[nread-1] = 0;
    if(nread == 0) {
        errno = ENODATA;
    } else if(nread < 0) {
        errno = ETIMEDOUT;
    } else if(nread > 17) {
        if(strncmp(ahp_xc_get_header(), (char*)buf, 16)) {
            errno = EINVAL;
            ahp_serial_AlignFrame('\r', -1);
        } else if(check_sof((char*)buf)) {
            errno = 0;
        } else if(strlen((char*)buf) < size-1) {
            errno = ERANGE;
        } else {
            errno = calc_checksum((char*)buf);
        }
    } else if(size == 17) {
        fprintf(stdout, "Model: %s\n", buf);
        errno = 0;
    }
    if(errno)
        goto err_end;
    if(timestamp != NULL)
        *timestamp = get_timestamp(buf);
    return buf;
err_end:
    fprintf(stderr, "%s error: %s\n", __func__, strerror(errno));
    free(buf);
    return NULL;
}

uint32_t ahp_xc_current_input()
{
    return xc_current_input;
}

void ahp_xc_select_input(uint32_t index)
{
    if(!ahp_xc_detected) return;
    int32_t idx = 0;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
    index >>= 2;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
    index >>= 2;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
    index >>= 2;
    ahp_xc_send_command(SET_INDEX, (unsigned char)((idx++)<<2)|(index&0x3));
    xc_current_input = index;
}

void ahp_xc_enable_crosscorrelator(int32_t enable)
{
    if(!ahp_xc_detected) return;
    ahp_xc_correlator_enabled = enable;
}

void ahp_xc_enable_intensity_crosscorrelator(int32_t enable)
{
    if(!ahp_xc_detected) return;
    ahp_xc_intensity_correlator_enabled = enable;
}

int32_t ahp_xc_intensity_crosscorrelator_enabled()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_intensity_correlator_enabled != 0 || !ahp_xc_has_crosscorrelator();
}

int32_t ahp_xc_has_crosscorrelator()
{
    if(!ahp_xc_detected) return 0;
    return (ahp_xc_flags & HAS_CROSSCORRELATOR ? ahp_xc_correlator_enabled : 0);
}

int32_t ahp_xc_has_psu()
{
    if(!ahp_xc_detected) return 0;
    return (ahp_xc_flags & HAS_PSU ? 1 : 0);
}

int32_t ahp_xc_has_leds()
{
    if(!ahp_xc_detected) return 0;
    return (ahp_xc_flags & HAS_LEDS ? 1 : 0);
}

int32_t ahp_xc_has_cumulative_only()
{
    if(!ahp_xc_detected) return 0;
    return (ahp_xc_flags & HAS_CUMULATIVE_ONLY ? 1 : 0);
}

char* ahp_xc_get_header()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_header;
}

int32_t ahp_xc_get_baudrate()
{
    if(!ahp_xc_detected) return 0;
    return XC_BASE_RATE << ahp_xc_rate;
}

uint32_t ahp_xc_get_bps()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_bps;
}

uint32_t ahp_xc_get_nlines()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_nlines;
}

uint32_t ahp_xc_get_nbaselines()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_get_nlines() * (ahp_xc_get_nlines() - 1) / 2;
}

uint32_t ahp_xc_get_nbaseprisms(int32_t order)
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_get_nlines() * (ahp_xc_get_nlines() - 1) / (pow(order, 2) / 2);
}

uint32_t ahp_xc_get_delaysize()
{
    if(!ahp_xc_detected) return 0;
    if(ahp_xc_delaysize == 0 || ahp_xc_delaysize == 4)
        return pow(2, 18);
    return ahp_xc_delaysize * 17;
}

uint32_t ahp_xc_get_autocorrelator_lagsize()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_auto_lagsize;
}

uint32_t ahp_xc_get_crosscorrelator_lagsize()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_cross_lagsize;
}

double ahp_xc_get_frequency()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_frequency;
}

double ahp_xc_get_sampletime()
{
    return (double)1.0/ahp_xc_get_frequency();
}

double ahp_xc_get_packettime()
{
    if(!ahp_xc_detected) return 0;
    return 9.0  * (double)ahp_xc_get_packetsize() / (double)ahp_xc_get_baudrate();
}

uint32_t ahp_xc_get_packetsize()
{
    return ahp_xc_packetsize;
}

int32_t ahp_xc_get_fd()
{
    return ahp_serial_GetFD();
}

int32_t ahp_xc_connect_fd(int32_t fd)
{
    if(ahp_xc_connected)
        return 0;
    ahp_xc_bps = 0;
    ahp_xc_nlines = 0;
    ahp_xc_nbaselines = 0;
    ahp_xc_delaysize = 0;
    ahp_xc_frequency = 0;
    ahp_xc_packetsize = 17;
    ahp_xc_rate = R_BASE;
    if(fd > -1) {
        ahp_xc_detected = 0;
        ahp_serial_SetFD(fd, XC_BASE_RATE);
        if(!ahp_xc_mutexes_initialized) {
            pthread_mutex_init(&ahp_xc_mutex, &ahp_serial_mutex_attr);
            ahp_xc_mutexes_initialized = 1;
        }
        nthreads = 0;
        xc_current_input = 0;
        ahp_xc_connected = 1;
        return 0;
    }
    return 1;
}

int32_t ahp_xc_connect(const char *port, int32_t high_rate)
{
    if(ahp_xc_connected)
        return 0;
    ahp_xc_header[0] = 0;
    ahp_xc_header[16] = 0;
    int32_t ret = 1;
    ahp_xc_bps = 0;
    ahp_xc_nlines = 0;
    ahp_xc_nbaselines = 0;
    ahp_xc_delaysize = 0;
    ahp_xc_frequency = 0;
    ahp_xc_packetsize = 17;
    ahp_xc_baserate = (high_rate ? XC_HIGH_RATE : XC_BASE_RATE);
    ahp_xc_rate = R_BASE;
    strcpy(ahp_xc_comport, port);
    if(!ahp_serial_OpenComport(ahp_xc_comport))
        ret = ahp_serial_SetupPort(ahp_xc_baserate, "8N2", 0);
    if(!ret) {
        if(!ahp_xc_mutexes_initialized) {
            pthread_mutex_init(&ahp_xc_mutex, &ahp_serial_mutex_attr);
            ahp_xc_mutexes_initialized = 1;
        }
        nthreads = 0;
        xc_current_input = 0;
        ahp_xc_connected = 1;
        ahp_xc_detected = 0;
    }
    return ret;
}
void ahp_xc_disconnect()
{
    if(ahp_xc_connected) {
        if(ahp_xc_mutexes_initialized) {
            pthread_mutex_unlock(&ahp_xc_mutex);
            pthread_mutex_destroy(&ahp_xc_mutex);
            ahp_xc_mutexes_initialized = 0;
        }
        ahp_xc_connected = 0;
        ahp_xc_detected = 0;
        ahp_xc_bps = 0;
        ahp_xc_nlines = 0;
        ahp_xc_nbaselines = 0;
        ahp_xc_delaysize = 0;
        ahp_xc_frequency = 0;
        ahp_xc_packetsize = 17;
        ahp_serial_CloseComport();
    }
}

uint32_t ahp_xc_is_connected()
{
    return ahp_xc_connected;
}

uint32_t ahp_xc_is_detected()
{
    return ahp_xc_detected;
}

ahp_xc_sample *ahp_xc_alloc_samples(uint64_t nlines, size_t size)
{
    uint64_t x;
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
    uint64_t x;
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

void ahp_xc_start_autocorrelation_scan(uint32_t index, off_t start, size_t size, size_t step)
{
    if(!ahp_xc_detected) return;
    ahp_xc_set_capture_flags((ahp_xc_get_capture_flags()|CAP_RESET_TIMESTAMP)&~CAP_ENABLE);
    ahp_xc_set_channel_auto(index, start, size, step);
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|SCAN_AUTO);
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_ENABLE);
}

void ahp_xc_end_autocorrelation_scan(uint32_t index)
{
    if(!ahp_xc_detected) return;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~SCAN_AUTO);
    ahp_xc_set_channel_auto(index, 0, 1, 1);
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~(CAP_ENABLE|CAP_RESET_TIMESTAMP));
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
    packet += 16;
    memcpy(subpacket, &packet[index*n], (unsigned int)n);
    uint64_t counts = strtoul(subpacket, NULL, 16)|1;
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
    if(nthreads > 0)
        nthreads--;
    return NULL;
}

void ahp_xc_get_autocorrelation(ahp_xc_sample *sample, int32_t index, const char *data, double lag)
{
    if(!ahp_xc_mutexes_initialized)
        return;
    autocorrelation_thread_args[index].sample = sample;
    autocorrelation_thread_args[index].index = index;
    autocorrelation_thread_args[index].data = data;
    autocorrelation_thread_args[index].lag = lag;
    _get_autocorrelation(&autocorrelation_thread_args[index]);
}

int32_t ahp_xc_scan_autocorrelations(uint32_t nlines, uint32_t *indexes, ahp_xc_sample **autocorrelations, off_t *starts, size_t *sizes, size_t *steps, int32_t *interrupt, double *percent)
{
    if(!ahp_xc_detected) return 0;
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
        starts[i] = (starts[i] < ahp_xc_get_delaysize()-2 ? starts[i] : (off_t)ahp_xc_get_delaysize()-2);
        sizes[i] = (starts[i]+(off_t)sizes[i] < ahp_xc_get_delaysize() ? (off_t)sizes[i] : (off_t)ahp_xc_get_delaysize()-1-starts[i]);
        len = fmax(len, sizes[i]/steps[i]);
        size += sizes[i]/steps[i];
    }
    ahp_xc_sample *correlations = ahp_xc_alloc_samples(size, (unsigned int)ahp_xc_get_autocorrelator_lagsize());
    char* data = (char*)malloc(ahp_xc_get_packetsize()*len);
    for(i = 0; i < nlines; i++)
        ahp_xc_start_autocorrelation_scan(indexes[i], starts[i], sizes[i], steps[i]);
    i = 0;
    char* buf = NULL;
    i = 0;
    while(i < len) {
        if(*interrupt)
            break;
        buf = grab_packet(NULL);
        if(!buf)
            continue;
        if(check_sof(buf))
            i = 0;
        memcpy(data+i*ahp_xc_get_packetsize(), buf, ahp_xc_get_packetsize());
        i++;
        free(buf);
        (*percent) += 100.0 / len;
        r++;
    }
    for(i = 0; i < nlines; i++)
        ahp_xc_end_autocorrelation_scan(indexes[i]);
    i = 0;
    while((int)i < r) {
        if(*interrupt)
            break;
        char *packet = (char*)data+i*ahp_xc_get_packetsize();
        ts = get_timestamp(packet);
        if(ts0 == 0.0)
            ts0 = ts;
        ts -= ts0;
        size_t off = 0;
        for(x = 0; x < nlines; x++) {
            if(i < sizes[x]/steps[x]) {
                ahp_xc_correlation correlation;
                memset(&correlation, 0, sizeof(ahp_xc_correlation));
                correlation.lag = ts;
                ahp_xc_sample *sample = ahp_xc_alloc_samples(1, (size_t)ahp_xc_get_autocorrelator_lagsize());
                ahp_xc_get_autocorrelation(sample, indexes[x], packet+y*ahp_xc_get_packetsize(), ts);
                double rad_p = (sample->correlations[0].phase+correlation.phase)/2.0;
                double rad_m = (sample->correlations[0].phase-correlation.phase)/2.0;
                correlation.counts += sample->correlations[0].counts;
                correlation.magnitude += sample->correlations[0].magnitude;
                correlation.real = 2*sin(rad_p)*cos(rad_m)*correlation.magnitude;
                correlation.imaginary = 2*cos(rad_p)*cos(rad_m)*correlation.magnitude;
                complex_phase_magnitude(&correlation);
                ahp_xc_free_samples(1, sample);
                memcpy(&correlations[i+off].correlations[0], &correlation, sizeof(ahp_xc_correlation));
                s++;
            }
            off += sizes[x]/steps[x];
        }
        wait_no_threads();
        i++;
    }
    free(data);
    *autocorrelations = correlations;
    return s;
}

void ahp_xc_start_crosscorrelation_scan(uint32_t index, off_t start, size_t size, size_t step)
{
    if(!ahp_xc_detected) return;
    ahp_xc_end_crosscorrelation_scan(index);
    ahp_xc_set_capture_flags((ahp_xc_get_capture_flags()|CAP_RESET_TIMESTAMP)&~CAP_ENABLE);
    if(!ahp_xc_intensity_crosscorrelator_enabled())
        ahp_xc_set_channel_auto(index, start, size, step);
    else
        ahp_xc_set_channel_cross(index, start, size, step);
    usleep(ahp_xc_get_packettime()*1000000);
    if(!ahp_xc_intensity_crosscorrelator_enabled())
        ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|SCAN_CROSS);
    else
        ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|SCAN_AUTO);
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_ENABLE);
}

void ahp_xc_end_crosscorrelation_scan(uint32_t index)
{
    if(!ahp_xc_detected) return;
    if(!ahp_xc_intensity_crosscorrelator_enabled())
        ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~SCAN_CROSS);
    else
        ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~SCAN_AUTO);
    if(!ahp_xc_intensity_crosscorrelator_enabled())
        ahp_xc_set_channel_auto(index, 0, 1, 0);
    else
        ahp_xc_set_channel_cross(index, 0, 1, 0);
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~(CAP_ENABLE|CAP_RESET_TIMESTAMP));
}

void *_get_crosscorrelation(void *o)
{
    thread_argument *arg = (thread_argument*)o;
    ahp_xc_sample *sample = arg->sample;
    int32_t *indexes = arg->indexes;
    int32_t index = arg->index;
    uint32_t num_indexes = arg->order;
    const char *data = arg->data;
    double lag = arg->lag;
    uint32_t x, y;
    int32_t n = ahp_xc_get_bps() / 4;
    const char *packet = data;
    sample->lag_size = (ahp_xc_get_crosscorrelator_lagsize()*2-1);
    sample->lag = lag;
    if(ahp_xc_intensity_crosscorrelator_enabled()) {
        ahp_xc_sample **samples = (ahp_xc_sample**)malloc(sizeof(ahp_xc_sample*)*num_indexes);
        for(y = 0; y < num_indexes; y++) {
            samples[y] = ahp_xc_alloc_samples(1, ahp_xc_get_autocorrelator_lagsize());
            ahp_xc_get_autocorrelation(samples[y], indexes[y], packet, lag);
        }
        wait_no_threads();
        for (y = 0; y < ahp_xc_get_autocorrelator_lagsize(); y++) {
            sample->correlations[y].lag = samples[0]->lag+y*ahp_xc_get_sampletime();
            sample->correlations[y].counts = samples[0]->correlations[y].counts;
            sample->correlations[y].magnitude = samples[0]->correlations[y].magnitude;
            sample->correlations[y].phase = samples[0]->correlations[y].phase;
            sample->correlations[y].real = 0.0;
            sample->correlations[y].imaginary = 0.0;
            ahp_xc_free_samples(1, samples[0]);
            for (x = 1; x < num_indexes; x++) {
                sample->correlations[y].counts += samples[x]->correlations[y].counts;
                sample->correlations[y].magnitude = pow(sample->correlations[y].magnitude * samples[x]->correlations[y].magnitude, 0.5);
                sample->correlations[y].lag = sample->lag+y*ahp_xc_get_sampletime();
                sample->correlations[y].real = (cos(sample->correlations[y].phase-samples[x]->correlations[y].phase)-cos(sample->correlations[y].phase+samples[x]->correlations[y].phase)) / 2.0 * sample->correlations[y].magnitude;
                sample->correlations[y].imaginary = (cos(sample->correlations[y].phase-samples[x]->correlations[y].phase)+cos(sample->correlations[y].phase+samples[x]->correlations[y].phase)) / 2.0 * sample->correlations[y].magnitude;
                complex_phase_magnitude(&sample->correlations[y]);
                ahp_xc_free_samples(1, samples[x]);
            }
        }
        free(samples);
    } else {
        char *subpacket = (char*)malloc(n+1);
        memset(subpacket, 0, n+1);
        packet += 16;
        uint64_t counts = 0;
        for(y = 0; y < num_indexes; y++) {
            memcpy(subpacket, &packet[indexes[y]*n], (unsigned int)n);
            counts += strtoul(subpacket, NULL, 16)|1;
        }
        packet += n*ahp_xc_get_nlines();
        packet += n*ahp_xc_get_autocorrelator_lagsize()*ahp_xc_get_nlines()*2;
        packet += n*index*2;
        for(y = 0; y < sample->lag_size; y++) {
            sample->correlations[y].lag = sample->lag+(y-ahp_xc_get_crosscorrelator_lagsize()+1)*ahp_xc_get_sampletime();
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
    if(nthreads > 0)
        nthreads--;
    return NULL;
}

void ahp_xc_get_crosscorrelation(ahp_xc_sample *sample, int32_t *indexes, int32_t order, const char *data, double lag)
{
    if(!ahp_xc_mutexes_initialized)
        return;
    int32_t index = get_crosscorrelation_index(indexes, order);
    crosscorrelation_thread_args[index].sample = sample;
    crosscorrelation_thread_args[index].index = index;
    crosscorrelation_thread_args[index].indexes = indexes;
    crosscorrelation_thread_args[index].order = order;
    crosscorrelation_thread_args[index].data = data;
    crosscorrelation_thread_args[index].lag = lag;
    _get_crosscorrelation(&crosscorrelation_thread_args[index]);
}

int32_t ahp_xc_scan_crosscorrelations(uint32_t index1, uint32_t index2, ahp_xc_sample **crosscorrelations, off_t head_start, size_t head_size, off_t tail_start, size_t tail_size, size_t step, int32_t *interrupt, double *percent)
{
    if(!ahp_xc_detected) return 0;
    size_t k = 0;
    int32_t i = 0;
    double ts = 0.0;
    double ts0 = 0.0;
    uint32_t n = ahp_xc_get_bps()/4;
    *crosscorrelations = NULL;
    uint32_t idx1 = (index1 < index2 ? index1 : index2);
    uint32_t idx2 = (index1 > index2 ? index1 : index2);
    int32_t size = (head_size+tail_size)/step;
    if(idx1 == idx2)
        return -1;
    head_start = (head_start < ahp_xc_get_delaysize()-2 ? head_start : (off_t)ahp_xc_get_delaysize()-2);
    tail_start = (tail_start < ahp_xc_get_delaysize()-2 ? tail_start : (off_t)ahp_xc_get_delaysize()-2);
    char* head = (char*)malloc(ahp_xc_get_packetsize()*head_size/step);
    char* tail = (char*)malloc(ahp_xc_get_packetsize()*tail_size/step);
    ahp_xc_sample *correlations = ahp_xc_alloc_samples((unsigned int)size, (unsigned int)ahp_xc_get_crosscorrelator_lagsize());
    char* sample = (char*)malloc((unsigned int)n+1);
    sample[n] = 0;
    (*percent) = 0;
    if(ahp_xc_intensity_crosscorrelator_enabled()) {
        ahp_xc_end_autocorrelation_scan(idx2);
        ahp_xc_set_channel_auto(idx2, head_start, 1, 0);
        ahp_xc_start_autocorrelation_scan(idx2, head_start, head_size, step);
    } else {
        ahp_xc_end_crosscorrelation_scan(idx2);
        ahp_xc_set_channel_cross(idx2, head_start, 1, 0);
        ahp_xc_start_crosscorrelation_scan(idx1, tail_start, tail_size, step);
    }
    i = 0;
    while(i < (int)(head_size/step)) {
        if(*interrupt)
            break;
        usleep(ahp_xc_get_packettime()*1000000);
        char* buf = grab_packet(NULL);
        if(!buf)
            continue;
        if(check_sof(buf))
            i = 0;
        memcpy(head+i*ahp_xc_get_packetsize(), buf, ahp_xc_get_packetsize());
        free(buf);
        (*percent) += 100.0 / size;
        i++;
    }
    if(ahp_xc_intensity_crosscorrelator_enabled()) {
        ahp_xc_end_autocorrelation_scan(idx1);
        ahp_xc_set_channel_auto(idx1, head_start, 1, 0);
        ahp_xc_start_autocorrelation_scan(idx1, head_start, head_size, step);
    } else {
        ahp_xc_end_crosscorrelation_scan(idx1);
        ahp_xc_set_channel_cross(idx1, head_start, 1, 0);
        ahp_xc_start_crosscorrelation_scan(idx2, tail_start, tail_size, step);
    }
    i = 0;
    while(i < (int)(tail_size/step)) {
        if(*interrupt)
            break;
        usleep(ahp_xc_get_packettime()*1000000);
        char* buf = grab_packet(NULL);
        if(!buf)
            continue;
        if(check_sof(buf))
            i = 0;
        memcpy(tail+i*ahp_xc_get_packetsize(), buf, ahp_xc_get_packetsize());
        free(buf);
        (*percent) += 100.0 / size;
        i++;
    }
    ahp_xc_end_crosscorrelation_scan(idx2);
    i = 0;
    k = 0;
    ts0 = 0.0;
    while(k < head_size/step) {
        if(*interrupt)
            break;
        char *packet = (char*)head+k*ahp_xc_get_packetsize();
        ts = get_timestamp(packet);
        if(ts0 == 0.0)
            ts0 = ts;
        ts -= ts0;
        ahp_xc_get_crosscorrelation(&correlations[i], (int[]){idx1, idx2}, 2, packet, -ts);
        wait_no_threads();
        i++;
        k++;
    }
    k = 0;
    ts0 = 0.0;
    while(k < tail_size/step) {
        if(*interrupt)
            break;
        char *packet = (char*)tail+k*ahp_xc_get_packetsize();
        ts = get_timestamp(packet);
        if(ts0 == 0.0)
            ts0 = ts;
        ts -= ts0;
        ahp_xc_get_crosscorrelation(&correlations[i], (int[]){idx1, idx2}, 2, packet, ts);
        wait_no_threads();
        i++;
        k++;
    }
    free(head);
    free(tail);
    free(sample);
    *crosscorrelations = correlations;
    return i;
}

int32_t ahp_xc_get_packet(ahp_xc_packet *packet)
{
    if(!ahp_xc_detected) return 0;
    char* data = NULL;
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
    data = grab_packet(&packet->timestamp);
    if(!data){
        ret = -ENOENT;
        goto end;
    }
    sample = (char*)malloc((unsigned int)n+1);
    packet->buf = data;
    const char *buf = packet->buf;
    buf += 16;
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
    for(x = 0; x < ahp_xc_get_nbaselines(); x++) {
        int32_t *inputs = (int*)malloc(sizeof(int)*ahp_xc_get_correlation_order());
        for(y = 0; y < (unsigned int)ahp_xc_get_correlation_order(); y++)
            inputs[y] = get_line_index(x, y);
        ahp_xc_get_crosscorrelation(&packet->crosscorrelations[x], inputs, ahp_xc_get_correlation_order(), data, 0.0);
    }
    wait_no_threads();
    for(x = 0; x < ahp_xc_get_nlines(); x++)
        ahp_xc_get_autocorrelation(&packet->autocorrelations[x], x, data, 0.0);
    wait_no_threads();
    ret = 0;
    goto free_end;
err_end:
    fprintf(stderr, "%s: %s\n", __func__, strerror(-ret));
free_end:
    free(sample);
    free(data);
end:
    pthread_mutex_unlock(((pthread_mutex_t*)packet->lock));
    return ret;
}

int32_t ahp_xc_get_properties()
{
    if(!ahp_xc_connected) return -ENOENT;
    char *data = NULL;
    int32_t n_read = 0;
    int32_t ntries = 16;
    int32_t _bps = -1, _nlines = -1, _delaysize = -1, _auto_lagsize = -1, _cross_lagsize = -1, _flags = -1, _tau = -1;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_ENABLE);
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_ENABLE);
    while(ntries-- > 0) {
        ahp_serial_AlignFrame('\r', -1);
        data = grab_packet(NULL);
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
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_ENABLE);
    if(n_read != 7)
        return -ENODEV;
    ahp_xc_flags = _flags;
    ahp_xc_bps = _bps;
    ahp_xc_nlines = _nlines+1;
    ahp_xc_nbaselines = (ahp_xc_flags & HAS_CROSSCORRELATOR) ? (ahp_xc_nlines*(ahp_xc_nlines-1)/2) : 0;
    ahp_xc_delaysize = _delaysize;
    ahp_xc_auto_lagsize = _auto_lagsize+1;
    ahp_xc_cross_lagsize = _cross_lagsize+1;
    ahp_xc_packetsize = (ahp_xc_nlines+ahp_xc_auto_lagsize*ahp_xc_nlines*2+(ahp_xc_cross_lagsize*2-1)*ahp_xc_nbaselines*2)*ahp_xc_bps/4+16+16+2+1;
    ahp_xc_frequency = 1000000000000.0/(!_tau?1:_tau);
    sign = (pow(2, ahp_xc_bps-1));
    fill = sign|(sign - 1);

    if(ahp_xc_mutexes_initialized) {
        int nbaselines = ahp_xc_nlines * (ahp_xc_nlines - 1) / 2;
        if(crosscorrelation_threads)
            crosscorrelation_threads = (pthread_t*)realloc(crosscorrelation_threads, sizeof(pthread_t)*nbaselines);
        else
            crosscorrelation_threads = (pthread_t*)malloc(sizeof(pthread_t)*nbaselines);
        memset(crosscorrelation_threads, 0, sizeof(pthread_t)*nbaselines);
        if(crosscorrelation_thread_args)
            crosscorrelation_thread_args = (thread_argument*)realloc(crosscorrelation_thread_args, sizeof(thread_argument)*nbaselines);
        else
            crosscorrelation_thread_args = (thread_argument*)malloc(sizeof(thread_argument)*nbaselines);
        memset(crosscorrelation_thread_args, 0, sizeof(thread_argument)*nbaselines);
        if(autocorrelation_threads)
            autocorrelation_threads = (pthread_t*)realloc(autocorrelation_threads, sizeof(pthread_t)*ahp_xc_nlines);
        else
            autocorrelation_threads = (pthread_t*)malloc(sizeof(pthread_t)*ahp_xc_nlines);
        memset(autocorrelation_threads, 0, sizeof(pthread_t)*ahp_xc_nlines);
        if(autocorrelation_thread_args)
            autocorrelation_thread_args = (thread_argument*)realloc(autocorrelation_thread_args, sizeof(thread_argument)*ahp_xc_nlines);
        else
            autocorrelation_thread_args = (thread_argument*)malloc(sizeof(thread_argument)*ahp_xc_nlines);
        memset(autocorrelation_thread_args, 0, sizeof(thread_argument)*ahp_xc_nlines);
    }
    nthreads = 0;
    if(ahp_xc_test)
        ahp_xc_test = (unsigned char*)realloc(ahp_xc_test, ahp_xc_nlines);
    else
        ahp_xc_test = (unsigned char*)malloc(ahp_xc_nlines);
    memset(ahp_xc_test, 0, ahp_xc_nlines);
    if(ahp_xc_leds)
        ahp_xc_leds = (unsigned char*)realloc(ahp_xc_leds, ahp_xc_nlines);
    else
        ahp_xc_leds = (unsigned char*)malloc(ahp_xc_nlines);
    memset(ahp_xc_leds, 0, ahp_xc_nlines);
    ahp_xc_detected = 1;
    return 0;
}

int32_t ahp_xc_set_capture_flags(xc_capture_flags flags)
{
    if(!ahp_xc_connected) return -ENOENT;
    ahp_xc_max_lost_packets = 1;
    ahp_xc_capture_flags = flags;
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
    xc_capture_flags old_flags = ahp_xc_get_capture_flags();
    ahp_xc_set_capture_flags(old_flags&~CAP_EXTRA_CMD);
    ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)rate);
    ahp_xc_set_capture_flags(old_flags);
    ahp_serial_CloseComport();
    ahp_serial_OpenComport(ahp_xc_comport);
    ahp_serial_SetupPort(ahp_xc_baserate*pow(2, (int)ahp_xc_rate), "8N2", 0);
}

void ahp_xc_set_correlation_order(uint32_t order)
{
    if(!ahp_xc_detected) return;
    if(order < 2) return;
    if(order > ahp_xc_get_nlines()) return;
    ahp_xc_correlation_order = order - 2;
    xc_capture_flags old_flags = ahp_xc_get_capture_flags();
    ahp_xc_set_capture_flags(old_flags|CAP_EXTRA_CMD);
    int index = 0;
    ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)(index<<2|(order&0x3)));
    index++;
    ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)(index<<2|(order&0x3)));
    index++;
    ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)(index<<2|(order&0x3)));
    index++;
    ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)(index<<2|(order&0x3)));
    index++;
    ahp_xc_set_capture_flags(old_flags);
}

int32_t ahp_xc_get_correlation_order()
{
    return ahp_xc_correlation_order + 2;
}

unsigned char ahp_xc_get_test_flags(uint32_t index)
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_test[index];
}

unsigned char ahp_xc_get_leds(uint32_t index)
{
    if(!ahp_xc_detected) return 0;
    if(!ahp_xc_has_leds())
        return 0;
    return ahp_xc_leds[index];
}

void ahp_xc_set_leds(uint32_t index, int32_t leds)
{
    if(!ahp_xc_detected) return;
    ahp_xc_leds[index] = (unsigned char)leds;
    ahp_xc_select_input(index);
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
    ahp_xc_send_command(SET_LEDS, (unsigned char)((leds & (0xf & ~AHP_XC_LEDS_MASK)) | (ahp_xc_has_leds() ? leds & AHP_XC_LEDS_MASK : 0)));
    leds >>= 4;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_EXTRA_CMD);
    ahp_xc_send_command(SET_LEDS, (unsigned char)((leds & (0xf & ~AHP_XC_LEDS_MASK)) | (ahp_xc_has_leds() ? leds & AHP_XC_LEDS_MASK : 0)));
}

void ahp_xc_set_channel_cross(uint32_t index, off_t value, size_t size, size_t step)
{
    if(!ahp_xc_detected) return;
    ahp_xc_select_input(index);
    int32_t idx = 0;
    int32_t test = 0;
    if(value+size >= ahp_xc_get_delaysize())
        return;
    idx = 0;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~TEST_STEP);
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    step >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    step >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
}

void ahp_xc_set_channel_auto(uint32_t index, off_t value, size_t size, size_t step)
{
    if(!ahp_xc_detected) return;
    ahp_xc_select_input(index);
    int32_t idx = 0;
    int32_t test = 0;
    if(value+size >= ahp_xc_get_delaysize())
        return;
    idx = 0;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~TEST_STEP);
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)|0x8));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)|0x8));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)|0x8));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)|0x8));
    step >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)|0x8));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)|0x8));
    step >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)|0x8));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)|0x8));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)|0x8));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)|0x8));
    size >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)|0x8));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)|0x8));
    size >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)|0x8));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)|0x8));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)|0x8));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)|0x8));
    value >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)|0x8));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)|0x8));
    value >>= 3;
    test++;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|(test<<5));
    idx = 0;
}

void ahp_xc_set_voltage(uint32_t index, unsigned char value)
{
    if(!ahp_xc_detected) return;
    ahp_xc_select_input(index);
    value = (unsigned char)(value < 0xff ? value : 0xff);
    int32_t idx = 0;
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

void ahp_xc_set_test_flags(uint32_t index, int32_t value)
{
    if(!ahp_xc_detected) return;
    ahp_xc_select_input(index);
    ahp_xc_test[index] = value;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_EXTRA_CMD);
    ahp_xc_send_command(ENABLE_TEST, (unsigned char)((ahp_xc_test[index]>>4)&0xf));
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
    ahp_xc_send_command(ENABLE_TEST, (unsigned char)(ahp_xc_test[index]&0xf));
}

 int32_t ahp_xc_send_command(xc_cmd c, unsigned char value)
{
    if(!ahp_xc_connected) return -ENOENT;
    int32_t ntries = 5;
    int32_t err = 0;
    ahp_serial_flushRX();
    while(ntries-- > 0)
        err |= ahp_serial_SendByte((unsigned char)(c|(((value<<4)|(value>>4))&0xf3)));
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
