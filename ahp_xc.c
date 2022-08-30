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
#include "rs232.c"
#include "ahp_xc.h"

#ifndef AIRY
#define AIRY 1.21966
#endif
#ifndef EULER
#define EULER 2.71828182845904523536028747135266249775724709369995
#endif
static int xc_current_input = 0;
static long sign = 1;
static long fill = 0;

typedef struct  {
    ahp_xc_sample *sample;
    int index;
    int index1;
    int index2;
    const char *data;
    double lag;
} thread_argument;

static unsigned long int nthreads = 0;
thread_argument *autocorrelation_thread_args = NULL;
thread_argument *crosscorrelation_thread_args = NULL;
static pthread_t *autocorrelation_threads = NULL;
static pthread_t *crosscorrelation_threads = NULL;
static pthread_mutex_t ahp_xc_mutex;
static int ahp_xc_mutexes_initialized = 0;

static int AHP_XC_MAX_THREADS = 1;
static unsigned char *ahp_xc_test = NULL;
static unsigned char *ahp_xc_leds = NULL;
static unsigned int ahp_xc_bps = 0;
static unsigned int ahp_xc_nlines = 0;
static unsigned int ahp_xc_nbaselines = 0;
static unsigned int ahp_xc_auto_lagsize = 0;
static unsigned int ahp_xc_cross_lagsize = 0;
static unsigned int ahp_xc_delaysize = 0;
static unsigned int ahp_xc_flags = 0;
static unsigned int ahp_xc_correlator_enabled = 1;
static unsigned int ahp_xc_intensity_correlator_enabled = 0;
static unsigned int ahp_xc_frequency = 1;
static unsigned int ahp_xc_voltage = 0;
static unsigned int ahp_xc_connected = 0;
static unsigned int ahp_xc_detected = 0;
static unsigned int ahp_xc_packetsize = 17;
static int ahp_xc_baserate = XC_BASE_RATE;
static baud_rate ahp_xc_rate = R_BASE;
static unsigned int ahp_xc_correlation_order = 1;
static char ahp_xc_comport[128];
static char ahp_xc_header[17] = { 0 };
static unsigned char ahp_xc_capture_flags = 0;
static unsigned char ahp_xc_max_lost_packets = 1;

unsigned long int ahp_xc_max_threads(unsigned long value)
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
            phase = -phase;
    }
    phase += M_PI;
    sample->magnitude = magnitude;
    sample->phase = phase;
}

double get_timestamp(char *data)
{
    char timestamp[16];
    strncpy(timestamp, &data[ahp_xc_get_packetsize()-19], 16);
    return (double)strtoul(timestamp, NULL, 16) / 1000000000.0;
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
        return EINVAL;
    }
    return 0;
}

static char * grab_packet()
{
    errno = 0;
    char *buf = (char*)malloc(ahp_xc_get_packetsize());
    memset(buf, 0, ahp_xc_get_packetsize());
    if(!ahp_xc_connected){
        errno = ENOENT;
        goto err_end;
    }
    unsigned int size = ahp_xc_get_packetsize();
    memset(buf, 0, (unsigned int)size);
    int nread = ahp_serial_RecvBuf((unsigned char*)buf, (int)size);
    if(nread < 0) {
        errno = ETIMEDOUT;
        goto err_end;
    } else {
        if(size > 17) {
            off_t len = (off_t)(strchr((char*)buf, '\r')-(char*)buf);
            if(len < size-1) {
                if(strncmp(ahp_xc_get_header(), (char*)buf, 16)) {
                    errno = EINVAL;
                } else {
                    errno = EPIPE;
                }
                ahp_serial_AlignFrame('\r', (int)size);
                goto err_end;
            }
        } else {
            if(buf[0] == '\r') {
                memcpy(buf, buf+1, 16);
                buf[16] = 13;
            } else {
                errno = EINVAL;
                goto err_end;
            }
        }
    }
    if(strlen((char*)buf) < size)
        errno = ENODATA;
    if(errno)
        goto err_end;
    if(size > 17)
        errno = calc_checksum((char*)buf);
    if(errno)
        goto err_end;
    return buf;
err_end:
    free(buf);
    return NULL;
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

void ahp_xc_enable_crosscorrelator(int enable)
{
    if(!ahp_xc_detected) return;
    ahp_xc_correlator_enabled = enable;
}

void ahp_xc_enable_intensity_crosscorrelator(int enable)
{
    if(!ahp_xc_detected) return;
    ahp_xc_intensity_correlator_enabled = enable;
}

int ahp_xc_intensity_crosscorrelator_enabled()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_intensity_correlator_enabled != 0 || !ahp_xc_has_crosscorrelator();
}

int ahp_xc_has_crosscorrelator()
{
    if(!ahp_xc_detected) return 0;
    return (ahp_xc_flags & HAS_CROSSCORRELATOR ? ahp_xc_correlator_enabled : 0);
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
    if(ahp_xc_intensity_crosscorrelator_enabled())
        return 0;
    return ahp_xc_nbaselines;
}

unsigned int ahp_xc_get_delaysize()
{
    if(!ahp_xc_detected) return 0;
    if(ahp_xc_delaysize == 0 || ahp_xc_delaysize == 4)
        return 1<<20;
    return ahp_xc_delaysize << 3;
}

unsigned int ahp_xc_get_autocorrelator_lagsize()
{
    if(!ahp_xc_detected) return 0;
    return ahp_xc_auto_lagsize;
}

unsigned int ahp_xc_get_crosscorrelator_lagsize()
{
    if(!ahp_xc_detected) return 0;
    if(ahp_xc_has_crosscorrelator())
        return ahp_xc_cross_lagsize;
    else
        return ahp_xc_auto_lagsize;
}

unsigned int ahp_xc_get_frequency()
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

unsigned int ahp_xc_get_packetsize()
{
    return ahp_xc_packetsize;
}

int ahp_xc_get_fd()
{
    return ahp_serial_GetFD();
}

int ahp_xc_connect_fd(int fd)
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
    ahp_xc_packetsize = 17;
    ahp_xc_baserate = (high_rate ? XC_HIGH_RATE : XC_BASE_RATE);
    ahp_xc_rate = R_BASE;
    strcpy(ahp_xc_comport, port);
    if(!ahp_serial_OpenComport(ahp_xc_comport))
        ret = ahp_serial_SetupPort(ahp_xc_baserate, "8N1", 0);
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
    packet->tau = (unsigned long)(1.0/ahp_xc_get_frequency());
    packet->n_lines = (unsigned long)ahp_xc_get_nlines();
    packet->n_baselines = (unsigned long)ahp_xc_get_nbaselines();
    packet->counts = (unsigned long*)malloc((unsigned long)ahp_xc_get_nlines() * sizeof(unsigned long));
    packet->autocorrelations = ahp_xc_alloc_samples((unsigned long)ahp_xc_get_nlines(), (unsigned long)ahp_xc_get_autocorrelator_lagsize());
    packet->crosscorrelations = ahp_xc_alloc_samples((unsigned long)ahp_xc_get_nbaselines(), (unsigned long)ahp_xc_get_crosscorrelator_lagsize()*2-1);
    return packet;
}

ahp_xc_packet *ahp_xc_copy_packet(ahp_xc_packet *packet)
{
    ahp_xc_packet *copy = ahp_xc_alloc_packet();
    memcpy(copy->counts, packet->counts, sizeof(unsigned long) * (unsigned long)ahp_xc_get_nlines());
    memcpy(copy->autocorrelations, packet->autocorrelations, sizeof(ahp_xc_sample) * (unsigned long)ahp_xc_get_nlines() * (unsigned long)ahp_xc_get_autocorrelator_lagsize());
    memcpy(copy->crosscorrelations, packet->crosscorrelations, sizeof(ahp_xc_sample) * (unsigned long)ahp_xc_get_nbaselines() * (unsigned long)ahp_xc_get_crosscorrelator_lagsize());
    return copy;
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

void ahp_xc_start_autocorrelation_scan(unsigned int index, off_t start, size_t size, size_t step)
{
    if(!ahp_xc_detected) return;
    ahp_xc_end_autocorrelation_scan(index);
    ahp_xc_set_capture_flags((ahp_xc_get_capture_flags()|CAP_RESET_TIMESTAMP)&~CAP_ENABLE);
    ahp_xc_set_channel_auto(index, start, size, step);
    usleep(ahp_xc_get_packettime()*1000000);
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|SCAN_AUTO);
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_ENABLE);
}

void ahp_xc_end_autocorrelation_scan(unsigned int index)
{
    if(!ahp_xc_detected) return;
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~SCAN_AUTO);
    ahp_xc_set_channel_auto(index, 0, 1, 0);
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~(CAP_ENABLE|CAP_RESET_TIMESTAMP));
}

static void* _get_autocorrelation(void *o)
{
    thread_argument *arg = (thread_argument*)o;
    ahp_xc_sample *sample = arg->sample;
    int index = arg->index;
    const char *data = arg->data;
    double lag = arg->lag;
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
    if(nthreads > 0)
        nthreads--;
    return NULL;
}

void ahp_xc_get_autocorrelation(ahp_xc_sample *sample, int index, const char *data, double lag)
{
    if(!ahp_xc_mutexes_initialized)
        return;
    autocorrelation_thread_args[index].sample = sample;
    autocorrelation_thread_args[index].index = index;
    autocorrelation_thread_args[index].data = data;
    autocorrelation_thread_args[index].lag = lag;
    _get_autocorrelation(&autocorrelation_thread_args[index]);
}

int ahp_xc_scan_autocorrelations(unsigned int nlines, unsigned int *indexes, ahp_xc_sample **autocorrelations, off_t *starts, size_t *sizes, size_t *steps, int *interrupt, double *percent)
{
    if(!ahp_xc_detected) return 0;
    int r = -1;
    unsigned int n = ahp_xc_get_bps()/4;
    unsigned int i = 0;
    unsigned int x = 0;
    *autocorrelations = NULL;
    char* sample = (char*)malloc((unsigned int)n+1);
    sample[n] = 0;
    (*percent) = 0;
    r++;
    size_t len = 0;
    for(i = 0; i < nlines; i++) {
        starts[i] = (starts[i] < ahp_xc_get_delaysize()-2 ? starts[i] : (off_t)ahp_xc_get_delaysize()-2);
        sizes[i] = (starts[i]+(off_t)sizes[i] < ahp_xc_get_delaysize() ? (off_t)sizes[i] : (off_t)ahp_xc_get_delaysize()-1-starts[i]);
        len = fmax(len, sizes[i]/steps[i]);
    }
    ahp_xc_sample *correlations = ahp_xc_alloc_samples(len*nlines, (unsigned int)ahp_xc_get_autocorrelator_lagsize());
    char* data = (char*)malloc(ahp_xc_get_packetsize()*len);
    for(i = 0; i < nlines; i++)
        ahp_xc_start_autocorrelation_scan(indexes[i], starts[i], sizes[i], steps[i]);
    i = 0;
    while(i < len) {
        if(*interrupt)
            break;
        usleep(ahp_xc_get_packettime()*1000000);
        char* buf = grab_packet();
        if(!buf)
            continue;
        memcpy(data+i*ahp_xc_get_packetsize(), buf, ahp_xc_get_packetsize());
        i++;
        free(buf);
        (*percent) += 100.0 / len;
        r++;
    }
    for(i = 0; i < nlines; i++)
        ahp_xc_end_autocorrelation_scan(indexes[i]);
    i = 0;
    double ts = 0.0;
    double ts0 = 0.0;
    int s = 0;
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
                ahp_xc_get_autocorrelation(&correlations[i+off], indexes[x], packet, ts);
                s++;
            }
            off += sizes[x]/steps[x];
        }
        wait_no_threads();
        i++;
    }
    free(data);
    free(sample);
    *autocorrelations = correlations;
    return s;
}

void ahp_xc_start_crosscorrelation_scan(unsigned int index, off_t start, size_t size, size_t step)
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

void ahp_xc_end_crosscorrelation_scan(unsigned int index)
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
    int index1 = arg->index1;
    int index2 = arg->index2;
    const char *data = arg->data;
    double lag = arg->lag;
    unsigned int y;
    int n = ahp_xc_get_bps() / 4;
    const char *packet = data;
    sample->lag_size = (ahp_xc_get_crosscorrelator_lagsize()*2-1);
    sample->lag = lag;
    if(ahp_xc_intensity_crosscorrelator_enabled()) {
        ahp_xc_sample *sample1 = ahp_xc_alloc_samples(1, ahp_xc_get_autocorrelator_lagsize());
        ahp_xc_sample *sample2 = ahp_xc_alloc_samples(1, ahp_xc_get_autocorrelator_lagsize());
        ahp_xc_get_autocorrelation(sample1, index1, packet, lag);
        ahp_xc_get_autocorrelation(sample2, index2, packet, lag);
        wait_no_threads();
        int z = ahp_xc_get_autocorrelator_lagsize()-1;
        for (y = 0; y < ahp_xc_get_autocorrelator_lagsize(); y++) {
            sample->correlations[z].counts = sample1->correlations[y].counts + sample2->correlations[0].counts;
            sample->correlations[z].lag = sample->lag+z*ahp_xc_get_sampletime();
            sample->correlations[z].magnitude = pow(sample1->correlations[y].magnitude * sample2->correlations[0].magnitude, 0.5);
            sample->correlations[z].phase = pow(sample1->correlations[y].phase * sample2->correlations[0].phase, 0.5);
            sample->correlations[z].real = sin(sample->correlations[y].phase) * sample->correlations[y].magnitude;
            sample->correlations[z].imaginary = cos(sample->correlations[y].phase) * sample->correlations[y].magnitude;
            z--;
        }
        z = ahp_xc_get_autocorrelator_lagsize()-1;
        for (y = 0; y < ahp_xc_get_autocorrelator_lagsize(); y++) {
            sample->correlations[z].counts = sample1->correlations[0].counts + sample2->correlations[y].counts;
            sample->correlations[z].lag = sample->lag+z*ahp_xc_get_sampletime();
            sample->correlations[z].magnitude = pow(sample1->correlations[0].magnitude * sample2->correlations[y].magnitude, 0.5);
            sample->correlations[z].phase = pow(sample1->correlations[0].phase * sample2->correlations[y].phase, 0.5);
            sample->correlations[z].real = sin(sample->correlations[y].phase) * sample->correlations[y].magnitude;
            sample->correlations[z].imaginary = cos(sample->correlations[y].phase) * sample->correlations[y].magnitude;
            z++;
        }
        ahp_xc_free_samples(1, sample1);
        ahp_xc_free_samples(1, sample2);
    } else {
        char *subpacket = (char*)malloc(n+1);
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

void ahp_xc_get_crosscorrelation(ahp_xc_sample *sample, int index1, int index2, const char *data, double lag)
{
    if(!ahp_xc_mutexes_initialized)
        return;
    int idx = index1*(ahp_xc_get_nlines()-index1)+index2-index1-1;
    crosscorrelation_thread_args[idx].sample = sample;
    crosscorrelation_thread_args[idx].index1 = index1;
    crosscorrelation_thread_args[idx].index2 = index2;
    crosscorrelation_thread_args[idx].data = data;
    crosscorrelation_thread_args[idx].lag = lag;
    _get_crosscorrelation(&crosscorrelation_thread_args[idx]);
}

int ahp_xc_scan_crosscorrelations(unsigned int index1, unsigned int index2, ahp_xc_sample **crosscorrelations, off_t head_start, size_t head_size, off_t tail_start, size_t tail_size, size_t step, int *interrupt, double *percent)
{
    if(!ahp_xc_detected) return 0;
    size_t k = 0;
    int i = 0;
    double ts = 0.0;
    double ts0 = 0.0;
    unsigned int n = ahp_xc_get_bps()/4;
    *crosscorrelations = NULL;
    unsigned int idx1 = (index1 < index2 ? index1 : index2);
    unsigned int idx2 = (index1 > index2 ? index1 : index2);
    int size = (head_size+tail_size)/step;
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
    ahp_xc_end_crosscorrelation_scan(idx2);
    ahp_xc_set_channel_cross(idx2, tail_start, 1, 0);
    ahp_xc_start_crosscorrelation_scan(idx1, head_start, head_size, step);
    if(ahp_xc_intensity_crosscorrelator_enabled())
        ahp_xc_start_crosscorrelation_scan(idx2, tail_start, tail_size, step);
    i = 0;
    while(i < (int)(head_size/step)) {
        if(*interrupt)
            break;
        usleep(ahp_xc_get_packettime()*1000000);
        char* buf = grab_packet();
        if(!buf)
            continue;
        memcpy(head+i*ahp_xc_get_packetsize(), buf, ahp_xc_get_packetsize());
        free(buf);
        (*percent) += 100.0 / size;
        i++;
    }
    ahp_xc_end_crosscorrelation_scan(idx1);
    ahp_xc_set_channel_cross(idx1, head_start, 1, 0);
    ahp_xc_start_crosscorrelation_scan(idx2, tail_start, tail_size, step);
    if(ahp_xc_intensity_crosscorrelator_enabled())
        ahp_xc_start_crosscorrelation_scan(idx1, head_start, head_size, step);
    i = 0;
    while(i < (int)(tail_size/step)) {
        if(*interrupt)
            break;
        usleep(ahp_xc_get_packettime()*1000000);
        char* buf = grab_packet();
        if(!buf)
            continue;
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
        ahp_xc_get_crosscorrelation(&correlations[i], idx1, idx2, packet, -ts);
        wait_no_threads();
        i++;
        k++;
    }
    free(head);
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
        ahp_xc_get_crosscorrelation(&correlations[i], idx1, idx2, packet, ts);
        wait_no_threads();
        i++;
        k++;
    }
    free(tail);
    free(sample);
    *crosscorrelations = correlations;
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
            goto err_end;
        }
        packet->counts[x] = (packet->counts[x] == 0 ? 1 : packet->counts[x]);
        buf += n;
    }
    int idx = 0;
    for(x = 0; x < ahp_xc_get_nlines()-ahp_xc_correlation_order; x++) {
        ahp_xc_get_autocorrelation(&packet->autocorrelations[x], x, data, 0.0);
        for(y = x+ahp_xc_correlation_order; y < ahp_xc_get_nlines(); y++) {
            ahp_xc_get_crosscorrelation(&packet->crosscorrelations[idx++], x, y, data, 0.0);
        }
    }
    wait_no_threads();
    packet->timestamp = get_timestamp(data);
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

    if(ahp_xc_mutexes_initialized) {
        if(crosscorrelation_threads)
            crosscorrelation_threads = (pthread_t*)realloc(crosscorrelation_threads, sizeof(pthread_t)*ahp_xc_nbaselines);
        else
            crosscorrelation_threads = (pthread_t*)malloc(sizeof(pthread_t)*ahp_xc_nbaselines);
        memset(crosscorrelation_threads, 0, sizeof(pthread_t)*ahp_xc_nbaselines);
        if(crosscorrelation_thread_args)
            crosscorrelation_thread_args = (thread_argument*)realloc(crosscorrelation_thread_args, sizeof(thread_argument)*ahp_xc_nbaselines);
        else
            crosscorrelation_thread_args = (thread_argument*)malloc(sizeof(thread_argument)*ahp_xc_nbaselines);
        memset(crosscorrelation_thread_args, 0, sizeof(thread_argument)*ahp_xc_nbaselines);
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

int ahp_xc_set_capture_flags(xc_capture_flags flags)
{
    if(!ahp_xc_connected) return -ENOENT;
    ahp_xc_max_lost_packets = 1;
    ahp_xc_capture_flags = flags;
    ahp_serial_flushRX();
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
    ahp_serial_SetupPort(ahp_xc_baserate<<((int)ahp_xc_rate), "8N2", 0);
}

void ahp_xc_set_correlation_order(unsigned int order)
{
    if(!ahp_xc_detected) return;
    if(order < 2) return;
    if(order > ahp_xc_get_nlines()) return;
    ahp_xc_correlation_order = order - 2;
    xc_capture_flags old_flags = ahp_xc_get_capture_flags();
    ahp_xc_set_capture_flags(old_flags|CAP_EXTRA_CMD);
    ahp_xc_send_command(SET_BAUD_RATE, (unsigned char)ahp_xc_correlation_order);
    ahp_xc_set_capture_flags(old_flags);
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

void ahp_xc_set_channel_cross(unsigned int index, off_t value, size_t size, size_t step)
{
    if(!ahp_xc_detected) return;
    ahp_xc_select_input(index);
    int idx = 0;
    if(value+size >= ahp_xc_get_delaysize())
        return;
    idx = 0;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_EXTRA_CMD);
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|TEST_STEP);
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)));
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~TEST_STEP);
    idx = 0;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)));
    size >>= 3;
    if(ahp_xc_delaysize > 4)
        size = round((double)size * 2.0 / ahp_xc_delaysize) + 1;
    ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)(0|(size&0x3)));
    size >>= 2;
    ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)(4|(size&0x3)));
    idx = 0;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)));
    value >>= 3;
    if(ahp_xc_delaysize > 4)
        value = round((double)value * 2.0 / ahp_xc_delaysize);
    ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)(0|(value&0x3)));
    value >>= 2;
    ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)(4|(value&0x3)));
}

void ahp_xc_set_channel_auto(unsigned int index, off_t value, size_t size, size_t step)
{
    if(!ahp_xc_detected) return;
    ahp_xc_select_input(index);
    int idx = 0;
    if(value+size >= ahp_xc_get_delaysize())
        return;
    idx = 3;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()|CAP_EXTRA_CMD);
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)|TEST_STEP);
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)|0x8));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)|0x8));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)|0x8));
    step >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(step&0x7)|0x8));
    ahp_xc_set_test_flags(index, ahp_xc_get_test_flags(index)&~TEST_STEP);
    idx = 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)|0x8));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)|0x8));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)|0x8));
    size >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(size&0x7)|0x8));
    size >>= 3;
    if(ahp_xc_delaysize > 4)
        size = round((double)size * 2.0 / ahp_xc_delaysize) + 1;
    ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)(0|(size&0x3)|0x8));
    size >>= 2;
    ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)(4|(size&0x3)|0x8));
    idx = 3;
    ahp_xc_set_capture_flags(ahp_xc_get_capture_flags()&~CAP_EXTRA_CMD);
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)|0x8));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)|0x8));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)|0x8));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (unsigned char)((idx++<<4)|(value&0x7)|0x8));
    value >>= 3;
    if(ahp_xc_delaysize > 4)
        value = round((double)value * 2.0 / ahp_xc_delaysize);
    ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)(0|(value&0x3)|0x8));
    value >>= 2;
    ahp_xc_send_command(SET_FREQ_DIV, (unsigned char)(4|(value&0x3)|0x8));
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

void ahp_xc_set_test_flags(unsigned int index, int value)
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
    int ntries = 5;
    int err = 0;
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
