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
#include <pthread.h>
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
static int ahp_xc_connected = 0;
static int ahp_xc_packetsize = 4096;
static baud_rate ahp_xc_rate = R_57600;
static char ahp_xc_comport[128];
static char ahp_xc_header[16] = { 0 };
static pthread_t readthread;
static int threads_running;
static int threads_stop;
#define NUM_THREADS 1
#define FIFO_SIZE 16
static pthread_mutex_t read_mutex[FIFO_SIZE];
static int threads_stopped = NUM_THREADS;
static char* read_buffer[FIFO_SIZE];
static int read_idx = 0;
static int write_idx = 0;
static int tail = 0;
static unsigned char* grab_next_valid_packet();
static int packet_error(unsigned char* packet);
static unsigned char ahp_xc_capture_flags = 0;
static void clear_tail()
{
    int err = 0;
    while(!err) {
        unsigned char *data = grab_next_valid_packet();
        err = packet_error(data);
        free(data);
    }
}

static int get_tail()
{
    tail = write_idx - read_idx;
    while(tail > FIFO_SIZE / 2)
        tail -= FIFO_SIZE;
    while(tail < -FIFO_SIZE / 2)
        tail += FIFO_SIZE;
    return tail;
}

static void* read_thread(void* arg)
{
    read_idx = 0;
    write_idx = 0;
    char err = 0;
    threads_stop = 0;
    threads_stopped--;
    int i;
    for(i = 0; i < FIFO_SIZE; i++) {
        read_buffer[i] = (unsigned char*)malloc(ahp_xc_packetsize);
        memset(read_buffer[i], 0, (unsigned int)ahp_xc_packetsize);
    }
    RS232_flushRX();
    RS232_AlignFrame('\r');
    fprintf(stdout, "%s: started\n", __func__);
    while(!threads_stop) {
        threads_running = 1;
        int size = ahp_xc_get_packetsize();
        pthread_mutex_lock(&read_mutex[write_idx]);
        memset(read_buffer[write_idx], 0, (unsigned int)size);
        ssize_t nread = RS232_PollComport(read_buffer[write_idx], size);
        if(nread < 0) {
            err = -ETIMEDOUT;
        } else {
            if(strlen(ahp_xc_get_header())==16) {
                off_t len = (off_t)(strchr(read_buffer[write_idx], '\r')-read_buffer[write_idx]);
                if(len < size-1) {
                    if(len>=16 && strncmp(ahp_xc_get_header(), read_buffer[write_idx], 16)) {
                        err = -EINVAL;
                    } else {
                        err = -EPIPE;
                    }
                    RS232_AlignFrame('\r');
                }
            }
            if(strlen(read_buffer[write_idx]) < size) {
                err = -ENODATA;
            }
        }
        if(err) {
            read_buffer[write_idx][0] = err;
            err = 0;
        }
        pthread_mutex_unlock(&read_mutex[write_idx]);
        write_idx++;
        write_idx %= FIFO_SIZE;
    }
    for(i = 0; i < FIFO_SIZE; i++) {
        free(read_buffer[i]);
    }
    fprintf(stdout, "%s: stopped\n", __func__);
    threads_running = 0;
    threads_stop = 1;
    threads_stopped++;
    return NULL;
}

static int packet_error(unsigned char* packet)
{
    if(packet == NULL)
        return -EOVERFLOW;
    char err = (char)packet[0];
    if(err < 0) {
        switch((int)packet[0]) {
        case -ETIMEDOUT:
        case -EINVAL:
        case -EPIPE:
        case -ENODATA:
            break;
        default:
            err = -EFAULT;
            break;
        }
        fprintf(stderr, "%s: %s\n", __func__, strerror(-err));
        return err;
    }
    return 0;
}

static unsigned char* grab_next_packet()
{
    if(get_tail() < 1) {
        usleep(ahp_xc_get_packettime());
        return NULL;
    }
    pthread_mutex_lock(&read_mutex[read_idx]);
    unsigned char *buf = (unsigned char*)malloc(ahp_xc_get_packetsize());
    memcpy(buf, read_buffer[read_idx], ahp_xc_get_packetsize());
    pthread_mutex_unlock(&read_mutex[read_idx]);
    read_idx++;
    read_idx %= FIFO_SIZE;
    return buf;
}

static unsigned char* grab_next_valid_packet()
{
    unsigned char *buf;
    int err = 0;
    int max_errored = FIFO_SIZE;
    while (err != -ETIMEDOUT && max_errored-- > 0) {
        buf = grab_next_packet();
        err = packet_error(buf);
        if(!err)
            break;
    }
    if(err || max_errored < 0)
        return NULL;
    return buf;
}

static unsigned char* grab_last_packet()
{
    read_idx = write_idx;
    usleep(ahp_xc_get_packettime()*2);
    return grab_next_valid_packet();
}

static void run_stop_threads(int start)
{
    if(!start&&threads_running) {
        fprintf(stdout, "%s: requesting stop", __func__);
        threads_stop = 1;
        while(threads_running) {
            fprintf(stdout, ".");
            usleep(1000);
        }
        fprintf(stdout, "\n");
        clear_tail();
        fprintf(stdout, "%s: threads stopped\n", __func__);
        RS232_flushRX();
    } else if(start&&!threads_running) {
        fprintf(stdout, "%s: requesting start\n", __func__);
        if(start) {
            pthread_create(&readthread, NULL, &read_thread, NULL);
        }
    }
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

char* ahp_xc_get_header()
{
    return ahp_xc_header;
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
    return ahp_xc_jittersize;
}

int ahp_xc_get_crosscorrelator_jittersize()
{
    return ahp_xc_jittersize;
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

int ahp_xc_connect_fd(int fd)
{
    ahp_xc_bps = -1;
    ahp_xc_nlines = -1;
    ahp_xc_nbaselines = -1;
    ahp_xc_delaysize = -1;
    ahp_xc_frequency = -1;
    ahp_xc_packetsize = 16;
    ahp_xc_rate = R_57600;
    if(fd > -1) {
        int i;
        for(i = 0; i < FIFO_SIZE; i++)
            pthread_mutex_init(&read_mutex[i], NULL);
        ahp_xc_connected = 1;
        RS232_SetFD(fd);
        return -1;
    }
    return 0;
}

int ahp_xc_connect(const char *port)
{
    if(ahp_xc_connected)
        return 1;
    ahp_xc_header[0] = 0;
    int ret = 1;
    ahp_xc_bps = -1;
    ahp_xc_nlines = -1;
    ahp_xc_nbaselines = -1;
    ahp_xc_delaysize = -1;
    ahp_xc_frequency = -1;
    ahp_xc_packetsize = 16;
    ahp_xc_rate = R_57600;
    strncpy(ahp_xc_comport, port, strlen(port));
    if(!RS232_OpenComport(ahp_xc_comport))
        ret = RS232_SetupPort(XC_BASE_RATE, "8N2", 0);
    if(!ret) {
        int i;
        for(i = 0; i < FIFO_SIZE; i++)
            pthread_mutex_init(&read_mutex[i], NULL);
        ahp_xc_connected = 1;
    }
    return ret;
}

void ahp_xc_disconnect()
{
    if(ahp_xc_connected) {
        ahp_xc_connected = 0;
        int i;
        for(i = 0; i < FIFO_SIZE; i++)
            pthread_mutex_destroy(&read_mutex[i]);
        ahp_xc_set_baudrate(R_57600);
        RS232_CloseComport();
    }
}

int ahp_xc_is_connected()
{
    return ahp_xc_connected;
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

void ahp_xc_start_crosscorrelation_scan(int index, int start)
{
    ahp_xc_set_capture_flag(CAP_ENABLE);
    ahp_xc_clear_capture_flag(CAP_ENABLE);
    ahp_xc_set_lag_cross(index, start-1<0?0:start-1);
    ahp_xc_set_test(index, SCAN_CROSS);
}

void ahp_xc_end_crosscorrelation_scan(int index)
{
    ahp_xc_clear_test(index, SCAN_CROSS);
    grab_last_packet();
    ahp_xc_clear_capture_flag(CAP_ENABLE);
}

int ahp_xc_scan_crosscorrelations(int index1, int index2, ahp_xc_sample **crosscorrelations, unsigned int start1, unsigned int start2, unsigned int size, int *interrupt, double *percent)
{
    int r = -1, x, y;
    int n = ahp_xc_get_bps()/4;
    *crosscorrelations = NULL;
    ahp_xc_sample *correlations = ahp_xc_alloc_samples((unsigned int)ahp_xc_get_delaysize(), (unsigned int)ahp_xc_get_autocorrelator_jittersize());
    char* sample = (char*)malloc((unsigned int)n+1);
    sample[n] = 0;
    (*percent) = 0;
    r++;
    start1 = (start1 > ahp_xc_get_delaysize()-2 ? start1 : ahp_xc_get_delaysize()-2);
    start2 = (start2 > ahp_xc_get_delaysize()-2 ? start2 : ahp_xc_get_delaysize()-2);
    size = (size < 5 ? 5 : size);
    ahp_xc_start_crosscorrelation_scan(index1, start1);
    ahp_xc_set_lag_cross(index2, (int)start2);
    ahp_xc_set_lag_auto(index1, 0);
    ahp_xc_set_lag_auto(index2, 0);
    int i = size/2;
    while(start1 < start1+size/2) {
        if((*interrupt) == 1 || i < 0)
            break;
        unsigned char* packet = grab_next_valid_packet();
        if(!packet)
            break;
        packet += 16;
        for(y = 0; y < ahp_xc_get_crosscorrelator_jittersize()*2-1; y++) {
            memcpy(sample, &packet[n*index1], (unsigned int)n);
            correlations[i].correlations[y].counts = strtoul(sample, NULL, 16);
            memcpy(sample, &packet[n*index2], (unsigned int)n);
            correlations[i].correlations[y].counts += strtoul(sample, NULL, 16);
            correlations[i].correlations[y].counts /= 2;
            packet += (ahp_xc_get_nlines()+ahp_xc_get_autocorrelator_jittersize()*ahp_xc_get_nlines())*n;
            int idx1 = (index1 < index2 ? index1 : index2);
            int idx2 = (index1 > index2 ? index1 : index2);
            packet += n*((idx1*(ahp_xc_get_nlines()*2-idx1-1))/2+idx2-idx1-1);
            memcpy(sample, packet, (unsigned int)n);
            correlations[i].correlations[y].correlations = strtoul(sample, NULL, 16);
            correlations[i].correlations[y].coherence = (double)correlations[i].correlations[y].correlations / (double)correlations[i].correlations[y].counts;
            (*percent) += 50.0 / ahp_xc_get_delaysize() / ahp_xc_get_autocorrelator_jittersize();
        }
        i--;
        start1++;
        r++;
        free(packet);
    }
    ahp_xc_start_crosscorrelation_scan(index2, start2);
    ahp_xc_set_lag_cross(index1, (int)start1);
    ahp_xc_set_lag_auto(index1, 0);
    ahp_xc_set_lag_auto(index2, 0);
    i = size/2;
    while(start2 < start2+size/2) {
        if((*interrupt) == 1 || i < 0)
            break;
        unsigned char* packet = grab_next_valid_packet();
        if(!packet)
            break;
        packet += 16;
        for(y = 0; y < ahp_xc_get_crosscorrelator_jittersize()*2-1; y++) {
            memcpy(sample, &packet[n*index1], (unsigned int)n);
            correlations[i].correlations[y].counts = strtoul(sample, NULL, 16);
            memcpy(sample, &packet[n*index2], (unsigned int)n);
            correlations[i].correlations[y].counts += strtoul(sample, NULL, 16);
            correlations[i].correlations[y].counts /= 2;
            packet += (ahp_xc_get_nlines()+ahp_xc_get_autocorrelator_jittersize()*ahp_xc_get_nlines())*n;
            int idx1 = (index1 < index2 ? index1 : index2);
            int idx2 = (index1 > index2 ? index1 : index2);
            packet += n*((idx1*(ahp_xc_get_nlines()*2-idx1-1))/2+idx2-idx1-1);
            memcpy(sample, packet, (unsigned int)n);
            correlations[i].correlations[y].correlations = strtoul(sample, NULL, 16);
            correlations[i].correlations[y].coherence = (double)correlations[i].correlations[y].correlations / (double)correlations[i].correlations[y].counts;
            (*percent) += 50.0 / ahp_xc_get_delaysize() / ahp_xc_get_autocorrelator_jittersize();
        }
        i++;
        start1++;
        r++;
        free(packet);
    }
cross_fail:
    ahp_xc_clear_capture_flag(CAP_ENABLE);
    ahp_xc_clear_test(index1, SCAN_CROSS);
    ahp_xc_clear_test(index2, SCAN_CROSS);
    free(sample);
    *crosscorrelations = correlations;
    return r;
}

void ahp_xc_start_autocorrelation_scan(int index, int start)
{
    ahp_xc_clear_capture_flag(CAP_ENABLE);
    ahp_xc_set_capture_flag(CAP_ENABLE);
    ahp_xc_set_lag_auto(index, start-1<0?0:start-1);
    ahp_xc_set_test(index, SCAN_AUTO);
}

void ahp_xc_end_autocorrelation_scan(int index)
{
    ahp_xc_clear_test(index, SCAN_AUTO);
    grab_last_packet();
    ahp_xc_clear_capture_flag(CAP_ENABLE);
}

int ahp_xc_scan_autocorrelations(int index, ahp_xc_sample **autocorrelations, int start, unsigned int len, int *interrupt, double *percent)
{
    int r = -1, x, y;
    int n = ahp_xc_get_bps()/4;
    *autocorrelations = NULL;
    ahp_xc_sample *correlations = ahp_xc_alloc_samples((unsigned int)ahp_xc_get_delaysize(), (unsigned int)ahp_xc_get_autocorrelator_jittersize());
    char* sample = (char*)malloc((unsigned int)n+1);
    sample[n] = 0;
    (*percent) = 0;
    r++;
    start = (start < ahp_xc_get_delaysize()-2 ? start : ahp_xc_get_delaysize()-2);
    int end = (start+len < ahp_xc_get_delaysize() ? start+len : ahp_xc_get_delaysize()-1);
    ahp_xc_start_autocorrelation_scan(index, start);
    ahp_xc_set_lag_cross(index, 0);
    while(start < end) {
        if((*interrupt) || start >= end)
            break;
        unsigned char* data = grab_next_valid_packet();
        if(!data)
            break;
        unsigned char *packet = data;
        for(y = 0; y < ahp_xc_get_autocorrelator_jittersize(); y++) {
            if((*interrupt) || start >= end)
                break;
            packet += 16;
            memcpy(sample, &packet[n*index], (unsigned int)n);
            correlations[start].correlations[y].counts = strtoul(sample, NULL, 16)|1;
            memcpy(sample, &packet[(ahp_xc_get_nlines()+ahp_xc_get_autocorrelator_jittersize()*index)*n], (unsigned int)n);
            correlations[start].correlations[y].correlations = strtoul(sample, NULL, 16);
            correlations[start].correlations[y].coherence = (double)correlations[start].correlations[y].correlations / (double)correlations[start].correlations[y].counts;
        }
        (*percent) += 100.0 / len;
        start++;
        r++;
        free(data);
    }
    ahp_xc_end_autocorrelation_scan(index);
    free(sample);
    *autocorrelations = correlations;
    return r;
}

int ahp_xc_get_packet(ahp_xc_packet *packet)
{
    int ret = 1;
    int x = 0, y = 0, z = 0;
    int n = ahp_xc_get_bps()/4;
    char *sample = (char*)malloc((unsigned int)n+1);
    if(packet == NULL) {
        return -EINVAL;
        goto err_end;
    }
    unsigned char* data = grab_last_packet();
    if(!data){
        goto err_end;
    }
    unsigned char *buf = data;
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
        for(y = 0; y < ahp_xc_get_autocorrelator_jittersize(); y++) {
            sample[n] = 0;
            memcpy(sample, buf, (unsigned int)n);
            if(1<sscanf(sample, "%lX",  &packet->autocorrelations[x].correlations[y].correlations)) {
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
                if(1<sscanf(sample, "%lX",  &packet->crosscorrelations[x].correlations[z].correlations)) {
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
    free(data);
    ret = 0;
    goto end;
err_end:
end:
    free(sample);
    return ret;
}

int ahp_xc_get_properties()
{
    unsigned char *data = NULL;
    ssize_t n_read;
    int ntries = 4096;
    int i;
    ahp_xc_clear_capture_flag(CAP_ENABLE);
    ahp_xc_set_capture_flag(CAP_ENABLE);
    while(ntries-- > 0) {
        data = grab_next_valid_packet();
        if(data)
            break;
    }
    ahp_xc_clear_capture_flag(CAP_ENABLE);
    if(ntries < 0 || data == NULL)
        return -EBUSY;
    int _bps, _nlines, _delaysize, _jittersize, _flags, _tau;
    n_read = sscanf((char*)data, "%02X%02X%03X%04X%01X%04X", &_bps, &_nlines, &_delaysize, &_jittersize, &_flags, &_tau);
    if(n_read != 6)
        return -EINVAL;
    strncpy(ahp_xc_header, data, 16);
    free(data);
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

int ahp_xc_set_capture_flag(xc_capture_flags flag)
{
    ahp_xc_capture_flags |= (1 << flag);
    int ret = ahp_xc_send_command(ENABLE_CAPTURE, (unsigned char)ahp_xc_capture_flags);
    if(flag == CAP_ENABLE)
        run_stop_threads(1);
    return ret;
}

int ahp_xc_clear_capture_flag(xc_capture_flags flag)
{
    ahp_xc_capture_flags &= ~(1 << flag);
    int ret = ahp_xc_send_command(ENABLE_CAPTURE, (unsigned char)ahp_xc_capture_flags);
    if(flag == CAP_ENABLE)
        run_stop_threads(0);
    return ret;
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
    ahp_xc_send_command(SET_DELAY, (idx<<4)|(value&0x7));
    idx++;
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (idx<<4)|(value&0x7));
    idx++;
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (idx<<4)|(value&0x7));
    idx++;
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (idx<<4)|(value&0x7));
    grab_last_packet();
}

void ahp_xc_set_lag_auto(int index, int value)
{
    ahp_xc_select_input(index);
    int idx = 0;
    ahp_xc_send_command(SET_DELAY, (idx++<<4)|0x8|(value&0x7));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (idx++<<4)|0x8|(value&0x7));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (idx++<<4)|0x8|(value&0x7));
    value >>= 3;
    ahp_xc_send_command(SET_DELAY, (idx++<<4)|0x8|(value&0x7));
    grab_last_packet();
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
    value = (unsigned char)(value < 0xff ? value : 0xff);
    int idx = 0;
    ahp_xc_send_command((xc_cmd)(SET_VOLTAGE), (idx++<<2)|(value&0x3));
    value >>= 2;
    ahp_xc_send_command((xc_cmd)(SET_VOLTAGE), (idx++<<2)|(value&0x3));
    value >>= 2;
    ahp_xc_send_command((xc_cmd)(SET_VOLTAGE), (idx++<<2)|(value&0x3));
    value >>= 2;
    ahp_xc_send_command((xc_cmd)(SET_VOLTAGE), (idx++<<2)|(value&0x3));
    value >>= 2;
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
    RS232_flushTX();
    return RS232_SendByte((unsigned char)(c|(((value<<4)|(value>>4))&0xf3)));
}
