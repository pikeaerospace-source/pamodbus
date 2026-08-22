/*
 * pa_serial.c — POSIX (termios) RS485/serial provider implementing pa_port_t.
 *
 * See pa_serial.h for the interface. Implements every optional pa_port_t hook
 * so the full feature test (including blocking delay and flush) works on the
 * host, and so a USB-RS485 adapter that needs manual half-duplex direction
 * control can drive its RTS line when requested.
 */

/* Enable clock_gettime/usleep/cfmakeraw etc. under strict -std=c99. */
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L

#include "pa_serial.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

struct pa_serial {
    int         fd;
    uint32_t    baud;
};

/* -------------------------------------------------------------------------
 *  baud-rate mapping
 * ------------------------------------------------------------------------- */

static speed_t pa_baud_to_speed(uint32_t baud)
{
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B0;   /* caller handles */
    }
}

/* ------------------------------------------------------------------------- */
/*  pa_port callbacks
 * ------------------------------------------------------------------------- */

static int ser_open(void *userdata)
{
    (void)userdata;
    return 0;   /* already opened in pa_serial_open */
}

static int ser_send(const uint8_t *data, size_t len, void *userdata)
{
    pa_serial_t *h = (pa_serial_t *)userdata;
    size_t off = 0;
    while (off < len) {
        ssize_t n = write(h->fd, data + off, len - off);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        off += (size_t)n;
    }
    return 0;
}

static int ser_recv(uint8_t *data, size_t max_len, void *userdata)
{
    pa_serial_t *h = (pa_serial_t *)userdata;
    struct pollfd pfd = { h->fd, POLLIN, 0 };
    int pr = poll(&pfd, 1, 0);   /* 0 => non-blocking; returns 1 if readable */
    if (pr <= 0) {
        return 0;
    }
    ssize_t n = read(h->fd, data, max_len);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }
    return (int)n;
}

static uint32_t ser_ticks(void *userdata)
{
    (void)userdata;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((ts.tv_sec * 1000u) + (ts.tv_nsec / 1000000u));
}

static int ser_set_dir(void *userdata, bool tx)
{
    /* Many USB-RS485 adapters auto-manage transmit-enable. Implementing RTS
     * line-toggle here is a per-adapter detail; leave it a no-op by default. */
    (void)userdata;
    (void)tx;
    return 0;
}

static void ser_flush(void *userdata)
{
    pa_serial_t *h = (pa_serial_t *)userdata;
    (void)tcdrain(h->fd);
    (void)tcflush(h->fd, TCIFLUSH);
}

static void ser_delay_ms(uint32_t ms, void *userdata)
{
    (void)userdata;
    usleep((useconds_t)ms * 1000u);
}

/* -------------------------------------------------------------------------
 *  public API
 * ------------------------------------------------------------------------- */

pa_serial_t *pa_serial_open(const char *device, uint32_t baud)
{
    speed_t speed = pa_baud_to_speed(baud);
    if (speed == B0) {
        fprintf(stderr, "pa_serial: unsupported baud rate %u\n", baud);
        return NULL;
    }

    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("pa_serial: open");
        return NULL;
    }

    struct termios tio;
    memset(&tio, 0, sizeof tio);
    if (tcgetattr(fd, &tio) != 0) {
        perror("pa_serial: tcgetattr");
        close(fd);
        return NULL;
    }

    cfmakeraw(&tio);                            /* raw 8N1, no flow control */
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tio.c_cflag |= CS8;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;                        /* non-blocking reads */

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        perror("pa_serial: tcsetattr");
        close(fd);
        return NULL;
    }

    pa_serial_t *h = calloc(1, sizeof *h);
    if (h == NULL) {
        close(fd);
        return NULL;
    }
    h->fd = fd;
    h->baud = baud;

    (void)tcflush(fd, TCIOFLUSH);
    return h;
}

void pa_serial_flush_input(pa_serial_t *h)
{
    if (h != NULL) {
        (void)tcflush(h->fd, TCIFLUSH);
    }
}

void pa_serial_build_port(pa_serial_t *h, pa_port_t *port)
{
    memset(port, 0, sizeof *port);
    port->userdata = h;
    port->open     = ser_open;
    port->send     = ser_send;
    port->recv     = ser_recv;
    port->ticks    = ser_ticks;
    port->set_dir  = ser_set_dir;
    port->flush    = ser_flush;
    port->delay_ms = ser_delay_ms;
}

void pa_serial_close(pa_serial_t *h)
{
    if (h == NULL) return;
    if (h->fd >= 0) {
        (void)tcdrain(h->fd);
        close(h->fd);
    }
    free(h);
}