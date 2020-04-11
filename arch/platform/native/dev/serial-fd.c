/*
 * Copyright (c) 2001, Adam Dunkels.
 * Copyright (c) 2009, 2010 Joakim Eriksson, Niclas Finne, Dogan Yazar.
 * Copyright (c) 2020 alexrayne <alexraynepe196@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

 /* Below define allows importing saved output into Wireshark as "Raw IP" packet type */
#define WIRESHARK_IMPORT_FORMAT 1
#include "contiki.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <err.h>

#include "dev/serial-line.h"
#include "serial-fd.h"
#include "platform-native.h"
#include "services/rpl-border-router/native/slip-config.h"

#ifndef BAUDRATE
#define BAUDRATE B115200
#endif

#ifdef SLIP_DEV_CONF_SEND_DELAY
#define SEND_DELAY SLIP_DEV_CONF_SEND_DELAY
#else
#define SEND_DELAY 0
#endif



//-----------------------------------------------------------------------------
struct SlipConfig fdsio_config = {
        0, //verbose
        0, //flowcontrol
        0, //timestamp
        NULL, //ipaddr;
        NULL, //siodev = NULL;
        NULL, //host = NULL;
        NULL, //port = NULL;
        { "" }, //tundev
        SEND_DELAY, //basedelay
        BAUDRATE, //b_rate
};

void fdsio_assign_slip_config(){
    fdsio_config = slip_config;
}
//-----------------------------------------------------------------------------


static int devopen(const char *dev, int flags);

static FILE *infdsio;

/* for statistics */
long fdsio_sent = 0;
long fdsio_received = 0;
unsigned long fdsio_packet_sent; //for statistics
struct sockaddr fdsio_sa; //used by udp service
socklen_t       fdsio_salen =0;


int fdsiofd = 0;

#define SLIP_END     0xC0
#define PROGRESS(s) do { } while(0)

/*---------------------------------------------------------------------------*/
static void fdsio_start();
static int  connect_to_server(const char *host, const char *port);

static int  start_server_udp(const char *host, const char *port);
static int  start_server_tcp(const char *host, const char *port);

void fdsio_init(void)
{
  setvbuf(stdout, NULL, _IOLBF, 0); /* Line buffered output. */

  if(fdsio_config.host != NULL) {
    if(fdsio_config.port == NULL) {
      fdsio_config.port = "60001";
    }
    if( fdsio_config.flowcontrol == 0)
        fdsiofd = connect_to_server(fdsio_config.host, fdsio_config.port);
    else if (fdsio_config.flowcontrol == FDSIO_SERVE_TCP){
        fdsiofd = start_server_tcp(fdsio_config.host, fdsio_config.port);
        return;
    }
    else if (fdsio_config.flowcontrol == FDSIO_SERVE_UDP){
        fdsiofd = start_server_udp(fdsio_config.host, fdsio_config.port);
        return;
    }
    if(fdsiofd == -1) {
      err(1, "can't connect to ``%s:%s''", fdsio_config.host, fdsio_config.port);
    }
  } else if(fdsio_config.siodev != NULL) {
    if(strcmp(fdsio_config.siodev, "null") == 0) {
      /* Disable slip */
      return;
    }
    fdsiofd = devopen(fdsio_config.siodev, O_RDWR | O_NONBLOCK);
    if(fdsiofd == -1) {
      err(1, "can't open siodev ``/dev/%s''", fdsio_config.siodev);
    }
  } else {
    static const char *siodevs[] = {
      "ttyUSB0", "cuaU0", "ucom0" /* linux, fbsd6, fbsd5 */
    };
    int i;
    for(i = 0; i < 3; i++) {
      fdsio_config.siodev = siodevs[i];
      fdsiofd = devopen(fdsio_config.siodev, O_RDWR | O_NONBLOCK);
      if(fdsiofd != -1) {
        break;
      }
    }
    if(fdsiofd == -1) {
      err(1, "can't open siodev");
    }
  }
  fdsio_start();
}

/*---------------------------------------------------------------------------*/
int is_sensible_string(const unsigned char *s, int len)
{
  int i;
  for(i = 1; i < len; i++) {
    if(s[i] == 0 || s[i] == '\r' || s[i] == '\n' || s[i] == '\t') {
      continue;
    } else if(s[i] < ' ' || '~' < s[i]) {
      return 0;
    }
  }
  return 1;
}

/*---------------------------------------------------------------------------*/
static int (* input_handler)(unsigned char) = NULL;

void fdsio_set_input(int (*f)(unsigned char))
{
  input_handler = f;
}
/*---------------------------------------------------------------------------*/
/*
 * Read from serial, when we have a packet call slip_packet_input. No output
 * buffering, input buffered by stdio.
 */

void fdsio_input(FILE *inslip)
{
  static unsigned char inbuf[2048];
  int inbufptr;
  int recv_len, i;

  while (1) {

  if (fdsio_config.flowcontrol != FDSIO_SERVE_UDP)
      recv_len = fread(inbuf, 1, sizeof(inbuf), infdsio);
  else
      recv_len = recvfrom(fdsiofd, inbuf, sizeof(inbuf), 0, &fdsio_sa, &fdsio_salen );

  if(recv_len < 0) {
    err(1, "fdsio_input: read");
  }

  if (recv_len == 0)
      break;

  fdsio_received+=recv_len;
  if (fdsio_config.verbose <= 1) {
  /* Notify specified rs232 handler */
  if(input_handler != NULL) {
    for (i=0; i < recv_len; i++) {
      input_handler(inbuf[i]);
    }
  } else {
    /* Notify serial process */
    for (i=0; i < recv_len; i++) {
      serial_line_input_byte(inbuf[i]);
    }
  }
  }
  else { // if (fdsio_config.verbose <= 1)
      inbufptr = 0;
      for (i=0; i < recv_len; i++) {
          unsigned char c = inbuf[i];

          if(input_handler != NULL) {
              input_handler(c);
          }
          else
              serial_line_input_byte(c);

          /* Echo lines as they are received for verbose=2,3,5+ */
          /* Echo all printable characters for verbose==4 */
          if(fdsio_config.verbose == 4) {
            if(c == 0 || c == '\r' || c == '\n' || c == '\t' || (c >= ' ' && c <= '~')) {
              fwrite(&c, 1, 1, stdout);
            }
          } else if(fdsio_config.verbose >= 2) {
            unsigned slen = (i - inbufptr);
            if(c == '\n' && is_sensible_string(inbuf, slen ))
            {
              fwrite(inbuf + inbufptr, slen, 1, stdout);
              inbufptr = i;
            }
          }
      }
  }
  } // while

}

unsigned char fdsio_buf[FDSIO_BUF_SIZE];
int fdsio_send_end;
static int slip_begin, slip_packet_end;
unsigned int fdsio_packet_count;
static struct timer send_delay_timer;
/* delay between slip packets */
static clock_time_t send_delay = SEND_DELAY;
/*---------------------------------------------------------------------------*/
static
void fdsio_slip_send(unsigned char c)
{
  if(fdsio_send_end >= sizeof(fdsio_buf)) {
    err(1, "fdsio_slip_send overflow");
  }
  fdsio_buf[fdsio_send_end] = c;
  fdsio_send_end++;
  fdsio_sent++;
  if(c == SLIP_END) {
    /* Full packet received. */
    fdsio_packet_count++;
    if(slip_packet_end == 0) {
      slip_packet_end = fdsio_send_end;
    }
  }
}

static
void fdsio_byte_send(unsigned char c)
{
  if(fdsio_send_end >= sizeof(fdsio_buf)) {
    err(1, "fdsio_byte_send overflow");
  }
  fdsio_buf[fdsio_send_end] = c;
  fdsio_send_end++;
  slip_packet_end = fdsio_send_end;
  fdsio_sent++;
}

void fdsio_send(unsigned char c)
{
    if(send_delay <= 0)
        fdsio_byte_send(c);
    else
        fdsio_slip_send(c);
}

static
int fdsio_send_bytes(const void* data, unsigned len){
        if (fdsio_send_end + len > FDSIO_BUF_SIZE)
            len = FDSIO_BUF_SIZE - fdsio_send_end;

        memcpy(fdsio_buf +fdsio_send_end, data, len );
        fdsio_send_end  += len;
        slip_packet_end = fdsio_send_end;
        fdsio_sent += len;
        return len;
}

static
int fdsio_send_slip(const void* data, unsigned len){
    if (fdsio_send_end + len > FDSIO_BUF_SIZE)
        len = FDSIO_BUF_SIZE - fdsio_send_end;


    const uint8_t* p = (const uint8_t*)data;
    for(unsigned i = 0; i < len; i++) {
        fdsio_slip_send(p[i]);
    }
    return len;
}
/*---------------------------------------------------------------------------*/
int fdsio_busy(void){
    if (fdsiofd != -1)
        return fdsio_send_end;
    else
        return -1;
}

int fdsio_space_avail(void){
    if (fdsiofd != -1)
        return  (sizeof(fdsio_buf) - fdsio_send_end) ;
    else
        return -1;
}

int slip_empty()
{
  return slip_packet_end == 0;
}

/*---------------------------------------------------------------------------*/
int fdsio_write_bytes(const void* data, unsigned len){
    if(send_delay <= 0)
        len = fdsio_send_bytes(data, len);
    else
        len = fdsio_send_slip(data, len);

    if (len > 0)
    if(fdsio_config.verbose > 2) {
      #ifdef __CYGWIN__
          printf("Packet from WPCAP of length %d\n", len);
      #else
          printf("Packet from TUN of length %d\n", len);
      #endif
    if(fdsio_config.verbose > 4) {
#if WIRESHARK_IMPORT_FORMAT
      printf("0000");
      const uint8_t* p = (const uint8_t*)data;
      for(unsigned i = 0; i < len; i++) {
        printf(" %02x", p[i]);
      }
#else
      printf("         ");
      for(i = 0; i < len; i++) {
        printf("%02x", p[i]);
        if((i & 3) == 3) {
          printf(" ");
        }
        if((i & 15) == 15) {
          printf("\n         ");
        }
      }
#endif
      printf("\n");
    }//if(fdsio_config.verbose > 4)
    }//if(fdsio_config.verbose > 2)

    return len;
}

/*---------------------------------------------------------------------------*/
static
void fdsio_flushbuf(int fd)
{
  int n;

  while(1) {

  if(slip_empty()) {
    return;
  }

  if (fd != -1){
    if (fdsio_config.flowcontrol != FDSIO_SERVE_UDP)
        n = write(fd, fdsio_buf + slip_begin, slip_packet_end - slip_begin);
    else
        n = sendto(fd, fdsio_buf + slip_begin, slip_packet_end - slip_begin
                        , 0, &fdsio_sa, fdsio_salen );
  }
  else
      // serial closed - just drop sent data
      n = (slip_packet_end - slip_begin);

  if(n == -1 && errno != EAGAIN) {
    err(1, "fdsio_flushbuf write failed");
  } else if(n == -1) {
    PROGRESS("Q");		/* Outqueue is full! */
    // free space in buf
    memmove(fdsio_buf, fdsio_buf + slip_begin,
           fdsio_send_end - slip_begin);
    fdsio_send_end -= slip_begin;
    slip_packet_end-= slip_begin;
    slip_begin = 0;
    return;
  }
  else {
    slip_begin += n;

    if(slip_begin >= slip_packet_end) {
      if(send_delay <= 0){
          fdsio_send_end -= slip_packet_end;
          slip_begin = slip_packet_end = 0;
          return;
      }

      fdsio_packet_count--;
      if(fdsio_send_end > slip_packet_end) {
        memmove(fdsio_buf, fdsio_buf + slip_packet_end,
               fdsio_send_end - slip_packet_end);
      }
      fdsio_send_end -= slip_packet_end;
      slip_begin = slip_packet_end = 0;
      if(fdsio_send_end > 0) {
        /* Find end of next slip packet */
        for(n = 1; n < fdsio_send_end; n++) {
          if(fdsio_buf[n] == SLIP_END) {
            slip_packet_end = n + 1;
            break;
          }
        }
        /* a delay between slip packets to avoid losing data */
          timer_set(&send_delay_timer, send_delay);
          return;
      }
    }//if(slip_begin == slip_packet_end)
    //else {}
  }
  } // while
}

/*---------------------------------------------------------------------------*/
static void
stty_telos(int fd)
{
  struct termios tty;
  speed_t speed = fdsio_config.b_rate;
  int i;

  if(tcflush(fd, TCIOFLUSH) == -1) {
    err(1, "tcflush");
  }

  if(tcgetattr(fd, &tty) == -1) {
    err(1, "tcgetattr");
  }

  cfmakeraw(&tty);

  /* Nonblocking read. */
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;
  if(fdsio_config.flowcontrol) {
    tty.c_cflag |= CRTSCTS;
  } else {
    tty.c_cflag &= ~CRTSCTS;
  }
  tty.c_cflag &= ~HUPCL;
  tty.c_cflag &= ~CLOCAL;

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  if(tcsetattr(fd, TCSAFLUSH, &tty) == -1) {
    err(1, "tcsetattr");
  }

#if 1
  /* Nonblocking read and write. */
  /* if(fcntl(fd, F_SETFL, O_NONBLOCK) == -1) err(1, "fcntl"); */

  tty.c_cflag |= CLOCAL;
  if(tcsetattr(fd, TCSAFLUSH, &tty) == -1) {
    err(1, "tcsetattr");
  }

  i = TIOCM_DTR;
  if(ioctl(fd, TIOCMBIS, &i) == -1) {
    err(1, "ioctl");
  }
#endif

  usleep(10 * 1000);    /* Wait for hardware 10ms. */

  /* Flush input and output buffers. */
  if(tcflush(fd, TCIOFLUSH) == -1) {
    err(1, "tcflush");
  }
}
/*---------------------------------------------------------------------------*/
static int
set_fd(fd_set *rset, fd_set *wset)
{
  /* Anything to flush? */
  if(!slip_empty() && (send_delay == 0 || timer_expired(&send_delay_timer))) {
    FD_SET(fdsiofd, wset);
  }

  FD_SET(fdsiofd, rset);	/* Read from slip ASAP! */
  return fdsiofd;
}
/*---------------------------------------------------------------------------*/
static void
handle_fd(fd_set *rset, fd_set *wset)
{
  if(FD_ISSET(fdsiofd, rset)) {
    fdsio_input(infdsio);
  }

  if(FD_ISSET(fdsiofd, wset)) {
    fdsio_flushbuf(fdsiofd);
  }
}

/*---------------------------------------------------------------------------*/
// TODO this is a shared function
static
int devopen(const char *dev, int flags)
{
  char t[32];
  strcpy(t, "/dev/");
  strncat(t, dev, sizeof(t) - 5);
  return open(t, flags);
}

/*---------------------------------------------------------------------------*/
static
void* get_in_addr(struct sockaddr *sa)
{
  if(sa->sa_family == AF_INET) {
    return &(((struct sockaddr_in *)sa)->sin_addr);
  }
  return &(((struct sockaddr_in6 *)sa)->sin6_addr);
}

static
int get_in_port(struct sockaddr *sa)
{
  if(sa->sa_family == AF_INET) {
    return ntohs( ((struct sockaddr_in *)sa)->sin_port );
  }
  return ntohs( ((struct sockaddr_in6 *)sa)->sin6_port );
}
/*---------------------------------------------------------------------------*/

// TODO implement connection to UDP service, if it have any sence.
static int
connect_to_server(const char *host, const char *port)
{
  /* Setup TCP connection */
  struct addrinfo hints;
  struct addrinfo* servinfo, *p;
  char s[INET6_ADDRSTRLEN];
  int rv, fd;

  memset(&hints, 0, sizeof(hints) );
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  if((rv = getaddrinfo(host, port, &hints, &servinfo)) != 0) {
    err(1, "getaddrinfo: %s", gai_strerror(rv));
    return -1;
  }

  /* loop through all the results and connect to the first we can */
  for(p = servinfo; p != NULL; p = p->ai_next) {
    if((fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      perror("client: socket");
      continue;
    }

    if(connect(fd, p->ai_addr, p->ai_addrlen) == -1) {
      close(fd);
      perror("client: connect");
      continue;
    }
    break;
  }

  if(p == NULL) {
    err(1, "can't connect to ``%s:%s''", host, port);
    return -1;
  }

  fcntl(fd, F_SETFL, O_NONBLOCK);

  inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
            s, sizeof(s));

  /* all done with this structure */
  freeaddrinfo(servinfo);
  return fd;
}
/*---------------------------------------------------------------------------*/
static const struct select_callback fdsio_callback = { set_fd, handle_fd };

static
void fdsio_start(){

  select_set_callback(fdsiofd, &fdsio_callback);

  if(fdsio_config.host != NULL) {
    fprintf(stderr, "********Serial opened to ``%s:%s''\n", fdsio_config.host,
            fdsio_config.port);
  } else {
    fprintf(stderr, "********Serial started on ``/dev/%s''\n", fdsio_config.siodev);
    stty_telos(fdsiofd);
  }

  send_delay = fdsio_config.basedelay;
  timer_set(&send_delay_timer, 0);

  infdsio = fdopen(fdsiofd, "r");
  if(infdsio == NULL) {
    err(1, "fdsio_init: fdopen");
  }
}
/*---------------------------------------------------------------------------*/
static void fdsio_accept(int fd);
int listen_fdsio = 0;

static
int listen_set_fd(fd_set *rset, fd_set *wset)
{
  FD_SET(listen_fdsio, rset);    /* Read from slip ASAP! */
  return listen_fdsio;
}

static
void listen_handle(fd_set *rset, fd_set *wset)
{
  if(FD_ISSET(listen_fdsio, rset)) {
    fdsio_accept(listen_fdsio);
  }
}

static const struct select_callback fdsio_listen_callback = { listen_set_fd, listen_handle };
/*---------------------------------------------------------------------------*/
static
int  start_server_udp(const char *host, const char *port){

    // TODO implement UDP service
    /* Setup UDP connection */
    struct addrinfo* servinfo;
    struct addrinfo hints;
    int rv;

    fdsiofd = -1;

    memset(&hints, 0, sizeof(hints) );
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    if((rv = getaddrinfo(host, port, &hints, &servinfo)) != 0) {
      err(1, "getaddrinfo: %s", gai_strerror(rv));
      return -1;
    }

    /* create listening socket */
    fdsiofd = socket(servinfo->ai_family , servinfo->ai_socktype
                                    , servinfo->ai_protocol);
    if ((fdsiofd) == -1)
    {
        err(1, "socket() failed: %s\n", strerror(errno));
        return -1;
    }

    /* bind to listening addr/port */
    if (bind(fdsiofd, servinfo->ai_addr, servinfo->ai_addrlen) == -1) {
        close(fdsiofd);
        err(1, "bind() failed: %s\n", strerror(errno));
        return -1;
    }

    printf("LISTENING ON PORT %d\n", get_in_port((struct sockaddr *)servinfo->ai_addr));
    fcntl(fdsiofd, F_SETFL, O_NONBLOCK);

    freeaddrinfo(servinfo);
    servinfo = NULL;

    return fdsiofd;
}

static
int  start_server_tcp(const char *host, const char *port){
    /* Setup TCP connection */
    struct addrinfo* servinfo;
    struct addrinfo hints;
    int rv;
    int rs;

    fdsiofd = -1;

    memset(&hints, 0, sizeof(hints) );
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags    = AI_NUMERICSERV;

    if((rv = getaddrinfo(host, port, &hints, &servinfo)) != 0) {
      err(1, "getaddrinfo: %s", gai_strerror(rv));
      return -1;
    }

    /* create listening socket */
    listen_fdsio = socket(servinfo->ai_family , servinfo->ai_socktype
                                    , servinfo->ai_protocol);
    if ((listen_fdsio) == -1)
    {
        err(1, "socket() failed: %s\n", strerror(errno));
        return -1;
    }

    /* reuse address option */
    rs = 1;
    setsockopt(listen_fdsio, SOL_SOCKET, SO_REUSEADDR, (char*)&rs, sizeof(rs));

    /* bind to listening addr/port */
    if (bind(listen_fdsio, servinfo->ai_addr, servinfo->ai_addrlen) == -1) {
        close(listen_fdsio);
        err(1, "bind() failed: %s\n", strerror(errno));
        return -1;
    }

    unsigned portno = get_in_port((struct sockaddr *)servinfo->ai_addr);
    printf("LISTENING ON PORT %u\n", portno);
    fcntl(listen_fdsio, F_SETFL, O_NONBLOCK);

    /* free address lookup info */
    freeaddrinfo(servinfo);
    servinfo = NULL;

    /* wait for client */
    if (listen(listen_fdsio, 1) == -1) {
        close(listen_fdsio);
        err(1, "listen() failed: %s\n", strerror(errno));
        return -11;
    }

    select_set_callback(listen_fdsio, &fdsio_listen_callback);
    return -1;
}

static
void fdsio_accept(int fd) {

    int client_sock = -1;

    client_sock = accept(listen_fdsio, (struct sockaddr *)&fdsio_sa, &fdsio_salen);
    if (client_sock == -1) {
        if ((errno == EAGAIN) || (errno == EWOULDBLOCK)){
            return; // ok polling
        }
        close(listen_fdsio);
        err(1, "accept() failed: %s\n", strerror(errno));
        return;
    }

    printf("CLIENT CONNECTION RECEIVED\n");

    /* stop listening now that we have a client */
    close(listen_fdsio);
    listen_fdsio = -1;
    select_set_callback(-1, &fdsio_listen_callback);

    fdsiofd = client_sock;
    fcntl(fdsiofd, F_SETFL, O_NONBLOCK);

    printf("SERVER CONNECTION ESTABLISHED\n");
    fdsio_start();
}

