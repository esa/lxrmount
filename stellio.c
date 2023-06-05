/*
 * TCP listener for stellarium telescope control plugin
 */

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <inttypes.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <math.h>
#include <string.h>
#include <termios.h>

#include <sys/time.h>
#include <time.h>

#include <libnova/libnova.h>

//#define TCP_PORT 10000

double lon_rad = 4.4730814 * (M_PI/180.0); /* Lange Voort */

void angle2str(char *s, double phi) {
  double a = phi*(180.0/M_PI);
  int d, m;
  d = (int)a;
  a = 60.0*(a - d);
  m = (int)a;
  a = 60.0*(a - m);
  sprintf(s, "%3u°%02u'%05.2f", d, m, a);
}

uint16_t get_u16(uint8_t *b) {
  return b[0] + (b[1]<<8);
}

uint32_t get_u32(uint8_t *b) {
  return b[0] + (b[1]<<8) + (b[2]<<16) + (b[3]<<24);
}

void put_u32(uint8_t *b, uint32_t u) {
  b[0] = u & 0xff;  u >>= 8;
  b[1] = u & 0xff;  u >>= 8;
  b[2] = u & 0xff;  u >>= 8;
  b[3] = u & 0xff;
}

void ra_dec2ha_de_now(double ra, double dec, double *ha, double *de) {
  double jd, ts;
  /* RA -> HA according to sidereal time */
  jd = ln_get_julian_from_sys();
  ts = ln_get_apparent_sidereal_time(jd);
  *ha = ts*(M_PI/12.0) + lon_rad - ra;
  /* FIXME: Dec-->DE also depends on nut/prec vs J2000.0 */
  *de = dec;
}

#if 0
void cel2stel(mount_t *mount, double *ra, double *sdec, double ha, double cdec) {
  double jd, ts;
  /* HA -> RA according to sidereal time */
  jd = ln_get_julian_from_sys();
  ts = ln_get_apparent_sidereal_time(jd);
  *ra = ts*(M_PI/12.0) + 14.6698733*(M_PI/180.0) - ha;
  *sdec = cdec;
}

void cel2mt(mount_t *mount, double *x1, double *x2, double ha, double dec) {
  *x1 = ha + mount->ha_off;
  *x2 = dec + mount->dec_off;
  /* 
     TODO1: mount rotational transform goes here
     TODO2: PEC goes here
  */
}

#endif

void stell_notify(int stell_fd, double ra, double dec) {
  uint32_t ra0;
  int32_t dec0;
  static uint8_t tx_msg[] = {
    24,0, 0,0, 0,0,0,0,0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
  };
  dec = fmod(dec + M_PI, 2*M_PI) - M_PI;
  if (dec > 0.5*M_PI) {
    dec = M_PI - dec;
    ra += M_PI;
  }
  else if (dec < -0.5*M_PI) {
    dec = -M_PI - dec;
    ra += M_PI;
  }
  ra = fmod(ra, 2*M_PI);
  ra *= 0x80000000/M_PI;
  dec *= 0x80000000/M_PI;
  ra0 = (uint32_t)(0.5 + ra);
  dec0 = (int32_t)(0.5 + dec);
  put_u32(tx_msg + 12, ra0);
  put_u32(tx_msg + 16, (uint32_t)dec0);
  fprintf(stderr, "ra = 0x%08x  dec = 0x%08x\n", ra0, dec0);
  send(stell_fd, tx_msg, sizeof(tx_msg), 0);
}

void write_track(double ha, double de) {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  FILE *f = fopen("goto.txt", "w");
  fprintf(f, "%lld.%09ld\t%.12f\t%.12f\n", (long long)ts.tv_sec, ts.tv_nsec, ha, de);
  ts.tv_sec += 1;
  ha += (366.24/365.24)*(2*M_PI/86400.0); /* FIXME: not exact */
  fprintf(f, "%lld.%09ld\t%.12f\t%.12f\n", (long long)ts.tv_sec, ts.tv_nsec, ha, de);
  fclose(f);
}

int stell_read(int stell_fd) {
  uint32_t ra0;
  int32_t dec0;
  double goto_ra, goto_dec;
  uint8_t buf[22];
  unsigned msg_len, msg_type;
  ssize_t n;
  n = recv(stell_fd, buf, 2, 0);
  if (n < 2) goto ret;
  msg_len = get_u16(buf);
  if (msg_len < 3) goto ret;
  msg_len -= 2;
  while (msg_len > sizeof(buf)) {
    n = recv(stell_fd, buf, sizeof(buf), 0);
    if (n < sizeof(buf)) goto ret;
    msg_len -= sizeof(buf);
  }
  n = recv(stell_fd, buf, msg_len, 0);
  if (n < msg_len) goto ret;
  msg_type = get_u16(buf);
  switch (msg_type) {
  case 0:
    ra0 = get_u32(buf + 10);
    dec0 = (int32_t)get_u32(buf + 14);
    goto_ra = ra0;
    goto_ra *= M_PI/0x80000000;
    goto_dec = dec0;
    goto_dec *= M_PI/0x80000000;
    double ha, de;
    ra_dec2ha_de_now(goto_ra, goto_dec, &ha, &de);
    printf("GOTO: RA,Dec = %f\t%f\tHA,DE = %f\t%f\n",
	   goto_ra, goto_dec, ha, de);
    write_track(ha, de);
    break;
  default:
    break;
  }
 ret:
  return n;
}

void *socket_thread(void *arg) {
  int *fd = (int*)arg;
  int sock_fd, stell_fd, rc;
  struct sockaddr_storage client_addr;
  socklen_t sin_size;
  char s[INET6_ADDRSTRLEN];

#if 0
  struct sockaddr_in bind_addr;

  if ((sock_fd = socket(PF_INET, SOCK_STREAM, 0)) == -1) {
    fprintf(stderr, "socket_thread: socket\n");
    return NULL;
  }
  memset(&bind_addr, 0, sizeof(bind_addr));
  bind_addr.sin_family = AF_INET;
  bind_addr.sin_addr.s_addr = INADDR_ANY;
  bind_addr.sin_port = htons(TCP_PORT);
  if (bind(sock_fd, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) == -1) {
    fprintf(stderr, "socket_thread: bind\n");
    return NULL;
  }
#endif
  struct addrinfo hints, *servinfo, *p;
  int on = 1;

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  if ((rc = getaddrinfo("127.0.0.1", "10000", &hints, &servinfo)) != 0) {
    fprintf(stderr, "error: getaddrinfo: %s\n", gai_strerror(rc));
    return NULL;
  }
  for (p = servinfo; p != NULL; p = p->ai_next) {
    if ((sock_fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      fprintf(stderr, "warn: socket\n");
      continue;
    }
    if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) == -1) {
      fprintf(stderr, "error: setsockopt\n");
      free(servinfo);
      return NULL;
    }
    if (bind(sock_fd, p->ai_addr, p->ai_addrlen) != -1)
      break;
    close(sock_fd);
    fprintf(stderr, "warn: bind\n");
  }
  if (p == NULL) {
    fprintf(stderr, "error: could not bind\n");
    free(servinfo);
    return NULL;
  }

  if (listen(sock_fd, 0) == -1) {
    fprintf(stderr, "socket_thread: listen\n");
    return NULL;
  }

  fprintf(stderr, "socket_thread: bound & listening\n");

  for (;;) {
    sin_size = sizeof(client_addr);
    stell_fd = accept(sock_fd,
		      (struct sockaddr*)&client_addr, &sin_size);
    if (stell_fd == -1) {
      fprintf(stderr, "socket_thread: accept\n");
      continue;
    }

    inet_ntop(client_addr.ss_family,
	      &((struct sockaddr_in*)&client_addr)->sin_addr,
	      s, sizeof(s));
    fprintf(stderr, "socket_thread: %s connected\n", s);

    uint8_t tx_msg[] =
    {24,0, 0,0, 0,0,0,0,0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,};
    send(stell_fd, tx_msg, sizeof(tx_msg), 0);

    *fd = stell_fd;

    for (;;) {
      rc = stell_read(stell_fd);
      if (rc < 0) {
	fprintf(stderr, "stell_read\n");
	break;
      }
      if (rc == 0) {
	fprintf(stderr, "stell_read: disconnected\n");
	break;
      }
    }
    *fd = -1;
    close(stell_fd);
  }
}

int main() {
  int stell_fd = -1;
  pthread_t socket_th;
  pthread_create(&socket_th, NULL, socket_thread, &stell_fd);

  double ra, dec;
  for (;;) {
    if (stell_fd == -1) {
      usleep(10000);
      continue;
    }
    puts("ra dec [rad]> ");
    if (scanf("%lf%lf", &ra, &dec) != 2)
      break;
    stell_notify(stell_fd, ra, dec);
  }  
  return 0;
}
