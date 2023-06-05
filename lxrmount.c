#include <sys/ioctl.h>
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
#include <signal.h>
#include <errno.h>

#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <sys/time.h>
#include <time.h>

#include <sys/types.h>
#include <linux/joystick.h>

#define NEQ5_MOTOR_STEPS    50 /* "1.8deg" = 200 1/4periods = 50 periods */
#define NEQ5_GEAR_RATIO    705
#define LXR_USTEPS         200 /* 50*705*200 ~~ <2^23 */
#define MAX_V             3000
#define MAX_V_GOTO        3000

#define TS_GE(a,b) (((a)->tv_sec == (b)->tv_sec) \
  ? ((a)->tv_nsec >= (b)->tv_nsec) \
  : ((a)->tv_sec >= (b)->tv_sec))
#define TS_LE(a,b) (((a)->tv_sec == (b)->tv_sec) \
  ? ((a)->tv_nsec <= (b)->tv_nsec) \
  : ((a)->tv_sec <= (b)->tv_sec))
#define TS_ADD(y,a,b) do {		   \
  (y)->tv_sec = (a)->tv_sec + (b)->tv_sec; \
  (y)->tv_nsec = (a)->tv_nsec + (b)->tv_nsec; \
    if ((y)->tv_nsec >= 1000000000) { \
      ++(y)->tv_sec; \
      (y)->tv_nsec -= 1000000000; \
    } \
  } while (0)

static const double
  deg2rad = M_PI/180.0,
  rad2deg = 180.0/M_PI,
  ustep2rad = (2*M_PI)/(NEQ5_MOTOR_STEPS*NEQ5_GEAR_RATIO*LXR_USTEPS),
  rad2ustep = (NEQ5_MOTOR_STEPS*NEQ5_GEAR_RATIO*LXR_USTEPS)/(2*M_PI);

static struct timespec local_t0;

typedef struct motion_t {
  struct timespec t; /* cas, kdy bylo provedeno rizeni a mereni */
  double x1, x2, xw1, xw2; /* [rad] */
  double vw1, vw2; /* [rad/s] */
} motion_t;

typedef struct mount_t {
  motion_t motion;
  FILE *pxmc;
  FILE *points_file;
  //int stell_fd;
  //int cmd_goto, cmd_save_point, cmd_calibrate;
  int finish; // stopped, socket_connected;
  /* mount offset calibration */
  double a_off, b_off;
  /* controller state */
  double s1, s2;
  /* joystick */
  double js_v1, js_v2;
  unsigned js_cmd;
} mount_t;

typedef struct traj_t {
  FILE *f;
  struct timespec t0, t1;
  double a0, b0, a1, b1;
  int eof, line_no;
} traj_t;

double delta_t(struct timespec *t1, struct timespec *t0) {
  long long dt_sec = t1->tv_sec - t0->tv_sec;
  long dt_nsec = t1->tv_nsec - t0->tv_nsec;
  double dt = dt_nsec;
  dt *= 0.000000001;
  dt += dt_sec;
  return dt;
}

void angle2str(char *s, double phi) {
  double a = phi*(180.0/M_PI);
  int d, m;
  d = (int)a;
  a = 60.0*(a - d);
  m = (int)a;
  a = 60.0*(a - m);
  sprintf(s, "%3u°%02u'%05.2f", d, m, a);
}

#ifdef JOY
void *js_control(void *arg) {
  mount_t *mount = (mount_t*)arg;
  double jvx, jvy;
  char *dev = "/dev/input/js0";
  int fd;
  unsigned char n_axes;
  int i, axis[256];
  struct js_event js;

  if ((fd = open(dev, O_RDONLY)) < 0) {
    perror("joy: open");
    return NULL;
  }
  fprintf(stderr, "=*= joystick OK =*=\n");
  ioctl(fd, JSIOCGAXES, &n_axes);
  for (;;) {
    if (read(fd, &js, sizeof(js)) != sizeof(js)) {
      perror("joy: read");
      return NULL;
    }
    if ((js.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS) {
      //fprintf(stderr, "***** %u : %d\n", js.number, js.value);
      axis[js.number] = js.value;
      float gain = (32767.0 - (float)axis[3])/(2*32767.0);
      jvx = (1.0/32768.0)*axis[0]*gain;
      jvy = (1.0/32768.0)*axis[1]*gain;
      mount->js_v1 = jvx;
      mount->js_v2 = jvy;
    }
    if ((js.type == JS_EVENT_BUTTON) && js.value) {
      fprintf(stderr, "*>* %u\n", 1+js.number);
      mount->js_cmd = 1+js.number;
    }
  }
}
#endif

/*
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
*/

int load_abs_offset(const char *fname, double *a0, double *b0) {

  return 0;
}

int save_point(FILE *f, double a, double b) {

  return 0;
}

int read_ab_trajectory_line(traj_t *tr, struct timespec *t, double *a, double *b) {
  char s[256];
  if (!fgets(s, sizeof(s), tr->f)) {
    tr->eof = 1;
    return -1;
  }
  //fprintf(stderr, "T: %s", s);
  unsigned long long sec;
  char sec_frac[16], *p;
  double sa, sb;
  int n = sscanf(s, "%llu.%12s%lf%lf", &sec, sec_frac, &sa, &sb);
  if (n != 4) {
  //int n = sscanf(s, "%lf%lf", &sa, &sb);
  //if (n != 2) {
    fprintf(stderr, "### T:EOF ###\n");
    tr->eof = 1;
    return -1;
  }

#ifndef FAKETIME
#pragma message("-- compiling for REAL time --")
  /* real time */
  t->tv_sec = sec;
  for (p = sec_frac; *p; p++);
  for ( ; p - sec_frac != 9; p++) {
    *p = '0';
  }
  *p = '\0';
  t->tv_nsec = atoll(sec_frac);
#else
#pragma message("-- compiling for FAKE time --")
  /* fake time */
  t->tv_sec = tr->line_no;
  t->tv_nsec = 0;
  struct timespec tmp;
  TS_ADD(&tmp, t, &local_t0); //debug
  *t = tmp;
#endif

  *a = sa;
  *b = sb;
  ++tr->line_no;

  fprintf(stderr, "T: %llu.%03ld  %+.4f %+.4f\n", (unsigned long long)t->tv_sec,
	  t->tv_nsec/1000000, (*a)*rad2deg, (*b)*rad2deg);
  return 0;
}

int open_ab_trajectory(traj_t *tr, const char *fname) {
  tr->f = fopen(fname, "r");
  if (!tr->f) {
    return -1;
  }
  tr->eof = 0;
  tr->line_no = 0;
  int rc;
  if (read_ab_trajectory_line(tr, &tr->t0, &tr->a0, &tr->b0) ||
      read_ab_trajectory_line(tr, &tr->t1, &tr->a1, &tr->b1)) {
    return -1;
  }
  return 0;
}

int get_ab_trajectory_pv(traj_t *tr, struct timespec *t,
			 double *a, double *b, double *da, double *db) {
  while (!tr->eof && TS_GE(t, &tr->t1)) {
    struct timespec tn;
    double an, bn;
    if (read_ab_trajectory_line(tr, &tn, &an, &bn) == 0) {
      tr->t0 = tr->t1;  tr->a0 = tr->a1;  tr->b0 = tr->b1;
      tr->t1 = tn;  tr->a1 = an;  tr->b1 = bn;
    }
  }
  double dt_t0 = delta_t(t, &tr->t0), dt1_t0 = delta_t(&tr->t1, &tr->t0);
  *da = (tr->a1 - tr->a0)/dt1_t0;
  *db = (tr->b1 - tr->b0)/dt1_t0;
  *a = (*da)*dt_t0 + tr->a0;
  *b = (*db)*dt_t0 + tr->b0;
  /*fprintf(stderr, "t(%f): %+.4f %+.4f %+.4f %+.4f\n",
    dt_t0, (*a)*rad2deg, (*da)*rad2deg, (*b)*rad2deg, (*db)*rad2deg);*/
  return 0;
}

double circle_dist(double length, double a, double b) {
  double d = a - b;
  if (fabs(d) <= length/2.0)
    return d;
  d = fmod(d, length);
  if (fabs(d) <= length/2.0)
    return d;
  return (d < 0.0) ? (d + length) : (d - length);
}

/*
 * xe: dilky
 * vw: dilky/periodu
 */
double ctrl(double xe, double vw, double *s) {
  /* xe = xw - x */
  double v;
  //v = vw + 0.25*(25.0 /*Hz*/)*xe;
  //v = vw + 0.25*xe * (0.1 /* T_s [s] */) * 0.5; //***Pysvejc:(***

  *s += 0.0004*xe;
  //*s += 0.00001*xe;
  if (*s > MAX_V) {
    *s = MAX_V;
  }
  else if (*s < -MAX_V) {
    *s = -MAX_V;
  }
  //v = (*s) + vw * 0.1 + 0.25*xe * 0.1 /* T_s [s] */;
  v = (*s) + (vw + 0.25*xe*0.1/* T_s [s] */)/15.625;
  
  if (v > MAX_V)
    return MAX_V;
  if (v < -MAX_V)
    return -MAX_V;
  return v;
}

void get_position(mount_t *m) {
  motion_t *mt = &m->motion;
  char s[80];
  int xi;

  fprintf(m->pxmc, "APA?\nAPB?\n");
  fflush(m->pxmc);
  clock_gettime(CLOCK_REALTIME, &mt->t);

  fgets(s, sizeof(s), m->pxmc);  fprintf(stderr, "p> %s", s);
  while (sscanf(s, "APA=%d", &xi) != 1);
  mt->x1 = xi * ustep2rad;

  fgets(s, sizeof(s), m->pxmc);  fprintf(stderr, "p> %s", s);
  while (sscanf(s, "APB=%d", &xi) != 1);
  mt->x2 = xi * ustep2rad;

  fprintf(stderr, "= %+9.4f\t%+9.4f\n",
	 rad2deg*mt->x1, rad2deg*mt->x2);
}

void motion_step(mount_t *m) {
  motion_t *mt = &m->motion;
  char s[80];
  double x, e;
  int xi, dx, v1, v2;

#ifdef JOY
  e = m->a_off + mt->xw1 - mt->x1; //circle_dist(IRC_PERIOD_RA, mt->xw1, mt->x1);
  v1 = (int)(0.5 + ctrl(e * rad2ustep, mt->vw1 * rad2ustep, &m->s1));
  e = m->b_off + mt->xw2 - mt->x2; //circle_dist(IRC_PERIOD_DEC, mt->xw2, mt->x2);
  v2 = (int)(0.5 + ctrl(e * rad2ustep, mt->vw2 * rad2ustep, &m->s2));
#else
  e = mt->xw1 - mt->x1; //circle_dist(IRC_PERIOD_RA, mt->xw1, mt->x1);
  v1 = (int)(0.5 + ctrl(e * rad2ustep, mt->vw1 * rad2ustep, &m->s1));
  e = mt->xw2 - mt->x2; //circle_dist(IRC_PERIOD_DEC, mt->xw2, mt->x2);
  v2 = (int)(0.5 + ctrl(e * rad2ustep, mt->vw2 * rad2ustep, &m->s2));
#endif
  //v1 = v2 = 0;
  
  fprintf(m->pxmc, "SPDA:%d\nSPDB:%d\nAPA?\nAPB?\n", v1, v2);
  fflush(m->pxmc);
  clock_gettime(CLOCK_REALTIME, &mt->t);

  fgets(s, sizeof(s), m->pxmc);  //fprintf(stderr, "s> %s", s);
  //sscanf(s, "APA=%lf", &x);    /*FIXME:kontrola!*/
  //xi = x*1000.0;
  sscanf(s, "APA=%d", &xi);
  //dx = (int)circle_dist(IRC_PERIOD_PXMC2, xi, m->x1_pxmc);
  mt->x1 = xi * ustep2rad; //(mt->x1 + dx + IRC_PERIOD_RA) % IRC_PERIOD_RA;

  fgets(s, sizeof(s), m->pxmc);  //fprintf(stderr, "s> %s", s);
  //sscanf(s, "APB=%lf", &x);    /*FIXME:kontrola!*/
  //xi = x*1000.0;
  sscanf(s, "APB=%d", &xi);
  //dx = (int)circle_dist(IRC_PERIOD_PXMC2, xi, m->x2_pxmc);
  mt->x2 = xi * ustep2rad; //(mt->x2 + dx + IRC_PERIOD_DEC) % IRC_PERIOD_DEC;
}

int motion_goto(mount_t *m) {
  motion_t *mt = &m->motion;
  char s[80];
  int x1, x2, flag = 0;
#ifdef JOY
  x1 = (int)(0.5 + rad2ustep * mt->xw1 + m->a_off);
  x2 = (int)(0.5 + rad2ustep * mt->xw2 + m->b_off);
#else
  x1 = (int)(0.5 + rad2ustep * mt->xw1);
  x2 = (int)(0.5 + rad2ustep * mt->xw2);
#endif
  fprintf(m->pxmc, "GA:%d\nGB:%d\nRA:\nRB:\n", x1, x2);
  fprintf(stderr, "GA:%d GB:%d RA: RB:\n", x1, x2);
  fflush(m->pxmc);
  do {
    if (!(fgets(s, sizeof(s), m->pxmc))) {
      return -1;
    }
    fprintf(stderr, "g> %s", s);
    if (strncmp(s, "RA!", 3) == 0) {
      flag |= 0x1;
    }
    else if (strncmp(s, "RB!", 3) == 0) {
      flag |= 0x2;
    }
    else {
      //return -1;
    }
  } while (flag != 0x3);
  get_position(m);
  return 0;
}

void *get_in_addr(struct sockaddr *sa) {
  if (sa->sa_family == AF_INET) {
    return &(((struct sockaddr_in*)sa)->sin_addr);
  }
  return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

int reply(FILE *f, const char *challenge, const char *response) {
  char s[64];
  for (;;) {
    if (!fgets(s, sizeof(s), f)) {
      return -1;
    }
    if (strncmp(challenge, s, strlen(challenge)) == 0) {
      fprintf(stderr, "=> %s", s);
      break;
    }
    fprintf(stderr, "?!> |%s|", s);
  }
  fprintf(f, "%s\n", response);
  fflush(f);
  return 0;
}

int mount_connect(mount_t *m, const char *dev_name) {
  /*
  int fd = open(dev_name, O_RDWR | O_SYNC);
  if (fd == -1) {
    return -1;
  }
  m->pxmc = fdopen(fd, "a+");
  */
  int sockfd;  
  struct addrinfo hints, *servinfo, *p;
  int rv;
  char s[INET6_ADDRSTRLEN];

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  if ((rv = getaddrinfo(dev_name, "23", &hints, &servinfo)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    return 1;
  }
  // loop through all the results and connect to the first we can
  for(p = servinfo; p != NULL; p = p->ai_next) {
    if ((sockfd = socket(p->ai_family, p->ai_socktype,
                         p->ai_protocol)) == -1) {
      perror("client: socket");
      continue;
    }
    if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
      close(sockfd);
      perror("client: connect");
      continue;
    }
    break;
  }
  if (p == NULL) {
    fprintf(stderr, "client: failed to connect\n");
    return 2;
  }
  inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
            s, sizeof s);
  fprintf(stderr, "client: connecting to %s\n", s);
  freeaddrinfo(servinfo);
  
  m->pxmc = fdopen(sockfd, "a+");
  if (!m->pxmc) {
    return -1;
  }
  if (reply(m->pxmc, "login:", "rocon") ||
      reply(m->pxmc, "password:", "pikron") ||
      reply(m->pxmc, "\r#ROCON", "ECHO:0")) {
    return -1;
  }
  
  fprintf(m->pxmc,
	  "ECHO:0\n"
	  "RELEASE:\n"
	  "REGMODEA:8\n"
	  "REGMODEB:8\n"
	  "REGMODEC:8\n"
	  "REGOUTMAP:\n"

	  "REGCURDPA:150\n"
	  "REGCURDIA:6000\n"
	  "REGCURQPA:150\n"
	  "REGCURQIA:6000\n"
	  "REGMEA:6000\n"
	  "REGCURHOLDA:200\n"
	  "REGMSA:%u\n"
	  "PURGEA:\n"
	  "REGPTIRCA:%u\n"
	  "REGPTPERA:1\n"
	  "REGPTMARKA:0\n"
	  "REGMODEA:8\n"
	  "PWMA:0\n"
	  "REGACCA:1\n"
	  "SPDA:0\n"

	  "REGCURDPB:150\n"
	  "REGCURDIB:6000\n"
	  "REGCURQPB:150\n"
	  "REGCURQIB:6000\n"
	  "REGMEB:6000\n"
	  "REGCURHOLDB:200\n"
	  "REGMSB:%u\n"
	  "PURGEB:\n"
	  "REGPTIRCB:%u\n"
	  "REGPTPERB:1\n"
	  "REGPTMARKB:0\n"
	  "REGMODEB:8\n"
	  "PWMB:0\n"
	  "REGACCB:1\n"
	  "SPDB:0\n",
	  MAX_V_GOTO, LXR_USTEPS, MAX_V_GOTO, LXR_USTEPS
	  );
  // REGMEx ~ 1/supply voltage, REGMSx, REGPTIRCx ~ LXR_USTEPS
  fflush(m->pxmc);
  return 0;
}

int mount_close(mount_t *m) {
  fprintf(m->pxmc,
	  "SPDA:0\n"
	  "SPDB:0\n");
  fflush(m->pxmc);
  fclose(m->pxmc);
  return 0;
}
		  
static mount_t *_mount = NULL;

void shutdown_exit() {
  mount_t *m = _mount;
  if (m && m->pxmc) {
    m->finish = 1;
  }
  fprintf(stderr, "== SHUTDOWN ==\n");
}

void shutdown_sig(int sig_no) {
  shutdown_exit();
}

int main(int argc, char *argv[]) {
  mount_t mount = {
    /*
    .cmd_goto = 0, .cmd_save_point = 0, .cmd_calibrate = 0,
    .finish = 0, .stopped = 0, .socket_connected = 0,
    .ra = 0.0, .dec = 0.0,
    */
    .finish = 0,
    .a_off = 0.0, .b_off = 0.0,
    .s1 = 0.0, .s2 = 0.0,
    .js_v1 = 0.0, .js_v2 = 0.0, .js_cmd = 0,
  };
  traj_t tr;
  pthread_t joy_th, socket_th;

  _mount = &mount;
  atexit(shutdown_exit);
  struct sigaction sig_act;
  sig_act.sa_handler = shutdown_sig;
  sigemptyset(&sig_act.sa_mask);
  sig_act.sa_flags = 0;
  sigaction(SIGINT,  &sig_act, NULL);
  sigaction(SIGTERM, &sig_act, NULL);
  //sigaction(SIGQUIT, &sig_act, NULL);
  sigaction(SIGHUP,  &sig_act, NULL);

  /*
  FILE *logf = fopen("lxrmount.log", "a");
  if (!logf) {
    perror("fopen");
    return -1;
  }
  */

  mount.points_file = fopen("mountpt.dat", "a");
  if (!mount.points_file) {
    perror("fopen");
    return -1;
  }
  fprintf(mount.points_file, "\n");

#ifdef JOY
  pthread_create(&joy_th, NULL, js_control, &mount);
#endif
 
  load_abs_offset("mountab.off", &mount.a_off, &mount.b_off);		
  mount_connect(&mount, argv[1]);

  clock_gettime(CLOCK_REALTIME, &local_t0);
  open_ab_trajectory(&tr, argv[2]);
  get_position(&mount);

  /*
  // random 2 bits -- debug
  mount.motion.xw1 = 10.0 * deg2rad;
  mount.motion.xw2 = 10.0 * deg2rad;
  if (local_t0.tv_nsec & 0x1) {
    mount.motion.xw1 = -mount.motion.xw1;
  }
  if (local_t0.tv_nsec & 0x2) {
    mount.motion.xw2 = -mount.motion.xw2;
  }
  */
  get_ab_trajectory_pv(&tr, &mount.motion.t,
			 &mount.motion.xw1, &mount.motion.xw2,
			 &mount.motion.vw1, &mount.motion.vw2);
  motion_goto(&mount);
  
  struct timespec t_period = {0, 100000000 /*0.1s*/}, t_next;
  while (!mount.finish) {
    TS_ADD(&t_next, &mount.motion.t, &t_period);
    while (clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &t_next, NULL) == EINTR);
    
    motion_step(&mount);
    get_ab_trajectory_pv(&tr, &mount.motion.t,
			 &mount.motion.xw1, &mount.motion.xw2,
			 &mount.motion.vw1, &mount.motion.vw2);

    printf("%+9.4f\t%+9.4f\t%+9.4f\t%+9.4f\n",
	   rad2deg*mount.motion.xw1, rad2deg*(mount.motion.x1 - mount.motion.xw1),
	   rad2deg*mount.motion.xw2, rad2deg*(mount.motion.x2 - mount.motion.xw2));
    //rad2deg*mount.a_off, rad2deg*mount.b_off);
    fflush(stdout);

#ifdef JOY
    mount.a_off += (0.2*M_PI/180.0)*mount.js_v1; /* 2deg/s max */
    mount.b_off += (0.2*M_PI/180.0)*mount.js_v2;
#endif
  }
  mount_close(&mount);
  fprintf(stderr, "Exit.\n");
  
  return 0;
}
