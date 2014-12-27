/*
 * Copyright (C) 2014 Wilds
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdlib.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <getopt.h>

#include "config.h"
#include "avrspi_commands.h"
#include "flightlog.h"
#include "mpu.h"

#include "routines.h"
#include "msg.h"

#include <string.h>
#include <assert.h>

int ret;
int err = 0;
volatile int stop = 0;

int avr_s[256];

unsigned long flight_time = 0;

int trim[3] = {0, 0, 0}; //in degrees * 1000
int mode = 0;
int rec_setting = 0;

int alt_hold = 0;
int throttle_hold = 0;
int throttle_target = 0;

int sock = 0;
char sock_path[256] = "127.0.0.1";
int portno = 1030;
struct sockaddr_in address;
struct hostent *server;

int ssock;
struct sockaddr_in saddress;
int sportno = 1032;
char data[256];

int verbose = 0;

int yprt[4] = {0, 0, 0, 0};

long dt_ms = 0;
static struct timespec ts, t1, t2, t3, *dt, lastPacketTime;


#define MSG_SIZE 3
#define QUEUE_SIZE 20 * MSG_SIZE
unsigned char queue[QUEUE_SIZE];
unsigned int queue_c = 0; //number of messages

int queueMsg(int t, int v) {
    if (queue_c >= QUEUE_SIZE - MSG_SIZE) {
        printf("Error: Queue full. Need to flush!\n");
        return -1;
    }
    queue[MSG_SIZE * queue_c] = t;
    packi16(queue + MSG_SIZE * queue_c + 1, v);

    queue_c++;
    return queue_c;
}

int sendQueue() {
    if (queue_c <= 0) return 0;
    if (write(sock, queue, MSG_SIZE * queue_c) < 0) {
        perror("writing");
        return -1;
    }
    queue_c = 0;
    return 0;
}

int sendMsg(int t, int v) {
    static unsigned char buf[3];
    buf[0] = t;
    packi16(buf + 1, v);
    if (write(sock, buf, 3) < 0) {
        perror("writing");
        return -1;
    }
    return 0;
}

char** str_split(char* a_str, const char a_delim) {
    char** result = 0;
    size_t count = 0;
    char* tmp = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    /* Count how many elements will be extracted. */
    while (*tmp) {
        if (a_delim == *tmp) {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);

    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;

    result = (char**) malloc(sizeof (char*) * count);

    if (result) {
        size_t idx = 0;
        char* token = strtok(a_str, delim);

        while (token) {
            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
        assert(idx == count - 1);
        *(result + idx) = 0;
    }

    return result;
}

const char * handle_packet(char * data) {

    char** tokens = str_split(data, ' ');

    if (!tokens)
        return "KO";

    clock_gettime(CLOCK_REALTIME, &lastPacketTime);

    if (strcmp(tokens[0], "hello") == 0) {

    } else if (strcmp(tokens[0], "rcinput") == 0) {
        yprt[0] = atoi(tokens[2]);
        yprt[1] = atoi(tokens[3]);
        yprt[2] = atoi(tokens[4]);
        yprt[3] = atoi(tokens[1]);

        dt = TimeSpecDiff(&lastPacketTime, &t3);
        dt_ms = dt->tv_sec * 1000 + dt->tv_nsec / 1000000;
        if (dt_ms < 50) {
            //mssleep(50 - dt_ms);
            return "FLOOD"; //do not flood AVR with data - will cause only problems; each loop sends 4 msg; 50ms should be enough for AVR to consume them
        }
        t3 = lastPacketTime;

        queueMsg(COMMAND_SET_YAW, yprt[0]);
        queueMsg(COMMAND_SET_PITCH, yprt[1]);
        queueMsg(COMMAND_SET_ROLL, yprt[2]);
        queueMsg(COMMAND_SET_THROTTLE, yprt[3]);
        sendQueue();
    } else if (strcmp(tokens[0], "heartbeat") == 0) {

    } else if (strcmp(tokens[0], "flymode") == 0) {
        if (strcmp(tokens[1], "stable"))
            sendMsg(COMMAND_SET_FLY_MODE, PARAMETER_SET_FLY_MODE_STABLE);
        else if (strcmp(tokens[1], "acro"))
            sendMsg(COMMAND_SET_FLY_MODE, PARAMETER_SET_FLY_MODE_ACRO);
        else
            sendMsg(COMMAND_SET_FLY_MODE, atoi(tokens[1]));

    } else if (strcmp(tokens[0], "altitudeholderenabled") == 0) {
        sendMsg(COMMAND_SET_ALTITUDE_HOLD, atoi(tokens[1])); // 0 or 1

    } else if (strcmp(tokens[0], "altitudeholdermod") == 0) {
        sendMsg(COMMAND_INCREMENT_ALTITUDE_TARGET, atoi(tokens[1]));
    } else if (strcmp(tokens[0], "altitudeholder") == 0) {
        sendMsg(COMMAND_SET_ALTITUDE, atoi(tokens[1]));
    } else if (strcmp(tokens[0], "tm") == 0) {
        sendMsg(COMMAND_TESTMOTOR_FL, atoi(tokens[1]));
        sendMsg(COMMAND_TESTMOTOR_BL, atoi(tokens[2]));
        sendMsg(COMMAND_TESTMOTOR_FR, atoi(tokens[3]));
        sendMsg(COMMAND_TESTMOTOR_BR, atoi(tokens[4]));
        sendQueue();

    } else if (strcmp(tokens[0], "takepicture") == 0) {
        char str[128];
        //take picture
        memset(str, '\0', 128);
        sprintf(str, "/usr/local/bin/picsnap.sh %05d ", config.cam_seq++);
        ret = system(str);
    } else if (strcmp(tokens[0], "vidsnap") == 0) {
        char str[128];
        memset(str, '\0', 128);
        sprintf(str, "/usr/local/bin/vidsnap.sh %05d ", config.cam_seq++);
        ret = system(str);
    } else if (strcmp(tokens[0], "querystatus") == 0) {
        char * resp = (char*) malloc(sizeof (char) * 255);
        memset(resp, '\0', 255);
        // yaw pitch roll altitudetarget altitude
        snprintf(resp, 255, "status %2.2f %2.2f %2.2f %i %i", avr_s[LOG_GYRO_YAW] / 100.0f, avr_s[LOG_GYRO_PITCH] / 100.0f, avr_s[LOG_GYRO_ROLL] / 100.0f, avr_s[LOG_ALTITUDE_HOLD_TARGET], avr_s[LOG_ALTITUDE]);
        return resp;
    }

    /*        case 0:
            if (js[0].yprt[3] < config.rec_t[2]) {
                flog_save();
                config_save();
                sync();
                fflush(NULL);
            }
            break;
        case 3:
            stop = 1;
            sendMsg(COMMAND_GET, PARAMETER_GET_RESET_AVR); //send reset event before exiting
            break;
        case 12:
            if (rec_setting) rec_setting = 0;
            else rec_setting = 1;
            rec_config(js, config.rec_t, config.rec_ypr[rec_setting]);
            break;
     */
    return "OK";
}

void recvMsgs() {
    static int sel = 0, i = 0, ret = 0;
    static unsigned char buf[4];
    static struct s_msg m;

    static fd_set fds;
    static struct timeval timeout;

    do {
        FD_ZERO(&fds);
        FD_SET(sock, &fds);
        FD_SET(ssock, &fds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 0; // If the timeout argument points to an object of type struct timeval whose members are 0, select() does not block
        int max_fd = ssock > sock ? ssock : sock;
        sel = select(max_fd + 1, &fds, NULL, NULL, &timeout);
        if ((sel < 0) && (errno != EINTR)) {
            perror("select");
            stop = 1;
        } else if (sel && !stop && FD_ISSET(sock, &fds)) {
            ret = read(sock, buf + i, 4 - i);
            if (ret < 0) {
                perror("reading");
                stop = 1;
            } else {
                i += ret;
                if (i == 4) {
                    if (buf[0] == 1) {
                        if (verbose) printf("Disconnect request.\n");
                        stop = 1;
                    } else {
                        m.t = buf[1];
                        m.v = unpacki16(buf + 2);
                        avr_s[m.t] = m.v;
                        i = 0;
                    }
                }
            }
        } else if (sel && !stop && FD_ISSET(ssock, &fds)) {
            /*
            int t = accept(ssock, 0, 0);
            if (t < 0) {
                perror("accept");
                continue;
            }

            if (verbose) printf("Client connected - sending data & disconnecting: %u\n", obuf_size);
            send_msgs(t);
            close(t);
             */
            sockaddr_in remoteAddr;
            int received_bytes = -1;
            //sockaddr_in from;
            socklen_t fromLength = sizeof (remoteAddr);

            received_bytes = recvfrom(ssock, data, sizeof (data), 0, (sockaddr*) & remoteAddr, &fromLength);
            if (received_bytes == -1) {
                perror("received bytes = -1 \n");
            } else {
                data[received_bytes] = 0;

                if (verbose >= 2) {
                    printf("Received: %s\n", data);
                }

                const char* resp = handle_packet(data);

                if (verbose >= 2) {
                    printf("Send response: %s\n", resp);
                }

                if (sendto(ssock, resp, strlen(resp), 0, (sockaddr*) & remoteAddr, (socklen_t) sizeof (remoteAddr)) == -1) {
                    perror(strerror(errno));
                }
            }
        }
    } while (!stop && sel && ret > 0); //no error happened; select picked up socket state change; read got some data back
}

void sendConfig() {
    sendMsg(COMMAND_SET_LOG_MODE, config.log_t);

    sendMsg(COMMAND_SET_FLY_MODE, mode);

    int gyro_orientation = inv_orientation_matrix_to_scalar(config.gyro_orient);
    sendMsg(COMMAND_SET_ORIENTATION, gyro_orientation);

    sendMsg(COMMAND_SET_MPU_ADDRESS, config.mpu_addr);

    sendMsg(COMMAND_SET_MOTOR_PWM_MIN, config.rec_t[0]);
    sendMsg(COMMAND_SET_MOTOR_PWM_INFLIGHT_THREASHOLD, config.rec_t[2]);

    for (int i = 0; i < 4; i++)
        sendMsg(COMMAND_SET_MOTOR_PIN_FL + i, config.motor_pin[i]);

    sendMsg(COMMAND_SET_BC, config.baro_f);
    //PIDS
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 5; j++) {
            sendMsg(BASE_COMMAND_SET_PID_RATE + i * 10 + j, config.r_pid[i][j]);
            sendMsg(BASE_COMMAND_SET_PID_STABLE + i * 10 + j, config.s_pid[i][j]);
        }

    for (int i = 0; i < 5; i++) {
        sendMsg(BASE_COMMAND_SET_PID_ACCEL + i, config.accel_pid[i]);
        sendMsg(BASE_COMMAND_SET_PID_ALT + i, config.alt_pid[i]);
        sendMsg(BASE_COMMAND_SET_PID_VZ + i, config.vz_pid[i]);
    }

    sendMsg(COMMAND_SET_PID_ACRO_P, config.a_pid[0]);

    int config_done = 1;
    sendMsg(COMMAND_GET, config_done);
    mssleep(1000);

}

void reset_avr() {
    //ensure AVR is properly rebooted
    while (avr_s[255] != 1 && !stop) { //once rebooted AVR will report status = 1;
        avr_s[255] = -1;
        sendMsg(COMMAND_GET, PARAMETER_GET_RESET_AVR);
        mssleep(1500);
        recvMsgs();
    }
}

void catch_signal(int sig) {
    printf("signal: %i\n", sig);
    stop = 1;
}

void log4() {
    flog_push(5,
            (float) t2.tv_sec - ts.tv_sec
            , (float) flight_time
            , avr_s[18] / 1.f, avr_s[19] / 1.f, avr_s[20] / 1.f
            );
}

void log4_print() {
    printf("T: %li\ttarget_alt: %i\talt: %i\tvz: %i\tp_accel: %i\n",
            flight_time, avr_s[18], avr_s[19], avr_s[20], avr_s[21]);
}

void log100_print() {
    printf("T: %li\tvz: %i\tpos_err: %i\taccel_err: %i\tpid_alt: %i\tpid_vz: %i\tpid_accel: %i\n",
            flight_time, avr_s[100], avr_s[101], avr_s[102], avr_s[103], avr_s[104], avr_s[105]);
}

void log3() {
    flog_push(10,
            (float) t2.tv_sec - ts.tv_sec
            , (float) flight_time
            , avr_s[4] / 100.0f, avr_s[5] / 100.0f, avr_s[6] / 100.0f, avr_s[7] / 100.0f
            , avr_s[8] / 1.f, avr_s[9] / 1.f, avr_s[10] / 1.f
            , avr_s[11] / 1.f
            );
}

void log3_print() {
    printf("T: %li\tfl: %i\tbl: %i\tfr: %i\tbr: %i\tqy: %f\tqp: %f\tqr: %f\tyt: %f\ty: %f\n",
            flight_time, avr_s[8], avr_s[9], avr_s[10], avr_s[11],
            avr_s[4] / 100.0f, avr_s[5] / 100.0f, avr_s[6] / 100.0f, avr_s[7] / 100.0f, (avr_s[7] - avr_s[4]) / 100.0f
            );
    printf("T: %li\tfl: %i\tbl: %i\tfr: %i\tbr: %i\n",
            flight_time, avr_s[10], avr_s[11], avr_s[12], avr_s[13]);
}

void log2() { //gyro & quat
    flog_push(9,
            (float) t2.tv_sec - ts.tv_sec
            , (float) flight_time
            , avr_s[1] / 100.0f, avr_s[2] / 100.0f, avr_s[3] / 100.0f
            , avr_s[8] / 1.f, avr_s[9] / 1.f, avr_s[10] / 1.f
            , avr_s[11] / 1.f
            );
}

void log2_print() {
    printf("T: %li\tgy: %2.2f\tgp: %2.2f\tgr: %2.2f\tfl: %i\tbl: %i\tfr: %i\tbr: %i\n",
            flight_time, avr_s[1] / 100.0f, avr_s[2] / 100.0f, avr_s[3] / 100.0f, avr_s[8], avr_s[9], avr_s[10], avr_s[11]);
}

void log1() {
    flog_push(8,
            (float) t2.tv_sec - ts.tv_sec
            , (float) flight_time
            , avr_s[12] / 1000.0f, avr_s[13] / 1000.0f, avr_s[14] / 1000.0f
            , avr_s[15] / 1000.0f, avr_s[16] / 1000.0f, avr_s[17] / 1000.0f
            );
}

void log1_print() {
    printf("T: %li\tax: %2.3f\t\ay: %2.3f\t\az: %2.3f\tbx: %2.3f\tby: %2.3f\tbz: %2.3f\n",
            flight_time, avr_s[12] / 1000.0f, avr_s[13] / 1000.0f, avr_s[14] / 1000.0f, avr_s[15] / 1000.0f, avr_s[16] / 1000.0f, avr_s[17] / 1000.0f);
}

void out_log() {
    switch (config.log_t) {
        case 0: break;
        case 1:
            log1();
            if (verbose == 2) log1_print();
            break;
        case 2:
            log2();
            if (verbose == 2) log2_print();
            break;
        case 3:
            log3();
            if (verbose == 2) log3_print();
            break;
        case 4:
            log4();
            if (verbose == 2) log4_print();
            break;
        case 100:
            if (verbose == 2) log100_print();
            break;
        default: break;
    }
}

unsigned long k = 0;

void init() {
    //feeds all config and waits for calibration
    int prev_status = avr_s[255];
    avr_s[255] = 0;

    reset_avr();

    if (verbose) printf("Initializing RPiCopter...\n");
    while (avr_s[255] != 5 && !stop) {
        sendMsg(255, 0); //query status
        sendMsg(255, 1); //crc status
        mssleep(350);
        recvMsgs();
        if (prev_status != avr_s[255]) {
            if (verbose) {
                if (avr_s[255] == -1) printf("Waiting for AVR status change.\n");
                else printf("AVR Status: %i\n", avr_s[255]);
            }
            prev_status = avr_s[255];
        }

        if (avr_s[254] != 0) { //AVR reported crc error when receiving data
            printf("AVR reports CRC errors %i\n", avr_s[254]);
            reset_avr();
            avr_s[254] = 0;
            avr_s[255] = 0;
        } else switch (avr_s[255]) {
                case -1: break;
                case 0: reset_avr();
                    break; //AVR should boot into status 1 so 0 means something wrong
                case 1: sendConfig();
                    mssleep(1000);
                    avr_s[255] = -1;
                    sendMsg(255, 2);
                    break;
                case 2: break; //AVR should arm motors and set status to 3
                case 3: break; //AVR is initializing MPU
                case 4: break; //AVR is calibration gyro
                case 5: break;
                case 255: printf("Gyro calibration failed!\n");
                    reset_avr();
                    avr_s[255] = 0;
                    break; //calibration failed
                default: printf("Unknown AVR status %i\n", avr_s[255]);
                    break;
            }
    }
}

void loop() {
    clock_gettime(CLOCK_REALTIME, &t3);
    lastPacketTime = ts = t1 = t2 = t3;
    if (verbose) printf("Starting main loop...\n");

    while (!err && !stop) {

        if (alt_hold && abs(yprt[3]) > (config.rec_t[1] - 50)) {
            alt_hold = 0;
            throttle_hold = 0;
            sendMsg(COMMAND_SET_ALTITUDE_HOLD, alt_hold);
        }


        clock_gettime(CLOCK_REALTIME, &t2);
        dt = TimeSpecDiff(&t2, &t1);
        dt_ms = dt->tv_sec * 1000 + dt->tv_nsec / 1000000;

        /*
        if (dt_ms < 50) {
            mssleep(50 - dt_ms);
            continue; //do not flood AVR with data - will cause only problems; each loop sends 4 msg; 50ms should be enough for AVR to consume them
        }
        */
        t1 = t2;

        if (alt_hold || yprt[3] > config.rec_t[2])
            flight_time += dt_ms;

        out_log();

        /*
        if (throttle_hold) {
            yprt[3] = throttle_target;
        }
         */

        // if no packet for 5sec maybe lost connection -> then try to land
        clock_gettime(CLOCK_REALTIME, &t2);
        dt = TimeSpecDiff(&t2, &lastPacketTime);
        dt_ms = dt->tv_sec * 1000 + dt->tv_nsec / 1000000;
        if (dt_ms > 2500) {
            printf("lost connection\n");
            queueMsg(COMMAND_SET_YAW, 0);
            queueMsg(COMMAND_SET_PITCH, 0);
            queueMsg(COMMAND_SET_ROLL, 0);
            queueMsg(COMMAND_SET_THROTTLE, config.rec_t[2]);
            sendQueue();
            lastPacketTime = t2;
        }

        recvMsgs();
    }
}

void print_usage() {
    printf("-v [level] - verbose mode\n");
    printf("-a [addr] - address to connect to (defaults to 127.0.0.1)\n");
    printf("-p [port] - port to connect to (default to 1030)\n");
    printf("-l [port] port to listen on (defaults to 1032)\n");
}

int main(int argc, char **argv) {

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    int option;
    verbose = 0;
    while ((option = getopt(argc, argv, "v:a:p:l:")) != -1) {
        switch (option) {
            case 'v': verbose = atoi(optarg);
                break;
            case 'a': strcpy(sock_path, optarg);
                break;
            case 'p': portno = atoi(optarg);
                break;
            case 'l': sportno = atoi(optarg);
                break;
            default:
                print_usage();
                return -1;
        }
    }

    for (int i = 0; i < 256; i++)
        avr_s[i] = 0;

    if (verbose) printf("Opening socket...\n");

    /* Create socket on which to send. */
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("opening socket");
        exit(1);
    }
    server = gethostbyname(sock_path);
    if (server == NULL) {
        fprintf(stderr, "ERROR, no such host\n");
        return -1;
    }
    bzero((char *) &address, sizeof (address));
    address.sin_family = AF_INET;
    bcopy((char *) server->h_addr, (char *) &address.sin_addr.s_addr, server->h_length);
    address.sin_port = htons(portno);

    if (connect(sock, (struct sockaddr *) &address, sizeof (struct sockaddr_in)) < 0) {
        close(sock);
        perror("connecting socket");
        exit(1);
    }
    /*

            //set non-blocking
               ret = fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
               if (ret == -1){
               perror("calling fcntl");
               return -1;
               }
     */
    if (verbose) printf("Connected to avrspi\n");



    ret = config_open("/var/local/rpicopter.config");
    if (ret < 0) {
        printf("Failed to initiate config! [%s]\n", strerror(ret));
        return -1;
    }

    ret = flog_open("/rpicopter");
    if (ret < 0) {
        printf("Failed to initiate log! [%s]\n", strerror(err));
        return -1;
    }

    /* Create socket to listen */
    ssock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ssock <= 0) {
        perror("opening server socket");
        exit(EXIT_FAILURE);
    }

    //int optval = 1;
    //setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

    bzero((char *) &saddress, sizeof (saddress));
    saddress.sin_family = AF_INET;
    saddress.sin_addr.s_addr = INADDR_ANY;
    saddress.sin_port = htons(sportno);

    //Binding to desired port number
    if (bind(ssock, (struct sockaddr *) &saddress, sizeof (struct sockaddr_in))) {
        perror("binding stream socket");
        exit(EXIT_FAILURE);
    }

    //setting Socket to non blocking mode
    int nonBlocking = 1;
    if (fcntl(ssock, F_SETFL, O_NONBLOCK, nonBlocking) == -1) {
        perror("failed to set non-blocking socket");
        exit(EXIT_FAILURE);
    }

    init();
    loop();
    close(sock);
    close(ssock);
    if (verbose) printf("Closing.\n");
    return 0;
}

