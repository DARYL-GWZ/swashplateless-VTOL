//
// Created by gwz on 2020/9/29.
//
/*
 * 串口读取函数
 */
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <systemlib/err.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sched.h>
#include <fcntl.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/iq.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/actuator_controls.h>
#include "generic_interface.hpp"
#include "propeller_motor_control_client.hpp"
#include "communication_interface.h"
#include "packet_finder.h"
#include "byte_queue.h"
#include "bipbuffer.h"
#include "brushless_drive_client.hpp"
#include<cmath>
#include <sys/time.h>
#include "serial_interface_client.hpp"
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include <stdlib.h>
#include "DriverFramework.hpp"
#include <sys/ioctl.h>
/*#include "DevObj.hpp"*/
#include "DevMgr.hpp"
#include "SyncObj.hpp"
/*#include "WorkItems.hpp"*/
#include <px4_time.h>
/*#include <unistd.h>*/
#define df_clock_gettime px4_clock_gettime
GenericInterface gwz;
PropellerMotorControlClient prope(0);
BrushlessDriveClient dric(0);
/*static int ReceiveMessages();*/
extern "C" __EXPORT int iq_pwm_main(int argc, char *argv[]);
static bool thread_should_exit = true;		/**< px4_uorb_subs exit flag */
static bool thread_kk = 0;		/**< px4_uorb_subs status flag */
static int read_uart_task;				/**< Handle of px4_uorb_subs task */
static int set_uart_pwm_baudrate(const int fd, unsigned int baud);
int iq_pwm_ss_main(int argc, char *argv[]);
/*int absoluteTime(struct timespec &ts)
{
// On NuttX we use CLOCK_REALTIME anyway.
// On macOS we can only use CLOCK_REALTIME unless we're using the lockstep
// scheduler.
#if defined(__DF_NUTTX) || (defined(__DF_APPLE) && !defined(ENABLE_LOCKSTEP_SCHEDULER))

    #ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif
	// CLOCK_MONOTONIC not available on NuttX or OSX
	return df_clock_gettime(CLOCK_REALTIME, &ts);
#else

#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif
    return df_clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
}
static uint64_t TsToAbstime(struct timespec *ts)
{
    uint64_t result = (uint64_t)(ts->tv_sec) * 1000000UL;
    result += ts->tv_nsec / 1000;

    return result;
}*/
static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: uart_read {start|stop|status} [-p <additional params>]\n\n");
}
int set_uart_pwm_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

/* 以新的配置填充结构体 */


/* 设置某个选项，那么就使用"|="运算，
     * 如果关闭某个选项就使用"&="和"~"运算
     * */

    tcgetattr(fd, &uart_config); // 获取终端参数


   /*clear ONLCR flag (which appends a CR for every LF)*/

    uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

/* 无偶校验，一个停止位 */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);// CSTOPB 使用两个停止位，PARENB 表示偶校验

/*    uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    uart_config.c_oflag &= ~OPOST;
    uart_config.c_cc[VTIME]=0;
    uart_config.c_cc[VMIN]=0;*/
/* 设置波特率 */

    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }
    // 设置与终端相关的参数，TCSANOW 立即改变参数
    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}
int iq_pwm_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {



       if (!thread_should_exit) {
            warnx("uart already running\n");
            /* this is not an error */
            return 0;
        }


        thread_should_exit = false;//定义一个守护进程*/
        read_uart_task = px4_task_spawn_cmd("iq_pwm",
                                            SCHED_DEFAULT,
                                            SCHED_PRIORITY_MAX - 5 ,
                /*SCHED_PRIORITY_SLOW_DRIVER,*/
                /* SCHED_PRIORITY_MAX - 50,*/
                                            5000,//堆栈分配大小
                                            iq_pwm_ss_main,
                                            (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        warnx("\tstop\n");
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_kk) {
            warnx("\trunning\n");
        }
        else {
            warnx("\tnot started\n");
        }

        return 0;
    }

    usage("unrecognized command");
    return 1;
}
/*struct timespec ts = {0, 0};*/
int iq_pwm_ss_main(int argc, char *argv[]){
    uint8_t  buff[64];
    uint8_t communication_buffer[64];
    uint8_t communication_length;
    memset(communication_buffer,0,64);
    int fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NONBLOCK);
//    int fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        err(1, "failed to open port 0 " );
        return -1;
    }

//   int fd3 = open("/dev/ttyS3", O_RDWR | O_NOCTTY | O_NONBLOCK);
//    if (fd3 < 0) {
//        err(1, "failed to open port 3 " );
//        return -1;*/
//    }
    if(false == set_uart_pwm_baudrate(fd,115200))
    {
        printf("[JXF]set_uart_baudrate is failed\n");
        return -1;
    }
/*    if(false == set_uart_pwm_baudrate(fd3,115200))
    {
        printf("[JXF]set_uart_baudrate is failed\n");
        return -1;
    }*/
/*    SerialInterfaceClient serial_interface(0);
    serial_interface.baud_rate_.gewz);
    serial_interface.baud_rate_.get(gwz,);*/
    double Speed = 0.0;

    static double velocity_to_set = 0.0;
    static double throttle_output_0 = 0.0;
    static double  pitch_output_0 = 0.0;
    static double  roll_output_0 = 0.0;
    /*static double bb = 0.0;*/
    double pitchCommand = 0.0;
    double rollCommand = 0.0;
    static double angle =0.0;
    static double velocity =0.0;
    static double addangle =0;
    thread_kk = 1;
    int _t_actuator_controls_0;
    struct actuator_controls_s  controls;
    _t_actuator_controls_0 = orb_subscribe(ORB_ID(actuator_controls_0));
    orb_set_interval(_t_actuator_controls_0, 0);
    bool updated;
/*    static uint64_t start = 0;
    static uint64_t start1 = 0;
    static uint64_t end = 0;*/
    struct iq_s iq;
    memset(&iq, 0, sizeof(iq));
    orb_advert_t att_pub = orb_advertise(ORB_ID(iq), &iq);
    while(!thread_should_exit){
       /* absoluteTime(ts);
        start=TsToAbstime(&ts);*/

        prope.ctrl_velocity_.set(gwz, velocity_to_set);
        dric.obs_angle_.get(gwz);
        dric.obs_velocity_.get(gwz);
        if (gwz.GetTxBytes(buff,communication_length))
        {
            write(fd, &buff, communication_length);
        }
        orb_check(_t_actuator_controls_0, &updated);
        if (updated) {
            orb_copy(ORB_ID(actuator_controls_0), _t_actuator_controls_0, &controls);
        }

        throttle_output_0 = controls.control[3];
        roll_output_0 = controls.control[0];
        pitch_output_0 = controls.control[1];

/*        printf("throttle_output_0 = %lf   ",throttle_output_0*100);
        printf("roll_output_0 = %lf    ",roll_output_0*100);
        printf("pitch_output_0 = %lf\n",pitch_output_0*100);*/

       if ( throttle_output_0 > 0.5)
       {
            Speed = (throttle_output_0-0.5) *1600;

            rollCommand = roll_output_0 *Speed*0.5;
           pitchCommand = pitch_output_0 *Speed*0.25;
       } else{
           pitchCommand = 0;
           rollCommand =0;
           Speed = 0;
       }
/*        absoluteTime(ts);
       start1=TsToAbstime(&ts);*/

        int readbyte = read(fd, &communication_buffer, 50);
//          int readbyte = 1;
         /* printf("readbyte = %d\n",readbyte);
*/
            if(readbyte>0)
            {
            gwz.SetRxBytes(communication_buffer, readbyte);
            uint8_t *rx_data; // temporary pointer to received type+data bytes
            uint8_t rx_length; // number of received type+data bytes

            while (gwz.PeekPacket(&rx_data, &rx_length)) {
                dric.ReadMsg(rx_data, rx_length);
                gwz.DropPacket();
            }

            if (dric.obs_angle_.IsFresh())
            {
                /*printf("angle=%d\n", (int)(angle / 3.14159 * 180 * 100));*/
                angle = dric.obs_angle_.get_reply();
                velocity = dric.obs_velocity_.get_reply();
            }
          }

        velocity_to_set = Speed + (pitchCommand * cos(angle + addangle)) + (rollCommand * sin(angle + addangle));
        iq.ob_speed=velocity / 2 / 3.14159 * 60;
        orb_publish(ORB_ID(iq), att_pub, &iq);
        /*int orb_stat(int att_pub, uint64_t *time)*/
/*       absoluteTime(ts);
         end= TsToAbstime(&ts);*/
        usleep(200);

        usleep(180-0.12*Speed);
/*       printf("velocity_to_set= %lf     ",velocity_to_set / 2 / 3.14159 * 60);
        printf("velocity_ob= %lf\n",velocity / 2 / 3.14159 * 60);*/
//        printf(" part1=%d, part2=%d, part3=%d\n",(int)(end-start),(int)(end-start1),(int)(start1-start));
//        printf(" part1=%d, part2=%d, part3=%d\n",(int)(start),(int)(start1),(int)(end));


    }

    warnx("code exiting.\n");
    thread_kk = 0;
    int dd = close(fd);
    printf("close ss: %d\n", dd);
    return 0;
}


