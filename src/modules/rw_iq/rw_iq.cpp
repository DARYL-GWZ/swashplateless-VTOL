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
#include <uORB/topics/iq.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/iq_motor_angle.h>
#include <uORB/topics/battery_status.h>
#include <systemlib/mavlink_log.h>
#include "generic_interface.hpp"
#include "propeller_motor_control_client.hpp"
#include "communication_interface.h"
#include "packet_finder.h"
#include "byte_queue.h"
#include "bipbuffer.h"
#include "brushless_drive_client.hpp"
// ORB_DEFINE(rw_uart_topic, struct rw_uart_topic_s);

GenericInterface com;
PropellerMotorControlClient prop(0);
BrushlessDriveClient dri(0);
/*static int ReceiveMessages();*/
extern "C" __EXPORT int rw_iq_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< px4_uorb_subs exit flag */
static bool thread_kk = 0;		/**< px4_uorb_subs status flag */
static int read_uart_task;				/**< Handle of px4_uorb_subs task */
static int set_uart_baudrate(const int fd, unsigned int baud);
int rw_iq_ss_main(int argc, char *argv[]);
static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: uart_read {start|stop|status} [-p <additional params>]\n\n");
}

int set_uart_baudrate(const int fd, unsigned int baud)
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

int rw_iq_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {



/*        if (!thread_kk) {
            warnx("uart already running\n");
            *//* this is not an error *//*
            return 0;
        }*/


        thread_should_exit = false;//定义一个守护进程*/
        read_uart_task = px4_task_spawn_cmd("rw_iq",
                                            SCHED_DEFAULT,
                                            SCHED_PRIORITY_DEFAULT,
                /*SCHED_PRIORITY_SLOW_DRIVER,*/
                /* SCHED_PRIORITY_MAX - 50,*/
                                            3600,//堆栈分配大小
                                            rw_iq_ss_main,
                                            (argv) ? (char * const *)&argv[2] : (char * const *)NULL);//正常命令形式为serv_sys_uart start /dev/ttyS2
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
int rw_iq_ss_main(int argc, char *argv[]){

    uint8_t  buff[64] ;
    uint8_t communication_length;
    /*uint8_t length;*/
    uint8_t communication_buffer[64];
    int fd =open("/dev/ttyS0", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        err(1, "failed to open port " );
        return -1;
    }

    if(false == set_uart_baudrate(fd,115200)){
        printf("[JXF]set_uart_baudrate is failed\n");
        return -1;
    }

     double Speed = 300.0;   // speed (in rad/s)
     double pitchCommand = Speed*0.3;
     double rollCommand = 0.0;
    static double angle =0.0;
    static double addangle =3.0;//-3~3
    static double velocity_to_set = 0.0;

    thread_kk = 1;
   /* struct iq_s iqangle;
    memset(&iqangle, 0, sizeof(iqangle));
    orb_advert_t iq_angle_pub = orb_advertise(ORB_ID(iq), &iqangle);*/
/*    struct iq_s iqangle;
    memset(&iqangle, 0, sizeof(iqangle));
    orb_advert_t iq_angle_pub = orb_advertise(ORB_ID(iq), &iqangle);
    iqangle.angle=angle*100;*/
    while(!thread_should_exit){

        prop.ctrl_velocity_.set(com, velocity_to_set);
        dri.obs_angle_.get(com);
        if (com.GetTxBytes(buff,communication_length)) {

            write(fd, &buff, communication_length);
        }

       int  readbyte=read(fd,&communication_buffer,50);


        com.SetRxBytes(communication_buffer,readbyte);
        uint8_t *rx_data; // temporary pointer to received type+data bytes
        uint8_t rx_length; // number of received type+data bytes
        while(com.PeekPacket(&rx_data,&rx_length))
        {
            dri.ReadMsg(rx_data,rx_length);
            com.DropPacket();
        }

        if(dri.obs_angle_.IsFresh()) {
            angle=dri.obs_angle_.get_reply();
            struct battery_status_s iqangle;
            memset(&iqangle, 0, sizeof(iqangle));
            orb_advert_t iq_angle_pub = orb_advertise(ORB_ID(iq), &iqangle);
            iqangle.angle=angle;
            orb_publish(ORB_ID(iq), iq_angle_pub, &iqangle);
        /*    printf("angle=%lf\n",angle*100);
            printf(" angle=%f speed =%f\n", angle*100,velocity_to_set);*/
        }
        /*addangle=addangle+0.01;*/
        velocity_to_set = Speed + (pitchCommand*cos(angle+addangle))+(rollCommand*sin(angle+addangle));
    }
    warnx("[uart_read] exiting.\n");
    thread_kk = 0;
    int dd = close(fd);
    printf("close ss: %d\n", dd);
    return 0;
}


