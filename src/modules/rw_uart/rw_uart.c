/* 
 * 串口读取函数
 * rw_uart.c 
 */
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>

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
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <systemlib/err.h>
//#include <systemlib/systemlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <systemlib/err.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>
#include <stdio.h>
#include <string.h>
#include <sched.h>
#include <px4_defines.h>
#include <px4_time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sched.h>
#include <uORB/uORB.h>
#include <uORB/topics/pm3901_with_tof.h>
#include <systemlib/mavlink_log.h>

// ORB_DEFINE(rw_uart_topic, struct rw_uart_topic_s);

__EXPORT int rw_uart_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< px4_uorb_subs exit flag */
static bool thread_kk = 0;		/**< px4_uorb_subs status flag */
static int read_uart_task;				/**< Handle of px4_uorb_subs task */
/*static int uart_init(char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);*/
int rw_uart_ss_main(int argc, char *argv[]);
static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: uart_read {start|stop|status} [-p <additional params>]\n\n");
}

/*
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

    */
/* 以新的配置填充结构体 *//*

    */
/* 设置某个选项，那么就使用"|="运算，
     * 如果关闭某个选项就使用"&="和"~"运算
     * *//*

    tcgetattr(fd, &uart_config); // 获取终端参数

    */
/* clear ONLCR flag (which appends a CR for every LF) *//*

    uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

    */
/* 无偶校验，一个停止位 *//*

    uart_config.c_cflag &= ~(CSTOPB | PARENB);// CSTOPB 使用两个停止位，PARENB 表示偶校验

     */
/* 设置波特率 *//*

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
*/


/*int uart_init(char * uart_name)
{
    printf("init code start %s\n");
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
    *//*Linux中，万物皆文件，打开串口设备和打开普通文件一样，使用的是open（）系统调用*//*
    // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    printf("Open the %s\n");
    return serial_fd;
}*/
int rw_uart_main(int argc, char *argv[])
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
        read_uart_task = px4_task_spawn_cmd("rw_uart",
                                            SCHED_DEFAULT,
                                        SCHED_PRIORITY_DEFAULT,
                /*SCHED_PRIORITY_SLOW_DRIVER,*/
                                           /* SCHED_PRIORITY_MAX - 50,*/
                                            2000,//堆栈分配大小
                                            rw_uart_ss_main,
                                            /*(argv) ? (char *const *) &argv[2] : (char *const *) NULL);*/
                                            NULL);

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
int rw_uart_ss_main(int argc, char *argv[]){
    warnx("\tmain function start =1\n");

    char buff[300] = "";
    /*
        GPS1:/dev/ttyS0
        TEL1:/dev/ttyS1
        TEL2:/dev/ttyS2
        TEL4:/dev/ttyS3
     */
/*    int uart_read = uart_init("/dev/ttyS2");*/
int fd =open("/dev/ttyS3", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        err(1, "failed to open port " );
        return -1;
    }
/*    if(false == uart_read)
        return -1;
    if(false == set_uart_baudrate(uart_read,9600)){
        printf("[JXF]set_uart_baudrate is failed\n");
        return -1;
    }*/

    printf("[JXF]uart init is successful\n");
    thread_kk = 1;
    while(!thread_should_exit){
        printf("   main cycle start\n");
        warnx("\tlen1\n");
        int len = read(fd, buff, 100);
        warnx("\tlen\n");
        printf(" len=%d\n",len);
        if (len > 0) {
            for (int i = 0; i < len; ++i) {
                printf("[JXF]uart init is already successful\n");
                putchar(buff[i]);
            }
        }
        warnx("\tthread_should_exit =1\n");
        /*printf("%s\n",buff);*/
        usleep(3000);
            }
    warnx("[uart_read] exiting.\n");
    thread_kk = 0;
    int dd = close(fd);
    printf("close stauts: %d\n", dd);
    return 0;
}
