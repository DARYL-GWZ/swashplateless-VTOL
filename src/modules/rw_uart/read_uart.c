/*
 * 串口读取函数
 * rw_uart.c
 */
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
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <stdio.h>
#include <string.h>
#include <sched.h>
#include <px4_defines.h>
#include <px4_time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <sched.h>

 __EXPORT int read_uart_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< px4_uorb_subs exit flag */
static bool thread_running = false;		/**< px4_uorb_subs status flag */
static int read_uart_task;				/**< Handle of px4_uorb_subs task / thread */
static int uart_init(char * uart_name);
/*static int set_uart_baudrate(const int fd, unsigned int baud);*/
int uart_read_thread_main(int argc, char *argv[]);
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

    tcgetattr(fd, &uart_config); // 获取终端参数

    /* clear ONLCR flag (which appends a CR for every LF) */
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
int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
    /*Linux中，万物皆文件，打开串口设备和打开普通文件一样，使用的是open（）系统调用*/
    // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        printf("failed to open port: %s\n", uart_name);
        return false;
    }
    printf("Open the %s\n",serial_fd);
    return serial_fd;
}
int read_uart_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        warnx("uart try\n");

        if (thread_running) {
            warnx("uart already running\n");
            /* this is not an error */
            return 0;
        }
        warnx("uart lsssssssss\n");
        thread_should_exit = false;//定义一个守护进程*/
        read_uart_task = px4_task_spawn_cmd("uart_read",
                                            SCHED_DEFAULT,
                           /*SCHED_PRIORITY_DEFAULT,//调度优先级*/
                           /*SCHED_PRIORITY_SLOW_DRIVER,*/
                                            SCHED_PRIORITY_MAX - 50,
                                            3600,//堆栈分配大小
                                            uart_read_thread_main,
                                            (argv) ? (char *const *) &argv[2] : (char *const *) NULL);

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
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
int uart_read_thread_main(int argc, char *argv[])
{


        /*char data = '0';*/
        char buff[300] = "";
        /*
         * TELEM1 : /dev/ttyS1
         * TELEM2 : /dev/ttyS2
         * GPS    : /dev/ttyS3
         * NSH    : /dev/ttyS5
         * SERIAL4: /dev/ttyS6
         * N/A    : /dev/ttyS4
         * IO DEBUG (RX only):/dev/ttyS0
         */

        warnx("22222222\n");

        int uart_read = uart_init("/dev/ttyS0");
    if(0) {
        if (false == uart_read)
            return -1;
        if (false == set_uart_baudrate(uart_read, 9600)) {
            printf("[JXF]set_uart_baudrate is failed\n");
            return -1;
        }
        printf("[JXF]uart init is successful\n");
        thread_running = true;
        while (!thread_should_exit) {

            int len = read(uart_read, buff, 200);
            if (len > 0) {
                for (int i = 0; i < len; ++i) {

                    putchar(buff[i]);
                }
            }

            /*printf("%s\n",buff);*/
            usleep(2000);

        }
        warnx("[uart_read] exiting.\n");
        thread_running = false;
        int fd = close(uart_read);
        printf("close stauts: %d\n", fd);
    }
    return 0;
}