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
#include <systemlib/mavlink_log.h>


 __EXPORT int iq_angle_sub_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< px4_uorb_subs exit flag */
static bool thread_kk = 0;		/**< px4_uorb_subs status flag */
static int read_uart_task;				/**< Handle of px4_uorb_subs task */

int iq_angle_sub_ss_main(int argc, char *argv[]);
static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: uart_read {start|stop|status} [-p <additional params>]\n\n");
}

int iq_angle_sub_main(int argc, char *argv[])
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
        read_uart_task = px4_task_spawn_cmd("iq_angle_sub",
                                            SCHED_DEFAULT,
                                            SCHED_PRIORITY_DEFAULT,
                /*SCHED_PRIORITY_SLOW_DRIVER,*/
                /* SCHED_PRIORITY_MAX - 50,*/
                                            3600,//堆栈分配大小
                                            iq_angle_sub_ss_main,
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
int iq_angle_sub_ss_main(int argc, char *argv[]){

    int error_counter = 0;
  int  iq_angle = orb_subscribe(ORB_ID(iq));
    orb_set_interval(iq_angle, 5);

    thread_kk = 1;
    px4_pollfd_struct_t fds[] = {
            { .fd = iq_angle,   .events = POLLIN },
            /* there could be more file descriptors here, in the form like:
             * { .fd = other_sub_fd,   .events = POLLIN },
             */
    };
    while(!thread_should_exit){
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;

        } else {

            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct iq_s angle_s;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(iq), iq_angle, &angle_s);
                double speed=angle_s.ob_speed;
                printf("speed=%lf\n",speed);

            }
        }

    }
    warnx("[iq_angle_sub] exiting.\n");
    thread_kk = 0;


    return 0;
}


