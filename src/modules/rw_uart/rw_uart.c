
#include "rw_uart.h"
#include "rw_uart_define.h"

/**
* daemon management function.
 */
__EXPORT int rw_uart_main(int argc, char *argv[]);

static bool rw_thread_should_exit = false;		/**< px4_uart exit flag */
static bool rw_uart_thread_running = false;		/**< px4_uart status flag */
//static uint64_t last_time_send;
static pthread_mutex_t mutex;

int uart_read;
uint8_t param_saved[62] = {};
Waypoint_saved wp_data;

MSG_orb_sub msg_fd;
MSG_orb_data msg_data;
MSG_orb_pub msg_pd;
MSG_param_hd msg_hd;

static int rw_uart_task;				/**< Handle of px4_uart task / thread */
static int rw_uart_init(void);
static int set_rw_uart_baudrate(const int fd, unsigned int baud);

void msg_orb_sub (void);
void msg_orb_data(void);
void msg_orb_unsub (void);
void msg_param_hd_cache (void);
void wp_data_init(void);

/**
 * Mainloop of daemon.
 */
int rw_uart_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void usage(const char *reason)
{
        if (reason) {
                printf("%s\n", reason);
        }

       printf("usage: rw_uart {start|stop|status} [-p <additional params>]\n\n");
}

int set_rw_uart_baudrate(const int fd, unsigned int baud)
{
        int speed;

        switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
                printf("ERR: baudrate: %d\n", baud);
                return -EINVAL;
        }

        struct termios uart_config;

        int termios_state;

        tcgetattr(fd, &uart_config); // 获取终端参数

        /* clear ONLCR flag (which appends a CR for every LF) */
        uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

        /* 无偶校验，一个停止位 */
        uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
        // CSTOPB 使用两个停止位，PARENB 表示偶校验, CRTSCTS 使用流控

        cfmakeraw(&uart_config);

         /* 设置波特率 */
        if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
                 printf("ERR: %d (cfsetispeed)\n", termios_state);
                return false;
        }

        if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
                  printf("ERR: %d (cfsetospeed)\n", termios_state);
                return false;
        }
        // 设置与终端相关的参数，TCSANOW 立即改变参数
        if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
                printf("ERR: %d (tcsetattr)\n", termios_state);
                return false;
        }

        return true;
}


int rw_uart_init (void)
{
       //char *uart_name = "/dev/ttyS3";
       char *uart_name = "/dev/ttyS1";
       int serial_fd = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY);
       //int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
        // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
        if (serial_fd < 0) {
                printf("failed to open port: %s\n", uart_name);
                return false;
        }
        printf("Open the %s\n",uart_name);
        return serial_fd;
}


void msg_orb_sub (void)
{
    memset(&msg_fd, 0, sizeof(msg_fd));
    msg_fd.arm_fd = orb_subscribe(ORB_ID(actuator_armed));
    msg_fd.gps_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
    //msg_fd.command_fd = orb_subscribe(ORB_ID(vehicle_command));
    msg_fd.mission_fd = orb_subscribe(ORB_ID(mission_result));
    msg_fd.manual_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
    msg_fd.status_fd = orb_subscribe(ORB_ID(vehicle_status));
    msg_fd.local_position_sp_fd = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
    msg_fd.local_position_fd = orb_subscribe(ORB_ID(vehicle_local_position));
    msg_fd.air_data_fd = orb_subscribe(ORB_ID(vehicle_air_data));
    msg_fd.attitude_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    msg_fd.battery_fd = orb_subscribe(ORB_ID(battery_status));
    msg_fd.geofence_fd = orb_subscribe(ORB_ID(geofence_result));
    //msg_fd->rc_input_fd = orb_subscribe(ORB_ID(input_rc));
    msg_fd.vibe_fd = orb_subscribe(ORB_ID(estimator_status));
    msg_fd.global_position_fd = orb_subscribe(ORB_ID(vehicle_global_position));
    msg_fd.attitude_sp_fd = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    msg_fd.home_position_fd = orb_subscribe(ORB_ID(home_position));
    msg_fd.dg_voltage_fd = orb_subscribe(ORB_ID(dg_voltage));
    msg_fd.position_setpoint_fd = orb_subscribe(ORB_ID(position_setpoint));
}


void msg_orb_data(void)
{
   orb_copy(ORB_ID(actuator_armed), msg_fd.arm_fd,&msg_data.arm_data);
   orb_copy(ORB_ID(vehicle_gps_position), msg_fd.gps_fd,&msg_data.gps_data);
   //orb_copy(ORB_ID(vehicle_command), msg_fd.command_fd,&msg_data.command_data);
   orb_copy(ORB_ID(mission_result), msg_fd.mission_fd, &msg_data.mission_data);
   orb_copy(ORB_ID(manual_control_setpoint), msg_fd.manual_fd, &msg_data.manual_data);
   orb_copy(ORB_ID(vehicle_status), msg_fd.status_fd, &msg_data.status_data);
   orb_copy(ORB_ID(vehicle_local_position_setpoint), msg_fd.local_position_sp_fd, &msg_data.local_position_sp_data);
   orb_copy(ORB_ID(vehicle_local_position), msg_fd.local_position_fd, &msg_data.local_position_data);
   orb_copy(ORB_ID(vehicle_air_data), msg_fd.air_data_fd, &msg_data.air_data);
   orb_copy(ORB_ID(vehicle_attitude), msg_fd.attitude_fd, &msg_data.attitude_data);
   orb_copy(ORB_ID(battery_status), msg_fd.battery_fd, &msg_data.battery_data);
   orb_copy(ORB_ID(geofence_result), msg_fd.geofence_fd, &msg_data.geofence_data);
   //orb_copy(ORB_ID(input_rc), msg_fd.rc_input_fd, &msg_data->input_rc_data);
   orb_copy(ORB_ID(estimator_status), msg_fd.vibe_fd, &msg_data.vibe_data);
   orb_copy(ORB_ID(vehicle_global_position), msg_fd.global_position_fd, &msg_data.global_position_data);
   orb_copy(ORB_ID(vehicle_attitude_setpoint), msg_fd.attitude_sp_fd, &msg_data.attitude_sp_data);
   orb_copy(ORB_ID(home_position), msg_fd.home_position_fd, &msg_data.home_position_data);
   orb_copy(ORB_ID(dg_voltage), msg_fd.dg_voltage_fd, &msg_data.dg_voltage_data);
   orb_copy(ORB_ID(position_setpoint), msg_fd.position_setpoint_fd, &msg_data.position_setpoint_data);
}

void msg_orb_unsub (void)
{
    orb_unsubscribe(msg_fd.arm_fd);
    orb_unsubscribe(msg_fd.gps_fd);
    //orb_unsubscribe(msg_fd.command_fd);
    orb_unsubscribe(msg_fd.mission_fd);
    orb_unsubscribe(msg_fd.manual_fd);
    orb_unsubscribe(msg_fd.status_fd);
    orb_unsubscribe(msg_fd.local_position_sp_fd);
    orb_unsubscribe(msg_fd.local_position_fd);
    orb_unsubscribe(msg_fd.air_data_fd);
    orb_unsubscribe(msg_fd.attitude_fd);
    //orb_unsubscribe(msg_fd->rc_input_fd);
    orb_unsubscribe(msg_fd.battery_fd);
    orb_unsubscribe(msg_fd.geofence_fd);
    orb_unsubscribe(msg_fd.vibe_fd);
    orb_unsubscribe(msg_fd.global_position_fd);
    orb_unsubscribe(msg_fd.attitude_sp_fd);
    orb_unsubscribe(msg_fd.home_position_fd);
    orb_unsubscribe(msg_fd.dg_voltage_fd);
    orb_unsubscribe(msg_fd.position_setpoint_fd);
}

void msg_param_hd_cache (void)
{
    memset(&msg_hd, 0, sizeof(msg_hd));
    msg_hd.roll_p_hd = param_find("MC_ROLLRATE_P");
    msg_hd.roll_i_hd = param_find("MC_ROLLRATE_I");
    msg_hd.roll_d_hd = param_find("MC_ROLLRATE_D");
    msg_hd.pitch_p_hd = param_find("MC_PITCHRATE_P");
    msg_hd.pitch_i_hd = param_find("MC_PITCHRATE_I");
    msg_hd.pitch_d_hd = param_find("MC_PITCHRATE_D");
//    msg_hd.yaw_p_hd = param_find("MC_YAWRATE_P");
//    msg_hd.yaw_i_hd = param_find("MC_YAWRATE_I");
//    msg_hd.yaw_d_hd = param_find("MC_YAWRATE_D");
    msg_hd.hor_p_hd = param_find("MPC_XY_VEL_P");
    msg_hd.hor_i_hd = param_find("MPC_XY_VEL_I");
    msg_hd.hor_d_hd = param_find("MPC_XY_VEL_D");
    msg_hd.ver_p_hd = param_find("MPC_Z_VEL_P");
    msg_hd.ver_i_hd = param_find("MPC_Z_VEL_I");
    msg_hd.ver_d_hd = param_find("MPC_Z_VEL_D");
    msg_hd.throttle_hd = param_find("THR_MDL_FAC");
    msg_hd.xy_p_hd = param_find("MPC_XY_P");
    msg_hd.z_p_hd = param_find("MPC_Z_P");
    msg_hd.throttle_hd = param_find("THR_MDL_FAC");
    msg_hd.up_vel_max_hd = param_find("MPC_Z_VEL_MAX_UP");
    msg_hd.xy_vel_max_hd = param_find("MPC_VEL_MANUAL");
    msg_hd.roll_rate_hd = param_find("MC_ROLLRATE_MAX");
    msg_hd.pitch_rate_hd = param_find("MC_PITCHRATE_MAX");
    msg_hd.yaw_rate_hd = param_find("MC_YAWRATE_MAX");
    msg_hd.acc_up_max_hd = param_find("MPC_ACC_UP_MAX");
    msg_hd.yaw_max_hd = param_find("MPC_MAN_Y_MAX");
    msg_hd.yaw_fast_hd = param_find("MC_YAWRATE_MAX");
    msg_hd.roll_max_hd = param_find("MPC_MAN_TILT_MAX");
    msg_hd.pitch_max_hd =param_find("MPC_TILTMAX_AIR");
    msg_hd.att_r_hd = param_find("MC_ROLL_P");
    msg_hd.att_p_hd = param_find("MC_PITCH_P");
    msg_hd.higt_max_hd = param_find("GF_MAX_VER_DIST");
    msg_hd.acc_hor_max_hd = param_find("MPC_ACC_HOR_MAX");
    msg_hd.dist_max_hd = param_find("GF_MAX_HOR_DIST");
    msg_hd.mav_type_hd = param_find("SYS_AUTOSTART");
    msg_hd.battery_n_cells_hd = param_find("BAT_N_CELLS");
    //msg_hd.battery_warn_hd = param_find("BAT_LOW_THR");
    //msg_hd.battery_crit_hd = param_find("BAT_CRIT_THR");
    msg_hd.battery_crit_hd = param_find("BAT_V_WARNING_DG");
    msg_hd.battery_fail_hd = param_find("COM_LOW_BAT_ACT");
    msg_hd.rc_lost_act_hd = param_find("NAV_RCL_ACT");
    msg_hd.dn_vel_max_hd = param_find("MPC_Z_VEL_MAX_DN");
    msg_hd.rc_on_off_hd = param_find("COM_RC_IN_MODE");
    //msg_hd->hover_thrust_hd = param_find("MPC_THR_HOVER");
    msg_hd.yaw_force_hd = param_find("MPC_YAW_MODE");
    msg_hd.pwm_min_hd = param_find("PWM_MIN");
}

void wp_data_init(void)
{
    memset(&wp_data, 0, sizeof(wp_data));
    wp_data.push = wp_data.setd;
    param_t cruise_speed_hd =  param_find("MPC_XY_CRUISE");
    float_t paramf;
    param_get(cruise_speed_hd, &paramf);
    wp_data.speed_pre = (uint8_t)(paramf*10);
}

int read_to_buff(uint8_t *buffer, int start, int end)
{
    uint8_t data = 0;
    int error_count = 0;
    int  i = start;
    for (; i < end;)
    {
        if (read(uart_read,&data,1) > 0){
            buffer[i] = data;
            printf("buffer[%d] is %x\n", i ,data);
            i++;
            error_count = 0;
        }
        else{
            error_count++;
        }
        if (error_count > 10) {
            //printf("buffer i is %d\n", i);
            return (i - start);
        }
    }
    return (i - start);
}


int rw_uart_main(int argc, char *argv[])
{
        if (argc < 2) {
                usage("missing command");
                return 1;
        }

        if (!strcmp(argv[1], "start")) {

                if (rw_uart_thread_running) {
                       printf("px4_uart already running\n");
                        /* this is not an error */
                        return 0;
                }

                rw_thread_should_exit = false;//定义一个守护进程
                rw_uart_task = px4_task_spawn_cmd("rw_uart",
                        SCHED_DEFAULT,
                        SCHED_PRIORITY_DEFAULT,//调度优先级
                        PX4_STACK_ADJUSTED(6000),//堆栈分配大小
                        rw_uart_thread_main,
                        (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
                return 0;
        }

        if (!strcmp(argv[1], "stop")) {
                rw_thread_should_exit = true;
                return 0;
        }

        if (!strcmp(argv[1], "status")) {
                if (rw_uart_thread_running) {
                        printf("\trunning\n");

                }
                else {
                        printf("\tnot started\n");
                }

                return 0;
        }

        usage("unrecognized command");
        return 1;
}

static void *receive_loop(void *arg)
{
    uint8_t buffer[150] ={};
    px4_pollfd_struct_t fds[1] = {
        { .fd = uart_read, .events = POLLIN }
    };
    int nread = 0;
    int read_finish = 0;
    int remain =0;
    int find_type_finish =0;
    int error_count = 0;

    while(!rw_thread_should_exit){

       if (error_count >20) {
           remain = 0;
           error_count =0;
       }

      if (error_count >0 || poll(&fds[0], 1, 20) > 0)
        {
          usleep(error_count * 1000);
          nread= read(uart_read, &buffer[remain], sizeof(buffer) - (size_t)remain);
          if (nread < 0) nread =0;
//           pthread_mutex_lock(&mutex);
          for ( read_finish = 0; read_finish < (nread + remain); ) {
               if ((nread + remain - read_finish) < 30) break;
               if (buffer[read_finish] == '$'){
                    find_type_finish = find_r_type(&buffer[read_finish], msg_data, &msg_pd, msg_hd);
                    if (find_type_finish < 0) {
                        error_count ++;
                        break;
                    }
                    else {
                        read_finish += find_type_finish;
                        error_count = 0;
                    }
                }
               else {
                   read_finish++;
               }
            }
            remain = nread + remain - read_finish;
            uint8_t buffer_move[150] = {};
            memcpy(buffer_move, &buffer[read_finish], (size_t)remain);
            memcpy(buffer, buffer_move, sizeof(buffer_move));
        }
    }
    return NULL;
}

void receive_start(pthread_t *thread)
{
    pthread_attr_t receiveloop_attr;
    pthread_attr_init(&receiveloop_attr);

    struct sched_param param;
    (void)pthread_attr_getschedparam(&receiveloop_attr, &param);
    param.sched_priority = SCHED_PRIORITY_MAX - 80;
    (void)pthread_attr_setschedparam(&receiveloop_attr, &param);

    pthread_attr_setstacksize(&receiveloop_attr, PX4_STACK_ADJUSTED(6000));
    pthread_create(thread, &receiveloop_attr, receive_loop, NULL);

    pthread_attr_destroy(&receiveloop_attr);
}


int rw_uart_thread_main(int argc, char *argv[])
{

        /*
                GPS1:/dev/ttyS0
                TEL1:/dev/ttyS1
                TEL2:/dev/ttyS2
                TEL4:/dev/ttyS3
         */

         uart_read = rw_uart_init();

         if (false == set_rw_uart_baudrate(uart_read, COM_PORT_BAUDRATE)) {
                 printf("set_rw_uart_baudrate is failed\n");
                 return -1;
         }
         printf("uart init is successful\n");

        //MSG_orb_sub msg_fd;
        msg_orb_sub();

        //MSG_orb_pub msg_pd;
        memset(&msg_pd, 0, sizeof(msg_pd));

        //MSG_param_hd msg_hd;
        msg_param_hd_cache();

        wp_data_init();

        //memset(param_saved, 0, sizeof(param_saved));
        msg_param_saved_get(msg_hd);

        pthread_mutex_init(&mutex, NULL);

        pthread_t receive_thread;

        rw_uart_thread_running = true;

        receive_start(&receive_thread);

        //last_time_send = hrt_absolute_time();
        printf("%s\n", __DATE__);

        while (!rw_thread_should_exit)
        {
            {
                pthread_mutex_lock(&mutex);
                memset(&msg_data, 0, sizeof(msg_data));
                msg_orb_data();
                pthread_mutex_unlock(&mutex);
                msg_pack_send(msg_data, &msg_pd);
                //last_time_send = hrt_absolute_time();
                //fflush(stdout);
                usleep(200000);
            }
        }

        msg_orb_unsub();
        //printf("[rw_uart] exiting\n");
        rw_uart_thread_running = false;
        close(uart_read);
        printf("uart close\n");

        fflush(stdout);
        return 0;
}
