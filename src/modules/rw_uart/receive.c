#include "rw_uart.h"
#include "rw_uart_define.h"

void wifi_pack(const uint8_t *buffer, MSG_orb_data *msg_data, MSG_type msg_type){
    int32_t *data;
    switch (msg_type.command) {
    case WIFI_COMM_WAYPOINT:
        msg_data->command_data.command = 16; //VEHICLE_CMD_NAV_WAYPOINT
        //data = ((int32_t) buffer[6]) + ((int32_t) buffer[7]<<8) +((int32_t) buffer[8]<<16) + ((int32_t)buffer[9]<<24);
        data = (int32_t*)((uint32_t)buffer + 6);
        msg_data->command_data.param5 = ((float_t)*data)/10000000.0;
        printf("lat: %d\n", *data);
        data = (int32_t*)((uint32_t)buffer + 10);
        msg_data->command_data.param6 = ((float_t)*data)/10000000.0;
        printf("lon: %d\n", *data);
        msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0;
        printf("Passing waypoint\n");
        msg_data->command_data.param1 = 0.0;
        msg_data->command_data.param2 = 0.0;
        msg_data->command_data.param3 = 0.0;
        msg_data->command_data.param4 = 0.0;
        break;
    case WIFI_COMM_AUTO_LAND:
        msg_data->command_data.command = 21; //VEHICLE_CMD_NAV_LAND
        msg_data->command_data.param4 = 0.0;
        msg_data->command_data.param5 = ((float_t)msg_data->gps_data.lat)/10000000.0;
        msg_data->command_data.param6 = ((float_t)msg_data->gps_data.lon)/10000000.0;
        msg_data->command_data.param7 = 0.0;
        printf("Passing land\n");
        break;
    case WIFI_COMM_AUTO_TAKEOFF:
        msg_data->command_data.command = 22; //VEHICLE_CMD_NAV_TAKEOFF
        msg_data->command_data.param1 = 0.0;
        msg_data->command_data.param4 = 0.0;
        msg_data->command_data.param5 = ((float_t)msg_data->gps_data.lat)/10000000.0;
        msg_data->command_data.param6 = ((float_t)msg_data->gps_data.lon)/10000000.0;
        msg_data->command_data.param7 = ((float_t) msg_data->gps_data.alt)/1000.0 +10.0;
        printf("Passing takeoff\n");
        break;
    default:
        break;
    }
}

bool change_param (MSG_param_hd msg_hd, uint8_t data, int i){
    bool changed = true;
    float_t paramf;
    int paramd;
    switch (i) {
    case 8:
        paramf = (float_t)data / 21.25;
        param_set(msg_hd.roll_p_hd, &paramf);
        //printf("Passing change\n");
        break;
//    case 11:
//        paramf = (float_t)data / 21.25;
//        param_set(msg_hd.pitch_p_hd, &paramf);
//        break;
//    case 14:
//        paramf = (float_t)data / 51.0;
//        param_set(msg_hd.yaw_p_hd, &paramf);
//        break;
//    case 17:
//        paramf = (float_t)data / 170.0;
//        param_set(msg_hd.z_p_hd, &paramf);
//        break;
//    case 21:
//        paramf = (float_t)data / 34.0 + 0.5;
//        param_set(msg_hd.up_vel_max_hd, &paramf);
//        break;
//    case 22:
//        paramf = (float_t)data / 15.0 + 3.0;
//        param_set(msg_hd.xy_vel_max_hd, &paramf);
//        break;
//    case 23:
//        paramf = (float_t)data / 0.142;
//        param_set(msg_hd.roll_rate_hd, &paramf);
//        break;
//    case 24:
//        paramf = (float_t)data / 0.142;
//        param_set(msg_hd.pitch_rate_hd, &paramf);
//        break;
//    case 25:
//        paramf = (float_t)data / 0.142;
//        param_set(msg_hd.yaw_rate_hd, &paramf);
//        break;
//    case 26:
//        paramf = (float_t)data / 19.62 + 2.0;
//        param_set(msg_hd.acc_up_max_hd, &paramf);
//        break;
//    case 28:
//        paramf = (float_t)data / 0.6375;
//        param_set(msg_hd.yaw_max_hd, &paramf);
//        break;
//    case 29:
//        paramf = (float_t)data / 2.84;
//        param_set(msg_hd.roll_max_hd, &paramf);
//        break;
//    case 30:
//        paramf = (float_t)data / 1.60 + 20.0;
//        param_set(msg_hd.pitch_max_hd, &paramf);
//        break;
//    case 33:
//        paramf = (float_t)data / 19.62 + 2.0;
//        param_set(msg_hd.acc_hor_max_hd, &paramf);
//        break;
//    case 38:
//        paramd = (int)data;
//        param_set(msg_hd.mav_type_hd, &paramd);
//        break;
//    case 39:
//        paramd = (int)data;
//        param_set(msg_hd.battery_n_cells_hd, &paramd);
//        break;
//    case 40:
//        paramf = (float_t)data / 910.8 + 0.12;
//        param_set(msg_hd.battery_warn_hd, &paramf);
//        break;
//    case 48:
//        paramd = (int)data;
//        param_set(msg_hd.battery_warn_hd, &paramd);
//        break;
//    case 49:
//        paramd = (int)data;
//        param_set(msg_hd.rc_lost_act_hd, &paramd);
//        break;
//    case 59:
//        paramf = (float_t)data / 63.75;
//        param_set(msg_hd.dn_vel_max_hd, &paramf);
//        break;
    default:
        changed = false;
        break;
    }
    printf("Passing change, changed is %d, i is %d\n", changed, i);
    return changed;
}

bool yfwi_param_set(const uint8_t *buffer, MSG_param_hd msg_hd, uint8_t *param_saved){
    bool changed = false;
    uint16_t  dist_max, dist_max_saved;

    dist_max = ((uint16_t) buffer[32]<< 8) + (uint16_t)buffer[31];
    dist_max_saved = ((uint16_t) param_saved[32]<< 8) + (uint16_t)param_saved[31];
    if(dist_max != dist_max_saved) {
        float_t paramf = (float_t)dist_max / 6.5535;
        param_set(msg_hd.higt_max_hd, &paramf);
        changed = true;
    }

    dist_max = ((uint16_t) buffer[35]<< 8) + (uint16_t)buffer[34];
    dist_max_saved = ((uint16_t) param_saved[35]<< 8) + (uint16_t)param_saved[34];
    if(dist_max != dist_max_saved) {
        float_t paramf = (float_t)dist_max / 6.5535;
        param_set(msg_hd.dist_max_hd, &paramf);
        changed = true;
        printf("higt_max changed\n");
    }

    for (int i = 8; i < 60; i++) {
        printf("i : %d, buffer[i] : %x, param_saved[i]: %x\n", i, buffer[i], param_saved[i]);
        if (buffer[i] != param_saved[i])
        {
            changed |= change_param(msg_hd, buffer[i], i);
            //printf("Passing change, changed is %d\n", changed);
        }
    }
    return changed;
}