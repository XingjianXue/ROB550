#ifndef MBOT_H
#define MBOT_H

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/defs/common_defs.h>
#include <rc/defs/mbot_diff_defs.h>
#include <rc/fram/fram.h>
#include <rc/math/filter.h>
#include <rc/mpu/mpu.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/mbot_messages.h>

#include <math.h>
#include <inttypes.h>

// Hardware info
#define MAX_FWD_VEL 0.8 // max forward speed (m/s)
#define MESSAGE_CONFIRMATION_CHANNEL "MSG_CONFIRM"
#define MAX_TURN_VEL 2.5 // max turning speed (rad/s)

// TODO: Enter the polarity values for your motors and encoders
#define LEFT_ENC_POL 1
#define RIGHT_ENC_POL -1
#define LEFT_MOTOR_POL -1
#define RIGHT_MOTOR_POL 1

// TODO: Populate with calibration data (recommended to generate these for reverse direction as well)
// change for each robot
// robot: 025
// #define SLOPE_L_p 0.006363150852208891
// #define SLOPE_R_p 0.006512770488858423
// #define INTERCEPT_L_p 0.13017367960157206
// #define INTERCEPT_R_p 0.12166174724123835

// #define SLOPE_L_n 0.00633573276638355
// #define SLOPE_R_n 0.006464232762117717
// #define INTERCEPT_L_n -0.05145179822465311
// #define INTERCEPT_R_n -0.03175599681592275

#define SLOPE_L_p 0.006638465729033319
#define SLOPE_R_p 0.006804774779897176
#define INTERCEPT_L_p 0.11885258529179651
#define INTERCEPT_R_p 0.11401058529944787

#define SLOPE_L_n 0.006447168436384867
#define SLOPE_R_n 0.006767546795515685
#define INTERCEPT_L_n -0.04787677185727659
#define INTERCEPT_R_n -0.029337356753979305

// TODO: Decide which controller is used, open loop = 1, PID = 0
#define OPEN_LOOP 0

// data to hold current mpu state (not used)
static rc_mpu_data_t mpu_data;
static i2c_inst_t *i2c;

uint64_t timestep_us = 0;

// data to hold calibration coefficients
float coeffs[4];

// data to hold the PID values
static mbot_pid_gains_t mbot_pid_gains;

typedef struct pid_parameters pid_parameters_t;
struct pid_parameters
{
    float kp;
    float ki;
    float kd;
    float dFilterHz;
};

float clamp_duty(float duty);

float clamp_angle(float angle);


// data to hold the IMU results
mbot_imu_t current_imu = {0};
// data to hold the received timestamp
timestamp_t received_time = {0};
// current odometry state
odometry_t current_odom = {0};
// current encoder states
mbot_encoder_t current_encoders = {0};
// current body frame command
mbot_motor_command_t current_cmd = {0};
// current control informaation
mbot_wheel_ctrl_t current_wheel_ctrl = {0};


/**
 * Example filter and PID parameter initialization
 *
 * rc_filter_t my_filter;
 *
 * pid_parameters_t pid_params = {
 *    .kp = 1.0,
 *    .ki = 0.0,
 *    .kd = 0.0,
 *    .dFilterHz = 25.0
 * };
 */

rc_filter_t left_pid;
rc_filter_t right_pid;
rc_filter_t fwd_vel_pid;
rc_filter_t turn_vel_pid;
rc_filter_t measured_vel_lowpass;
rc_filter_t wheel_speed_lowpass;
rc_filter_t wheel_turn_lowpass;

rc_filter_t measured_fwd_vel_lowpass;
rc_filter_t measured_turn_vel_lowpass;

// 0.4 0.5
// pid_parameters_t left_pid_params = {
//     .kp = 0.2,
//     .ki = 0.8,
//     .kd = 0.20,
//     .dFilterHz = 12.5,
// };
// //without lowpass 0.1,0.7,0.05
// pid_parameters_t right_pid_params = {
//     .kp = 0.2,
//     .ki = 0.8,
//     .kd = 0.20,
//     .dFilterHz = 12.5,
// };

// 0.1 0.6 0.1
pid_parameters_t left_pid_params = {
    .kp = 4.0,
    .ki = 0.01, //0.7
    .kd = 0.02,
    .dFilterHz = 12.5,
};
//without lowpass 0.1,0.7,0.05
pid_parameters_t right_pid_params = {
    .kp = 4.0,
    .ki = 0.01, //0.7
    .kd = 0.02,
    .dFilterHz = 12.5,
};

pid_parameters_t fwd_vel_pid_params = {
    .kp = 2.0, //0.5
    .ki = 0.0, //0.3
    .kd = 0.01, //0.01
    .dFilterHz = 12.5,
};
pid_parameters_t turn_vel_pid_params = { //4,1,0.1
    // .kp = 4.5, //0.3 
    // .ki = 0.1, //0.0
    // .kd = 0.02, //0.05
    .dFilterHz = 12.5,
    .kp = 2.0,
    .ki = 0.00,
    .kd = 0.1,
    // .dFilterHz = 12.5,
    
};

float clamp_duty(float duty);
float calibration(float speed);

//test
void odom_test(mbot_motor_command_t* current_cmd, float* l_duty, float* r_duty);

#endif
