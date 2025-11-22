#ifndef IMU_TASK_HPP
#define IMU_TASK_HPP
#include "A_HERO_SELECTION.hpp"
#include "calibrate_task.hpp"
#include "io/bmi088/bmi088.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"
#include "stdint.h"
#include "io/vision/vision.hpp"

extern sp::BMI088 bmi088;
extern sp::Mahony imu;

// -------------------- 控制参数 --------------------
//IMU温度
constexpr float IMU_TEMP = 50.0f;
//PID参数
constexpr float IMU_TEMP_KP = 1600.0f;
constexpr float IMU_TEMP_KI = 0.2f;
constexpr float IMU_TEMP_KD = 0.0f;
constexpr float IMU_TEMP_MAXOUT = 4500.0f;
constexpr float IMU_TEMP_MAXIOUT = 1000.0f;

//温度控制函数
void imu_temp_control(float temp);

#ifdef HERO_DOG

constexpr float GyroXZero = 0.00620746519f;
constexpr float GyroYZero = -0.00352368597f;
constexpr float GyroZZero = 0.000081818027f;

#endif

#ifdef HERO_THREE_FRIC

constexpr float GyroXZero = -0.0062657f;
constexpr float GyroYZero = -0.0010400f;
constexpr float GyroZZero = -0.0008904f;

#endif

#ifdef MPC
inline sp::Vision vis;
#endif

// -------------------- 对外接口 --------------------
extern uint8_t calibrate_flag;
// extern float gyro_x_zero;
// extern float gyro_y_zero;
// extern float gyro_z_zero;
extern float ins_gyro[3];
extern float ins_gyro_filter[3];
extern float imu_vyaw_filter;
extern float imu_vpitch_filter;
extern float ins_eulers[3];
extern sp::PID imu_temp_pid;
extern sp::Vision vis;

#endif