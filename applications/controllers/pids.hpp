#ifndef PIDS_HPP
#define PIDS_HPP
#include "A_HERO_SELECTION.hpp"
#include "tools/pid/pid.hpp"

constexpr float T_CONTROL = 1e-3f;  // 控制周期, 单位: s

//chassis pids
constexpr float MAX_WHEEL_TORQUE = 5.859f;  // 底盘电机3508最大扭矩，单位N.m

#ifdef HERO_DOG
inline sp::PID speed_lf_pid(T_CONTROL, 0.3f, 0.0f, 0.0f, MAX_WHEEL_TORQUE, 0.5f, 0.15f);
inline sp::PID speed_lr_pid(T_CONTROL, 0.3f, 0.0f, 0.0f, MAX_WHEEL_TORQUE, 0.8f, 0.15f);
inline sp::PID speed_rf_pid(T_CONTROL, 0.3f, 0.0f, 0.0f, MAX_WHEEL_TORQUE, 0.5f, 0.15f);
inline sp::PID speed_rr_pid(T_CONTROL, 0.3f, 0.0f, 0.0f, MAX_WHEEL_TORQUE, 0.5f, 0.15f);
#endif

//gimbal pids
constexpr float MAX_4310_TORQUE = 10.0f;    // 达妙4310电机最大扭矩，单位N.m
constexpr float MAX_XIAOMI_TORQUE = 10.0f;  // 小米电机最大扭矩，单位N.m

#ifdef HERO_DOG
//位置控制PID
inline sp::PID yaw_pos_pid(T_CONTROL, 30.0f, 0.0f, 0.3f, 7, 3, 1.0f, true, false);
inline sp::PID yaw_speed_pid(
  T_CONTROL, 2.0f, 0.0f, 0.25f, MAX_4310_TORQUE, MAX_4310_TORQUE / 3.0f, 1.0f, false, false);

inline sp::PID pitch_pos_pid(T_CONTROL, 50.0f, 0.0f, 1.1f, 6, 0.18, 1.0f, true, false);
inline sp::PID pitch_speed_pid(
  T_CONTROL, 0.6f, 0.0f, 0.0f, MAX_4310_TORQUE, 0.2, 1.0f, false, false);

//初始化PID
inline sp::PID yaw_encode_pos_pid(T_CONTROL, 10.0f, 0.0f, 0.45f, 3, 1.5, 1.0f, true, false);
inline sp::PID yaw_encode_speed_pid(
  T_CONTROL, 1.5f, 0.0f, 0.12f, MAX_4310_TORQUE, MAX_4310_TORQUE / 3.0f, 1.0, false, false);

inline sp::PID pitch_encode_pos_pid(T_CONTROL, 13.0f, 100.0f, 0.05f, 5, 0.8, 1.0f, true, false);
inline sp::PID pitch_encode_speed_pid(
  T_CONTROL, 0.2f, 0.0f, 0.0f, MAX_XIAOMI_TORQUE, 0.1, 1.0f, false, false);
#endif

//shoot pids
constexpr float MAX_FRIC_MOTOR_TORQUE = 5.859f;  // N.m
constexpr float TRIGGER_MAX_TORQUE = 1.8f;       // N.m

#ifdef HERO_DOG
//单发外环PID
inline sp::PID trigger_pos_pid1(T_CONTROL, 16.05f, 6.0f, 0.0f, 10, 0.5, 0.5f, true, true);
//单发内环PID
inline sp::PID trigger_speed_pid2(
  T_CONTROL, 0.53f, 0.0f, 0.002f, TRIGGER_MAX_TORQUE, TRIGGER_MAX_TORQUE, 1, false, true);

//反转外环PID
inline sp::PID triggerback_pos_pid1(T_CONTROL, 40.05f, 3.5f, 0.1f, 10, 10, 1.0f, true, true);
//反转内环PID
inline sp::PID triggerback_speed_pid2(
  T_CONTROL, 0.53f, 0.0f, 0.002f, TRIGGER_MAX_TORQUE, TRIGGER_MAX_TORQUE, 1, false, true);

//摩擦轮3508单环速度环PID
inline sp::PID fricmotor1_pid(T_CONTROL, 0.0032f, 0.0f, 0.000001f, 0.34, 0.1, 0.8);
inline sp::PID fricmotor2_pid(T_CONTROL, 0.0032f, 0.0f, 0.000001f, 0.34, 0.1, 0.8);
inline sp::PID fricmotor3_pid(T_CONTROL, 0.0032f, 0.0f, 0.000001f, 0.34, 0.1, 0.8);
#endif

#endif