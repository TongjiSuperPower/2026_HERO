#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP
#include "A_HERO_SELECTION.hpp"
#include "controllers/chassis_controller/chassis_task.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "io/can/can.hpp"
#include "io/vision/vision.hpp"

// -------------------- 控制参数 --------------------
//计算力矩控制参数
#ifdef HERO_DOG
// m = 0.0617, b = 0.3535, c = 0.3400
// m = 0.0716, b = 0.2665, c = 0.3082
// m = 0.0787, b = 0.2249, c = 0.3071
// m = 0.0786, b = 0.2343, c = 0.3100
constexpr float YAW_INERTIA = 0.0786f;
constexpr float YAW_DAMPING_COEFF = 0.2343f;
constexpr float YAW_COULOMB_FORCE = 0.3100f;

// m = 0.0331, b = 0.2112, c = 0.1037, d = 1.8725
// m = 0.0349, b = 0.2383, c = 0.0796, d = 1.8778
constexpr float PITCH_INERTIA = 0.0349f;
constexpr float PITCH_DAMPING_COEFF = 0.2383f;
constexpr float PITCH_COULOMB_FORCE = 0.0796f;
constexpr float PITCH_GRAVITY_TORQUE = 1.8778f;
#endif

extern float pitch_torque;
extern float a;
extern float b;
extern float c;
extern float d;

#endif