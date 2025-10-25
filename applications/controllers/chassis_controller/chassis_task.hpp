#ifndef CHASSIS_TASK_HPP
#define CHASSIS_TASK_HPP
#include "A_HERO_SELECTION.hpp"
#include "controllers/detect_task.hpp"
#include "controllers/keys.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

//根据剩余能量和电容情况确定最大功率
void Pmax_get();
//基于缓冲能量的功率控制
void buff_energy_p_limited();

// -------------------- 控制结构 --------------------
typedef struct
{
  float vx;
  float vy;
  float wz;
} Chassis_Speed;

typedef struct
{
  float lf;
  float lr;
  float rf;
  float rr;
} Wheel_Speed;

typedef struct
{
  float lf;
  float lr;
  float rf;
  float rr;
} Wheel_Torque;

// -------------------- 控制参数 --------------------
constexpr float T_CHASSIS = 1e-3f;  // 控制周期, 单位: s

constexpr float CAP_MAX_POWER = 400.0f;  //16V时电容最大放电功率
//电容最大放电功率=25*电压

//遥控器平移最大速度
constexpr float REMOTE_CONTROL_V = 3.0f;
//键鼠平移速度
constexpr float KEYBOARD_CONTROL_V = 3.2f;
//电容模式键鼠平移速度m/s(飞坡)
constexpr float KEYBOARD_CAP_CONTROL_V = 4.0f;  //400cm/s能飞坡
//小陀螺角速度rad/s
constexpr float SPIN_W = 10.0f;  //大约112w
constexpr float SPIN_V = 1.3f;
//底盘3508转轴最大转速限制rad/s
constexpr float AXLE_WMAX = 45.0f;
//底盘速度死区
constexpr float ZeroXY = 1.0e-6f;

#ifdef HERO_DOG
//底盘跟随pid
inline sp::PID chassis_follow_wz_pid(T_CHASSIS, 8.5f, 0.0f, 0.7f, 5.0f, 3.0f, 0.5f);

#endif

#ifdef HERO_DOG
//filter
inline sp::LowPassFilter chassis_follow_vx_filter(0.1f);
inline sp::LowPassFilter chassis_follow_vy_filter(0.1f);
inline sp::LowPassFilter chassis_follow_wz_filter(0.5f);
inline sp::LowPassFilter chassis_follow_cap_vx_filter(0.006f);
inline sp::LowPassFilter chassis_follow_cap_vy_filter(0.006f);
inline sp::LowPassFilter chassis_spin_vx_filter(0.2f);
inline sp::LowPassFilter chassis_spin_vy_filter(0.2f);
//为了使升级时候功率阶跃不明显
inline sp::LowPassFilter power_filter(0.15f);
inline sp::LowPassFilter yaw_relative_angle_filter(0.01f);
//平滑3508速度
inline sp::LowPassFilter wheel_lf_filter(0.03f);
inline sp::LowPassFilter wheel_lr_filter(0.03f);
inline sp::LowPassFilter wheel_rf_filter(0.03f);
inline sp::LowPassFilter wheel_rr_filter(0.03f);
//平滑底盘wz观测值
inline sp::LowPassFilter chassis_wz_measured_filter(0.1f);

inline sp::LowPassFilter correctx(0.5);
inline sp::LowPassFilter correcty(0.5);
#endif

//电容部分
//充分利用电容的最大功率限制pid
inline sp::PID power_limit_pid(T_CHASSIS, 1, 0.2, 0, 50, 10);
inline sp::PID buff_limit_pid(T_CHASSIS, 100, 2, 1, 800, 10, 0.5);

// -------------------- 机械参数 --------------------
#ifdef HERO_DOG
constexpr float WHEEL_RADIUS = 77e-3f;       // m
constexpr float CHASSIS_LENGTH = 396.2e-3f;  // m
constexpr float CHASSIS_WIDTH = 356.47e-3f;  // m
#endif

#ifdef HERO_THREE_WHEELS
constexpr float WHEEL_RADIUS = 77e-3f;       // m
constexpr float CHASSIS_LENGTH = 396.2e-3f;  // m
constexpr float CHASSIS_WIDTH = 356.47e-3f;  // m
#endif

// -------------------- 对外接口 --------------------
//状态机
extern bool chassis_init_flag;
//超级电容
inline sp::SuperCap super_cap(sp::SuperCapMode::AUTOMODE);
extern uint8_t low_vol_flag;

// -------------------- 对外硬件 --------------------
#ifdef HERO_DOG
inline sp::RM_Motor wheel_lf(1, sp::RM_Motors::M3508, RADUCTION_RATIO);  // left front
inline sp::RM_Motor wheel_lr(4, sp::RM_Motors::M3508, RADUCTION_RATIO);  // left rear
inline sp::RM_Motor wheel_rf(2, sp::RM_Motors::M3508, RADUCTION_RATIO);  // right front
inline sp::RM_Motor wheel_rr(3, sp::RM_Motors::M3508, RADUCTION_RATIO);  // right rear
#endif

// -------------------- 对外调试 --------------------
extern sp::Mecanum chassis;
extern sp::Mecanum corrector;
extern Wheel_Speed chassis_target_speed;
extern Wheel_Speed chassis_target_speed_before;
extern sp::SuperCap super_cap;
extern Chassis_Speed chassis_speed;
extern Wheel_Speed wheel_speed;
extern Wheel_Torque wheel_give_torque;
extern float wheel_lf_speed_filter;
extern float wheel_lr_speed_filter;
extern float wheel_rf_speed_filter;
extern float wheel_rr_speed_filter;
extern float energy_sum;
extern float chassis_wz_filter;
// -------------------------------------------------

#endif