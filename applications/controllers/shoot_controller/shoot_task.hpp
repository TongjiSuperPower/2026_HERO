#ifndef SHOOT_TASK_HPP
#define SHOOT_TASK_HPP
#include "A_HERO_SELECTION.hpp"
#include "io/vision/vision.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "referee/pm02/pm02.hpp"
#include "tools/pid/pid.hpp"
// -------------------- 控制参数 --------------------
constexpr float T_SHOOT = 1e-3f;  // 控制周期, 单位: s
//摩擦轮转速rad/s
#ifdef HERO_DOG
constexpr float FRIC_SPEED = 390.0f;
#endif

#ifdef HERO_THREE_WHEELS
constexpr float FRIC_SPEED = 670.0f;  //23.5度 21.5射速
#endif

// -------------------- SHOOT_INIT相关 --------------------
//拨弹轮堵转反转角度
constexpr float TRIGGER_BACK_ANGLE = 0.6f;  //rad
//拨弹轮双发反转角度
constexpr float TRIGGER_SINGLE_BACK_ANGLE = 0.4f;  //rad

// -------------------- SHOOT_READY_SINGLE相关 --------------------
//每次射击冷却时间
constexpr uint16_t SHOOT_COLD_TIME = 500;  //ms
//每次射击拨弹轮旋转角度
constexpr float SHOOT_ANGLE_ADD = 1.0472f;  //rad  3.1415926*2/6=1.0472
//双发判断一次射击摩擦轮降速阈值
#ifdef HERO_DOG
constexpr float FRIC_SPEED_REDUCE = 17.5f;
#endif
#ifdef HERO_THREE_WHEELS
constexpr float FRIC_SPEED_REDUCE = 17.0f;
#endif
//双发检测持续时间
constexpr uint16_t DETECT_TIME = 500;
constexpr float HEAT_PER_SHOT = 40.0f;

// -------------------- 对外硬件 --------------------
inline sp::RM_Motor trigger_motor(5, sp::RM_Motors::M3508, RADUCTION_RATIO);
inline sp::RM_Motor fric_motor1(1, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor2(2, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor3(3, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor4(4, sp::RM_Motors::M3508, 1);

// -------------------- 对外接口 --------------------
extern uint8_t shoot_mode_flag;
//发送给拨弹轮的扭矩（用于判断拨弹轮是否堵转）
extern float trigger_give_torque;

extern float fric_target_speed;
extern float aim_trigger_speed_before;
extern float trigger_target_speed;
extern float trigger_target_angle;
extern uint16_t fric_target_speed_up_count;

extern bool shoot_continue_permission;
extern uint16_t single_shoot_over_time;
extern uint16_t single_shoot_time;
extern uint16_t shooting_single_count;
extern bool double_shoot_flag;

extern uint16_t shoot_init_time;
extern uint16_t shoot_init_over_time;
extern float speed;
extern float last_initial_speed;
extern float initial_speed;

//摩擦轮降速时间阈值
extern uint64_t shoot_speed_reduce_time;
//摩擦轮升速时间阈值
extern uint64_t shoot_speed_up_time;

extern int64_t shoot_single_double_time;
extern bool double_shoot_flag;
extern float shoot_time_first;
extern float shoot_time_second;

extern uint16_t shoot_time;
extern float shoot_speed;
extern uint16_t ShootTime;
extern float cooling_remain;
extern float cal_heat;
extern uint32_t shoot_count;
extern bool shoot_done_flag;
extern float fric_on_speed;
extern uint16_t fric_target_change_count;
extern bool fric_target_change_flag;

//状态机初始化
void shoot_mode_init(void);
//摩擦轮状态机控制函数
void fric_mode_control(void);
//拨弹轮状态机控制函数
void trigger_mode_control(void);
//摩擦轮解算
void fric_cmd(void);
//拨弹轮解算
void trigger_cmd(void);
//SHOOT_INIT模式下拨弹轮反转解算
void shoot_init_cmd(void);
//SHOOT_SINGLE_RREADY下允许射击标识符+目标角度
void shoot_single_permission(void);
//双发检测
void shoot_double_detect();
#endif