#include "shoot_task.hpp"

#include "A_HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "controllers/detect_task.hpp"
#include "controllers/keys.hpp"
#include "controllers/mode.hpp"
#include "controllers/pids.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "tools/math_tools/math_tools.hpp"

//目标摩擦轮转速
float fric_target_speed = FRIC_SPEED;
//摩擦轮设定开启转速
float fric_on_speed = FRIC_SPEED;
//目标拨弹轮位置
float trigger_target_angle = 0.0f;
//目标拨弹轮转速
float trigger_target_speed = 0.0f;
//摩擦轮转速正常（从down切换到on）
bool fric_speed_normal_flag = false;
//上一次射速
float last_initial_speed = 0.0f;
//射速
float initial_speed = 0.0f;
//摩擦轮变速完成标识位：1为已经完成变速，0为未完成变速
bool fric_speed_change_done_flag = false;
//卡弹/测速模块离线检测时间
uint32_t shoot_error_time = 0;
//在一次连续打弹中摩擦轮变速标识符（超射速阈值降速）：1为有变速，0为未变速（摩擦轮速度接近目标值之后置为0）
bool fric_target_change_flag = false;
//射速调控按键按下计数器
uint16_t fric_target_change_count = 500;

// -------------------- 状态机与射击标识符相关 --------------------
extern gimbal_mode Gimbal_Mode;

// -------------------- SHOOT_INIT相关 --------------------
//发送给拨弹轮的扭矩（用于判断拨弹轮是否堵转）
float trigger_give_torque = 0.0f;
//倒转时间
uint16_t shoot_init_time;
//倒转完成时间
uint16_t shoot_init_over_time;
//判断处于倒转状态标识符
bool shoot_init_state_flag = false;

// -------------------- SHOOT_READY_SINGLE相关 --------------------
//每次射击冷却时间
uint16_t single_shoot_cold_time = 0;
//射击时间
uint16_t single_shoot_time;
//射击完成时间
uint16_t single_shoot_over_time;
//判断上一发是否射击完成
bool single_shoot_over_flag = true;

//双发检测：若双发（考虑正好转到小弹丸位置），则反转10°
//双发标识符（检测到双发准备反转）
bool double_shoot_flag = false;
//记录热量变化时间（第一发）
float shoot_time_first = 0.0f;
//记录热量变化时间（第二发）
float shoot_time_second = 0.0f;
//表示开火
bool shoot_fire_flag = false;
//保存上一次目标拨弹轮转速
float aim_trigger_speed_before = 0.0f;
//剩余热量，只要大于100就可以发射
float heat_remain = 0.0f;

//射击模式初始化
void shoot_mode_init(void)
{
  Fric_Mode = FRIC_DOWN;
  Trigger_Mode = SHOOT_READY_SINGLE;
  fric_target_speed = FRIC_SPEED;
}

void fric_mode_control(void)
{
#ifdef VT03
  static bool last_remote_fn_l = vt03.fn_l;
  if (Global_Mode == ZERO_FORCE || pm02.robot_status.power_management_shooter_output == 0) {
    Fric_Mode = FRIC_DOWN;
    Shoot_Mode = FIRE_DOWN;
    fric_speed_normal_flag = false;
  }
  else {
    if (vt03.fn_l && !last_remote_fn_l) {
      if (Fric_Mode == FRIC_DOWN || Fric_Mode == FRIC_OFF) {
        Fric_Mode = FRIC_ON;
      }
      else {
        Fric_Mode = FRIC_OFF;
        Shoot_Mode = FIRE_DOWN;
        fric_speed_normal_flag = false;
      }
    }
  }
  last_remote_fn_l = vt03.fn_l;
#endif

#ifdef DT7
  static bool last_fric_flag = remote.sw_l == sp::DBusSwitchMode::UP;
  if (Global_Mode == ZERO_FORCE || pm02.robot_status.power_management_shooter_output == 0) {
    Fric_Mode = FRIC_DOWN;
    Shoot_Mode = FIRE_DOWN;
    fric_speed_normal_flag = false;
  }
  else {
    if (remote.sw_l == sp::DBusSwitchMode::UP && !last_fric_flag) {
      if (Fric_Mode == FRIC_DOWN || Fric_Mode == FRIC_OFF) {
        Fric_Mode = FRIC_ON;
      }
      else {
        Fric_Mode = FRIC_OFF;
        Shoot_Mode = FIRE_DOWN;
        fric_speed_normal_flag = false;
      }
    }
  }
  last_fric_flag = remote.sw_l == sp::DBusSwitchMode::UP;
#endif
}

//根据全局状态与摩擦轮状态决定拨弹轮状态
void trigger_mode_control(void)
{
#ifdef VT03
  static bool last_remote_pause = vt03.pause;
  static bool last_fn_r = vt03.fn_r;
  //down掉之后（一局结束后）重置为单发
  if (Global_Mode == ZERO_FORCE) {
    Trigger_Mode = SHOOT_READY_SINGLE;
    shoot_init_state_flag = false;
  }
  //拨弹轮初始化反转时不选择状态机
  else if (shoot_init_state_flag) {
    return;
  }
  //拨弹轮堵转或者双发则初始化反转
  else if (trigger_block_flag) {
    Trigger_Mode = SHOOT_INIT;
    shoot_init_state_flag = true;
    trigger_target_angle = trigger_motor.angle - TRIGGER_BACK_ANGLE;
    trigger_block_flag = false;
    double_shoot_flag = false;
  }
  else if (double_shoot_flag) {
    Trigger_Mode = SHOOT_INIT;
    shoot_init_state_flag = true;
    trigger_target_angle = trigger_target_angle - TRIGGER_SINGLE_BACK_ANGLE;
    trigger_block_flag = false;
    double_shoot_flag = false;
  }
  //REMOTE模式下状态机选择
  else if (Global_Mode == REMOTE) {
    if (Last_Global_Mode != REMOTE) {
      Trigger_Mode = SHOOT_READY_SINGLE;
    }
  }

  //KEYBOARD模式下状态机选择
  else if (Global_Mode == KEYBOARD) {
//按z切换单发连发
#ifdef PLAYER
    if (mode_time > 0) {
      mode_time--;
    }
    if (Trigger_Mode == SHOOT_INIT) {
      Trigger_Mode = Last_Trigger_Mode;
    }
    if (Trigger_Mode == SHOOT_CLEAR) {
      Trigger_Mode = SHOOT_READY_CONTINUE;
    }
    //打符自动进单发
    if (key_small_buff || key_big_buff) {
      if (!buff_mode_flag) {
        Buff_Before_Trigger_Mode = Trigger_Mode;
      }
      Trigger_Mode = SHOOT_READY_SINGLE;
      Last_Trigger_Mode = Trigger_Mode;
      buff_mode_flag = true;
    }
    if (buff_mode_flag) {
      if ((!key_small_buff) && (!key_big_buff)) {
        buff_mode_flag = false;
        Trigger_Mode = Buff_Before_Trigger_Mode;
        Last_Trigger_Mode = Trigger_Mode;
      }
    }
#endif
#ifdef SHOOT_DEBUG
    Trigger_Mode = SHOOT_READY_CONTINUE;
#endif
#ifdef VISION
    //授权单发射击
    Trigger_Mode = SHOOT_READY_SINGLE;
#endif
  }
  // 更新历史按键状态
  last_fn_r = vt03.fn_r;
  last_remote_pause = vt03.pause;

#endif

#ifdef DT7
  if (Global_Mode == ZERO_FORCE) {
    Trigger_Mode = SHOOT_DOWN;
    shoot_init_state_flag = false;
  }
  //拨弹轮初始化反转时不选择状态机
  else if (shoot_init_state_flag) {
    return;
  }
  //拨弹轮堵转或者双发则初始化反转
  else if (trigger_block_flag) {
    Trigger_Mode = SHOOT_INIT;
    shoot_init_state_flag = true;
    trigger_target_angle = trigger_motor.angle - TRIGGER_BACK_ANGLE;
    trigger_block_flag = false;
    double_shoot_flag = false;
  }
  else if (double_shoot_flag) {
    Trigger_Mode = SHOOT_INIT;
    shoot_init_state_flag = true;
    trigger_target_angle = trigger_target_angle - TRIGGER_SINGLE_BACK_ANGLE;
    trigger_block_flag = false;
    double_shoot_flag = false;
  }
  //REMOTE模式下状态机选择
  else if (Global_Mode == REMOTE) {
    if (Fric_Mode == FRIC_ON) {
      Trigger_Mode = SHOOT_READY_SINGLE;
    }
    if (Fric_Mode == FRIC_OFF || Fric_Mode == FRIC_DOWN) {
      Trigger_Mode = SHOOT_DOWN;
    }
  }
  else if (Global_Mode == KEYBOARD) {
    if (Fric_Mode == FRIC_ON) {
      Trigger_Mode = SHOOT_READY_SINGLE;
    }
    if (Fric_Mode == FRIC_OFF || Fric_Mode == FRIC_DOWN) {
      Trigger_Mode = SHOOT_DOWN;
    }
  }
#endif
}

//摩擦轮目标速度设置与调整
void fric_cmd(void)
{
  float speed = (fabs(fric_motor1.speed) + fabs(fric_motor2.speed)) / 2.0f;
  //摩擦轮转速正常（从down切换到on到转速稳定了;变目标速度稳定之后再打弹）
  if (!fric_speed_normal_flag && fric_target_speed != 0) {
    if (fabs(speed - fric_target_speed) < 2.0f) {
      fric_speed_normal_flag = true;
    }
  }
  //摩擦轮速度切换的标志位
  if (!fric_target_change_flag) {
    if (fabs(speed - fric_target_speed) < 2.0f) {
      fric_target_change_flag = true;
    }
  }

  if (Fric_Mode == FRIC_ON) {
    last_initial_speed = initial_speed;
    initial_speed = pm02.shoot.initial_speed;
    //摩擦轮设置目标速度
    if (Last_Fric_Mode != FRIC_ON) {
      fric_target_speed = fric_on_speed;
    }
    //最高射速限制12：超射速摩擦轮自动降速
    if ((initial_speed != last_initial_speed) && pm02.shoot.initial_speed > 12.0f && key_shoot) {
      fric_target_speed -= 20.0f;
      fric_target_change_flag = false;
    }
    //操作手手动调整摩擦轮转速
    if (pm02.game_status.game_progress == 1) {
      if (fric_target_change_count > 0) {
        fric_target_change_count--;
      }
      if (key_fric_up && fric_target_change_count == 0) {
        fric_target_change_count = 300;
        fric_on_speed += 10.0f;
        fric_target_speed = fric_on_speed;
      }
      if (key_fric_down && fric_target_change_count == 0) {
        fric_target_change_count = 300;
        fric_on_speed -= 10.0f;
        fric_target_speed = fric_on_speed;
      }
    }
    // fric_target_speed = sp::limit_min_max(fric_target_speed, 300.0f, 600.0f);
    Last_Fric_Mode = FRIC_ON;
  }
  if (Fric_Mode == FRIC_OFF) {
    Last_Fric_Mode = FRIC_OFF;
    fric_target_speed = 0.0f;
    fric_speed_normal_flag = false;
  }
  if (Fric_Mode == FRIC_DOWN) {
    Last_Fric_Mode = FRIC_DOWN;
    fric_speed_normal_flag = false;
  }
}

//拨弹轮
void trigger_cmd(void)
{
  if (Global_Mode == ZERO_FORCE) {
    trigger_motor.cmd(0.0f);
  }
  else {
    if (Trigger_Mode == SHOOT_INIT) {
      shoot_init_cmd();
    }
    else if (Fric_Mode == FRIC_ON && Trigger_Mode == SHOOT_READY_SINGLE) {
      shoot_single_permission();
      shoot_double_detect();
    }
    else {
      trigger_motor.cmd(0.0f);
    }
  }
}

//SHOOT_INIT模式下拨弹轮反转解算
void shoot_init_cmd(void)
{
  if (fabs(trigger_target_angle - trigger_motor.angle) < 0.05f) {
    shoot_init_over_time++;
  }
  shoot_init_time++;

  if (shoot_init_time == 1000 || shoot_init_over_time > 50) {
    shoot_init_time = 0;
    shoot_init_over_time = 0;
    shoot_init_state_flag = false;
    trigger_block_flag = false;
    //保证进入单发之后的连续性（保证single_shoot_over_flag判断上一发（单发连发均可判断）是否射击完成）
    // trigger_target_angle = trigger_motor.angle;
  }
}

//permission——>赋值目标+重置冷却——>射击——>冷却——>permission
//SHOOT_SINGLE_READY下允许射击标识符+目标角度
void shoot_single_permission(void)
{
  if (!single_shoot_over_flag) {
    //计算上一次射击完成时间
    if (fabs(trigger_target_angle - trigger_motor.angle) < 0.04f) {
      single_shoot_over_time++;
    }
    //计算上一次射击持续时间
    single_shoot_time++;
  }

  //判断上一次射击是否完成
  if (single_shoot_over_time > 50 || single_shoot_time > 149) {
    single_shoot_over_flag = true;
    single_shoot_over_time = 0;
    single_shoot_time = 0;
  }

  //上次射击完成则开始减少冷却
  if (single_shoot_over_flag) {
    if (single_shoot_cold_time > 0) {
      single_shoot_cold_time--;
    }
  }

  //根据剩余热量进行热量控制
  heat_remain = pm02.robot_status.shooter_barrel_heat_limit - HEAT_PER_SHOT;

  //SHOOT_READY_SINGLE射击条件：摩擦轮开+冷却结束+左拨杆下档/左键
  if (Global_Mode == REMOTE) {
    if (Fric_Mode == FRIC_ON && remote_shoot && single_shoot_cold_time == 0 && heat_remain > 0.0f) {
      trigger_target_angle = trigger_motor.angle - SHOOT_ANGLE_ADD;
      single_shoot_cold_time = SHOOT_COLD_TIME;
      single_shoot_over_flag = false;
    }
  }
  if (Global_Mode == KEYBOARD) {
    if (
      Fric_Mode == FRIC_ON &&
      (key_shoot || (vis.fire && vis.control && Gimbal_Mode == GIMBAL_AUTO)) &&
      single_shoot_cold_time == 0 && heat_remain > 0.0f) {
      trigger_target_angle = trigger_motor.angle - SHOOT_ANGLE_ADD;
      single_shoot_cold_time = SHOOT_COLD_TIME;
      single_shoot_over_flag = false;
    }
  }
}

//双发检测
void shoot_double_detect(void)
{
  auto stamp_ms = osKernelSysTick();  // 获取当前的系统时间戳（以毫秒为单位）
  if (shoot_fire_flag) {
    if (shoot_time_first < shoot_time_second) {
      shoot_time_first = stamp_ms;
    }
    else {
      shoot_time_second = stamp_ms;
    }
  }
  if (
    fabs(shoot_time_first - shoot_time_second) < (SHOOT_COLD_TIME - 10) &&
    shoot_time_first != 0.0f && shoot_time_second != 0.0f) {
    double_shoot_flag = true;
    shoot_time_first = 0.0f;
    shoot_time_second = 0.0f;
  }
  else {
    double_shoot_flag = false;
  }
}
