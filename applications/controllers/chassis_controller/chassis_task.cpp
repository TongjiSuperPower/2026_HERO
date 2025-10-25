#include "chassis_task.hpp"

#include <algorithm>
#include <iostream>

#include "cmsis_os.h"
#include "controllers/mode.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "power_control.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"

//底盘初始化
bool chassis_init_flag = false;
uint32_t chassis_init_time = 0;
uint32_t chassis_init_over_time = 0;
//麦轮解算
sp::Mecanum chassis(WHEEL_RADIUS, (CHASSIS_LENGTH / 2), (CHASSIS_WIDTH / 2));
//修正
sp::Mecanum corrector(WHEEL_RADIUS, (CHASSIS_LENGTH / 2), (CHASSIS_WIDTH / 2));

//低压放电保护 0：电压低不可使用电容  1：可使用电容
uint8_t low_vol_flag = 1;
//底盘期望前后、旋转速度
Chassis_Speed chassis_speed = {0.0f, 0.0f, 0.0f};

//经过偏移修正之后的轮子目标速度
Wheel_Speed chassis_target_speed = {0.0f, 0.0f, 0.0f, 0.0f};
//当前底盘四个电机轮子转速
Wheel_Speed wheel_speed = {0.0f, 0.0f, 0.0f, 0.0f};
//轮子平滑后速度（仅用作pid控制）
float wheel_lf_speed_filter = 0.0f;
float wheel_lr_speed_filter = 0.0f;
float wheel_rf_speed_filter = 0.0f;
float wheel_rr_speed_filter = 0.0f;
//pid算出来的预期扭矩
Wheel_Torque wheel_give_torque = {0.0f, 0.0f, 0.0f, 0.0f};

//根据yaw相对角度将底盘目标速度投影到云台坐标系下
extern float yaw_relative_angle;
//记录wz死区
uint16_t count_w = 500;

//小陀螺改变转向flag（检录使用）
bool spin_revert_flag;

//功率控制相关预期功率
float infact_Pmax = 0.0f;
//总使用能量
float energy_sum = 0.0f;
//滤波后底盘速度
float chassis_wz_filter = 0.0f;

//底盘模式选择
void chassis_mode_control();
//chassis_follow,spin下的遥控器/键鼠对应速度
void remote_speedcontrol_follow();
void keyboard_speedcontrol_follow(bool key);
void keyboard_speedcontrol_spin();
//chassis_follow通用坐标系变至云台系+底盘跟随的函数
void chassis_coordinate_converter(Chassis_Speed * chassis_speed_given, float yaw_angle);
//底盘层面pid解算
Wheel_Torque chassis_pid_cal(float lf, float lr, float rf, float rr);

//底盘模式与目标速度确定
extern "C" void Chassis_Task()
{
  while (1) {
    //总能量更新
    if (game_start_flag) {
      energy_sum += super_cap.power_in * 0.001f;
    }
    else {
      energy_sum = 0.0f;
    }

    //底盘模式选择
    chassis_mode_control();
    //Chassis_init退出条件：
    if (chassis_init_flag) {
      if (chassis_alive) {
        chassis_init_over_time++;
      }
      chassis_init_time++;
      if (chassis_init_over_time == 100 || chassis_init_time == 7000) {
        chassis_init_over_time = 0;
        chassis_init_time = 0;
        chassis_init_flag = false;
      }
    }
    //底盘逆运动学，由四轮速度求解底盘速度
    chassis.update(wheel_lf.speed, wheel_lr.speed, wheel_rf.speed, wheel_rr.speed);
    //平滑底盘角速度
    chassis_wz_measured_filter.update(chassis.wz);
    chassis_wz_filter = chassis_wz_measured_filter.out;
    //平滑3508速度
    wheel_lf_filter.update(wheel_lf.speed);
    wheel_lf_speed_filter = wheel_lf_filter.out;
    wheel_lr_filter.update(wheel_lr.speed);
    wheel_lr_speed_filter = wheel_lr_filter.out;
    wheel_rf_filter.update(wheel_rf.speed);
    wheel_rf_speed_filter = wheel_rf_filter.out;
    wheel_rr_filter.update(wheel_rr.speed);
    wheel_rr_speed_filter = wheel_rr_filter.out;

    if (Chassis_Mode == CHASSIS_FOLLOW) {
      //遥控器模式
      if (Global_Mode == REMOTE) {
        remote_speedcontrol_follow();
      }
      //键鼠模式
      if (Global_Mode == KEYBOARD) {
        keyboard_speedcontrol_follow(key_cap);
      }
    }
    if (Chassis_Mode == CHASSIS_SPIN) {
      chassis_speed.wz = (spin_revert_flag ? -SPIN_W : SPIN_W);
      keyboard_speedcontrol_spin();
    }
    // chassis_coordinate_converter(&chassis_speed, yaw_relative_angle);
    // chassis.calc(chassis_speed.vx, chassis_speed.vy, chassis_speed.wz);
    // wheel_speed.lf = wheel_lf.speed;
    // wheel_speed.lr = wheel_lr.speed;
    // wheel_speed.rf = wheel_rf.speed;
    // wheel_speed.rr = wheel_rr.speed;

    // chassis_target_speed.lf = chassis.speed_lf;
    // chassis_target_speed.lr = chassis.speed_lr;
    // chassis_target_speed.rf = chassis.speed_rf;
    // chassis_target_speed.rr = chassis.speed_rr;

    chassis_coordinate_converter(&chassis_speed, yaw_relative_angle);
    chassis.calc(chassis_speed.vx, chassis_speed.vy, chassis_speed.wz);
    wheel_speed.lf = wheel_lf.speed;
    wheel_speed.lr = wheel_lr.speed;
    wheel_speed.rf = wheel_rf.speed;
    wheel_speed.rr = wheel_rr.speed;

    chassis_target_speed.lf = chassis.speed_lf;
    chassis_target_speed.lr = chassis.speed_lr;
    chassis_target_speed.rf = chassis.speed_rf;
    chassis_target_speed.rr = chassis.speed_rr;

    osDelay(1);
  }
}

void chassis_mode_control()
{
  static bool chassis_output_zero = pm02.robot_status.power_management_chassis_output;
  if (!chassis_output_zero && pm02.robot_status.power_management_chassis_output) {
    chassis_init_flag = true;
  }
  chassis_output_zero = pm02.robot_status.power_management_chassis_output;
  //底盘初始化优先级最高
  if (chassis_init_flag) {
    Chassis_Mode = CHASSIS_INIT;
    return;
  }
  //无力状态或云台初始化状态下底盘无力
  if (Global_Mode == ZERO_FORCE || Gimbal_Mode == GIMBAL_INIT) {
    Chassis_Mode = CHASSIS_DOWN;
  }

  else {
#ifdef DT7
    //DT7遥控器模式
    if (Global_Mode == REMOTE) {
      if (remote.ch_lu < 0) {
        Chassis_Mode = CHASSIS_SPIN;
        spin_revert_flag = true;
      }
      else if (remote.ch_lu > 0) {
        Chassis_Mode = CHASSIS_SPIN;
        spin_revert_flag = false;
      }
      else {
        Chassis_Mode = CHASSIS_FOLLOW;
      }
    }

    //键鼠模式
    if (Global_Mode == KEYBOARD) {
      //不在小陀螺模式时，底盘进入跟随模式
      if (!key_spin) {
        Chassis_Mode = CHASSIS_FOLLOW;
      }
      //SHIFT放在后面保证任何模式下按下shift都进入小陀螺
      if (key_spin) {
        Chassis_Mode = CHASSIS_SPIN;
      }
    }
#endif

#ifdef VT03
    //VT03图传链路
    if (Global_Mode == REMOTE) {
      if (vt03.wheel < 0) {
        Chassis_Mode = CHASSIS_SPIN;
        spin_revert_flag = true;
      }
      else if (vt03.wheel > 0) {
        Chassis_Mode = CHASSIS_SPIN;
        spin_revert_flag = false;
      }
      else {
        Chassis_Mode = CHASSIS_FOLLOW;
      }
    }
    //键鼠模式
    if (Global_Mode == KEYBOARD) {
      //不在小陀螺和打符模式时，底盘进入跟随模式
      if (!key_spin) {
        Chassis_Mode = CHASSIS_FOLLOW;
      }
      //SHIFT放在后面保证任何模式下按下shift都进入小陀螺
      if (key_spin) {
        Chassis_Mode = CHASSIS_SPIN;
      }
    }
#endif
  }
}

//chassis_follow下的遥控器对应速度
void remote_speedcontrol_follow(void)
{
  // w:左正
  // vx+：前
  // vy+：左
  chassis_speed.vx = REMOTE_CONTROL_V * remote_move_x;
  chassis_speed.vy = -REMOTE_CONTROL_V * remote_move_y;
  //底盘跟随角速度
  //平滑控制底盘跟随，解决启停扭动问题
  yaw_relative_angle_filter.update(yaw_relative_angle);
  yaw_relative_angle = yaw_relative_angle_filter.out;
  chassis_follow_wz_pid.calc(0.0f, yaw_relative_angle); //底盘跟随：设为底盘与yaw轴相对角度为0
  chassis_speed.wz = -chassis_follow_wz_pid.out;
}

//chassis_follow下的键鼠对应速度
void keyboard_speedcontrol_follow(bool key)
{
  float vx = 0.0f;
  float vy = 0.0f;
  //不开电容
  if (key == 0) {
    //按下按键后速度为3.2m/s，左右键同时按理论上底盘速度应该是零
    vx =
      (key_move_x_up ? KEYBOARD_CONTROL_V : 0.0f) + (key_move_x_down ? -KEYBOARD_CONTROL_V : 0.0f); 
    vy =
      (key_move_y_up ? KEYBOARD_CONTROL_V : 0.0f) + (key_move_y_down ? -KEYBOARD_CONTROL_V : 0.0f);
  }
  //开电容后速度为4.0m/s
  else {
    vx = (key_move_x_up ? KEYBOARD_CAP_CONTROL_V : 0.0f) +
         (key_move_x_down ? -KEYBOARD_CAP_CONTROL_V : 0.0f);
    vy = (key_move_y_up ? KEYBOARD_CAP_CONTROL_V : 0.0f) +
         (key_move_y_down ? -KEYBOARD_CAP_CONTROL_V : 0.0f);
  }
  //底盘一阶低通滤波，目的是平滑速度
  chassis_follow_vx_filter.update(vx);
  chassis_speed.vx = chassis_follow_vx_filter.out;
  chassis_follow_vy_filter.update(vy);
  chassis_speed.vy = chassis_follow_vy_filter.out;
  //在原本速度也很小时，若滤波之后速度输出在死区内，则限制为0（停下迅速）
  if (fabs(vx) < ZeroXY || fabs(vy) < ZeroXY) {
    if (fabs(chassis_speed.vx) < ZeroXY) {
      chassis_speed.vx = 0.0f;
    }
    if (fabs(chassis_speed.vy) < ZeroXY) {
      chassis_speed.vy = 0.0f;
    }
  }
  //平滑控制底盘跟随，解决启停扭动问题
  yaw_relative_angle_filter.update(yaw_relative_angle);
  yaw_relative_angle = yaw_relative_angle_filter.out;
  chassis_follow_wz_pid.calc(0.0f, yaw_relative_angle);
  chassis_speed.wz = -chassis_follow_wz_pid.out;
  chassis_follow_wz_filter.update(chassis_speed.wz);
  chassis_speed.wz = chassis_follow_wz_filter.out;
}

//chassis_spin下的键鼠对应速度
void keyboard_speedcontrol_spin(void)
{
  float vx = 0.0f;
  float vy = 0.0f;

  if (infact_Pmax <= 150.0f) {
    // vx:前后  vy：左右
    vx = (key_move_x_up ? SPIN_V * chassis_speed.wz / SPIN_W : 0.0f) +
         (key_move_x_down ? -SPIN_V * chassis_speed.wz / SPIN_W : 0.0f) +
         (key_move_y_up ? 0.7f * SPIN_V * chassis_speed.wz / SPIN_W : 0.0f) +
         (key_move_y_down ? -0.7f * SPIN_V * chassis_speed.wz / SPIN_W : 0.0f);
    vy = (key_move_x_up ? -0.7f * SPIN_V * chassis_speed.wz / SPIN_W : 0.0f) +
         (key_move_x_down ? 0.7f * SPIN_V * chassis_speed.wz / SPIN_W : 0.0f) +
         (key_move_y_up ? SPIN_V * chassis_speed.wz / SPIN_W : 0.0f) +
         (key_move_y_down ? -SPIN_V * chassis_speed.wz / SPIN_W : 0.0f);
    vx = sp::limit_max(vx, SPIN_V);
    vy = sp::limit_max(vy, SPIN_V);
  }
  else {
    vx = (key_move_x_up ? 1.3f : 0.0f) + (key_move_x_down ? -1.3f : 0.0f);
    vy = (key_move_y_up ? 1.3f : 0.0f) + (key_move_y_down ? -1.3f : 0.0f);
  }

  chassis_spin_vx_filter.update(vx);
  chassis_speed.vx = chassis_spin_vx_filter.out;
  chassis_spin_vy_filter.update(vy);
  chassis_speed.vy = chassis_spin_vy_filter.out;

  // // 死区
  // if (fabs(chassis_speed.vx) < 1e-6) {
  //   chassis_speed.vx = 0.0f;
  // }
  // if (fabs(chassis_speed.vy) < 1e-6) {
  //   chassis_speed.vy = 0.0f;
  // }
}

//chassis_follow通用坐标系变至云台系，解决小陀螺下平移的问题，以及底盘跟随状态下，移动以操作手视角移动
void chassis_coordinate_converter(Chassis_Speed * chassis_speed_given, float yaw_angle)
{
  //底盘解算
  float vx = chassis_speed_given->vx;
  float vy = chassis_speed_given->vy;
  chassis_speed_given->vx = vx * cos(yaw_angle) - vy * sin(yaw_angle);
  chassis_speed_given->vy = vx * sin(yaw_angle) + vy * cos(yaw_angle);
}

//根据剩余能量和电容情况确定最大功率
void Pmax_get()
{
#ifdef RMUC
  //电容低电保护
  if (low_vol_flag == 0 && super_cap.voltage >= 11.2) {
    low_vol_flag = 1;
  }
  if (low_vol_flag == 1 && super_cap.voltage < 6.2) {
    low_vol_flag = 0;
  }
  //剩余能量大于30%时 01111B = 16D
  if (pm02.buff.remaining_energy > 16) {
    if (!key_cap) {
      infact_Pmax = pm02.robot_status.chassis_power_limit;
    }
    else {
      //根据电容的电压大小增加额外功率
      if (super_cap.voltage >= 6 && low_vol_flag == 1) {
        infact_Pmax = std::min(
          CAP_MAX_POWER, (super_cap.voltage - 6.0f) * 20 + pm02.robot_status.chassis_power_limit);
      }
      else {
        infact_Pmax = pm02.robot_status.chassis_power_limit;
      }
    }
  }
  else {
    //剩余能量不足时限制最大功率为80W
    if (!key_cap) {
      if (pm02.robot_status.chassis_power_limit > 80.0f) {
        infact_Pmax = 80.0f;
      }
      else {
        infact_Pmax = pm02.robot_status.chassis_power_limit;
      }
    }
    else {
      //根据电容的电压大小增加额外功率
      if (super_cap.voltage >= 6 && low_vol_flag == 1) {
        infact_Pmax = std::min(
          CAP_MAX_POWER, (super_cap.voltage - 6.0f) * 20 + pm02.robot_status.chassis_power_limit);
      }
      else {
        infact_Pmax = pm02.robot_status.chassis_power_limit;
      }
    }
  }
#endif
  buff_energy_p_limited();
  power_filter.update(infact_Pmax);
  infact_Pmax = power_filter.out;
}

void buff_energy_p_limited()
{
  if (pm02.power_heat.buffer_energy < 20.0f) {
    //infact_Pmax 确定原则：x<45/3(对抗赛的一级功率/3：虚弱时最小功率限制)；x-3(后面留的余量)>K3，防止功率控制无解
    infact_Pmax = K3 + 4.0f;
  }
}