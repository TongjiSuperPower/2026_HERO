#include "gimbal_task.hpp"

#include "cmsis_os.h"
#include "controllers/keys.hpp"
#include "controllers/mode.hpp"
#include "controllers/shoot_controller/shoot_task.hpp"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "io/imu_task.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"

//上坡角度
//pitch相对角度滤波
sp::LowPassFilter pitch_relative_angle_filter(0.1f);
extern float slope_angle;

//云台回中模式下回中后的时间
uint16_t gimbal_init_over_time = 0;
//云台进入回中模式的时间
uint16_t gimbal_init_time = 0;
//云台是否在回中模式
uint8_t gimbal_init_flag = 0;
//掉头冷却时间1s
uint16_t turnover_cold_time = TURNOVER_COLDTIME;
//发送给上位机的射击状态标识符
uint8_t shoot_mode_flag = 0;

//云台初始化
void gimbal_init();
//云台状态选择
void gimbal_mode_control();
//云台电流解算
void gimbal_cmd();

//变量们
//当中码盘值等效换算的角度
//区间：-Π~Π
float yaw_offecd_ecd_angle = 0.0f;
float pitch_offecd_ecd_angle = 0.0f;
//yaw解算的当前码盘值相对于正中码盘值的差
float yaw_relative_angle = 0.0f;
//pit解算的当前码盘值相对于正中码盘值的差
float pitch_relative_angle = 0.0f;
//yaw，pitch目标参数
float gyro_yaw_angle_add = 0.0f;
float gyro_pitch_angle_add = 0.0f;
float yaw_target_angle = 0.0f;
float pitch_target_angle = 0.0f;
//小陀螺补偿
extern sp::Mecanum chassis;
//输入给yaw的扭矩
float yaw_cmd_torque = 0.0f;

extern "C" void Gimbal_Task()
{
  osDelay(700);  // 等待各个任务初始化完成
  //云台初始化
  gimbal_init();
  //射击初始化
  shoot_mode_init();

  while (1) {
    //云台电机选择模式
    gimbal_mode_control();
    //计算云台目标速度
    gimbal_cmd();
    //摩擦轮选择模式
    fric_mode_control();
    //拨弹轮选择模式
    trigger_mode_control();
    //计算摩擦轮目标速度
    fric_cmd();
    //计算拨弹轮目标速度
    trigger_cmd();

    osDelay(1);
  }
}

//云台初始化（被打死之后上电回中）
void gimbal_init()
{
#ifdef HERO_DOG
  yaw_offecd_ecd_angle = -0.53349f;
  pitch_offecd_ecd_angle = 0.62146f;
#endif
#ifdef HERO_THREE_WHEELS
  yaw_offecd_ecd_angle = 2.3814f;
  pitch_offecd_ecd_angle = -0.780f;
#endif
}

//云台状态选择
void gimbal_mode_control()
{
  //正在回中过程中无法调整模式
  if (gimbal_init_flag == 1) {
    return;
  }

  //DOWN
  if (Global_Mode == ZERO_FORCE) {
    Last_Gimbal_Mode = Gimbal_Mode;
    Gimbal_Mode = GIMBAL_ZERO_FORCE;
    shoot_mode_flag = 0;
  }

  //遥控器模式
  if (Global_Mode == REMOTE) {
    Last_Gimbal_Mode = Gimbal_Mode;
    Gimbal_Mode = GIMBAL_GYRO;
    shoot_mode_flag = 0;
  }

  //键鼠 长按鼠标右键自瞄
  if (Global_Mode == KEYBOARD) {
    if (!key_autoaim) {
      Last_Gimbal_Mode = Gimbal_Mode;
      Gimbal_Mode = GIMBAL_GYRO;
      shoot_mode_flag = 0;
    }
    else {
      Last_Gimbal_Mode = Gimbal_Mode;
      Gimbal_Mode = GIMBAL_AUTO;
    }
#ifdef VISION
    Gimbal_Mode = GIMBAL_AUTO;
#endif
  }
  //判断是否进入回中模式
  if (Last_Gimbal_Mode == GIMBAL_ZERO_FORCE && Gimbal_Mode != GIMBAL_ZERO_FORCE) {
    Gimbal_Mode = GIMBAL_INIT;
    gimbal_init_flag = 1;
    shoot_mode_flag = 0;
  }

#ifdef DUAL_PID
  yaw_autoaim_pos_pid.data.iout = 0.0f;
#endif
}

void gimbal_cmd()
{
  yaw_relative_angle = sp::limit_angle(yaw_motor.angle - yaw_offecd_ecd_angle);
  pitch_relative_angle = sp::limit_angle(pitch_motor.angle - pitch_offecd_ecd_angle);
  pitch_relative_angle_filter.update(pitch_relative_angle);
  pitch_relative_angle = pitch_relative_angle_filter.out;

  //云台模式为GYRO
  if (Gimbal_Mode == GIMBAL_GYRO) {
    //遥控器
    if (Global_Mode == REMOTE) {
      gyro_yaw_angle_add = -remote_yaw * W_MAX;
      gyro_pitch_angle_add = -remote_pitch * W_MAX;
      yaw_target_angle = sp::limit_angle(yaw_target_angle + gyro_yaw_angle_add);
      pitch_target_angle = sp::limit_angle(pitch_target_angle + gyro_pitch_angle_add);
      //pitch轴限角
#ifdef RMUL
      pitch_target_angle =
        sp::limit_min_max(pitch_target_angle, IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX);
#endif
#ifdef RMUC
      pitch_target_angle = sp::limit_min_max(
        pitch_target_angle, IMU_PITCH_ANGLE_MIN + slope_angle, IMU_PITCH_ANGLE_MAX + slope_angle);
#endif
    }
    //键鼠
    if (Global_Mode == KEYBOARD) {
      //陀螺仪控云台
      gyro_yaw_angle_add = -mouse_yaw * MOUSE_DPI;
      gyro_pitch_angle_add = -mouse_pitch * MOUSE_DPI;

      yaw_target_angle = sp::limit_angle(yaw_target_angle + gyro_yaw_angle_add);
      pitch_target_angle = sp::limit_angle(pitch_target_angle + gyro_pitch_angle_add);
      //掉头冷却减少
      if (turnover_cold_time > 0) {
        turnover_cold_time--;
      }
      if (key_yaw_left_90 && turnover_cold_time == 0) {
        yaw_target_angle += sp::PI / 2;
        turnover_cold_time = TURNOVER_COLDTIME;
      }
      if (key_yaw_right_90 && turnover_cold_time == 0) {
        yaw_target_angle += -sp::PI / 2;
        turnover_cold_time = TURNOVER_COLDTIME;
      }
      //按下X回头
      if (key_yaw_180 && turnover_cold_time == 0) {
        yaw_target_angle += sp::PI;
        turnover_cold_time = TURNOVER_COLDTIME;
      }
      //pitch轴限角
#ifdef RMUL
      pitch_target_angle =
        sp::limit_min_max(pitch_target_angle, IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX);
#endif
#ifdef RMUC
      pitch_target_angle = sp::limit_min_max(
        pitch_target_angle, IMU_PITCH_ANGLE_MIN + slope_angle, IMU_PITCH_ANGLE_MAX + slope_angle);
#endif
    }
  }

  //自瞄控云台
  if (Gimbal_Mode == GIMBAL_AUTO) {
    shoot_mode_flag = 1;  // 普通自瞄
    //赋予自瞄坐标
    if (vis.control) {
      yaw_target_angle = vis.yaw;
#ifdef RMUL
      pitch_target_angle = vis.pitch;
      pitch_target_angle =
        sp::limit_min_max(pitch_target_angle, IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX);
#endif
#ifdef RMUC
      pitch_target_angle = sp::limit_min_max(
        vis.pitch, IMU_PITCH_ANGLE_MIN + slope_angle, IMU_PITCH_ANGLE_MAX + slope_angle);
#endif
    }
  }

  if (Gimbal_Mode == GIMBAL_INIT) {
    yaw_target_angle = 0.0f;
    pitch_target_angle = 0.0f;
    if ((fabs(yaw_relative_angle)) < 0.05f && (fabs(pitch_relative_angle)) < 0.05f) {
      gimbal_init_over_time++;
    }
    gimbal_init_time++;

    //判断初始化完成
    if (gimbal_init_time == 1000 || gimbal_init_over_time == 500) {
      yaw_target_angle = imu.yaw;
      pitch_target_angle = imu.pitch;
      gimbal_init_over_time = 0;
      gimbal_init_time = 0;
      gimbal_init_flag = false;
    }
  }
}
