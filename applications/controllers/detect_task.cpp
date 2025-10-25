#include "detect_task.hpp"

#include "A_HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "controllers/chassis_controller/chassis_task.hpp"
#include "controllers/gimbal_controller/gimbal_task.hpp"
#include "controllers/keys.hpp"
#include "controllers/mode.hpp"
#include "controllers/shoot_controller/shoot_task.hpp"
#include "data_interfaces/uart/uart_task.hpp"
// #include "io/adc/adc.hpp"
#include "io/imu_task.hpp"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f4xx_hal_cortex.h"

float voltage = 0.0f;
//拨弹轮堵转检测
bool trigger_block_flag = false;
//电机掉线检测
bool motor_alive = true;
bool yaw_motor_alive = true;
bool pitch_motor_alive = false;
bool fric_motor_alive = true;
bool chassis_alive = true;
bool gimbal_alive = true;
bool shoot_alive = true;
//记录拨弹轮速度异常时间
uint16_t trigger_num = 0;
//坡度检测
//10°坡计数器
uint32_t slope_time_10 = 0;
//20°坡计数器
uint32_t slope_time_20 = 0;
//坡角度
float slope_angle = 0.0f;
//初始化C板重置倒计时
uint8_t reset_count = 50;
//被墙卡住检测
bool stuck_flag = false;
//被墙卡住检测倒计时
uint32_t stuck_count = 0;

//C板重置
void Cboard_reset(bool key1);
//拨弹轮堵转检测函数
bool trigger_motor_block(void);
//电机掉线检测函数
void motor_dead();

extern "C" void Detect_Task()
{
  while (1) {
#ifdef VT03
    Cboard_reset(vt03.keys.g);
#endif
#ifdef DT7
    Cboard_reset(remote.keys.g);
#endif
    //拨弹轮堵转检测
    trigger_block_flag = trigger_motor_block();
    //电机掉线检测
    motor_dead();
  }
}

//C板重置
void Cboard_reset(bool key1)
{
  if (key1 && reset_count > 0) {
    reset_count--;
  }
  else {
    reset_count = 50;
  }
  if (reset_count == 0) {
    HAL_NVIC_SystemReset();
  }
}

//电机掉线检测
void motor_dead()
{
  auto stamp_ms = osKernelSysTick();  // 获取当前的系统时间戳（以毫秒为单位）
  yaw_motor_alive = yaw_motor.is_alive(stamp_ms);
  pitch_motor_alive = pitch_motor.is_alive(stamp_ms);
  if (pm02.robot_status.power_management_shooter_output) {
    fric_motor_alive = fric_motor1.is_alive(stamp_ms) && fric_motor2.is_alive(stamp_ms);
  }
  else {
    fric_motor_alive = true;
  }

  if (pm02.robot_status.power_management_chassis_output) {
    chassis_alive = wheel_lf.is_alive(stamp_ms) && wheel_lr.is_alive(stamp_ms) &&
                    wheel_rf.is_alive(stamp_ms) && wheel_rr.is_alive(stamp_ms);
  }
  else {
    chassis_alive = true;
  }

  if (pm02.robot_status.power_management_gimbal_output) {
    gimbal_alive = yaw_motor.is_alive(stamp_ms) && pitch_motor.is_alive(stamp_ms);
  }
  else {
    gimbal_alive = true;
  }

  if (pm02.robot_status.power_management_shooter_output) {
    shoot_alive = fric_motor1.is_alive(stamp_ms) && fric_motor2.is_alive(stamp_ms) &&
                  trigger_motor.is_alive(stamp_ms);
  }
  else {
    shoot_alive = true;
  }
  motor_alive = chassis_alive && gimbal_alive && shoot_alive;
}

bool trigger_motor_block(void)
{
  //记录上一次拨弹轮速度
  float v_trigger_before = trigger_motor.speed;
  if (!trigger_block_flag) {
    if (
      fabs(trigger_give_torque) > 0.9f &&
      fabs((v_trigger_before + trigger_motor.speed) / 2.0f) < 0.5f) {
      trigger_num++;
    }
    else {
      trigger_num = 0;
    }
  }
//堵转判断
#ifdef HERO_DOG
  if (trigger_num > 25) {
    trigger_num = 0;
    return true;
  }
  return false;
#endif
}
