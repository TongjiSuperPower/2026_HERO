#include "power_control.hpp"

#include <iostream>

#include "chassis_task.hpp"
#include "cmsis_os.h"
#include "controllers/pids.hpp"
#include "math.h"
#include "referee/pm02/pm02.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"

//扭矩衰减系数
float K[4] = {0.0f, 0.0f, 0.0f, 0.0f};  //缩放系数(-1,1)

static inline bool float_equal(float a, float b) { return fabs(a - b) < 1e-5f; }

//底盘功率限制函数
//Wheel_Torque * Data1为pid计算出来的目标扭矩, Wheel_speed * Data2为当前转速， Wheel_Speed * Data3为预期转速
void chassis_power_control(
  Wheel_Torque * Data1, Wheel_Speed * Data2, Wheel_Speed * Data3, float P_max)
{
  auto stamp_ms = osKernelSysTick();  // 获取当前的系统时间戳（以毫秒为单位）
  float wheel_give_torque[4] = {Data1->lf, Data1->lr, Data1->rf, Data1->rr};
  float wheel_speed[4] = {Data2->lf, Data2->lr, Data2->rf, Data2->rr};
  float wheel_target_speed[4] = {Data3->lf, Data3->lr, Data3->rf, Data3->rr};

  //所需总功率
  float sum_cmd_power = 0.0f;
  //实际可分配功率（考虑动能回收）
  float allocatable_power = P_max;
  //轮速小于目标速度的轮电机所需总功率
  float sum_required_power = 0.0f;
  //判断各轮电机与目标转速衰减率
  float K_damping[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  float K_damping_max = 0.0f;
  //所有电机按照最大速度衰减率缩放后的速度
  float wheel_speed_damping[4] = {1.0f, 1.0f, 1.0f, 1.0f};

  //单个轮电机所需功率
  float cmd_power[4];
  //轮电机速度与衰减后目标速度误差
  float wheel_speed_error[4];
  //轮电机速度与衰减后目标速度总误差
  float sum_wheel_error = 0.0f;
  //误差置信度
  float error_confidence = 0.0f;

  //判断轮组电机是否在线
  bool wheel_is_alive[4];
  wheel_is_alive[0] = wheel_lf.is_alive(stamp_ms);
  wheel_is_alive[1] = wheel_lr.is_alive(stamp_ms);
  wheel_is_alive[2] = wheel_rf.is_alive(stamp_ms);
  wheel_is_alive[3] = wheel_rr.is_alive(stamp_ms);

  //根据各电机当前速度和目标速度得到最大速度衰减率
  for (int i = 0; i < 4; i++) {
    if (wheel_is_alive[i]) {
      K_damping[i] = fabs(wheel_speed[i] / wheel_target_speed[i]);
    }
    else {
      K_damping[i] = 0.0f;
    }
  }
  K_damping_max =
    std::max(std::max(K_damping[0], K_damping[1]), std::max(K_damping[2], K_damping[3]));

  //计算允许分配功率、误差
  for (int i = 0; i < 4; i++) {
    if (wheel_is_alive[i]) {
      wheel_speed_damping[i] = K_damping_max * wheel_target_speed[i];
      cmd_power[i] = wheel_give_torque[i] * wheel_speed[i] +
                     K1 * wheel_give_torque[i] * wheel_give_torque[i] +
                     K2 * wheel_speed[i] * wheel_speed[i] + K3 / 4.0f;
      wheel_speed_error[i] = fabs(wheel_speed[i] - wheel_speed_damping[i]);
      sum_cmd_power += cmd_power[i];
      if (float_equal(cmd_power[i], 0.0f) || cmd_power[i] < 0.0f) {
        allocatable_power += -cmd_power[i]; //动能回收？
      }
      else {
        sum_wheel_error += wheel_speed_error[i];
        sum_required_power += cmd_power[i];
      }
    }
    else {
      cmd_power[i] = 0.0f;
      wheel_speed_error[i] = 0.0f;
    }
  }

  //功率重分配
  //所需功率过大则立刻减速，不按误差分配
  if ((super_cap.power_in - super_cap.power_out - allocatable_power) > 20.0f) {
    error_confidence = 0.0f;
  }
  else if ((super_cap.power_in - super_cap.power_out - allocatable_power) > 10.0f) {
    error_confidence = sp::limit_min_max(
      ((super_cap.power_in - super_cap.power_out - allocatable_power) - 10.0f) / (20.0f - 10.0f),
      0.0f, 1.0f);
  }
  else {
    error_confidence = 1.0f;
  }

  //计算各轮电机所需力矩
  for (int i = 0; i < 4; i++) {
    if (!wheel_is_alive[i]) {
      K[i] = 0.0f;
    }
    else {
      //动能回收
      if (float_equal(cmd_power[i], 0.0f) || cmd_power[i] < 0.0f) {
        K[i] = 1.0f;
        continue;
      }
      float powerWeight_Error = wheel_speed_error[i] / sum_wheel_error;
      float powerWeight_Prop = cmd_power[i] / sum_required_power;
      float powerWeight =
        error_confidence * powerWeight_Error + (1.0f - error_confidence) * powerWeight_Prop;

      float Ewt = wheel_give_torque[i] * wheel_speed[i];
      float Et2 = wheel_give_torque[i] * wheel_give_torque[i];
      float Ew2 = wheel_speed[i] * wheel_speed[i];
      float a = K1 * Et2;
      float b = Ewt;
      float c = K2 * Ew2 + K3 / 4.0f - powerWeight * allocatable_power;

      if ((b * b - 4 * a * c) >= 0.0f) {
        K[i] = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
      }
      else {
        K[i] = 1.0f;
      }

      if (K[i] < -1.0f || K[i] > 1.0f) {
        K[i] = 1.0f;
      }
    }
  }

  //力矩衰减后输出
  Data1->lf = K[0] * Data1->lf;
  Data1->lr = K[1] * Data1->lr;
  Data1->rf = K[2] * Data1->rf;
  Data1->rr = K[3] * Data1->rr;
}