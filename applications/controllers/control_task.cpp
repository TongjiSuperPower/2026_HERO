#include "controllers/control_task.hpp"

#include "A_HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "controllers/chassis_controller/energy_control.hpp"
#include "controllers/chassis_controller/power_control.hpp"
#include "controllers/gimbal_controller/gimbal_task.hpp"
#include "controllers/shoot_controller/shoot_task.hpp"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "detect_task.hpp"
#include "io/imu_task.hpp"
#include "mode.hpp"
#include "pids.hpp"

uint8_t cap_send_num = 0;

//射击数据发送频率控制符
uint16_t shoot_mode_send_num = 0;
uint16_t q_send_num = 0;
//达妙使能帧控制符
uint16_t yaw_enable_num = 0;
uint16_t pitch_enable_num = 0;

void motor_enable();
void chassis_control();
Wheel_Torque chassis_pid_cal(float lf, float lr, float rf, float rr);
void shoot_init_control();
void shoot_single_control();
void pitch_init();

extern "C" void Control_Task()
{
  osDelay(500);  // 等待各个任务初始化完成
  can1.config();
  can1.start();
  can2.config();
  can2.start();
  yaw_motor.write_enable(can2.tx_data);
  can2.send(yaw_motor.tx_id);
  pitch_init();

  while (1) {
    // 确定全局状态机
    global_mode_control();
    // 失能检测,发送使能帧
    motor_enable();
    // pitch_motor.cmd_get_id();
    // pitch_motor.write(can1.tx_data);
    // can1.send_ext(pitch_motor.communication_type, 0, pitch_motor.motor_id, pitch_motor.master_id);

    // yaw
    // 清除错误帧
    if (yaw_motor.error != 1 && yaw_motor.error != 0) {
      yaw_motor.write_clear_error(can2.tx_data);
      can2.send(yaw_motor.tx_id);
      yaw_motor.write_enable(can2.tx_data);
      can2.send(yaw_motor.tx_id);
    }
    //底盘控制
    chassis_control();
    // can1 send一次；can2 send一次
    chassis_send();

    osDelay(1);
  }
}

//控制函数
//底盘轮组电机控制
void chassis_control()
{
  if (Chassis_Mode == CHASSIS_DOWN || Chassis_Mode == CHASSIS_INIT) {
    wheel_lf.cmd(0.0f);
    wheel_lr.cmd(0.0f);
    wheel_rf.cmd(0.0f);
    wheel_rr.cmd(0.0f);
    return;
  }
  wheel_give_torque = chassis_pid_cal(
    chassis_target_speed.lf, chassis_target_speed.lr, chassis_target_speed.rf,
    chassis_target_speed.rr);
  Pmax_get();
  chassis_power_control(
    &wheel_give_torque, &wheel_speed, &chassis_target_speed, infact_Pmax - 3.0f);
  chassis_energy_control(wheel_speed, &wheel_give_torque);
  wheel_lf.cmd(wheel_give_torque.lf);
  wheel_lr.cmd(wheel_give_torque.lr);
  wheel_rf.cmd(wheel_give_torque.rf);
  wheel_rr.cmd(wheel_give_torque.rr);
}

//底盘层面pid解算
Wheel_Torque chassis_pid_cal(float lf, float lr, float rf, float rr)
{
  Wheel_Torque wheel_given_torque_temp;
  speed_lf_pid.calc(lf, wheel_lf_speed_filter);
  speed_lr_pid.calc(lr, wheel_lr_speed_filter);
  speed_rf_pid.calc(rf, wheel_rf_speed_filter);
  speed_rr_pid.calc(rr, wheel_rr_speed_filter);
  wheel_given_torque_temp.lf = speed_lf_pid.out;
  wheel_given_torque_temp.lr = speed_lr_pid.out;
  wheel_given_torque_temp.rf = speed_rf_pid.out;
  wheel_given_torque_temp.rr = speed_rr_pid.out;
  return wheel_given_torque_temp;
}

void pitch_init()
{
  pitch_motor.cmd_set_single_parameter(run_mode, 0);  //设置为运行模式
  pitch_motor.write(can1.tx_data);
  can1.send_ext(pitch_motor.communication_type, 0, pitch_motor.master_id, pitch_motor.motor_id);
  osDelay(1);
  pitch_motor.cmd_motor_enable();  //电机使能
  pitch_motor.write(can1.tx_data);
  can1.send_ext(pitch_motor.communication_type, 0, pitch_motor.master_id, pitch_motor.motor_id);
}

//达妙发送使能帧
void motor_enable(void)
{
  if (yaw_motor.error == 0) {
    yaw_motor.write_enable(can2.tx_data);
    can2.send(yaw_motor.tx_id);
  }
  if (pitch_motor.error != 0) {
    pitch_motor.cmd_motor_enable();  //电机使能
    pitch_motor.write(can1.tx_data);
    can1.send_ext(pitch_motor.communication_type, 0, pitch_motor.master_id, pitch_motor.motor_id);
  }
  if (!yaw_motor_alive) {
    if (yaw_enable_num == 1000) {
      yaw_motor.write_enable(can2.tx_data);
      can2.send(yaw_motor.tx_id);
      yaw_enable_num = 0;
    }
    yaw_enable_num++;
  }
  if (!pitch_motor_alive) {
    if (pitch_enable_num == 1000) {
      pitch_motor.cmd_motor_enable();  //电机使能
      pitch_motor.write(can1.tx_data);
      can1.send_ext(pitch_motor.communication_type, 0, pitch_motor.master_id, pitch_motor.motor_id);
      pitch_enable_num = 0;
    }
    pitch_enable_num++;
  }
}

//摩擦轮控制
void fric_control()
{
  if (Fric_Mode == FRIC_DOWN || !pm02.robot_status.power_management_shooter_output) {
    fric_motor1.cmd(0.0f);
    fric_motor2.cmd(0.0f);
    fricmotor1_pid.data.iout = 0.0f;
    fricmotor2_pid.data.iout = 0.0f;
    fricmotor3_pid.data.iout = 0.0f;
    return;
  }
  fricmotor1_pid.calc(-fric_target_speed, fric_motor1.speed);
  fricmotor2_pid.calc(fric_target_speed, fric_motor2.speed);
  fricmotor3_pid.calc(fric_target_speed, fric_motor3.speed);

  fric_motor1.cmd(fricmotor1_pid.out);
  fric_motor2.cmd(fricmotor2_pid.out);
  fric_motor3.cmd(fricmotor3_pid.out);
}

void trigger_control()
{
  if (Global_Mode == ZERO_FORCE || !pm02.robot_status.power_management_shooter_output) {
    trigger_motor.cmd(0.0f);
    trigger_give_torque = 0.0f;
    return;
  }
  else {
    if (Trigger_Mode == SHOOT_INIT) {
      shoot_init_control();
    }
    else if (Fric_Mode == FRIC_ON && Trigger_Mode == SHOOT_READY_SINGLE) {
      shoot_single_control();
    }
    else {
      trigger_motor.cmd(0.0f);
      trigger_give_torque = 0.0f;
    }
  }
}

void shoot_init_control()
{
  triggerback_pos_pid1.calc(trigger_target_angle, trigger_motor.angle);
  //防超调之后回到原目标慢
  if (triggerback_pos_pid1.data.iout * (trigger_target_angle - trigger_motor.angle) < 0) {
    triggerback_pos_pid1.data.iout = 0.0f;
  }
  triggerback_speed_pid2.calc(triggerback_pos_pid1.out, trigger_motor.speed);
  trigger_motor.cmd(triggerback_speed_pid2.out);
}

void shoot_single_control()
{
  trigger_pos_pid1.calc(trigger_target_angle, trigger_motor.angle);
  trigger_speed_pid2.calc(trigger_pos_pid1.out, trigger_motor.speed);
  trigger_give_torque = trigger_speed_pid2.out;
  trigger_motor.cmd(trigger_speed_pid2.out);
  if (trigger_pos_pid1.out * (trigger_target_angle - trigger_motor.angle) < 0) {
    trigger_pos_pid1.data.iout = 0.0f;
  }
}