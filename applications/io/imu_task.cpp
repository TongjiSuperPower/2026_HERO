#include "imu_task.hpp"

#include "cmsis_os.h"
#include "tim.h"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/pid/pid.hpp"

extern uint8_t shoot_mode_flag;

const float r_ab[3][3] = {{0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

uint8_t first_temperate;
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(1e-3f);
sp::PID imu_temp_pid(
  1e-3, IMU_TEMP_KP, IMU_TEMP_KI, IMU_TEMP_KD, IMU_TEMP_MAXOUT, IMU_TEMP_MAXIOUT, 1.0f);

//不用作imu解算，仅用pid计算
#ifdef HERO_DOG
sp::LowPassFilter gyro_0_filter(0.006f);
//pitch
sp::LowPassFilter gyro_1_filter(0.006f);
//yaw
sp::LowPassFilter gyro_2_filter(0.006f);
//vyaw
sp::LowPassFilter vyaw_filter(0.012f);
//vpitch
sp::LowPassFilter vpitch_filter(0.015f);
#endif

#ifdef HERO_THREE_FRIC
sp::LowPassFilter gyro_0_filter(0.006f);
//pitch
sp::LowPassFilter gyro_1_filter(0.006f);
//yaw
sp::LowPassFilter gyro_2_filter(0.006f);
//vyaw
sp::LowPassFilter vyaw_filter(0.012f);
//vpitch
sp::LowPassFilter vpitch_filter(0.015f);
#endif

//零漂校准后的角速度xyz
float ins_gyro[3] = {0.0f, 0.0f, 0.0f};
//加速度计数据
float ins_accel[3] = {0.0f, 0.0f, 0.0f};
//角速度计数据
float ins_gyro_filter[3] = {0.0f, 0.0f, 0.0f};
//滤波后vyaw、vpitch
float imu_vyaw_filter = 0.0f;
float imu_vpitch_filter = 0.0f;

//接收视觉数据的回调函数
#ifdef MPC
extern "C" void vision_callback(uint8_t * buf, uint32_t len) { vis.update(buf, len); }
#endif

extern "C" void IMU_task()
{
  osDelay(7);
  bmi088.init();
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

  while (true) {
#ifdef MPC
    vis.send(
      shoot_mode_flag, imu.q, imu.yaw, imu.vyaw, imu.pitch, imu.vpitch, pm02.shoot.initial_speed,
      0);
#endif
    imu_temp_control(bmi088.temp);
    bmi088.update();
    //传感器角速度减去零飘值
    ins_gyro[0] = bmi088.gyro[0] - gyro_x_zero;  //roll
    ins_gyro[1] = bmi088.gyro[1] - gyro_y_zero;  //pitch
    ins_gyro[2] = bmi088.gyro[2] - gyro_z_zero;  //yaw

    gyro_0_filter.update(ins_gyro[0]);
    gyro_1_filter.update(ins_gyro[1]);
    gyro_2_filter.update(ins_gyro[2]);
    ins_gyro_filter[0] = gyro_0_filter.out;
    ins_gyro_filter[1] = gyro_1_filter.out;
    ins_gyro_filter[2] = gyro_2_filter.out;

    vyaw_filter.update(imu.vyaw);
    vpitch_filter.update(imu.vpitch);
    imu_vyaw_filter = vyaw_filter.out;
    imu_vpitch_filter = vpitch_filter.out;

    osDelay(1);
    //解算四元数，欧拉角
    imu.update(bmi088.acc, ins_gyro);
  }
}

//陀螺仪温度控制函数
void imu_temp_control(float temp)
{
  uint16_t tempPWM;
  static uint8_t temp_constant_time = 0;
  if (first_temperate) {
    imu_temp_pid.calc(IMU_TEMP, bmi088.temp);
    if (imu_temp_pid.out < 0.0f) {
      imu_temp_pid.out = 0.0f;
    }
    tempPWM = (uint16_t)imu_temp_pid.out;
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, tempPWM);
  }
  else {
    //在没有达到设置的温度，一直最大功率加热
    //in beginning, max power
    if (temp > IMU_TEMP) {
      temp_constant_time++;
      if (temp_constant_time > 200) {
        first_temperate = 1;
      }
    }
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 4999);
  }
}