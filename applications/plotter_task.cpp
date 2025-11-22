#include "cmsis_os.h"
#include "controllers/shoot_controller/shoot_task.hpp"
#include "io/plotter/plotter.hpp"
#include "io/imu_task.hpp"
#include "controllers/gimbal_controller/gimbal_task.hpp"
#include "controllers/control_task.hpp"

sp::Plotter plotter(&huart1);

extern "C" void Plotter_Task()
{
  /* USER CODE BEGIN Plotter_Task */
  /* Infinite loop */
  while (1) {
    plotter.plot(
      // trigger_motor.angle
      // imu.pitch,pitch_torque,1
      imu.pitch,pitch_target_angle, pitch_torque,a,b,c,d
      // imu.pitch,pitch_target_angle, pitch_torque, gravity_compensation,pitch_encode_speed_pid.out,pitch_encode_pos_pid.out
      // pitch_target_angle,pitch_relative_angle,pitch_encode_pos_pid.out,pitch_encode_pos_pid.data.pout,pitch_encode_pos_pid.data.iout,pitch_encode_pos_pid.data.dout
    );
    osDelay(10);
  }
  /* USER CODE END Plotter_Task */
}