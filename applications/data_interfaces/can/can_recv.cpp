#include "can_recv.hpp"

#include "A_HERO_SELECTION.hpp"
#include "can.hpp"
#include "cmsis_os.h"
#include "controllers/chassis_controller/chassis_task.hpp"
#include "controllers/gimbal_controller/gimbal_task.hpp"
#include "controllers/shoot_controller/shoot_task.hpp"

extern "C" {

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();  // 获取当前的系统时间戳（以毫秒为单位）
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    // if (hcan == &hcan1) {
    //     can1.recv();
    //     if (can1.rx_id == fric_motor1.rx_id) {
    //         fric_motor1.read(can1.rx_data, stamp_ms);
    //     }
    //     else if (can1.rx_id == fric_motor2.rx_id) {
    //         fric_motor2.read(can1.rx_data, stamp_ms);
    //     }
    //     else if (can1.rx_id == pitch_motor.rx_id)
    //         pitch_motor.read(can1.rx_data, stamp_ms);
    // }
    if (hcan == &hcan2) {
      can2.recv();
      if (can2.rx_id == wheel_lf.rx_id)
        wheel_lf.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == wheel_lr.rx_id)
        wheel_lr.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == wheel_rf.rx_id)
        wheel_rf.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == wheel_rr.rx_id)
        wheel_rr.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == trigger_motor.rx_id)
        trigger_motor.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == yaw_motor.rx_id)
        yaw_motor.read(can2.rx_data, stamp_ms);
      // else if (can2.rx_id == CAN_CAP_ID)
      //     super_cap.read(can2.rx_data, stamp_ms);
    }
    if (hcan == &hcan1) {
      can1.recv();
      if (!can1.frame_type) {
        if (can1.rx_id == fric_motor1.rx_id) {
          fric_motor1.read(can1.rx_data, stamp_ms);
        }
        else if (can1.rx_id == fric_motor2.rx_id) {
          fric_motor2.read(can1.rx_data, stamp_ms);
        }
        else if (can1.rx_id == fric_motor3.rx_id) {
          fric_motor3.read(can1.rx_data, stamp_ms);
        }
        else if (can1.rx_id == fric_motor4.rx_id) {
          fric_motor4.read(can1.rx_data, stamp_ms);
        }
      }
      else if (can1.frame_type) {
        pitch_motor.read(can1.rx_id, can1.rx_data, stamp_ms);
      }
    }
  }
}

}  // extern "C"