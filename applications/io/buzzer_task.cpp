#include "buzzer_task.hpp"

#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

uint8_t BuzzerCount = 0;
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

//dragon 天空之城相关定义
//音符&音调 设计音乐
const uint32_t btnDelay2 = 1.5 * 300;  //0.5
const uint32_t btnDelay3 = 600;        //1
const uint32_t btnDelay4 = 2 * 150;    //0.25
const uint32_t btnDelay5 = 700;        //1+0.5

void BuzzerPrompt(void);

extern "C" void Buzzer_Task()
{
  osDelay(200);
  buzzer.set(5000, 0.3);

#ifdef HERO_DOG
  for (uint8_t i = 0; i < 3; ++i) {
    // buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);
  }
  buzzer.set(0, 0);
  // buzzer.start();
#endif

  while (true) {
    //进入陀螺仪校准或校准完成提示音
    if ((calibrate_flag == 1 && BuzzerCount == 0) || (calibrate_flag == 0 && BuzzerCount == 1)) {
      buzzer.start();
      BuzzerPrompt();
      BuzzerCount++;
    }
    if (calibrate_flag == 0 && BuzzerCount == 2) {
      buzzer.stop();
      BuzzerCount = 0;
    }
    // if (!motor_alive) {
    //   // for (uint8_t i = 0; i < 3; ++i) {
    //   //   buzzer.set(5000, 0.3);
    //   //   buzzer.start();
    //   //   osDelay(100);
    //   //   buzzer.stop();
    //   //   osDelay(100);
    //   // }
    //   // BuzzerPrompt();
    //   // Castle_in_the_sky();
    //   // Senbonzakura();
    //   // final_elysian();
    //   // Senbonzakura(Music1);
    // }B
    // if (motor_alive) {
    if (1) {
      buzzer.set(0, 0);
      buzzer.stop();
    }
    osDelay(100);
  }
}

void tim1(uint16_t tune)
{
  buzzer.set(tune, 0.5);  // 播放 0.5 节拍
  osDelay(btnDelay2);     // 延时 450ms
}

void tim2(uint16_t tune)
{
  buzzer.set(tune, 0.5);  // 播放 1 节拍
  osDelay(btnDelay3);     // 延时 600ms
}

void tim3(uint16_t tune)
{
  buzzer.set(tune, 0.5);  // 播放 0.25 节拍
  osDelay(btnDelay4);     // 延时 300ms
}

void tim4(uint16_t tune)
{
  buzzer.set(tune, 0.5);  // 播放 1.5 节拍
  osDelay(btnDelay5);     // 延时 700ms
}
//*******************
//提示音
void BuzzerPrompt(void)
{
  tim1(M6);  //450
  tim1(M7);  //450

  tim3(H1);  //300
  tim1(M7);  //450

  tim2(0);
}