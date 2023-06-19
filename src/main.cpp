#include <Pid.h>
#include <mbed.h>

#include "swap_endian.h"

// 速度コントローラID 1~8まで
constexpr unsigned motor_id = 5;
// 目標rpm -7600~7600rpmまで
constexpr int target_rpm = 3000;

BufferedSerial pc{USBTX, USBRX, 115200};
CAN can{PA_11, PA_12, (int)1e6};
// CAN can{PB_12, PB_13, (int)1e6};
Timer timer;
InterruptIn p_i{PC_4};
CANMessage msg;

struct C620Sender {
  // index operator
  static constexpr int max = 16384;
  int16_t pwm[8];

  CANMessage fw() {
    int16_t data[4];
    data[0] = swap_endian(pwm[0]);
    data[1] = swap_endian(pwm[1]);
    data[2] = swap_endian(pwm[2]);
    data[3] = swap_endian(pwm[3]);
    return CANMessage{0x200, (uint8_t*)data, 8};
  }
  CANMessage bw() {
    int16_t data[4];
    data[0] = swap_endian(pwm[4]);
    data[1] = swap_endian(pwm[5]);
    data[2] = swap_endian(pwm[6]);
    data[3] = swap_endian(pwm[7]);
    return CANMessage{0x1FF, (uint8_t*)data, 8};
  }
  // TODO global変数に頼らないAPIにしたい
  bool send() {
    return can.write(fw()) && can.write(bw());
  }
};
// c620から受け取るデータ
struct C620Reader {
  // index operator
  void read(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.len == 8 && 0x200 <= msg.id && msg.id <= 0x208) {
      data[msg.id - 0x201].set(msg.data);
    }
  }
  //C620ReadVal
  struct {
    uint16_t angle;
    int16_t rpm;
    int16_t ampere;
    uint8_t temp;
    uint8_t padding;

    void set(const uint8_t data[8]) {
      angle = uint16_t(data[0] << 8 | data[1]);
      rpm = int16_t(data[2] << 8 | data[3]);
      ampere = int16_t(data[4] << 8 | data[5]);
      temp = data[6];
    }
  } data[8];
};
struct SensorBoard {
  void set_forward(const CANMessage& msg) {
    time_us = uint64_t{msg.data[4]} << 32 | msg.data[3] << 24 | msg.data[2] << 16 | msg.data[1] << 8 | msg.data[0];
    lim = msg.data[5];
    enc[0] = msg.data[7] << 8 | msg.data[6];
  }
  void set_backward(const CANMessage& msg) {
    enc[1] = msg.data[1] << 8 | msg.data[0];
    enc[2] = msg.data[3] << 8 | msg.data[2];
    enc[3] = msg.data[5] << 8 | msg.data[4];
    enc[4] = msg.data[7] << 8 | msg.data[6];
  }
  void read(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.len == 8) {
      if(msg.id == id[0]) {
        set_forward(msg);
      } else if(msg.id == id[1]) {
        set_backward(msg);
      }
    }
  }
  unsigned id[2];

  uint64_t time_us = {};
  uint8_t lim = {};
  int16_t enc[5] = {};
};
// struct SteerUnit {
//   void set_target_pos(int) {}
//   void set_target_rpm(int) {}
//   float steer(const int pos, const std::chrono::microseconds& delta_time) {
//     return pid_steer.calc(target_pos, pos, delta_time);
//   }
//   float drive(const int rpm, const std::chrono::microseconds& delta_time) {
//     return pid_drive.calc(target_rpm, rpm, delta_time);
//   }
//   rct::Pid<float> pid_drive{{0.008f, 0.005f}};
//   rct::Pid<float> pid_steer{{0.008f, 0.005f}};
//   int zero_pos;
//   int target_pos;
//   int target_rpm;
// };

SensorBoard sensor_board{10u, 11u};
C620Reader reader{};
C620Sender sender{};
rct::Pid<int> pid{{0.8f, 0.5f}};
volatile int zero_pos = 0;

int main() {
  // put your setup code here, to run once:
  printf("\nsetup\n");

  p_i.fall([&] {
    zero_pos = sensor_board.enc[0];
  });

  timer.start();
  auto pre = timer.elapsed_time();
  auto pre_alive = timer.elapsed_time();
  while(1) {
    auto now = timer.elapsed_time();

    if(can.read(msg)) {
      sensor_board.read(msg);
      reader.read(msg);
      if(msg.id == (0x200u | motor_id)) {
        pre_alive = now;
        // printf("%x\t", msg.id);
        // printf("%d\t", reader.data[motor_id - 1].angle);
        // printf("%d\t", reader.data[motor_id - 1].rpm);
        // printf("%d\t", reader.data[motor_id - 1].ampere);
        // printf("%d\n", reader.data[motor_id - 1].temp);
      } else {
        // printf("%d\n", msg.id);
      }
    } else {
      // printf("no msg\n");
    }

    if(now - pre > 10ms) {
      {
        // DC
        int16_t pwm[4] = {5000};
        msg = CANMessage{0x02, (uint8_t*)pwm, 8};
        // can.write(msg);
      }

      {
        // dji
        int out = sender.max * 0.5;
        // pid
        if(now - pre_alive > 100ms) {
          out = sender.pwm[motor_id - 1] / 2;
          pid.refresh();
        } else {
          out = pid.calc(target_rpm, reader.data[motor_id - 1].rpm, now - pre);
          // out -16384~16384まで
          out = std::clamp(out, -8000, 8000);
        }

        sender.pwm[motor_id - 1] = out;
        printf("%d\t", out);
        sender.send();
        // if(!sender.send()) printf("\nfailed send to c620");
      }

      printf("%d\t", reader.data[motor_id - 1].angle);
      printf("%d\t", reader.data[motor_id - 1].rpm);
      printf("%d\t", reader.data[motor_id - 1].ampere);
      printf("%d\n", reader.data[motor_id - 1].temp);
      // printf("zero:% 8d\t", zero_pos);
      // printf("enc :% 8d\t", sensor_board.enc[0]);
      // printf("fix :% 8d\n", sensor_board.enc[0] - zero_pos);
      pre = now;
    }
  }
}
