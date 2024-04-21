#include <Arduino.h>

// MotorController.h
class Encoder {
  public:
    Encoder(uint8_t port1, uint8_t port2) : port1_(port1), port2_(port2), last_port1_(digitalRead(port1)), last_port2_(digitalRead(port2)) {}
    int8_t read() {
      int8_t current_port1 = digitalRead(port1_);
      int8_t current_port2 = digitalRead(port2_);
      if (current_port1 != last_port1_) {
        if (current_port2 == last_port2_) {
          position_ += current_port1 == HIGH ? 1 : -1;
        } else {
          position_ += current_port1 == HIGH ? -1 : 1;
        }
      }
      if (current_port2 != last_port2_) {
        if (current_port1 == last_port1_) {
          position_ += current_port2 == HIGH ? -1 : 1;
        } else {
          position_ += current_port2 == HIGH ? 1 : -1;
        }
      }
      last_port1_ = current_port1;
      last_port2_ = current_port2;
      return position_;
    }
    void zero() { position_ = 0; }

  private:
    uint8_t port1_;
    uint8_t port2_;
    int8_t last_port1_;
    int8_t last_port2_;
    int8_t position_ = 0;
};

class PID {
  public:
    PID(float kP, float kI, float kD) : kP_(kP), kI_(kI), kD_(kD), integral_(0), last_error_(0) {}
    float output(float error) {
      float proportional = error * kP_;
      integral_ += error * kI_;
      float derivative = (error - last_error_) * kD_;
      last_error_ = error;
      return proportional + integral_ + derivative;
    }

  private:
    float kP_;
    float kI_;
    float kD_;
    float integral_;
    float last_error_ = 0;
};

class Motor {
  public:
    Motor(uint8_t port_EN, uint8_t port_PH, uint8_t port_SLP, uint8_t port_encoder1, uint8_t port_encoder2, float kP, float kI, float kD)
        : encoder_(port_encoder1, port_encoder2), pid_(kP, kI, kD) {
      pinMode(port_EN, OUTPUT);
      pinMode(port_PH, OUTPUT);
      pinMode(port_SLP, OUTPUT);
      pinMode(port_encoder1, INPUT);
      pinMode(port_encoder2, INPUT);
      set_target(0);
      off();
    }
    void set_target(int8_t target) { target_ = target; }
    void update_position() { position_ = encoder_.read(); }
    void drive() {
      digitalWrite(port_SLP, HIGH);
      int8_t speed = pid_.output(target_ - position_);
      digitalWrite(port_PH, speed > 0);
      analogWrite(port_EN, abs(speed));
    }
    void off() { analogWrite(port_EN, 0); }
    int8_t get_position() const { return position_; }
    int8_t get_target() const { return target_; }

  private:
    Encoder encoder_;
    PID pid_;
    uint8_t port_EN;
    uint8_t port_PH;
    uint8_t port_SLP;
    int8_t position_ = 0;
    int8_t target_ = 0;
};

class MotorController {
  public:
    MotorController(Motor& center, Motor& left, Motor& right) : center_(center), left_(left), right_(right) {}
    void update_positions() {
      center_.update_position();
      left_.update_position();
      right_.update_position();
    }
    void set_targets(int8_t center, int8_t left, int8_t right) {
      center_.set_target(center);
      left_.set_target(left);
      right_.set_target(right);
    }
    void drive() {
      center_.drive();
      left_.drive();
      right_.drive();
    }
    void off() {
      center_.off();
      left_.off();
      right_.off();
    }
    bool stopped() const {
      return center_.get_position() == center_.get_target() && left_.get_position() == left_.get_target() && right_.get_position() == right_.get_target();
    }
    void zero() {
      center_.zero();
      left_.zero();
      right_.zero();
    }

  private:
    Motor& center_;
    Motor& left_;
    Motor& right_;
};

class SerialHandler {
  public:
    SerialHandler(MotorController& motor_controller) : motor_controller_(motor_controller) {}
    void update_targets() {
      if (Serial.available() >= 3) {
        int8_t center = Serial.read();
        int8_t left = Serial.read();
        int8_t right = Serial.read();
        motor_controller_.set_targets(center, left, right);
      }
    }

  private:
    MotorController& motor_controller_;
};

// main.cpp
#include "MotorController.h"

Motor center(5, 4, 12, A2, A3, 10.0, 0.0, 0.0);
Motor left(6, 7, 13, A4, A5, 10.0, 0.0, 0.0);
Motor right(3, 2, 8, A0, A1, 10.0, 0.0, 0.0);

MotorController motor_controller(center, left, right);
SerialHandler serial_handler(motor_controller);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1000);
  motor_controller.zero();
}

void loop() {
  motor_controller.update_positions();
  serial_handler.update_targets();
  motor_controller.drive();
  if (motor_controller.stopped()) {
    motor_controller.off();
    delay(1000);
  }
}

// calibrate.cpp
#include "MotorController.h"

Motor center(5, 4, 12, A2, A3, 10.0, 0.0, 0.0);
Motor left(6, 7, 13, A4, A5, 10.0, 0.0, 0.0);
Motor right(3, 2, 8, A0, A1, 10.0, 0.0, 0.0);

MotorController motor_controller(center, left, right);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1000);
  motor_controller.zero();
}

void loop() {
  if (Serial.available() >= 3) {
    int8_t center = Serial.read();
    int8_t left = Serial.read();
    int8_t right = Serial.read();
    motor_controller.set_targets(center, left, right);
    motor_controller.drive();
    while (!motor_controller.stopped()) {
      motor_controller.update_positions();
      motor_controller.drive();
    }
    motor_controller.off();
    Serial.println("Calibration complete");
    delay(1000);
  }
}
