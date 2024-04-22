#include <Arduino.h>

class Encoder {
public:
    Encoder(uint8_t port1_temp, uint8_t port2_temp) {
        pinMode(port1_temp, INPUT);
        pinMode(port2_temp, INPUT);
        port1 = port1_temp;
        port2 = port2_temp;
        updateState();
    }

    long read() {
        updateState();
        processEncoderChanges();
        return position;
    }

    void zero() {
        position = 0;
    }

private:
    uint8_t port1;
    uint8_t port2;
    bool current_port1;
    bool current_port2;
    bool last_port1;
    bool last_port2;
    long position = 0;

    void updateState() {
        last_port1 = current_port1;
        last_port2 = current_port2;
        current_port1 = digitalRead(port1);
        current_port2 = digitalRead(port2);
    }

    void processEncoderChanges() {
        if (current_port1 != last_port1 || current_port2 != last_port2) {
            if (current_port1 == last_port2) {
                position++;
            } else {
                position--;
            }
        }
    }
};

class PID {
public:
    PID(float kP_temp, float kI_temp, float kD_temp) : kP(kP_temp), kI(kI_temp), kD(kD_temp) {}

    float output(float input) {
        float proportional = input * kP;
        // Placeholder for integral and derivative calculations
        return proportional;
    }

private:
    float kP;
    float kI;
    float kD;
};

class Motor {
public:
    Motor(uint8_t port_EN_temp, uint8_t port_PH_temp, uint8_t port_SLP_temp,
          uint8_t port_encoder1, uint8_t port_encoder2, float kP, float kI, float kD)
        : encoder(port_encoder1, port_encoder2), pid(kP, kI, kD) {
        pinMode(port_EN_temp, OUTPUT);
        pinMode(port_PH_temp, OUTPUT);
        pinMode(port_SLP_temp, OUTPUT);
        port_EN = port_EN_temp;
        port_PH = port_PH_temp;
        port_SLP = port_SLP_temp;
    }

    void set_target(long target) {
        this->target = target;
    }

    void update_position() {
        position = encoder.read();
    }

    void drive() {
        digitalWrite(port_SLP, HIGH);
        long speed = pid.output(target - position);
        digitalWrite(port_PH, (speed < 0) ? HIGH : LOW);
        analogWrite(port_EN, abs(speed));
    }

    void off() {
        analogWrite(port_EN, 0);
    }

    long get_position() {
        return position;
    }

    long get_target() {
        return target;
    }

    bool stopped() {
        bool isStopped = (last_position == position);
        if (!isStopped) {
            last_moving_time = millis();
        } else if (millis() - last_moving_time >= 1000 && position == target) {
            isStopped = true;
        }
        last_position = position;
        return isStopped;
    }

    void zero() {
        encoder.zero();
    }

private:
    uint8_t port_EN;
    uint8_t port_PH;
    uint8_t port_SLP;
    Encoder encoder;
    PID pid;
    long target;
    long position;
    long last_position;
    unsigned long last_moving_time;
};

struct motor_targets {
    uint8_t center = 0;
    uint8_t left = 0;
    uint8_t right = 0;
};

Motor center(5, 4, 12, A2, A3, 10.0, 0.0, 0.0);
Motor left(6, 7, 13, A4, A5, 10.0, 0.0, 0.0);
Motor right(3, 2, 8, A0, A1, 10.0, 0.0, 0.0);

void turn_off_all_motors() {
    center.off();
    left.off();
    right.off();
}

void set_motor_targets(struct motor_targets *targets) {
    center.set_target(targets->center * 12);
    left.set_target(targets->left * 12);
    right.set_target(targets->right * 12);
}

void check_encoder_time() {
    unsigned long update_start = micros();
    center.update_position();
    left.update_position();
    right.update_position();
    unsigned long time_taken = micros() - update_start;
    if (time_taken > 400) {
        while (true) {
            turn_off_all_motors();
            Serial.println("ERROR: Too long to read encoders, time: " + String(time_taken));
            delay(500);
        }
    }
}

class SerialCustom {
public:
    void setup() {
        Serial.begin(9600);
    }

    void read(struct motor_targets *targets) {
        if (Serial.availableForWrite() == 63 && Serial.available() == 1) {
            int serial = Serial.read();
            Serial.write(serial);
            switch (serial_count) {
                case 0:
                    targets->center = (uint8_t)serial;
                    serial_count = 1;
                    break;
                case 1:
                    targets->left = (uint8_t)serial;
                    serial_count = 2;
                    break;
                case 2:
                    targets->right = (uint8_t)serial;
                    serial_count = 0;
                    break;
                default:
                    while (true) {
                        Serial.println("ERROR: serial_count not in bounds");
                        turn_off_all_motors();
                        delay(500);
                    }
                    break;
            }
        }
        if (Serial.available() > 1) {
            while (true) {
                Serial.println("ERROR: Too much serial data received");
                turn_off_all_motors();
                delay(500);
            }
        }
    }

private:
    uint8_t serial_count = 0;
};

SerialCustom serial;

void setup() {
    serial.setup();
    Serial.setTimeout(2147483647); // Set timeout to maximum
    Serial.println("STARTED");
    delay(500);
}

void loop() {
    static bool calibrated = false;
    static uint8_t calibration_serial_count = 0;

    check_encoder_time();
    struct motor_targets targets;
    serial.read(&targets);
    set_motor_targets(&targets);

    if (!calibrated && center.stopped() && left.stopped() && right.stopped()) {
        turn_off_all_motors();
        String calibration_data;
        switch (calibration_serial_count) {
            case 0:
                Serial.println("center motor position");
                calibration_data = Serial.readStringUntil('|');
                center.set_target(calibration_data.toInt());
                Serial.println(String(center.get_target()));
                calibration_serial_count = 1;
                break;
            case 1:
                Serial.println("left motor position");
                calibration_data = Serial.readStringUntil('|');
                left.set_target(calibration_data.toInt());
                Serial.println(String(left.get_target()));
                calibration_serial_count = 2;
                break;
            case 2:
                Serial.println("right motor position");
                calibration_data = Serial.readStringUntil('|');
                right.set_target(calibration_data.toInt());
                Serial.println(String(right.get_target()));
                calibration_serial_count = 3;
                break;
            case 3:
                Serial.println("Finished calibration? (y/n)");
                calibration_data = Serial.readStringUntil('|');
                if (calibration_data == "y") {
                    calibrated = true;
                    center.zero();
                    left.zero();
                    right.zero();
                    Serial.println("Calibration finished");
                } else {
                    calibration_serial_count = 0;
                    Serial.println("Calibration continued");
                }
                break;
            default:
                while (true) {
                    Serial.println("ERROR: calibration_serial_count not in bounds");
                    turn_off_all_motors();
                    delay(500);
                }
                break;
        }
    }
    
    check_encoder_time();
    center.drive();
    left.drive();
    right.drive();
}
