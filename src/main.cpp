#include <Arduino.h>
#include "Icm20948Imu.h"
#include "PinMapping.h"
#include "OledDisplay.h"
#include "L293dDcMotor.h"

//sensors
Icm20948Imu imu(1);

//actuators
OledDisplay oled(OledDisplay::Controller::SH1106, 0x3C, "OLED Display");
L293dDcMotor motor(PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_EN, "Drive Motor");

// Polymorphic registry
ISensor* sensors[] = { &imu };
constexpr size_t kNumSensors = sizeof(sensors) / sizeof(sensors[0]);

IActuator* actuators[] = {&oled, &motor};
constexpr size_t kNumActuators = sizeof(actuators) / sizeof(actuators[0]);

// Basic scheduling
uint32_t lastPrintMs = 0;

int32_t hysterisisThreshold = 25; // mg threshold for motor control

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(200);

  for (size_t i = 0; i < kNumSensors; i++) {
    bool ok = sensors[i]->begin();
    Serial.print("Begin ");
    Serial.print(sensors[i]->name());
    Serial.print(": ");
    Serial.println(ok ? "OK" : "FAIL");
  }
  for (size_t i = 0; i < kNumActuators; i++) {
    bool ok = actuators[i]->begin();
    Serial.print("Begin ");
    Serial.print(actuators[i]->name());
    Serial.print(": ");
    Serial.println(ok ? "OK" : "FAIL");
  }

  if(oled.healthy()) {
    oled.clear();
    oled.printLine(0, "Hello, world!");
    oled.printLine(1, "IMU and OLED");
    oled.printLine(2, "are working!");
    oled.update();
  }



}


void loop() {
  // put your main code here, to run repeatedly:
    for (size_t i = 0; i < kNumSensors; i++) {
    sensors[i]->sample();
  }

  // Update actuators (e.g. buzzer timing)
  for (size_t i = 0; i < kNumActuators; i++) {
    actuators[i]->update();
  }



  // Print at a slower cadence to avoid spamming the serial output and overwhelming the OLED's I2C bandwidth
  const uint32_t now = millis();
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;

    Serial.println("\n=== Sensor Readings ===");

    Serial.print("[IMU] ");
    if(!imu.hasReading()) {
      Serial.println("no reading yet");
    } else {
      Serial.print("A=(mg) ");
      Serial.print(imu.ax_mg(), 1);
      Serial.print(",");
      Serial.print(imu.ay_mg(), 1);
      Serial.print(",");
      Serial.print(imu.az_mg(), 1);

      Serial.print(" G=(dps) ");
      Serial.print(imu.gx_dps(), 1);
      Serial.print(",");
      Serial.print(imu.gy_dps(), 1);
      Serial.print(",");
      Serial.print(imu.gz_dps(), 1);

      Serial.print(" M=(uT) ");
      Serial.print(imu.mx_uT(), 1);
      Serial.print(",");
      Serial.print(imu.my_uT(), 1);
      Serial.print(",");
      Serial.print(imu.mz_uT(), 1);

      Serial.print(" T=");
      Serial.print(imu.temp_C(), 1);
      Serial.println("C");

      oled.printfLine(0, "A %.0f %.1f %.1f", imu.ax_mg(), imu.ay_mg(), imu.az_mg());
      oled.printfLine(1, "G %.0f %.1f %.1f", imu.gx_dps(), imu.gy_dps(), imu.gz_dps());
      oled.printfLine(2, "M %.0f %.1f %.1f", imu.mx_uT(), imu.my_uT(), imu.mz_uT());
      oled.printfLine(3, "T %.1fC", imu.temp_C());

    } 

    if(imu.hasReading()) {
      int speed = map(abs(imu.ay_mg()), 0, 1024, 0, 255);
      speed = constrain(speed, 0, 255);
      if (imu.ay_mg() < -hysterisisThreshold) {
        motor.reverse((uint8_t)speed);
      } else if (imu.ay_mg() > hysterisisThreshold) {
        motor.forward((uint8_t)speed);
      } else {
        motor.stop();
      }
    }


    Serial.println("=======================\n");

  }

  delay(50);
}
