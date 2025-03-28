/*
   Test ESCs.  Make sure to run Calibrate sketch first

   This file is part of Teensy-OneShot125.

   Teensy-OneShot125 is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Teensy-OneShot125 is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Teensy-OneShot125. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <vector>

#include <oneshot125.hpp>
#include "../lib/sbus/sbus.h"

bfs::SbusRx sbus_rx(&Serial2);  // Teensy 4.1 pin 7
bfs::SbusData data;

static const std::vector<uint8_t> MOTOR_PINS = {0};
static auto motors = OneShot125(MOTOR_PINS);

void setup() {
    Serial.begin(115200);

    sbus_rx.Begin();
    motors.arm();
}

void loop() {
  // Test SBUS
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    // Display the received data
    for (int8_t i = 0; i < data.NUM_CH; i++) {
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }
    // Display lost frames and failsafe data
    Serial.print(data.lost_frame);
    Serial.print("\t");
    Serial.println(data.failsafe);
  }

  // Test Oneshot125
  float throttle = 0.5;  // 0.0 - 1.0
  motors.set(0, throttle);
  motors.run();
}