/*
   Credit: https://github.com/simondlevy/OneShot125/blob/main/src/oneshot125.hpp

   Header-only library for running OneShot125 ESCs from a Teensy board

   Adapted from the code in https://github.com/nickrehm/dRehmFlight

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

#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <vector>

class OneShot125 {

    public:

        OneShot125(
            const std::vector<uint8_t> pins, 
            const uint32_t loopFrequency=2000)
        {
            for (auto pin : pins) {
                _pins.push_back(pin);
                _pulseWidths.push_back(1250);  // 125.0 * 10; default zero throttle
                _flags.push_back(false);
            }

            _loopFrequency = loopFrequency;
        }

        void arm(void) 
        {
            for (auto pin : _pins) {

                pinMode(pin, OUTPUT);

                for (uint8_t i=0; i<50; i++) {
                    digitalWrite(pin, LOW);
                    delay(2);
                }
            }
        }

        // Set pulse width in microseconds (e.g., 125.0 to 250.0)
        void set(const uint8_t index, const float throttle)
        {
            // convert throttle from 0-1 to pulse width in tenths of microseconds (1250 to 2500)
            uint16_t fixedPoint = static_cast<uint16_t>((125 * (throttle + 1)) * 10.0f);
            
            // Check that pin index is valid and pulse width is within range
            // Valid range is 1250 (125.0µs) to 2500 (250.0µs)
            // If so, set the pulse width, otherwise set to 1250 (zero throttle)
            _pulseWidths[index] = 
                index < _pins.size() && 
                (fixedPoint >= 1250 && fixedPoint <= 2500) ? fixedPoint : 
                1250;
        }

        void run(void)
        {
            // Set all pins to HIGH and reset flags
            for (uint8_t k=0; k<_pins.size(); ++k) {
                digitalWrite(_pins[k], HIGH);
                _flags[k] = false;
            }

            // Start the timer
            const auto pulseStart = micros() * 10;  // tenths of microseconds

            uint8_t wentLow = 0;

            // Loop until all pins have gone LOW
            while (wentLow < _pins.size()) {

                const auto timer = micros() * 10;  // tenths of microseconds

                // Loop through pins and set low if pulse width has passed
                for (uint8_t k=0; k<_pins.size(); ++k) {

                    if ((_pulseWidths[k] <= timer - pulseStart) && !_flags[k]) {
                        digitalWrite(_pins[k], LOW);
                        wentLow++;
                        _flags[k] = true;
                    }
                }
            }
        }

    private:
        std::vector<uint8_t> _pins;
        std::vector<uint16_t> _pulseWidths;  // stores tenths of microseconds
        std::vector<bool> _flags;
        uint8_t _loopFrequency;
};