#pragma once

// One lock per Arduino I2C controller, held across a whole register
// transaction.
//
// Arduino's TwoWire only locks inside endTransmission()/requestFrom(); read()
// and available() then copy from a shared rxBuffer with no lock. When two tasks
// share a bus (X4 Pro: GT911 touch on the input task, CW2017 gauge and RTC on
// the render task and web server), one task's requestFrom() can overwrite the
// bytes another task is still copying out, so a touch frame or battery value
// comes back corrupt. Every SDK I2C access takes this lock around its full
// write + read + copy-out. Recursive, so register helpers can nest inside a
// larger locked sequence. Keep delay() calls outside the locked scope so other
// bus users (touch polling) never wait on a sleep.

#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace freeink {

// Static storage: no heap, created once on first use (thread-safe local statics).
inline SemaphoreHandle_t i2cBusMutex(const TwoWire& wire) {
  static StaticSemaphore_t bus0Storage;
  static const SemaphoreHandle_t bus0 = xSemaphoreCreateRecursiveMutexStatic(&bus0Storage);
#if SOC_I2C_NUM > 1
  static StaticSemaphore_t bus1Storage;
  static const SemaphoreHandle_t bus1 = xSemaphoreCreateRecursiveMutexStatic(&bus1Storage);
  if (&wire == &Wire1) return bus1;
#endif
  (void)wire;
  return bus0;
}

class I2cBusLock {
 public:
  explicit I2cBusLock(const TwoWire& wire = Wire) : mutex_(i2cBusMutex(wire)) {
    xSemaphoreTakeRecursive(mutex_, portMAX_DELAY);
  }
  ~I2cBusLock() { xSemaphoreGiveRecursive(mutex_); }

  I2cBusLock(const I2cBusLock&) = delete;
  I2cBusLock& operator=(const I2cBusLock&) = delete;

 private:
  SemaphoreHandle_t mutex_;
};

}  // namespace freeink
