#include "io_expander.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace pimoroni_mics6814 {

static const char *const TAG = "pimoroni_mics6814.ioe";

IOExpander::IOExpander(i2c::I2CDevice *dev, bool debug) : dev_(dev), debug_(debug) {}

bool IOExpander::initialise(bool skip_chip_id_check) {
  if (skip_chip_id_check) return true;
  uint16_t id = this->get_chip_id();
  if (id != CHIP_ID) {
    ESP_LOGE(TAG, "Chip ID invalid: 0x%04X expected 0x%04X", id, CHIP_ID);
    return false;
  }
  return true;
}

uint16_t IOExpander::get_chip_id() {
  uint8_t lo = read8_(REG_CHIP_ID_L);
  uint8_t hi = read8_(REG_CHIP_ID_H);
  return (uint16_t) lo | ((uint16_t) hi << 8);
}

uint8_t IOExpander::reg_m1_(uint8_t port) {
  switch (port) {
    case 0: return REG_P0M1;
    case 1: return REG_P1M1;
    case 3: return REG_P3M1;
    default: return 0;
  }
}
uint8_t IOExpander::reg_m2_(uint8_t port) {
  switch (port) {
    case 0: return REG_P0M2;
    case 1: return REG_P1M2;
    case 3: return REG_P3M2;
    default: return 0;
  }
}
uint8_t IOExpander::reg_p_(uint8_t port) {
  switch (port) {
    case 0: return REG_P0;
    case 1: return REG_P1;
    case 3: return REG_P3;
    default: return 0;
  }
}
uint8_t IOExpander::reg_ps_(uint8_t port) {
  switch (port) {
    case 0: return REG_P0S;
    case 1: return REG_P1S;
    case 3: return REG_P3S;
    default: return 0;
  }
}

IOExpander::PinInfo IOExpander::get_pininfo_(uint8_t pin) {
  // Mapping copied from IOExpander_Library constructor pin table:
  // pin1  = P1.5, PWM ch5 (PIOCON1)
  // pin11 = P0.6, ADC ch3
  // pin12 = P0.5, ADC ch4, PWM ch2 (PIOCON1)
  // pin13 = P0.7, ADC ch2
  // pin14 = P1.7, ADC ch0
  switch (pin) {
    case 1:  return {1, 5, 0xFF, 5, REG_PIOCON1};
    case 11: return {0, 6, 3,    0xFF, 0};
    case 12: return {0, 5, 4,    2,    REG_PIOCON1};
    case 13: return {0, 7, 2,    0xFF, 0};
    case 14: return {1, 7, 0,    0xFF, 0};
    default: return {0xFF, 0, 0xFF, 0xFF, 0};
  }
}

bool IOExpander::write8_(uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  return dev_->write(buf, 2);
}

uint8_t IOExpander::read8_(uint8_t reg) {
  dev_->write(&reg, 1);
  uint8_t v = 0;
  dev_->read(&v, 1);
  return v;
}

bool IOExpander::set_bit_(uint8_t reg, uint8_t bit) {
  uint8_t v = read8_(reg);
  v |= (1u << bit);
  return write8_(reg, v);
}
bool IOExpander::clr_bit_(uint8_t reg, uint8_t bit) {
  uint8_t v = read8_(reg);
  v &= (uint8_t) ~(1u << bit);
  return write8_(reg, v);
}
bool IOExpander::change_bit_(uint8_t reg, uint8_t bit, bool state) {
  return state ? set_bit_(reg, bit) : clr_bit_(reg, bit);
}
bool IOExpander::get_bit_(uint8_t reg, uint8_t bit) {
  uint8_t v = read8_(reg);
  return (v >> bit) & 0x1;
}
bool IOExpander::set_bits_(uint8_t reg, uint8_t mask) {
  uint8_t v = read8_(reg);
  v |= mask;
  return write8_(reg, v);
}
bool IOExpander::clr_bits_(uint8_t reg, uint8_t mask) {
  uint8_t v = read8_(reg);
  v &= (uint8_t) ~mask;
  return write8_(reg, v);
}

void IOExpander::set_mode(uint8_t pin, uint8_t mode, bool schmitt, bool invert) {
  auto info = get_pininfo_(pin);
  if (info.port == 0xFF) {
    ESP_LOGW(TAG, "Unsupported pin %u in this minimal port", pin);
    return;
  }

  // If pin supports PWM and we're NOT using PWM mode: clear mapping.
  if (info.pwm_chan != 0xFF && mode != PIN_MODE_PWM) {
    // Clear corresponding bit in PIOCON register (disables PWM mapping)
    clr_bit_(info.reg_iopwm, info.pwm_chan);
  }

  // GPIO mode encoding:
  // gpioMode = mode & 0b11;  (QB/PP/IN/OD)
  uint8_t gpio_mode = mode & 0b11;
  uint8_t reg_m1 = reg_m1_(info.port);
  uint8_t reg_m2 = reg_m2_(info.port);

  uint8_t pm1 = read8_(reg_m1);
  uint8_t pm2 = read8_(reg_m2);

  pm1 &= (uint8_t) ~(1u << info.bit);
  pm2 &= (uint8_t) ~(1u << info.bit);

  pm1 |= ((gpio_mode >> 1) & 0x1u) << info.bit;
  pm2 |= (gpio_mode & 0x1u) << info.bit;

  write8_(reg_m1, pm1);
  write8_(reg_m2, pm2);

  // Schmitt trigger setup for input modes (matches original intent)
  if (mode == PIN_MODE_PU || mode == PIN_MODE_IN) {
    change_bit_(reg_ps_(info.port), info.bit, schmitt);
  }

  // Default output state encoded in bit4 of mode -> (initialState << 3) | pinbit
  uint8_t initial_state = (mode >> 4) & 0x1;
  write8_(reg_p_(info.port), (uint8_t) ((initial_state << 3) | (info.bit & 0x7)));
}

void IOExpander::output(uint8_t pin, uint16_t value) {
  auto info = get_pininfo_(pin);
  if (info.port == 0xFF) {
    ESP_LOGW(TAG, "Unsupported pin %u in this minimal port", pin);
    return;
  }
  uint8_t regp = reg_p_(info.port);
  if (value == 0) {
    clr_bit_(regp, info.bit);
  } else {
    set_bit_(regp, info.bit);
  }
}

float IOExpander::input_as_voltage(uint8_t pin, uint32_t adc_timeout_ms) {
  auto info = get_pininfo_(pin);
  if (info.port == 0xFF) return -1.0f;

  // Only ADC path is used in this component
  if (info.adc_chan == 0xFF) {
    // Fallback: digital read
    bool pv = get_bit_(reg_p_(info.port), info.bit);
    return pv ? vref_ : 0.0f;
  }

  // Select ADC channel (low nibble)
  clr_bits_(REG_ADCCON0, 0x0f);
  set_bits_(REG_ADCCON0, (uint8_t) (info.adc_chan & 0x0f));

  // Enable analog input for that channel
  write8_(REG_AINDIDS, 0);
  set_bit_(REG_AINDIDS, info.adc_chan);

  // Enable ADC
  set_bit_(REG_ADCCON1, 0);

  // Clear ADCF (bit7), start conversion (bit6)
  clr_bit_(REG_ADCCON0, 7);
  set_bit_(REG_ADCCON0, 6);

  uint32_t start = millis();
  while (!get_bit_(REG_ADCCON0, 7)) {
    delay(1);
    if (millis() - start >= adc_timeout_ms) {
      ESP_LOGW(TAG, "Timeout waiting for ADC conversion on pin %u (ch %u)", pin, info.adc_chan);
      return -1.0f;
    }
  }

  uint8_t hi = read8_(REG_ADCRH);
  uint8_t lo = read8_(REG_ADCRL);
  uint16_t raw12 = (uint16_t) (hi << 4) | (uint16_t) lo;
  float v = ((float) raw12 / 4095.0f) * vref_;
  return v;
}

}  // namespace pimoroni_mics6814
}  // namespace esphome
