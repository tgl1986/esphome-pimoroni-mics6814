#pragma once
#include <cstdint>
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace pimoroni_mics6814 {

class IOExpander {
 public:
  explicit IOExpander(i2c::I2CDevice *dev, bool debug=false);

  // From IOExpander_Library (subset)
  bool initialise(bool skip_chip_id_check=false);
  uint16_t get_chip_id();

  void set_adc_vref(float vref) { vref_ = vref; }
  float get_adc_vref() const { return vref_; }

  // Mode constants (matching Arduino port)
  static constexpr uint8_t PIN_MODE_QB  = 0b00000;   // Output, Quasi-Bidirectional
  static constexpr uint8_t PIN_MODE_PP  = 0b00001;   // Output, Push-Pull
  static constexpr uint8_t PIN_MODE_IN  = 0b00010;   // Input-only (hi-z)
  static constexpr uint8_t PIN_MODE_PU  = 0b10000;   // Input with pull-up
  static constexpr uint8_t PIN_MODE_OD  = 0b00011;   // Open-drain output
  static constexpr uint8_t PIN_MODE_PWM = 0b00101;   // PWM output (not used here)
  static constexpr uint8_t PIN_MODE_ADC = 0b01010;   // ADC input

  static constexpr uint8_t PIN_IN  = PIN_MODE_IN;
  static constexpr uint8_t PIN_OUT = PIN_MODE_PP;
  static constexpr uint8_t PIN_ADC = PIN_MODE_ADC;

  void set_mode(uint8_t pin, uint8_t mode, bool schmitt=false, bool invert=false);
  void output(uint8_t pin, uint16_t value);
  float input_as_voltage(uint8_t pin, uint32_t adc_timeout_ms);

 private:
  // Chip constants
  static constexpr uint16_t CHIP_ID = 0xE26A;

  // Registers (subset copied from IOExpander_Library)
  static constexpr uint8_t REG_CHIP_ID_L = 0xfa;
  static constexpr uint8_t REG_CHIP_ID_H = 0xfb;

  static constexpr uint8_t REG_P0 = 0x40;
  static constexpr uint8_t REG_P1 = 0x50;
  static constexpr uint8_t REG_P3 = 0x70;

  static constexpr uint8_t REG_P3M1 = 0x6c;
  static constexpr uint8_t REG_P3M2 = 0x6d;
  static constexpr uint8_t REG_P0M1 = 0x71;
  static constexpr uint8_t REG_P0M2 = 0x72;
  static constexpr uint8_t REG_P1M1 = 0x73;
  static constexpr uint8_t REG_P1M2 = 0x74;

  // Schmitt trigger regs are on page 1 in Pimoroni protocol mapping
  static constexpr uint8_t REG_P3S = 0xc0;
  static constexpr uint8_t REG_P0S = 0xc2;
  static constexpr uint8_t REG_P1S = 0xc4;

  static constexpr uint8_t REG_ADCRL = 0x82;
  static constexpr uint8_t REG_ADCRH = 0x83;

  static constexpr uint8_t REG_ADCCON1 = 0xa1;
  static constexpr uint8_t REG_ADCCON0 = 0xa8;
  static constexpr uint8_t REG_AINDIDS = 0xb6;

  // For clearing PWM mapping on pins that can do ADC/PWM
  static constexpr uint8_t REG_PIOCON1 = 0xc9;
  static constexpr uint8_t REG_PIOCON0 = 0x9e;

  // PWM invert register used in original lib; not needed here
  static constexpr uint8_t REG_PNP = 0x81;
  static constexpr uint8_t REG_PWMCON0 = 0x98;

  struct PinInfo {
    uint8_t port;       // 0,1,3
    uint8_t bit;        // 0..7
    uint8_t adc_chan;   // 0..7 or 0xFF
    uint8_t pwm_chan;   // 0..7 or 0xFF
    uint8_t reg_iopwm;  // PIOCON0/1 or 0
  };

  // Only define pins we need (1, 11, 12, 13, 14)
  static PinInfo get_pininfo_(uint8_t pin);

  static uint8_t reg_m1_(uint8_t port);
  static uint8_t reg_m2_(uint8_t port);
  static uint8_t reg_p_(uint8_t port);
  static uint8_t reg_ps_(uint8_t port);

  bool write8_(uint8_t reg, uint8_t val);
  uint8_t read8_(uint8_t reg);

  bool set_bit_(uint8_t reg, uint8_t bit);
  bool clr_bit_(uint8_t reg, uint8_t bit);
  bool change_bit_(uint8_t reg, uint8_t bit, bool state);
  bool get_bit_(uint8_t reg, uint8_t bit);

  bool set_bits_(uint8_t reg, uint8_t mask);
  bool clr_bits_(uint8_t reg, uint8_t mask);

  i2c::I2CDevice *dev_;
  bool debug_{false};
  float vref_{3.3f};
};

}  // namespace pimoroni_mics6814
}  // namespace esphome
