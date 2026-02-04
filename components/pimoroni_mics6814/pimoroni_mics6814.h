#pragma once
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"

namespace esphome {
namespace pimoroni_mics6814 {

// Forward
class IOExpander;

// Mimic GasBreakout mapping from IOExpander_Library
struct GasReading {
  float ref_v;
  float reducing_ohm;
  float nh3_ohm;
  float oxidising_ohm;
};

class PimoroniMics6814 : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  // Sensors
  void set_oxidising_sensor(sensor::Sensor *s) { oxidising_ = s; }
  void set_reducing_sensor(sensor::Sensor *s) { reducing_ = s; }
  void set_nh3_sensor(sensor::Sensor *s) { nh3_ = s; }
  void set_ref_sensor(sensor::Sensor *s) { ref_ = s; }

  void set_r0_oxidising_sensor(sensor::Sensor *s) { r0_oxidising_ = s; }
  void set_r0_reducing_sensor(sensor::Sensor *s) { r0_reducing_ = s; }
  void set_r0_nh3_sensor(sensor::Sensor *s) { r0_nh3_ = s; }

  void set_ratio_oxidising_sensor(sensor::Sensor *s) { ratio_oxidising_ = s; }
  void set_ratio_reducing_sensor(sensor::Sensor *s) { ratio_reducing_ = s; }
  void set_ratio_nh3_sensor(sensor::Sensor *s) { ratio_nh3_ = s; }

  // Public actions for helper entities
  bool set_heater_enabled(bool enabled);
  // Starts a baseline calibration.
  // The Arduino IOExpander_Library does not implement an on-board calibration.
  // We therefore do a host-side baseline capture: sample Rs for a short period,
  // then persist R0 for each channel.
  bool start_calibration();

  // Set/get baseline R0 values (host-side)
  void set_r0_by_channel(uint8_t channel, float value_ohm);
  float get_r0_by_channel(uint8_t channel) const;

  // Config knobs (can be extended later)
  void set_adc_timeout_ms(uint32_t ms) { adc_timeout_ms_ = ms; }
  void set_vref(float vref) { vref_ = vref; }

 protected:
  // Pimoroni pins (1..14) from Arduino GasBreakout
  static constexpr uint8_t PIN_HEATER_EN = 1;
  static constexpr uint8_t PIN_VREF = 14;
  static constexpr uint8_t PIN_RED = 13;      // reducing
  static constexpr uint8_t PIN_NH3 = 11;
  static constexpr uint8_t PIN_OX = 12;

  // Fixed resistor used on Pimoroni board (from Arduino GasBreakout.cpp)
  static constexpr float LOAD_RESISTOR_OHM = 56000.0f;

  sensor::Sensor *oxidising_{nullptr};
  sensor::Sensor *reducing_{nullptr};
  sensor::Sensor *nh3_{nullptr};
  sensor::Sensor *ref_{nullptr};

  sensor::Sensor *r0_oxidising_{nullptr};
  sensor::Sensor *r0_reducing_{nullptr};
  sensor::Sensor *r0_nh3_{nullptr};

  sensor::Sensor *ratio_oxidising_{nullptr};
  sensor::Sensor *ratio_reducing_{nullptr};
  sensor::Sensor *ratio_nh3_{nullptr};

  std::unique_ptr<IOExpander> ioe_;

  // Preferences for baseline calibration
  ESPPreferenceObject pref_;
  bool pref_loaded_{false};

  struct Baseline {
    float r0_ox{NAN};
    float r0_red{NAN};
    float r0_nh3{NAN};
  } baseline_;

  // --- Calibration state machine (non-blocking) ---
  bool calib_active_{false};
  uint32_t calib_next_sample_ms_{0};
  uint16_t calib_target_samples_{30};   // ~30 seconds at 1Hz sampling
  uint16_t calib_samples_{0};
  // Running sums for mean Rs
  double calib_sum_ox_{0};
  double calib_sum_red_{0};
  double calib_sum_nh3_{0};

  uint32_t adc_timeout_ms_{1000};
  float vref_{3.3f};

  bool init_board_();
  GasReading read_all_();

  static float calc_resistance_(float vref, float vgas);
  void publish_baseline_();
  void publish_ratios_(const GasReading &r);
};

class PimoroniHeaterSwitch : public switch_::Switch {
 public:
  explicit PimoroniHeaterSwitch(PimoroniMics6814 *parent) : parent_(parent) {}
  void write_state(bool state) override;

 protected:
  PimoroniMics6814 *parent_;
};

class PimoroniCalibrateButton : public button::Button {
 public:
  explicit PimoroniCalibrateButton(PimoroniMics6814 *parent) : parent_(parent) {}
  void press_action() override;

 protected:
  PimoroniMics6814 *parent_;
};

// Optional helper: expose baseline (R0) as settable numbers
class PimoroniBaselineNumber : public number::Number {
 public:
  explicit PimoroniBaselineNumber(PimoroniMics6814 *parent) : parent_(parent) {}
  void set_channel(uint8_t ch) { channel_ = ch; }

 protected:
  void control(float value) override;

  PimoroniMics6814 *parent_;
  uint8_t channel_{0};
};

}  // namespace pimoroni_mics6814
}  // namespace esphome
