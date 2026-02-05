#include "pimoroni_mics6814.h"
#include "io_expander.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace pimoroni_mics6814 {

static const char *const TAG = "pimoroni_mics6814";

void PimoroniMics6814::setup() {
  ESP_LOGI(TAG, "Setting up Pimoroni MICS6814 Breakout (IOExpander-based) ...");

  this->pref_ = global_preferences->make_preference<Baseline>(this->get_object_id_hash());
  this->pref_loaded_ = this->pref_.load(&this->baseline_);

  this->ioe_ = std::make_unique<IOExpander>(this, false);
  this->ioe_->set_adc_vref(this->vref_);

  if (!this->init_board_()) {
    this->mark_failed();
    ESP_LOGE(TAG, "Init failed. Check I2C wiring/address. (Default address in Arduino GasBreakout is 0x19.)");
  } else {
    // Publish stored baselines once at boot if present
    this->publish_baseline_();
  }
}

void PimoroniMics6814::loop() {
  // Non-blocking calibration sampling at ~1Hz.
  if (!this->calib_active_) return;
  uint32_t now = millis();
  if ((int32_t) (now - this->calib_next_sample_ms_) < 0) return;

  this->calib_next_sample_ms_ = now + 1000;

  GasReading r = this->read_all_();
  if (!std::isfinite(r.oxidising_ohm) || !std::isfinite(r.reducing_ohm) || !std::isfinite(r.nh3_ohm)) {
    ESP_LOGW(TAG, "Calibration sample skipped: invalid readings");
    return;
  }

  this->calib_sum_ox_ += r.oxidising_ohm;
  this->calib_sum_red_ += r.reducing_ohm;
  this->calib_sum_nh3_ += r.nh3_ohm;
  this->calib_samples_++;

  ESP_LOGD(TAG, "Calibration sampling %u/%u", this->calib_samples_, this->calib_target_samples_);

  if (this->calib_samples_ >= this->calib_target_samples_) {
    // Finalize: store mean as R0
    this->baseline_.r0_ox = (float) (this->calib_sum_ox_ / (double) this->calib_samples_);
    this->baseline_.r0_red = (float) (this->calib_sum_red_ / (double) this->calib_samples_);
    this->baseline_.r0_nh3 = (float) (this->calib_sum_nh3_ / (double) this->calib_samples_);

    this->pref_.save(&this->baseline_);
    this->pref_loaded_ = true;
    this->calib_active_ = false;

    ESP_LOGI(TAG, "Calibration saved (mean over %u samples): ox=%.1fΩ red=%.1fΩ nh3=%.1fΩ",
             this->calib_samples_, this->baseline_.r0_ox, this->baseline_.r0_red, this->baseline_.r0_nh3);
    this->publish_baseline_();
  }
}

void PimoroniMics6814::dump_config() {
  ESP_LOGCONFIG(TAG, "Pimoroni MICS6814");
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  ADC timeout: %u ms", this->adc_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  ADC vref: %.3f V", this->vref_);
  if (this->pref_loaded_) {
    ESP_LOGCONFIG(TAG, "  Baseline loaded: ox=%.1fΩ red=%.1fΩ nh3=%.1fΩ",
                  this->baseline_.r0_ox, this->baseline_.r0_red, this->baseline_.r0_nh3);
  } else {
    ESP_LOGCONFIG(TAG, "  Baseline loaded: no");
  }
}

bool PimoroniMics6814::init_board_() {
  if (!this->ioe_->initialise(false)) {
    return false;
  }

  // Configure ADC pins (as in Pimoroni mics6814-python + Arduino GasBreakout)
  // VREF is a spare ADC channel tied to 3v3 on the breakout and is used as reference.
  this->ioe_->set_mode(PIN_VREF, IOExpander::PIN_ADC);
  this->ioe_->set_mode(PIN_RED, IOExpander::PIN_ADC);
  this->ioe_->set_mode(PIN_NH3, IOExpander::PIN_ADC);
  this->ioe_->set_mode(PIN_OX, IOExpander::PIN_ADC);

  // Heater enable pin
  this->ioe_->set_mode(PIN_HEATER_EN, IOExpander::PIN_OUT);
  // Arduino port sets LOW here; since heater is active-low, this is effectively "ON".
  this->ioe_->output(PIN_HEATER_EN, 0);

  return true;
}

float PimoroniMics6814::calc_resistance_(float vref, float vgas) {
  // Copied from GasBreakout::readGas
  if (vgas < 0.0f) return NAN;
  if (vref == vgas) return 0.0f;
  float res = (vgas * LOAD_RESISTOR_OHM) / (vref - vgas);
  return res;
}

GasReading PimoroniMics6814::read_all_() {
  GasReading r{};
  float ref_v = this->ioe_->input_as_voltage(PIN_VREF, this->adc_timeout_ms_);
  if (ref_v < 0.0f) ref_v = 0.0f;
  r.ref_v = ref_v;

  const float adc_vref = this->ioe_->get_adc_vref();  // typically 3.3V
  float v_red = this->ioe_->input_as_voltage(PIN_RED, this->adc_timeout_ms_);
  float v_nh3 = this->ioe_->input_as_voltage(PIN_NH3, this->adc_timeout_ms_);
  float v_ox  = this->ioe_->input_as_voltage(PIN_OX, this->adc_timeout_ms_);

  r.reducing_ohm  = calc_resistance_(adc_vref, v_red);
  r.nh3_ohm       = calc_resistance_(adc_vref, v_nh3);
  r.oxidising_ohm = calc_resistance_(adc_vref, v_ox);

  return r;
}

void PimoroniMics6814::update() {
  if (this->is_failed()) return;

  GasReading r = this->read_all_();

  if (this->ref_)       this->ref_->publish_state(r.ref_v);
  if (this->reducing_)  this->reducing_->publish_state(r.reducing_ohm);
  if (this->nh3_)       this->nh3_->publish_state(r.nh3_ohm);
  if (this->oxidising_) this->oxidising_->publish_state(r.oxidising_ohm);

  this->publish_baseline_();
  this->publish_ratios_(r);
}

void PimoroniMics6814::publish_baseline_() {
  if (this->r0_oxidising_ && std::isfinite(this->baseline_.r0_ox)) this->r0_oxidising_->publish_state(this->baseline_.r0_ox);
  if (this->r0_reducing_  && std::isfinite(this->baseline_.r0_red)) this->r0_reducing_->publish_state(this->baseline_.r0_red);
  if (this->r0_nh3_       && std::isfinite(this->baseline_.r0_nh3)) this->r0_nh3_->publish_state(this->baseline_.r0_nh3);
}

void PimoroniMics6814::publish_ratios_(const GasReading &r) {
  if (this->ratio_oxidising_) {
    if (std::isfinite(this->baseline_.r0_ox) && this->baseline_.r0_ox > 0.0f && std::isfinite(r.oxidising_ohm))
      this->ratio_oxidising_->publish_state(r.oxidising_ohm / this->baseline_.r0_ox);
    else
      this->ratio_oxidising_->publish_state(NAN);
  }
  if (this->ratio_reducing_) {
    if (std::isfinite(this->baseline_.r0_red) && this->baseline_.r0_red > 0.0f && std::isfinite(r.reducing_ohm))
      this->ratio_reducing_->publish_state(r.reducing_ohm / this->baseline_.r0_red);
    else
      this->ratio_reducing_->publish_state(NAN);
  }
  if (this->ratio_nh3_) {
    if (std::isfinite(this->baseline_.r0_nh3) && this->baseline_.r0_nh3 > 0.0f && std::isfinite(r.nh3_ohm))
      this->ratio_nh3_->publish_state(r.nh3_ohm / this->baseline_.r0_nh3);
    else
      this->ratio_nh3_->publish_state(NAN);
  }
}

bool PimoroniMics6814::set_heater_enabled(bool enabled) {
  if (!this->ioe_) return false;
  // Arduino GasBreakout uses active-low: enabled -> LOW, disabled -> HIGH
  this->ioe_->output(PIN_HEATER_EN, enabled ? 0 : 1);
  return true;
}

bool PimoroniMics6814::start_calibration() {
  // NOTE: The Arduino IOExpander_Library does not implement an on-board calibration flow.
  // This starts a host-side baseline capture: we sample Rs for ~30 seconds and store the mean as R0.
  // This idea mirrors typical calibration guidance (take a stable, clean-air baseline rather than a single instant read).
  if (this->calib_active_) {
    ESP_LOGW(TAG, "Calibration already running");
    return false;
  }

  this->calib_active_ = true;
  this->calib_samples_ = 0;
  this->calib_sum_ox_ = 0;
  this->calib_sum_red_ = 0;
  this->calib_sum_nh3_ = 0;
  this->calib_next_sample_ms_ = millis();
  ESP_LOGI(TAG, "Calibration started: collecting %u samples (~%u seconds)",
           this->calib_target_samples_, this->calib_target_samples_);
  return true;
}

void PimoroniMics6814::set_r0_by_channel(uint8_t channel, float value_ohm) {
  switch (channel) {
    case 0: this->baseline_.r0_ox = value_ohm; break;
    case 1: this->baseline_.r0_red = value_ohm; break;
    case 2: this->baseline_.r0_nh3 = value_ohm; break;
    default: return;
  }
  this->pref_.save(&this->baseline_);
  this->publish_baseline_();
}

float PimoroniMics6814::get_r0_by_channel(uint8_t channel) const {
  switch (channel) {
    case 0: return this->baseline_.r0_ox;
    case 1: return this->baseline_.r0_red;
    case 2: return this->baseline_.r0_nh3;
    default: return NAN;
  }
}

// -------- Helper entities --------

void PimoroniHeaterSwitch::write_state(bool state) {
  bool ok = parent_->set_heater_enabled(state);
  this->publish_state(ok && state);
}

void PimoroniCalibrateButton::press_action() {
  parent_->start_calibration();
}

void PimoroniBaselineNumber::control(float value) {
  // value is in ohms
  parent_->set_r0_by_channel(this->channel_, value);
  this->publish_state(value);
}

}  // namespace pimoroni_mics6814
}  // namespace esphome
