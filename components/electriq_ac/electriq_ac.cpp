#include "electriq_ac.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace electriq_ac {

static const char *const TAG = "electriq_ac";

// Custom fan mode string constants
static const char *const FAN_SPEED_1 = "Low";
static const char *const FAN_SPEED_2 = "Low-Mid";
static const char *const FAN_SPEED_3 = "Mid";
static const char *const FAN_SPEED_4 = "Mid-High";
static const char *const FAN_SPEED_5 = "Max";

// Sleep mode is bit 6 (0x40) in protocol byte 4 (b[2]), same byte as swing (0x0C)
static const uint8_t SLEEP_BIT = 0x40;

// Smart Cool is a distinct operating mode, encoded in the mode nibble (b[1] & 0x0F)
// alongside Cool=0x01, Dry=0x02, Fan=0x03, Heat=0x04. Confirmed from hardware logs:
// when Smart Cool is selected the MCU reports b[1]=0x90, i.e. mode nibble 0x00.
// (The old firmware's "case 0x01: default:" caught this and mislabelled it Cool.)
// Mapped to CLIMATE_MODE_AUTO because ESPHome's climate modes are a fixed enum and
// the five standard slots are already used; Home Assistant will label it "Auto".
static const uint8_t SMART_COOL_NIBBLE = 0x00;

void ElectriqAC::setup() {
  this->set_interval("heartbeat", 1800, [this] { SendHeartbeat(); });
}

// only used during debugging
//void ElectriqAC::dump_config() {
//  ESP_LOGCONFIG(TAG, "Electriq AC Climate");
//  LOG_CLIMATE("", "Electriq AC", this);
//  this->check_uart_settings(9600);
//}

// calculate checksum and write out the serial message
void ElectriqAC::SendToMCU() {
  uint8_t tuyacmd;
  tuyacmd = (ac_mode_ + fan_speed_);
  // ensure we have obtained the MCU settings and published before commanding the MCU
  if (target_temp_ != 0) {
    uint8_t flags = swing_ | (sleep_ ? SLEEP_BIT : 0x00);
    uint8_t checksum = (0xAA + 0x03 + tuyacmd + flags + target_temp_ + 0x0B);
    write_array({0xAA, 0x03, tuyacmd, flags, target_temp_, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x00, checksum});
    ESP_LOGD(TAG, "SendToMCU fan/mode: %s, target_temp: %s", format_hex_pretty(tuyacmd).c_str(),
             format_hex_pretty(target_temp_).c_str());
    // we wrote something, so ensure it's published back to HA too
    this->publish_state();
  } else {
    ESP_LOGD(TAG, "Something to write but target_temp zero? %s", format_hex_pretty(target_temp_).c_str());
  }
}

// send regular heartbeat and check for any response
void ElectriqAC::SendHeartbeat() {
  ReadMCU(); // read data from previous heartbeat first
  write_array({0xAA, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAC}); // the magic message
  flush();  // Ensure data is actually sent, probably not needed
}

// Select command nibble for fan speed
void ElectriqAC::AcFanSpeed() {
  if (this->has_custom_fan_mode()) {
    const char* mode = this->get_custom_fan_mode().c_str();
    if (strcmp(mode, FAN_SPEED_1) == 0) fan_speed_ = 0x90;
    else if (strcmp(mode, FAN_SPEED_2) == 0) fan_speed_ = 0xA0;
    else if (strcmp(mode, FAN_SPEED_3) == 0) fan_speed_ = 0xB0;
    else if (strcmp(mode, FAN_SPEED_4) == 0) fan_speed_ = 0xC0;
    else if (strcmp(mode, FAN_SPEED_5) == 0) fan_speed_ = 0xD0;
  }
}

// Select command nibble for mode
void ElectriqAC::AcModes() {
  switch (this->mode) {
    case climate::CLIMATE_MODE_COOL:
      ac_mode_ = 0x01;
      break;
    case climate::CLIMATE_MODE_DRY:
      ac_mode_ = 0x02;
      break;
    case climate::CLIMATE_MODE_FAN_ONLY:
      ac_mode_ = 0x03;
      break;
    case climate::CLIMATE_MODE_HEAT:
      ac_mode_ = 0x04;
      break;
    case climate::CLIMATE_MODE_AUTO:  // Smart Cool
      ac_mode_ = SMART_COOL_NIBBLE;
      break;
    case climate::CLIMATE_MODE_OFF:
    default:
      fan_speed_ = 0x10;
      break;
  }
}

// Select command nibble for swing
void ElectriqAC::AcSwing() {
  switch (this->swing_mode) {
    case climate::CLIMATE_SWING_OFF:
    default:
      swing_ = 0x00;
      break;
    case climate::CLIMATE_SWING_VERTICAL:
      swing_ = 0x0C;
      break;
  }
}

bool ElectriqAC::CheckIdle(uint8_t &a) {
  if (a == 0x00) {
    return true;
  }
  return false;
}

// read and parse messages from MCU serial
void ElectriqAC::ReadMCU() {
  uint8_t pos = 0;
  uint8_t csum = 0;
  uint8_t c;
  uint8_t b[16];

  ESP_LOGD(TAG, "ReadMCU called, bytes available: %d", this->available());

  // find header byte, read further 16 bytes into array
  while (this->available()) {
    read_byte(&c);
    if (c == 0xAA) {
      ESP_LOGD(TAG, "Found header byte 0xAA, reading 16 bytes...");
      read_array(b, 16);
      // --- ADDED: full hex dump for sleep mode byte identification ---
      ESP_LOGD(TAG, "Raw: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
               b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
               b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
      ESP_LOGD(TAG, "Read complete. Byte[1]=%02X, Byte[3]=%02X (target_temp)", b[1], b[3]);
      // if any more bytes available in the serial buffer, read each one to clear them out
      while (this->available()) {
        read_byte(&c);
      }
      // validate the checksum before progressing
      while (pos < 14) {
        csum += b[pos];
        ++pos;
      }
      if ((csum += 0xAA) != b[15]) {
        ESP_LOGD(TAG, "Bad received checksum %s, should be %s", format_hex_pretty(csum).c_str(),
                 format_hex_pretty(b[15]).c_str());
        return;
      }
      // Simple bitwise AND ops to get fan, mode, swing and action nibbles
      uint8_t f = (b[1] & 0xF0);
      uint8_t m = (b[1] & 0x0F);
      uint8_t s = (b[2] & 0x0F);
      uint8_t a = (b[11] & 0x0F);
      static uint8_t last_b1;   // command
      static uint8_t last_b2;   // swing
      static uint8_t last_b3;   // set temp
      static uint8_t last_b7;   // temp probe
      static uint8_t last_b11;  // active state

      if (f == 0x10) {
        ESP_LOGD(TAG, "Detected mode: Standby");
        this->action = climate::CLIMATE_ACTION_OFF;
        this->mode = climate::CLIMATE_MODE_OFF;
        AcModes();
      } else {  // not in standby, report mode / idle at all times
        switch (m) {
          case 0x01:
            if (!CheckIdle(a)) {
              this->action = climate::CLIMATE_ACTION_COOLING;
            }
            ESP_LOGD(TAG, "Detected mode: Cool");
            this->mode = climate::CLIMATE_MODE_COOL;
            AcModes();
            break;
          case 0x02:
            if (!CheckIdle(a)) {
              this->action = climate::CLIMATE_ACTION_DRYING;
            }
            ESP_LOGD(TAG, "Detected mode: Dry");
            this->mode = climate::CLIMATE_MODE_DRY;
            AcModes();
            break;
          case 0x03:
            if (!CheckIdle(a)) {
              this->action = climate::CLIMATE_ACTION_FAN;
            }
            ESP_LOGD(TAG, "Detected mode: Fan");
            this->mode = climate::CLIMATE_MODE_FAN_ONLY;
            AcModes();
            break;
          case 0x04:
            if (!CheckIdle(a)) {
              this->action = climate::CLIMATE_ACTION_HEATING;
            }
            ESP_LOGD(TAG, "Detected mode: Heat");
            this->mode = climate::CLIMATE_MODE_HEAT;
            AcModes();
            break;
          case SMART_COOL_NIBBLE:
            if (!CheckIdle(a)) {
              this->action = climate::CLIMATE_ACTION_COOLING;
            }
            ESP_LOGD(TAG, "Detected mode: Smart Cool");
            this->mode = climate::CLIMATE_MODE_AUTO;
            AcModes();
            break;
          default:
            // Unrecognised mode nibble. Logged loudly so a not-yet-mapped mode
            // (e.g. Smart Cool, if SMART_COOL_NIBBLE is wrong) can be identified.
            ESP_LOGW(TAG, "Detected mode: UNKNOWN nibble 0x%02X - treating as Cool", m);
            if (!CheckIdle(a)) {
              this->action = climate::CLIMATE_ACTION_COOLING;
            }
            this->mode = climate::CLIMATE_MODE_COOL;
            AcModes();
            break;
        }
        if (CheckIdle(a)) {
          ESP_LOGD(TAG, "Detected action: Idle");
          this->action = climate::CLIMATE_ACTION_IDLE;
        }
      }
      // update fan speed
      switch (f) {
        case 0x90:
        default:
          ESP_LOGD(TAG, "Detected fan: low");
          this->set_custom_fan_mode_(FAN_SPEED_1);
          break;
        case 0xA0:
          ESP_LOGD(TAG, "Detected fan: low_medium");
          this->set_custom_fan_mode_(FAN_SPEED_2);
          break;
        case 0xB0:
          ESP_LOGD(TAG, "Detected fan: medium");
          this->set_custom_fan_mode_(FAN_SPEED_3);
          break;
        case 0xC0:
          ESP_LOGD(TAG, "Detected fan: medium_high");
          this->set_custom_fan_mode_(FAN_SPEED_4);
          break;
        case 0xD0:
          ESP_LOGD(TAG, "Detected fan: high");
          this->set_custom_fan_mode_(FAN_SPEED_5);
          break;
      }
      // update swing
      if (s == 0x0C) {
        ESP_LOGD(TAG, "Detected swing: on");
        this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
      } else {
        ESP_LOGD(TAG, "Detected swing: off");
        this->swing_mode = climate::CLIMATE_SWING_OFF;
      }

      // update sleep
      sleep_ = (b[2] & SLEEP_BIT) != 0;
      this->preset = sleep_ ? climate::CLIMATE_PRESET_SLEEP : climate::CLIMATE_PRESET_NONE;

      this->current_temperature = b[7];
      this->target_temperature = b[3];
      target_temp_ = b[3];

      // only publish state if something changes
      if ((last_b1 != b[1]) || (last_b2 != b[2]) || (last_b3 != b[3]) || (last_b7 != b[7]) || (last_b11 != b[11])) {
        ESP_LOGD(TAG, "Publishing new state...");
        this->publish_state();
        last_b1 = b[1];
        last_b2 = b[2];
        last_b3 = b[3];
        last_b7 = b[7];
        last_b11 = b[11];
      }
    }
  }
}

void ElectriqAC::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    ESP_LOGD(TAG, "New mode value seen");
    climate::ClimateMode mode = *call.get_mode();
    this->mode = mode;
    AcModes();
    // if the mode isn't standby (denoted by 0x10 fan speed), set the fan speed
    if (this->mode != climate::CLIMATE_MODE_OFF) {
      AcFanSpeed();
    }
    SendToMCU();
  } else if (call.get_target_temperature().has_value()) {
    target_temp_ = *call.get_target_temperature();
    // Set fan speed nibble here to avoid unexpected switch-off on temp changes
    AcFanSpeed();
    SendToMCU();
  } else if (call.has_custom_fan_mode()) {
    auto mode = call.get_custom_fan_mode();
    this->set_custom_fan_mode_(mode);
    AcFanSpeed();
    SendToMCU();
  } else if (call.get_swing_mode().has_value()) {
    climate::ClimateSwingMode swing_mode = *call.get_swing_mode();
    this->swing_mode = swing_mode;
    AcSwing();
    // Set fan speed nibble here to avoid unexpected switch-off on temp changes
    AcFanSpeed();
    SendToMCU();
  } else if (call.get_preset().has_value()) {
    this->preset = *call.get_preset();
    sleep_ = (this->preset == climate::CLIMATE_PRESET_SLEEP);
    AcFanSpeed();
    SendToMCU();
  }
}

climate::ClimateTraits ElectriqAC::traits() {
  auto traits = climate::ClimateTraits();

  // ESPHome 2026.5 removed the set_supports_* setters in favour of feature flags
  // (ClimateFeature enum in climate_mode.h). We support reporting the current
  // temperature and the current action; single-point target temperature is the
  // default, so no two-point flag is set.
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE | climate::CLIMATE_SUPPORTS_ACTION);

  traits.set_visual_min_temperature(16);
  traits.set_visual_max_temperature(32);
  traits.set_visual_temperature_step(1);

  traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_COOL, climate::CLIMATE_MODE_HEAT,
                              climate::CLIMATE_MODE_DRY, climate::CLIMATE_MODE_FAN_ONLY,
                              climate::CLIMATE_MODE_AUTO});  // AUTO = Smart Cool

  traits.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_VERTICAL});

  traits.set_supported_custom_fan_modes({
    FAN_SPEED_1,
    FAN_SPEED_2,
    FAN_SPEED_3,
    FAN_SPEED_4,
    FAN_SPEED_5
  });

  traits.set_supported_presets({
    climate::CLIMATE_PRESET_NONE,
    climate::CLIMATE_PRESET_SLEEP,
  });

  return traits;
}

}  // namespace electriq_ac
}  // namespace esphome
