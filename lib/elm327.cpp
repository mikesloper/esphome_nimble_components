#include "elm327.h"
#include "esphome.h"

#include <cmath>
#include <cstdlib>

static const char *const TAG = "elm327";

std::map<int, int> elm327_pid_status;

static int hex_nibble_(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return -1;
}

static int parse_hex_byte_(const std::string &s, size_t pos) {
  if (pos + 1 >= s.size())
    return -1;
  int hi = hex_nibble_(s[pos]);
  int lo = hex_nibble_(s[pos + 1]);
  if (hi < 0 || lo < 0)
    return -1;
  return (hi << 4) | lo;
}

static int get_uptime_minutes_() {
  float uptime = id(uptime_minutes).get_raw_state();
  if (std::isnan(uptime) || uptime < 0.0f)
    return 0;
  return static_cast<int>(uptime);
}

void init_elm327_pid_status() { elm327_pid_status.clear(); }

void handle_eml_reponse(const std::string &raw) {
  std::string s;
  s.reserve(raw.size());
  for (char c : raw) {
    if (c != '\r' && c != '\n' && c != '>' && c != ' ')
      s.push_back(c);
  }
  if (s.empty())
    return;

  ESP_LOGV(TAG, "RX (%u bytes): %s", static_cast<unsigned>(raw.size()), s.c_str());

  int uptime = get_uptime_minutes_();

  size_t obd_pos = s.find("41");
  if (obd_pos != std::string::npos && obd_pos + 6 <= s.size()) {
    int pid = parse_hex_byte_(s, obd_pos + 2);
    if (pid == 0x0C && obd_pos + 8 <= s.size()) {
      int a = parse_hex_byte_(s, obd_pos + 4);
      int b = parse_hex_byte_(s, obd_pos + 6);
      if (a >= 0 && b >= 0) {
        float rpm = ((a * 256) + b) / 4.0f;
        id(odbii_rpm).publish_state(rpm);
        elm327_pid_status[12] = uptime;
        ESP_LOGD(TAG, "RPM: %.1f", rpm);
      }
    } else if (pid == 0x0D && obd_pos + 6 <= s.size()) {
      int a = parse_hex_byte_(s, obd_pos + 4);
      if (a >= 0) {
        id(odbii_kph).publish_state(static_cast<float>(a));
        elm327_pid_status[5] = uptime;
        ESP_LOGD(TAG, "KPH: %d", a);
      }
    } else if (pid == 0x05 && obd_pos + 6 <= s.size()) {
      int a = parse_hex_byte_(s, obd_pos + 4);
      if (a >= 0) {
        id(odbii_coolant_temp).publish_state(static_cast<float>(a - 40));
        elm327_pid_status[13] = uptime;
        ESP_LOGD(TAG, "Coolant: %d C", a - 40);
      }
    }
    return;
  }

  if (s.rfind("ATRV", 0) == 0 || (s.find('.') != std::string::npos && s.find('V') != std::string::npos)) {
    size_t v_pos = s.find('V');
    if (v_pos != std::string::npos && v_pos > 0) {
      std::string num = s.substr(0, v_pos);
      float volts = strtof(num.c_str(), nullptr);
      if (volts > 0.0f) {
        id(odbii_device_voltage).publish_state(volts);
        ESP_LOGD(TAG, "Voltage: %.1f V", volts);
      }
    }
  }
}

void handle_eml_reponse(const std::vector<uint8_t> &raw) {
  handle_eml_reponse(std::string(raw.begin(), raw.end()));
}
