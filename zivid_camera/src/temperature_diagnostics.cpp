// Copyright 2026 Wilder Systems
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Wilder Systems nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <Zivid/CameraState.h>

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <zivid_camera/temperature_diagnostics.hpp>

namespace zivid_camera
{
namespace
{
std::string formatDegrees(double value, int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value;
  return key_value;
}

diagnostic_msgs::msg::DiagnosticStatus makeStatus(
  uint8_t level, const std::string & message, const std::string & serial_number)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string{diagnostics_device_id} + "/temperature";
  status.level = level;
  status.message = message;
  status.hardware_id = diagnostics_device_id;
  status.values = {keyValue("serial_number", serial_number)};
  return status;
}

// The whole degree the message shows for a reading. It moves only once the reading is a full
// degree away from it, so a reading that wobbles around half a degree doesn't change the message.
double shownDegrees(std::optional<double> & shown, double reading)
{
  if (!shown || std::abs(reading - *shown) >= 1.0) {
    shown = std::round(reading);
  }
  return *shown;
}

// "LED 45 °C (limit 70 °C)". A reading over its limit is given to 0.1 °C, so it never reads as
// equal to the limit.
std::string reading(const std::string & name, double shown, double value, double limit)
{
  const auto degrees = value > limit ? formatDegrees(value, 1) : formatDegrees(shown, 0);
  return name + " " + degrees + " °C (limit " + formatDegrees(limit, 0) + " °C)";
}
}  // namespace

diagnostic_msgs::msg::DiagnosticStatus TemperatureDiagnostics::status(
  const Zivid::CameraState & state, const std::string & serial_number)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  if (!state.isConnected().value()) {
    shown_led_.reset();
    shown_dmd_.reset();
    return makeStatus(DiagnosticStatus::STALE, "Zivid not connected", serial_number);
  }

  const auto & temperature = state.temperature();
  const double led = temperature.led().value();
  const double dmd = temperature.dmd().value();
  const bool led_over = led > led_temperature_limit_celsius;
  const bool dmd_over = dmd > dmd_temperature_limit_celsius;

  std::string message =
    reading("LED", shownDegrees(shown_led_, led), led, led_temperature_limit_celsius) + ", " +
    reading("DMD", shownDegrees(shown_dmd_, dmd), dmd, dmd_temperature_limit_celsius);
  if (led_over && dmd_over) {
    message += "; LED and DMD over their limits: the camera rejects captures";
  } else if (led_over || dmd_over) {
    message += std::string{"; "} + (led_over ? "LED" : "DMD") +
               " over its limit: the camera rejects captures";
  }

  auto status = makeStatus(
    led_over || dmd_over ? DiagnosticStatus::ERROR : DiagnosticStatus::OK, message, serial_number);
  const std::vector<diagnostic_msgs::msg::KeyValue> readings = {
    keyValue("dmd_c", formatDegrees(dmd, 1)),
    keyValue("led_c", formatDegrees(led, 1)),
    keyValue("lens_c", formatDegrees(temperature.lens().value(), 1)),
    keyValue("pcb_c", formatDegrees(temperature.pcb().value(), 1)),
    keyValue("general_c", formatDegrees(temperature.general().value(), 1)),
    keyValue("led_limit_c", formatDegrees(led_temperature_limit_celsius, 0)),
    keyValue("dmd_limit_c", formatDegrees(dmd_temperature_limit_celsius, 0)),
  };
  status.values.insert(status.values.end(), readings.begin(), readings.end());
  return status;
}

diagnostic_msgs::msg::DiagnosticStatus TemperatureDiagnostics::unreadable(
  const std::string & serial_number)
{
  return makeStatus(
    diagnostic_msgs::msg::DiagnosticStatus::STALE, "Could not read the Zivid's temperatures",
    serial_number);
}

}  // namespace zivid_camera
