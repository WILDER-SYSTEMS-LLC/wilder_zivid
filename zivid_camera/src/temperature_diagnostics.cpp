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

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <zivid_camera/temperature_diagnostics.hpp>

namespace zivid_camera
{
namespace
{
constexpr auto status_name = "zivid_temperature";

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
  uint8_t level, const std::string & message, const std::string & hardware_id)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = status_name;
  status.level = level;
  status.message = message;
  status.hardware_id = hardware_id;
  return status;
}
}  // namespace

diagnostic_msgs::msg::DiagnosticStatus temperatureDiagnosticStatus(
  const Zivid::CameraState & state, const std::string & hardware_id)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  if (!state.isConnected().value()) {
    return makeStatus(DiagnosticStatus::STALE, "Zivid not connected", hardware_id);
  }

  const auto & temperature = state.temperature();
  const double led = temperature.led().value();
  const double dmd = temperature.dmd().value();

  // The message names only the limits that are exceeded, so it changes only when the level does.
  std::vector<std::string> over_limit;
  if (led > led_temperature_limit_celsius) {
    over_limit.push_back("LED above " + formatDegrees(led_temperature_limit_celsius, 0) + " °C");
  }
  if (dmd > dmd_temperature_limit_celsius) {
    over_limit.push_back("DMD above " + formatDegrees(dmd_temperature_limit_celsius, 0) + " °C");
  }

  auto status =
    makeStatus(DiagnosticStatus::OK, "Zivid temperatures within Zivid's limits", hardware_id);
  if (!over_limit.empty()) {
    std::string message = "Zivid " + over_limit.front();
    if (over_limit.size() > 1) {
      message += " and " + over_limit.back();
    }
    status =
      makeStatus(DiagnosticStatus::ERROR, message + "; the camera rejects captures", hardware_id);
  }

  status.values = {
    keyValue("dmd_c", formatDegrees(dmd, 1)),
    keyValue("led_c", formatDegrees(led, 1)),
    keyValue("lens_c", formatDegrees(temperature.lens().value(), 1)),
    keyValue("pcb_c", formatDegrees(temperature.pcb().value(), 1)),
    keyValue("general_c", formatDegrees(temperature.general().value(), 1)),
  };
  return status;
}

diagnostic_msgs::msg::DiagnosticStatus unreadableTemperatureDiagnosticStatus(
  const std::string & hardware_id)
{
  return makeStatus(
    diagnostic_msgs::msg::DiagnosticStatus::STALE, "Could not read the Zivid's temperatures",
    hardware_id);
}

}  // namespace zivid_camera
