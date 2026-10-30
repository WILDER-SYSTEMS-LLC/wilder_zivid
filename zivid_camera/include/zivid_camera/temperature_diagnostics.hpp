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

#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <optional>
#include <string>

namespace Zivid
{
class CameraState;
}

namespace zivid_camera
{
// Zivid's limits: above them the camera's firmware rejects captures, and Zivid's camera health
// check reports an error. See "Monitoring Camera Temperature" in the Zivid knowledge base:
// https://support.zivid.com/en/latest/camera/reference-articles/monitoring-camera-temperature.html
constexpr double led_temperature_limit_celsius = 70.0;
constexpr double dmd_temperature_limit_celsius = 60.0;

// Builds the `zivid_temperature` status for /diagnostics: the camera's temperatures in degrees
// Celsius, ERROR above Zivid's limits, and STALE while the camera isn't connected.
//
// The message gives the LED and DMD readings with their limits, e.g.
// "LED 45 °C (limit 70 °C), DMD 38 °C (limit 60 °C)". The robot's diagnostics recorder stores a
// status each time its message changes, so the message gives whole degrees, and a reading it shows
// changes only once the camera's reading is a full degree away from it. The values carry every
// reading to 0.1 °C.
class TemperatureDiagnostics
{
public:
  diagnostic_msgs::msg::DiagnosticStatus status(
    const Zivid::CameraState & state, const std::string & hardware_id);

  // The status when the camera's state can't be read.
  static diagnostic_msgs::msg::DiagnosticStatus unreadable(const std::string & hardware_id);

private:
  std::optional<double> shown_led_;
  std::optional<double> shown_dmd_;
};

}  // namespace zivid_camera
