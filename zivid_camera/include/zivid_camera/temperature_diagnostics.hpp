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

// The `zivid_temperature` status for /diagnostics: the camera's temperatures in degrees Celsius,
// ERROR above Zivid's limits, and STALE while the camera isn't connected.
diagnostic_msgs::msg::DiagnosticStatus temperatureDiagnosticStatus(
  const Zivid::CameraState & state, const std::string & hardware_id);

// The `zivid_temperature` status when the camera's state can't be read.
diagnostic_msgs::msg::DiagnosticStatus unreadableTemperatureDiagnosticStatus(
  const std::string & hardware_id);

}  // namespace zivid_camera
