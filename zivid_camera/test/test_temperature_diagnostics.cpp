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
#include <gtest/gtest.h>

#include <string>
#include <zivid_camera/temperature_diagnostics.hpp>

using diagnostic_msgs::msg::DiagnosticStatus;
using Temperature = Zivid::CameraState::Temperature;
using zivid_camera::TemperatureDiagnostics;

namespace
{
Zivid::CameraState connectedCamera(double led, double dmd)
{
  return Zivid::CameraState{
    Zivid::CameraState::Connected{true},
    Temperature{
      Temperature::DMD{dmd}, Temperature::General{35.04}, Temperature::LED{led},
      Temperature::Lens{38.46}, Temperature::PCB{41.27}}};
}

Zivid::CameraState disconnectedCamera()
{
  return Zivid::CameraState{
    Zivid::CameraState::Connected{false},
    Temperature{Temperature::DMD{80.0}, Temperature::LED{80.0}}};
}

std::string valueOf(const DiagnosticStatus & status, const std::string & key)
{
  for (const auto & key_value : status.values) {
    if (key_value.key == key) {
      return key_value.value;
    }
  }
  return "(missing)";
}
}  // namespace

TEST(TemperatureDiagnosticsTest, testMessageGivesTheReadingsAndTheirLimits)
{
  TemperatureDiagnostics diagnostics;

  const auto status = diagnostics.status(connectedCamera(45.2, 37.8), "SN1");

  EXPECT_EQ(status.name, "zivid/temperature");
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "LED 45 °C (limit 70 °C), DMD 38 °C (limit 60 °C)");
  // The camera's device id, the same on every robot; the unit's serial number is a value.
  EXPECT_EQ(status.hardware_id, "zivid");
  EXPECT_EQ(valueOf(status, "serial_number"), "SN1");
}

TEST(TemperatureDiagnosticsTest, testValuesGiveEveryReadingAndTheLimits)
{
  TemperatureDiagnostics diagnostics;

  const auto status = diagnostics.status(connectedCamera(45.0, 38.0), "SN1");

  EXPECT_EQ(valueOf(status, "dmd_c"), "38.0");
  EXPECT_EQ(valueOf(status, "led_c"), "45.0");
  EXPECT_EQ(valueOf(status, "lens_c"), "38.5");
  EXPECT_EQ(valueOf(status, "pcb_c"), "41.3");
  EXPECT_EQ(valueOf(status, "general_c"), "35.0");
  EXPECT_EQ(valueOf(status, "led_limit_c"), "70");
  EXPECT_EQ(valueOf(status, "dmd_limit_c"), "60");
}

TEST(TemperatureDiagnosticsTest, testReadingsAtZividsLimitsAreOk)
{
  TemperatureDiagnostics diagnostics;

  const auto status = diagnostics.status(connectedCamera(70.0, 60.0), "SN1");

  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "LED 70 °C (limit 70 °C), DMD 60 °C (limit 60 °C)");
}

TEST(TemperatureDiagnosticsTest, testLedOverItsLimitIsAnError)
{
  TemperatureDiagnostics diagnostics;

  const auto status = diagnostics.status(connectedCamera(70.4, 50.0), "SN1");

  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(
    status.message,
    "LED 70.4 °C (limit 70 °C), DMD 50 °C (limit 60 °C); LED over its limit: the camera rejects "
    "captures");
}

TEST(TemperatureDiagnosticsTest, testDmdOverItsLimitIsAnError)
{
  TemperatureDiagnostics diagnostics;

  const auto status = diagnostics.status(connectedCamera(50.0, 61.0), "SN1");

  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(
    status.message,
    "LED 50 °C (limit 70 °C), DMD 61.0 °C (limit 60 °C); DMD over its limit: the camera rejects "
    "captures");
}

TEST(TemperatureDiagnosticsTest, testBothOverTheirLimitsIsOneError)
{
  TemperatureDiagnostics diagnostics;

  const auto status = diagnostics.status(connectedCamera(75.0, 65.0), "SN1");

  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(
    status.message,
    "LED 75.0 °C (limit 70 °C), DMD 65.0 °C (limit 60 °C); LED and DMD over their limits: the "
    "camera rejects captures");
}

TEST(TemperatureDiagnosticsTest, testMessageKeepsAReadingUntilItMovesAFullDegree)
{
  // The robot's diagnostics recorder stores a status each time its message changes.
  TemperatureDiagnostics diagnostics;
  EXPECT_EQ(
    diagnostics.status(connectedCamera(45.4, 38.0), "").message,
    "LED 45 °C (limit 70 °C), DMD 38 °C (limit 60 °C)");

  EXPECT_EQ(
    diagnostics.status(connectedCamera(45.6, 38.4), "").message,
    "LED 45 °C (limit 70 °C), DMD 38 °C (limit 60 °C)");
  EXPECT_EQ(
    diagnostics.status(connectedCamera(44.5, 37.6), "").message,
    "LED 45 °C (limit 70 °C), DMD 38 °C (limit 60 °C)");

  EXPECT_EQ(
    diagnostics.status(connectedCamera(46.1, 38.0), "").message,
    "LED 46 °C (limit 70 °C), DMD 38 °C (limit 60 °C)");
}

TEST(TemperatureDiagnosticsTest, testDisconnectedCameraIsStaleWithoutTemperatures)
{
  TemperatureDiagnostics diagnostics;

  const auto status = diagnostics.status(disconnectedCamera(), "SN1");

  EXPECT_EQ(status.level, DiagnosticStatus::STALE);
  EXPECT_EQ(status.message, "Zivid not connected");
  EXPECT_EQ(status.values.size(), 1u);
  EXPECT_EQ(valueOf(status, "serial_number"), "SN1");
}

TEST(TemperatureDiagnosticsTest, testReconnectedCameraShowsItsReadingsAfresh)
{
  TemperatureDiagnostics diagnostics;
  diagnostics.status(connectedCamera(45.0, 38.0), "");
  diagnostics.status(disconnectedCamera(), "");

  EXPECT_EQ(
    diagnostics.status(connectedCamera(45.6, 38.0), "").message,
    "LED 46 °C (limit 70 °C), DMD 38 °C (limit 60 °C)");
}

TEST(TemperatureDiagnosticsTest, testUnreadableStateIsStale)
{
  const auto status = TemperatureDiagnostics::unreadable("SN1");

  EXPECT_EQ(status.name, "zivid/temperature");
  EXPECT_EQ(status.level, DiagnosticStatus::STALE);
  EXPECT_EQ(status.message, "Could not read the Zivid's temperatures");
  EXPECT_EQ(status.hardware_id, "zivid");
  EXPECT_EQ(valueOf(status, "serial_number"), "SN1");
}
