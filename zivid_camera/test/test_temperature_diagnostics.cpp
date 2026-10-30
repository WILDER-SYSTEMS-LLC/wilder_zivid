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

TEST(TemperatureDiagnosticsTest, testTemperaturesWithinZividsLimitsAreOk)
{
  const auto status = zivid_camera::temperatureDiagnosticStatus(connectedCamera(45.0, 38.0), "SN1");

  EXPECT_EQ(status.name, "zivid_temperature");
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "Zivid temperatures within Zivid's limits");
  EXPECT_EQ(status.hardware_id, "SN1");
  EXPECT_EQ(valueOf(status, "dmd_c"), "38.0");
  EXPECT_EQ(valueOf(status, "led_c"), "45.0");
  EXPECT_EQ(valueOf(status, "lens_c"), "38.5");
  EXPECT_EQ(valueOf(status, "pcb_c"), "41.3");
  EXPECT_EQ(valueOf(status, "general_c"), "35.0");
}

TEST(TemperatureDiagnosticsTest, testTemperaturesAtZividsLimitsAreOk)
{
  const auto status = zivid_camera::temperatureDiagnosticStatus(connectedCamera(70.0, 60.0), "SN1");

  EXPECT_EQ(status.level, DiagnosticStatus::OK);
}

TEST(TemperatureDiagnosticsTest, testLedAboveZividsLimitIsAnError)
{
  const auto status = zivid_camera::temperatureDiagnosticStatus(connectedCamera(70.4, 50.0), "SN1");

  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Zivid LED above 70 °C; the camera rejects captures");
  EXPECT_EQ(valueOf(status, "led_c"), "70.4");
}

TEST(TemperatureDiagnosticsTest, testDmdAboveZividsLimitIsAnError)
{
  const auto status = zivid_camera::temperatureDiagnosticStatus(connectedCamera(50.0, 61.0), "SN1");

  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Zivid DMD above 60 °C; the camera rejects captures");
}

TEST(TemperatureDiagnosticsTest, testBothAboveZividsLimitsIsOneError)
{
  const auto status = zivid_camera::temperatureDiagnosticStatus(connectedCamera(75.0, 65.0), "SN1");

  EXPECT_EQ(status.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(
    status.message, "Zivid LED above 70 °C and DMD above 60 °C; the camera rejects captures");
}

TEST(TemperatureDiagnosticsTest, testMessageDoesNotChangeWithTheTemperatures)
{
  // The diagnostics recorder stores a status only when its level or message changes.
  const auto cooler = zivid_camera::temperatureDiagnosticStatus(connectedCamera(40.0, 35.0), "");
  const auto warmer = zivid_camera::temperatureDiagnosticStatus(connectedCamera(55.5, 48.2), "");

  EXPECT_EQ(cooler.message, warmer.message);
}

TEST(TemperatureDiagnosticsTest, testDisconnectedCameraIsStaleWithoutTemperatures)
{
  const auto state = Zivid::CameraState{
    Zivid::CameraState::Connected{false},
    Temperature{Temperature::DMD{80.0}, Temperature::LED{80.0}}};

  const auto status = zivid_camera::temperatureDiagnosticStatus(state, "SN1");

  EXPECT_EQ(status.level, DiagnosticStatus::STALE);
  EXPECT_EQ(status.message, "Zivid not connected");
  EXPECT_TRUE(status.values.empty());
}

TEST(TemperatureDiagnosticsTest, testUnreadableStateIsStale)
{
  const auto status = zivid_camera::unreadableTemperatureDiagnosticStatus("SN1");

  EXPECT_EQ(status.name, "zivid_temperature");
  EXPECT_EQ(status.level, DiagnosticStatus::STALE);
  EXPECT_EQ(status.message, "Could not read the Zivid's temperatures");
  EXPECT_EQ(status.hardware_id, "SN1");
}
