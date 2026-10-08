#include "Ssd2677Driver.h"
#include "../StickyDisplayTrace.h"

#include <BoardConfig.h>

#include <cstring>

#include <esp_heap_caps.h>

namespace freeink {
namespace {

constexpr uint8_t CMD_PANEL_SETTING = 0x00;
constexpr uint8_t CMD_POWER_SETTING = 0x06;
constexpr uint8_t CMD_POWER_ON = 0x04;
constexpr uint8_t CMD_POWER_OFF = 0x02;
constexpr uint8_t CMD_REFRESH = 0x12;
constexpr uint8_t CMD_DTM1 = 0x10;
constexpr uint8_t CMD_DEEP_SLEEP = 0x07;
constexpr uint32_t BUSY_TIMEOUT_MS = 30000;

}  // namespace

void Ssd2677Driver::PsramDeleter::operator()(uint8_t* ptr) const { heap_caps_free(ptr); }

Ssd2677Driver::Ssd2677Driver() = default;

uint8_t Ssd2677Driver::fullWaveform(uint8_t temp) {
  if (temp <= 10) return 0xE8;
  if (temp <= 20) return 0xEB;
  if (temp <= 30) return 0xEE;
  return temp <= 127 ? 0xF1 : 0xE8;
}

uint8_t Ssd2677Driver::partialWaveform(uint8_t temp) {
  if (temp <= 10) return 0x08;
  if (temp <= 20) return 0x0F;
  if (temp <= 30) return 0x19;
  return temp <= 127 ? 0x23 : 0x08;
}

uint8_t Ssd2677Driver::grayWaveform(uint8_t temp) { return temp <= 15 ? 0xF1 : temp <= 127 ? 0xFA : 0xF1; }

uint8_t Ssd2677Driver::reverseBits(uint8_t value) {
  value = static_cast<uint8_t>((value >> 4) | (value << 4));
  value = static_cast<uint8_t>(((value & 0xCC) >> 2) | ((value & 0x33) << 2));
  return static_cast<uint8_t>(((value & 0xAA) >> 1) | ((value & 0x55) << 1));
}

void Ssd2677Driver::packByte(uint8_t high, uint8_t low, uint8_t* out) {
  out[0] = out[1] = 0;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    const uint8_t pair = static_cast<uint8_t>((((high >> (7 - bit)) & 1) << 1) | ((low >> (7 - bit)) & 1));
    out[bit / 4] |= static_cast<uint8_t>(pair << (6 - (bit % 4) * 2));
  }
}

void Ssd2677Driver::writeRegister(EpdBus& bus, uint8_t command, uint8_t value) {
  bus.cmd(command);
  bus.data(value);
}

void Ssd2677Driver::begin(EpdBus& bus) {
  if (!_previous)
    _previous.reset(static_cast<uint8_t*>(heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
  _previousValid = false;
  _grayValid = false;
  _activeMode = PanelMode::Unselected;
  initializePanel(bus);
}

void Ssd2677Driver::initializePanel(EpdBus& bus) {
  _initialized = false;
  _powered = false;
  // Seeed's SSD2677 OTP initialization, including the panel's 800x680 RAM setting.
  bus.reset(0, 20);
  if (!bus.waitReadyLow(BUSY_TIMEOUT_MS)) {
    traceStickyDisplay("SSD2677 reset BUSY timeout", 0, false, true);
    return;
  }
  traceStickyDisplay("SSD2677 reset ready");
  static constexpr uint8_t psr[] = {0x2F, 0x0E};
  static constexpr uint8_t booster[] = {0x0F, 0x8B, 0x93, 0xC1};
  static constexpr uint8_t timing[] = {0x76, 0x76, 0x76, 0x5A, 0x9D, 0x8A, 0x76, 0x62};
  static constexpr uint8_t resolution[] = {0x03, 0x20, 0x02, 0xA8};
  static constexpr uint8_t gsst[] = {0, 0, 0, 0};
  bus.cmd(CMD_PANEL_SETTING);
  bus.data(psr, sizeof(psr));
  if (!bus.waitReadyLow(BUSY_TIMEOUT_MS)) {
    traceStickyDisplay("SSD2677 PSR BUSY timeout", 0, false, true);
    return;
  }
  bus.cmd(CMD_POWER_SETTING);
  bus.data(booster, sizeof(booster));
  writeRegister(bus, 0xE7, 0xC1);
  writeRegister(bus, 0x30, 0x08);
  writeRegister(bus, 0x50, 0x77);
  bus.cmd(0x62);
  bus.data(timing, sizeof(timing));
  bus.cmd(0x61);
  bus.data(resolution, sizeof(resolution));
  writeRegister(bus, 0xE0, 0x10);
  bus.cmd(0x65);
  bus.data(gsst, sizeof(gsst));
  writeRegister(bus, 0xE9, 0x01);
  if (!bus.waitReadyLow(BUSY_TIMEOUT_MS)) {
    traceStickyDisplay("SSD2677 init BUSY timeout", 0, false, true);
    return;
  }
  _initialized = true;
  traceStickyDisplay("SSD2677 OTP ready");
}

bool Ssd2677Driver::prepareMode(EpdBus& bus, PanelMode mode) {
  if (!_initialized) return false;
  if (_activeMode == mode) return true;
  if (_activeMode != PanelMode::Unselected) {
    initializePanel(bus);
    if (!_initialized) return false;
  }
  _activeMode = mode;
  return true;
}

void Ssd2677Driver::latchWaveform(EpdBus& bus, RefreshMode mode, bool gray) {
  bus.cmd(0x40);
  bus.waitBusy(" SSD2677 temperature");
  const uint8_t temperature = bus.readData8ThreeWire();
  const uint8_t waveform = gray ? grayWaveform(temperature)
                                : mode == RefreshMode::Fast ? partialWaveform(temperature)
                                                            : fullWaveform(temperature);
  writeRegister(bus, 0xE0, 0x12);
  writeRegister(bus, 0xE6, waveform);
  bus.cmd(0xA5);
  bus.waitBusy(" SSD2677 waveform");
  delay(10);
}

void Ssd2677Driver::writeMono(EpdBus& bus, const uint8_t* current, const uint8_t* previous) {
  // Align the SSD2677 stream with Sticky's SSD1677 framebuffer coordinates.
  const bool mirrorX = BoardConfig::ACTIVE.orientation.mirrorX;
  const bool mirrorY = !BoardConfig::ACTIVE.orientation.mirrorY;
  uint16_t buffered = 0;
  bus.cmd(CMD_DTM1);
  for (uint16_t row = 0; row < ROW_COUNT; ++row) {
    const uint32_t rowStart = static_cast<uint32_t>(mirrorY ? ROW_COUNT - 1 - row : row) * ROW_BYTES;
    for (uint16_t column = 0; column < ROW_BYTES; ++column) {
      const uint32_t sourceIndex = rowStart + (mirrorX ? ROW_BYTES - 1 - column : column);
      uint8_t old = previous ? previous[sourceIndex] : current[sourceIndex];
      uint8_t next = current[sourceIndex];
      if (mirrorX) {
        old = reverseBits(old);
        next = reverseBits(next);
      }
      packByte(old, next, &_transfer[buffered * 2]);
      if (++buffered == CHUNK_BYTES) {
        bus.data(_transfer, buffered * 2);
        buffered = 0;
      }
    }
  }
  if (buffered) bus.data(_transfer, buffered * 2);
}

void Ssd2677Driver::writeGray(EpdBus& bus, const uint8_t* base) {
  const bool mirrorX = BoardConfig::ACTIVE.orientation.mirrorX;
  const bool mirrorY = !BoardConfig::ACTIVE.orientation.mirrorY;
  uint16_t buffered = 0;
  bus.cmd(CMD_DTM1);
  for (uint16_t row = 0; row < ROW_COUNT; ++row) {
    const uint32_t rowStart = static_cast<uint32_t>(mirrorY ? ROW_COUNT - 1 - row : row) * ROW_BYTES;
    for (uint16_t column = 0; column < ROW_BYTES; ++column) {
      const uint32_t sourceIndex = rowStart + (mirrorX ? ROW_BYTES - 1 - column : column);
      uint8_t high = _grayMsb.get()[sourceIndex];
      uint8_t low = _grayLsb.get()[sourceIndex];
      if (_grayMode == GrayscaleMode::Overlay) {
        const uint8_t dark = static_cast<uint8_t>(low & high);
        const uint8_t light = static_cast<uint8_t>(~low & high);
        const uint8_t unchanged = static_cast<uint8_t>(base[sourceIndex] & ~(dark | light));
        high = static_cast<uint8_t>(unchanged | light);
        low = static_cast<uint8_t>(unchanged | dark);
      }
      if (mirrorX) {
        high = reverseBits(high);
        low = reverseBits(low);
      }
      packByte(high, low, &_transfer[buffered * 2]);
      if (++buffered == CHUNK_BYTES) {
        bus.data(_transfer, buffered * 2);
        buffered = 0;
      }
    }
  }
  if (buffered) bus.data(_transfer, buffered * 2);
}

void Ssd2677Driver::powerOff(EpdBus& bus) {
  if (!_powered) return;
  writeRegister(bus, CMD_POWER_OFF, 0);
  bus.waitBusy(" SSD2677 power off");
  _powered = false;
}

void Ssd2677Driver::activate(EpdBus& bus, bool keepPowered) {
  if (!_powered) {
    bus.cmd(CMD_POWER_ON);
    bus.waitBusy(" SSD2677 power on");
    _powered = true;
  }
  writeRegister(bus, CMD_REFRESH, 0);
  bus.waitBusy(" SSD2677 refresh");
  if (!keepPowered) powerOff(bus);
}

void Ssd2677Driver::display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) {
  if (!_initialized || !fb) return;
  const uint8_t* baseline = prev ? prev : (_previousValid ? _previous.get() : nullptr);
  const bool partial = mode == RefreshMode::Fast && baseline != nullptr;
  if (!prepareMode(bus, partial ? PanelMode::Partial : PanelMode::Full)) return;
  latchWaveform(bus, partial ? RefreshMode::Fast : RefreshMode::Full, false);
  writeMono(bus, fb, partial ? baseline : nullptr);
  activate(bus, partial && !turnOff);
  if (_previous) {
    memcpy(_previous.get(), fb, FRAME_BYTES);
    _previousValid = true;
  }
  _grayValid = false;
}

void Ssd2677Driver::displayWindow(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, uint16_t, uint16_t,
                                  uint16_t, uint16_t, bool turnOff) {
  // Seeed's SSD2677 partial path sends a complete previous/current frame pair.
  display(bus, fb, prev, RefreshMode::Fast, turnOff);
}

GrayscaleCapabilities Ssd2677Driver::grayscaleCapabilities(GrayscaleMode mode) const {
  if (!_previous) return {};
  if (mode == GrayscaleMode::Overlay)
    return {GrayscaleEncoding::OverlayMasks, GrayscaleBase::Combined, true, false, false};
  return {GrayscaleEncoding::AbsolutePlanes, GrayscaleBase::Combined, true, false, false};
}

void Ssd2677Driver::beginGrayscale(EpdBus&, const uint8_t* fb, GrayscaleMode mode, RefreshMode, bool) {
  _grayMode = mode;
  _grayValid = false;
  if (_previous && fb) memcpy(_previous.get(), fb, FRAME_BYTES);
}

void Ssd2677Driver::ensureGrayBuffers() {
  if (!_grayLsb)
    _grayLsb.reset(static_cast<uint8_t*>(heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
  if (!_grayMsb)
    _grayMsb.reset(static_cast<uint8_t*>(heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
}

void Ssd2677Driver::writeGrayscalePlaneStrip(EpdBus&, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                              uint16_t numRows) {
  if (!rows || yStart >= 480 || numRows > 480 - yStart) return;
  ensureGrayBuffers();
  uint8_t* target = plane == GrayPlane::Lsb ? _grayLsb.get() : _grayMsb.get();
  if (!target) return;
  memcpy(target + static_cast<uint32_t>(yStart) * 100, rows, static_cast<uint32_t>(numRows) * 100);
  if (plane == GrayPlane::Msb && yStart + numRows == 480) _grayValid = true;
}

void Ssd2677Driver::copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) {
  writeGrayscalePlaneStrip(bus, GrayPlane::Lsb, lsb, 0, 480);
}

void Ssd2677Driver::copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) {
  writeGrayscalePlaneStrip(bus, GrayPlane::Msb, msb, 0, 480);
}

void Ssd2677Driver::displayGray(EpdBus& bus, const uint8_t* fb, bool, const unsigned char*, bool) {
  if (!_initialized) return;
  if (!_grayValid || !_grayLsb || !_grayMsb) {
    display(bus, fb, nullptr, RefreshMode::Full, true);
    return;
  }
  if (!prepareMode(bus, PanelMode::Gray)) return;
  const uint8_t* base = _previous ? _previous.get() : fb;
  latchWaveform(bus, RefreshMode::Full, true);
  writeGray(bus, base);
  activate(bus, false);
  _previousValid = false;
  _grayValid = false;
}

void Ssd2677Driver::cleanupGrayscaleBuffers(EpdBus&, const uint8_t*) {
  _grayValid = false;
  _previousValid = false;
}

void Ssd2677Driver::deepSleep(EpdBus& bus) {
  powerOff(bus);
  writeRegister(bus, CMD_DEEP_SLEEP, 0xA5);
  _previousValid = false;
}

PanelDriver& ssd2677Driver() {
  static Ssd2677Driver instance;
  return instance;
}

}  // namespace freeink
