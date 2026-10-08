#pragma once

#include <memory>

#include "PanelDriver.h"

namespace freeink {

// Seeed Sticky SSD2677. The panel accepts a 2-bit DTM1 stream even for B/W
// updates; partial updates interleave the previous and current 1-bit frames.
class Ssd2677Driver final : public PanelDriver {
 public:
  Ssd2677Driver();

  uint32_t spiHz() const override { return 20000000; }
  BusyPolarity busyPolarity() const override { return BusyPolarity::ActiveLow; }
  PanelGeometry geometry() const override { return {800, 480, 100, FRAME_BYTES}; }

  void begin(EpdBus& bus) override;
  void deepSleep(EpdBus& bus) override;
  void display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) override;
  void displayWindow(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, uint16_t x, uint16_t y, uint16_t w,
                     uint16_t h, bool turnOff) override;

  GrayscaleCapabilities grayscaleCapabilities(GrayscaleMode mode = GrayscaleMode::Overlay) const override;
  void beginGrayscale(EpdBus& bus, const uint8_t* fb, GrayscaleMode mode, RefreshMode fallback,
                      bool turnOff) override;
  void writeGrayscalePlaneStrip(EpdBus& bus, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                uint16_t numRows) override;
  void copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) override;
  void copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) override;
  void displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut,
                   bool factoryMode) override;
  void cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) override;
  void requestResync(uint8_t) override { _previousValid = false; }

 private:
  enum class PanelMode : uint8_t { Unselected, Full, Partial, Gray };

  static constexpr uint16_t ROW_BYTES = 800U / 8U;
  static constexpr uint16_t ROW_COUNT = 480;
  static constexpr uint32_t FRAME_BYTES = static_cast<uint32_t>(ROW_BYTES) * ROW_COUNT;
  static constexpr uint16_t CHUNK_BYTES = 512;
  struct PsramDeleter {
    void operator()(uint8_t* ptr) const;
  };
  using PsramBuffer = std::unique_ptr<uint8_t, PsramDeleter>;

  static uint8_t fullWaveform(uint8_t temperature);
  static uint8_t partialWaveform(uint8_t temperature);
  static uint8_t grayWaveform(uint8_t temperature);
  static uint8_t reverseBits(uint8_t value);
  static void packByte(uint8_t high, uint8_t low, uint8_t* output);

  void writeRegister(EpdBus& bus, uint8_t command, uint8_t value);
  void initializePanel(EpdBus& bus);
  bool prepareMode(EpdBus& bus, PanelMode mode);
  void latchWaveform(EpdBus& bus, RefreshMode mode, bool gray);
  void writeMono(EpdBus& bus, const uint8_t* current, const uint8_t* previous);
  void writeGray(EpdBus& bus, const uint8_t* base);
  void activate(EpdBus& bus, bool keepPowered);
  void powerOff(EpdBus& bus);
  void ensureGrayBuffers();

  PsramBuffer _previous;
  PsramBuffer _grayLsb;
  PsramBuffer _grayMsb;
  uint8_t _transfer[CHUNK_BYTES * 2] = {};
  bool _previousValid = false;
  bool _powered = false;
  bool _grayValid = false;
  bool _initialized = false;
  PanelMode _activeMode = PanelMode::Unselected;
  GrayscaleMode _grayMode = GrayscaleMode::Overlay;
};

PanelDriver& ssd2677Driver();

}  // namespace freeink
