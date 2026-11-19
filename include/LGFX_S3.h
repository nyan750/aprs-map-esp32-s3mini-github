#pragma once
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// ESP32-S3 Super Mini + 2.8" TFT SPI 240x320 V1.1 module
//   Classic red module, ILI9341-class panel + XPT2046 resistive touch.
//   Display bus : SPI SCLK=GPIO12 MOSI=GPIO11 MISO=GPIO13 DC=GPIO9
//   Panel       : CS=GPIO10 RST=GPIO8, LED -> 3.3V (Q1 transistor on module)
//                 VCC = 5V (module regulator fitted, jumper J1 open)
//   Touch bus   : separate SPI, T_CLK=GPIO6 T_DIN=GPIO5 T_DO=GPIO4
//                 T_CS=GPIO2 T_IRQ=GPIO1
//   SD          : module SD slot NOT used (SD_CS stays unconnected)
//   Buttons     : none (touch replaces them)
//   Note: this panel returns byte-swapped RGB565 from pushImage;
//   the firmware's colour map data is stored byte-swapped to match
//   (see src/main.cpp panelPack565 / colorAt).
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341   _panel;
  lgfx::Bus_SPI         _bus;
  lgfx::Touch_XPT2046   _touch;

public:
  LGFX() {
    {
      auto cfg = _bus.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 40000000;
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 12;
      cfg.pin_mosi = 11;
      cfg.pin_miso = 13;
      cfg.pin_dc   = 9;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs   = 10;
      cfg.pin_rst  = 8;
      cfg.pin_busy = -1;
      cfg.panel_width   = 240;
      cfg.panel_height  = 320;
      cfg.memory_width  = 240;
      cfg.memory_height = 320;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.readable  = true;
      cfg.invert    = false;   // HYPOTHESIS: typical ILI9341 module, inversion OFF
      cfg.rgb_order = false;   // HYPOTHESIS: RGB order
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel.config(cfg);
    }
    {
      auto cfg = _touch.config();
      cfg.freq     = 1000000;
      cfg.x_min    = 300;
      cfg.x_max    = 3900;
      cfg.y_min    = 400;
      cfg.y_max    = 3900;
      cfg.pin_int  = 1;
      cfg.bus_shared = false;
      cfg.offset_rotation = 0;
      cfg.spi_host = SPI3_HOST;
      cfg.pin_sclk = 6;
      cfg.pin_mosi = 5;
      cfg.pin_miso = 4;
      cfg.pin_cs   = 2;
      _touch.config(cfg);
      _panel.setTouch(&_touch);
    }
    setPanel(&_panel);
  }
};
