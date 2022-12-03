#pragma GCC optimize("O3,unroll-loops")
#include <Arduino.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include <gfx.hpp>
#include "pin_config.h"
using namespace gfx;

// the size of the screen
constexpr static const size16 screen_size(LCD_H_RES,LCD_V_RES);

using frame_buffer_t = bitmap<rgb_pixel<16>>;
constexpr static const int frame_buffer_parts = 5;
static uint8_t frame_buffer_data[frame_buffer_t::sizeof_buffer({screen_size.width,screen_size.height/frame_buffer_parts})];
frame_buffer_t frame_buffer({screen_size.width,screen_size.height/frame_buffer_parts},frame_buffer_data);
esp_lcd_panel_io_handle_t io_handle = NULL;
esp_lcd_panel_handle_t panel_handle = NULL;
const int16_t
  res_bits        = 12,   // Fractional resolution
  pixelWidth  = screen_size.width,  // TFT dimensions
  pixelHeight = screen_size.height,
  iterations  = 20,  // Fractal iteration limit or 'dwell'
  moveX=0,
  moveY=0;
float
  centerReal(-0.6), // Image center point in complex plane
  centerImag(0.0),
  rangeReal(3.0), // Image coverage in complex plane
  rangeImag(3.0),
  incRange(.95);

static bool flush_ready_cb(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
  return false;
}

void setup() {
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  Serial.begin(115200);
  
  pinMode(PIN_LCD_RD, OUTPUT);
  digitalWrite(PIN_LCD_RD, HIGH);
  
  esp_lcd_i80_bus_handle_t i80_bus = NULL;
  esp_lcd_i80_bus_config_t bus_config = {
      .dc_gpio_num = PIN_LCD_DC,
      .wr_gpio_num = PIN_LCD_WR,
      .clk_src = LCD_CLK_SRC_PLL160M,
      .data_gpio_nums =
          {
              PIN_LCD_D0,
              PIN_LCD_D1,
              PIN_LCD_D2,
              PIN_LCD_D3,
              PIN_LCD_D4,
              PIN_LCD_D5,
              PIN_LCD_D6,
              PIN_LCD_D7,
          },
      .bus_width = 8,
      .max_transfer_bytes = frame_buffer_t::sizeof_buffer(frame_buffer.dimensions()),
  };
  esp_lcd_new_i80_bus(&bus_config, &i80_bus);

  esp_lcd_panel_io_i80_config_t io_config = {
      .cs_gpio_num = PIN_LCD_CS,
      .pclk_hz = LCD_PIXEL_CLOCK_HZ,
      .trans_queue_depth = 20,
      .on_color_trans_done = flush_ready_cb,
      .user_ctx = NULL,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
      .dc_levels =
          {
              .dc_idle_level = 0,
              .dc_cmd_level = 0,
              .dc_dummy_level = 0,
              .dc_data_level = 1,
          },
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));
  
  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = PIN_LCD_RES,
      .color_space = ESP_LCD_COLOR_SPACE_RGB,
      .bits_per_pixel = 16,
  };
  esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);
  esp_lcd_panel_invert_color(panel_handle, true);

  esp_lcd_panel_swap_xy(panel_handle, true);
  esp_lcd_panel_mirror(panel_handle, false, true);
  // the gap is LCD panel specific, even panels with the same driver IC, can
  // have different gap value
  esp_lcd_panel_set_gap(panel_handle, 0, 35);

  // Lighten the screen with gradient 
  ledcSetup(0, 10000, 8);
  ledcAttachPin(PIN_LCD_BL, 0);
  for (uint8_t i = 0; i < 0xFF; i++) {
    ledcWrite(0, i);
    delay(2);
  }
  // make the bitmap all black
  draw::filled_rectangle(frame_buffer,frame_buffer.bounds(),color<rgb_pixel<16>>::black);
  int y = 0;
  // fill the screen with the black bitmap
  for(int i = 0;i<frame_buffer_parts;++i) {
    esp_lcd_panel_draw_bitmap(panel_handle, 0, y, screen_size.width, y+(screen_size.height/frame_buffer_parts), frame_buffer.begin());
    y+=(screen_size.height/frame_buffer_parts);
  }
  
}

void loop() {
  int64_t       n, a, b, a2, b2, posReal, posImag;
  uint32_t      startTime,elapsedTime;


  int32_t
    startReal   = (int64_t)((centerReal - rangeReal * 0.5)   * (float)(1 << res_bits)),
    startImag   = (int64_t)((centerImag + rangeImag * 0.5)   * (float)(1 << res_bits)),
    incReal     = (int64_t)((rangeReal / (float)pixelWidth)  * (float)(1 << res_bits)),
    incImag     = (int64_t)((rangeImag / (float)pixelHeight) * (float)(1 << res_bits));
  
  startTime = millis();
  posImag = startImag;
  int yy = 0;
  // draw the mandelbrot
  for (int y = 0;; y++) {
    // if we reached the end of one of the frame buffer segments...
    if(yy==(screen_size.height/frame_buffer_parts)) {
      yy=0; // reset the counter
      // draw the bitmap
      esp_lcd_panel_draw_bitmap(panel_handle, 0, y-(screen_size.height/frame_buffer_parts), screen_size.width, y, frame_buffer.begin());
      if(y==screen_size.height) {
        // nothing left to draw
        break;
      }
    }
    // compute the next point
    posReal = startReal;
    for (int x = 0; x < screen_size.width; x++) {
      a = posReal;
      b = posImag;
      for (n = iterations; n > 0 ; n--) {
        // use fixed precision for speed
        a2 = (a * a) >> res_bits;
        b2 = (b * b) >> res_bits;
        if ((a2 + b2) >= (4 << res_bits)) 
          break;
        b  = posImag + ((a * b) >> (res_bits - 1));
        a  = posReal + a2 - b2;
      }
      frame_buffer_t::pixel_type px;
      px.native_value =  (n * 29)<<8 | (n * 67);
      frame_buffer.point(point16(x,y%(screen_size.height/frame_buffer_parts)),px);
      posReal += incReal;
    }
    posImag -= incImag;
    ++yy;
  }
  elapsedTime = millis()-startTime;
  Serial.print("Took "); Serial.print(elapsedTime); Serial.println(" ms");

  rangeReal *= incRange;
  rangeImag *= incRange;
}
