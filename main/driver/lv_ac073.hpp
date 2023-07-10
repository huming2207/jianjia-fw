#pragma once

#include <cstdint>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <lvgl.h>

#include "ac073tc1.hpp"

namespace ac073_def
{
    // From Waveshare demo code
    static const constexpr uint8_t palette[8 * 3] = {
            0, 0, 0,
            255, 255, 255,
            67, 138, 28,
            100, 64, 255,
            191, 0, 0,
            255, 243, 56,
            232, 126, 0,
            194 ,164 , 244 // Why we need this line (21-23)??
    };

    enum colors : uint8_t {
        BLACK = 0,
        WHITE = 1,
        GREEN = 2,
        BLUE = 3,
        RED = 4,
        YELLOW = 5,
        ORANGE = 6,
    };

    enum lv_state_bits : uint32_t {
        STATE_RENDER_IDLE = BIT0,
        STATE_XFER_IDLE = BIT1,
    };

    static const constexpr uint8_t palette_lut[] = {
            BLACK, BLUE, BLUE, BLUE, // 0x00
            BLACK, BLUE, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE, // 0x10
            GREEN, GREEN, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE,
            BLACK, BLUE, BLUE, BLUE, // 0x20
            BLACK, BLUE, BLUE, BLUE,
            GREEN, BLUE, BLUE, BLUE,
            GREEN, BLUE, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE, // 0x30
            GREEN, GREEN, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE,
            GREEN, GREEN, GREEN, BLUE,
            BLACK, BLUE, BLUE, BLUE, // 0x40
            RED, BLUE, BLUE, BLUE,
            GREEN, BLUE, BLUE, BLUE,
            GREEN, BLUE, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE, // 0x50
            GREEN, GREEN, BLUE, BLUE,
            GREEN, GREEN, GREEN, BLUE,
            GREEN, GREEN, GREEN, BLUE,
            RED, BLUE, BLUE, BLUE, // 0x60
            RED, BLUE, BLUE, BLUE,
            ORANGE, BLUE, BLUE, BLUE,
            YELLOW, BLUE, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE, // 0x70
            GREEN, GREEN, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE,
            RED, BLUE, BLUE, BLUE, // 0x80
            RED, BLUE, BLUE, BLUE,
            ORANGE, BLUE, BLUE, BLUE,
            ORANGE, BLUE, BLUE, BLUE,
            ORANGE, GREEN, BLUE, BLUE, // 0x90
            GREEN, GREEN, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE,
            GREEN, GREEN, BLUE, BLUE,
            RED, BLUE, BLUE, BLUE, // 0xa0
            RED, BLUE, BLUE, BLUE,
            ORANGE, ORANGE, BLUE, BLUE,
            YELLOW, YELLOW, BLUE, BLUE,
            YELLOW, YELLOW, BLUE, BLUE, // 0xb0
            YELLOW, YELLOW, BLUE, BLUE,
            YELLOW, GREEN, BLUE, BLUE,
            YELLOW, GREEN, BLUE, BLUE,
            RED, BLUE, BLUE, BLUE, // 0xc0
            RED, RED, BLUE, BLUE,
            ORANGE, RED, BLUE, BLUE,
            ORANGE, ORANGE, BLUE, BLUE,
            YELLOW, ORANGE, BLUE, BLUE, // 0xd0
            YELLOW, YELLOW, BLUE, BLUE,
            YELLOW, YELLOW, WHITE, BLUE,
            YELLOW, YELLOW, WHITE, WHITE,
            RED, RED, BLUE, BLUE, // 0xe0
            RED, RED, BLUE, BLUE,
            ORANGE, RED, BLUE, BLUE,
            ORANGE, ORANGE, BLUE, BLUE,
            ORANGE, YELLOW, BLUE, BLUE, // 0xf0
            ORANGE, YELLOW, WHITE, WHITE,
            ORANGE, YELLOW, WHITE, WHITE,
            YELLOW, YELLOW, WHITE, WHITE,
    };
}



class lv_ac073
{
public:
    static lv_ac073 *instance()
    {
        static lv_ac073 _instance;
        return &_instance;
    }

    lv_ac073(lv_ac073 const &) = delete;
    void operator=(lv_ac073 const &) = delete;

public:
    esp_err_t init(gpio_num_t _sda = GPIO_NUM_16, gpio_num_t _scl = GPIO_NUM_15, gpio_num_t _cs = GPIO_NUM_7,
                   gpio_num_t _dc = GPIO_NUM_6, gpio_num_t _rst = GPIO_NUM_5, gpio_num_t _busy = GPIO_NUM_4, spi_host_device_t _spi_periph = SPI3_HOST);

    esp_err_t render(uint32_t timeout_ms = 10000);
    esp_err_t commit(uint32_t timeout_ms = 60000);

private:
    lv_ac073() = default;
    static void flush_handler(lv_disp_drv_t *disp_drv, const lv_area_t * area, lv_color_t * color_p);
    static inline uint8_t find_nearest_color(uint8_t pix);

private:
    static const constexpr char TAG[] = "lv_ac073";
    static const constexpr size_t lv_fb_len = (ac073_def::hor_size * ac073_def::ver_size);
    static const constexpr size_t disp_fb_len = (ac073_def::hor_size * ac073_def::ver_size) / 2;
    uint8_t *lv_fb = nullptr;
    uint8_t *disp_fb = nullptr;

private:
    ac073tc1 *epd = ac073tc1::instance();
    lv_disp_t *lv_disp = nullptr;
    EventGroupHandle_t lv_state = nullptr;
    lv_disp_draw_buf_t lv_draw_buf = {};
};
