#include <lvgl.h>
#include <esp_log.h>

#include "lv_ac073.hpp"

uint8_t lv_ac073::lv_fb[ac073_def::hor_size * ac073_def::ver_size] = {};
uint8_t lv_ac073::disp_fb[ac073_def::hor_size * ac073_def::ver_size / 2] = {};

esp_err_t lv_ac073::init(gpio_num_t _sda, gpio_num_t _scl, gpio_num_t _cs, gpio_num_t _dc, gpio_num_t _rst, gpio_num_t _busy, spi_host_device_t _spi_periph)
{
    auto ret = epd->init(_sda, _scl, _cs, _dc, _rst, _busy, _spi_periph);
    if (ret != ESP_OK) {
        return ret;
    }

    lv_init();
    lv_disp_draw_buf_init(&lv_draw_buf, lv_fb, nullptr, ac073_def::hor_size * ac073_def::ver_size);

    lv_disp_drv_t disp_drv = {};
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = flush_handler;
    disp_drv.antialiasing = 0;
    disp_drv.full_refresh = 1;
    disp_drv.draw_buf = &lv_draw_buf;
    disp_drv.hor_res = ac073_def::hor_size;
    disp_drv.ver_res = ac073_def::ver_size;
    disp_drv.user_data = this;
    lv_disp = lv_disp_drv_register(&disp_drv);
    if (lv_disp == nullptr) {
        ESP_LOGE(TAG, "LVGL init seems failed??");
        return ESP_FAIL;
    }


    return ret;
}

void lv_ac073::flush_handler(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    auto *ctx = (lv_ac073 *)disp_drv->user_data;

    size_t buf_len = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    if (buf_len < (ac073_def::ver_size * ac073_def::hor_size)) {
        ESP_LOGE(TAG, "Framebuffer is not full, x1=%d, y1=%d; x2=%d, y2=%d", area->x1, area->y1, area->x2, area->y2);
        return;
    }

    size_t processed_pix_cnt = 0;
    for (size_t idx = 0; idx < buf_len; idx += 2) {
        if (processed_pix_cnt >= sizeof(disp_fb)) {
            ESP_LOGE(TAG, "Out of bound!");
            break;
        }

        disp_fb[processed_pix_cnt] = (((find_nearest_color(color_p[idx].full) & 0b1111) << 4) | (find_nearest_color(color_p[idx + 1].full) & 0b1111));
        processed_pix_cnt += 1;
    }

    auto ret = ctx->epd->commit_framebuffer(disp_fb, sizeof(disp_fb));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "EPD buffer commit returned non-OK result: 0x%x", ret);
    }

    lv_disp_flush_ready(disp_drv);
}

uint8_t lv_ac073::find_nearest_color(uint8_t color)
{
    return ac073_def::palette_lut[color];
}
