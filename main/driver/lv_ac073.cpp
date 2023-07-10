#include <lvgl.h>
#include <esp_log.h>
#include <esp_heap_caps.h>

#include "lv_ac073.hpp"

esp_err_t lv_ac073::init(gpio_num_t _sda, gpio_num_t _scl, gpio_num_t _cs, gpio_num_t _dc, gpio_num_t _rst, gpio_num_t _busy, spi_host_device_t _spi_periph)
{
    lv_state = xEventGroupCreate();
    if (lv_state == nullptr) {
        ESP_LOGE(TAG, "Failed to create state event group");
        return ESP_ERR_NO_MEM;
    }

    auto ret = epd->init(_sda, _scl, _cs, _dc, _rst, _busy, _spi_periph);
    if (ret != ESP_OK) {
        return ret;
    }

    lv_fb = static_cast<uint8_t *>(heap_caps_calloc(1, ac073_def::hor_size * ac073_def::ver_size, MALLOC_CAP_SPIRAM));
    if (lv_fb == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate LVGL framebuffer");
        return ESP_ERR_NO_MEM;
    }

    disp_fb = static_cast<uint8_t *>(heap_caps_calloc(1, ac073_def::hor_size * ac073_def::ver_size, MALLOC_CAP_SPIRAM));
    if (disp_fb == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate display framebuffer");
        return ESP_ERR_NO_MEM;
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

    auto ret_bits = xEventGroupWaitBits(ctx->lv_state, ac073_def::STATE_RENDER_IDLE, pdTRUE, pdTRUE, pdMS_TO_TICKS(10000));
    if ((ret_bits & ac073_def::STATE_RENDER_IDLE) != 0) {
        xEventGroupSetBits(ctx->lv_state, ac073_def::STATE_RENDER_IDLE);
        ESP_LOGE(TAG, "Timeout when waiting previous render");
    }

    ESP_LOGI(TAG, "Framebuffer flush begin, x1=%d, y1=%d; x2=%d, y2=%d", area->x1, area->y1, area->x2, area->y2);

    size_t processed_pix_cnt = 0;
    for (size_t idx = 0; idx < lv_fb_len; idx += 2) {
        ctx->disp_fb[processed_pix_cnt] = (((find_nearest_color(color_p[idx].full) & 0b1111) << 4) | (find_nearest_color(color_p[idx + 1].full) & 0b1111));
        processed_pix_cnt += 1;
    }

    ESP_LOGI(TAG, "Framebuffer flush end");
    xEventGroupSetBits(ctx->lv_state, ac073_def::STATE_RENDER_IDLE);
    lv_disp_flush_ready(disp_drv);
}

uint8_t lv_ac073::find_nearest_color(uint8_t color)
{
    return ac073_def::palette_lut[color];
}

esp_err_t lv_ac073::render(uint32_t timeout_ms)
{
    auto ret_bits = xEventGroupWaitBits(lv_state, ac073_def::STATE_RENDER_IDLE, pdTRUE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
    if ((ret_bits & ac073_def::STATE_RENDER_IDLE) != 0) {
        ESP_LOGE(TAG, "Timeout when waiting previous render");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Render begin!");
    uint32_t remain_timeout = timeout_ms;
    while(lv_disp->inv_p > 0) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
        if (remain_timeout > 1) {
            remain_timeout -= 10;
        } else {
            xEventGroupSetBits(lv_state, ac073_def::STATE_RENDER_IDLE);
            ESP_LOGE(TAG, "Timeout when rendering!");
            return ESP_ERR_TIMEOUT;
        }
    }

    ESP_LOGI(TAG, "Render end!");
    xEventGroupSetBits(lv_state, ac073_def::STATE_RENDER_IDLE);
    return ESP_OK;
}

esp_err_t lv_ac073::commit(uint32_t timeout_ms)
{
    auto ret_bits = xEventGroupWaitBits(lv_state, ac073_def::STATE_XFER_IDLE, pdTRUE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
    if ((ret_bits & ac073_def::STATE_XFER_IDLE) != 0) {
        ESP_LOGE(TAG, "Timeout when waiting previous SPI transfer");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "EPD power on!");
    auto ret = epd->power_on();
    ret = ret ?: epd->commit_framebuffer(disp_fb, disp_fb_len);
    if (ret != ESP_OK) {
        xEventGroupSetBits(lv_state, ac073_def::STATE_XFER_IDLE);
        ESP_LOGE(TAG, "EPD buffer commit returned non-OK result: 0x%x", ret);
        epd->sleep(); // Try sleep anyway
        return ret;
    }

    ESP_LOGI(TAG, "Refresh begin!");
    ret = epd->refresh(timeout_ms);
    if (ret != ESP_OK) {
        xEventGroupSetBits(lv_state, ac073_def::STATE_XFER_IDLE);
        ESP_LOGE(TAG, "EPD refresh returned non-OK result: 0x%x", ret);
        epd->sleep(); // Try sleep anyway
        return ret;
    }

    ESP_LOGI(TAG, "Refresh OK, going to sleep!");
    ret = epd->sleep();
    ESP_LOGI(TAG, "Slept!");
    xEventGroupSetBits(lv_state, ac073_def::STATE_XFER_IDLE);
    return ret;
}
