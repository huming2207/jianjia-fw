#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ac073tc1.hpp"

esp_err_t ac073tc1::init(gpio_num_t _sda, gpio_num_t _scl, gpio_num_t _cs, gpio_num_t _dc, gpio_num_t _rst, gpio_num_t _busy, spi_host_device_t _spi_periph)
{
    gpio_config_t pin_default_cfg = {};
    pin_default_cfg.mode = GPIO_MODE_DISABLE;
    pin_default_cfg.pin_bit_mask = ((1ULL << _sda) | (1ULL << _scl) | (1ULL << _cs) | (1ULL << _dc) | (1ULL << _rst) | (1ULL << _busy));
    pin_default_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    pin_default_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pin_default_cfg.intr_type = GPIO_INTR_DISABLE;
    auto ret = gpio_config(&pin_default_cfg);

    gpio_config_t out_pin_cfg = {};
    out_pin_cfg.mode = GPIO_MODE_OUTPUT;
    out_pin_cfg.pin_bit_mask = ((1ULL << _cs) | (1ULL << _dc) | (1ULL << _rst));
    out_pin_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    out_pin_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out_pin_cfg.intr_type = GPIO_INTR_DISABLE;
    ret = ret ?: gpio_config(&out_pin_cfg);

    ESP_LOGI(TAG, "Restting device");
    ret = ret ?: gpio_set_level(_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    ret = ret ?: gpio_set_level(_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config or reset failed");
        return ret;
    }

    spi_bus_config_t spi_bus_cfg = {};
    spi_bus_cfg.mosi_io_num = _sda,
    spi_bus_cfg.miso_io_num = GPIO_NUM_NC;
    spi_bus_cfg.sclk_io_num = _scl;
    spi_bus_cfg.data2_io_num = GPIO_NUM_NC;
    spi_bus_cfg.data3_io_num = GPIO_NUM_NC;
    spi_bus_cfg.data4_io_num = GPIO_NUM_NC;
    spi_bus_cfg.data5_io_num = GPIO_NUM_NC;
    spi_bus_cfg.data6_io_num = GPIO_NUM_NC;
    spi_bus_cfg.data7_io_num = GPIO_NUM_NC;
    spi_bus_cfg.max_transfer_sz = ac073_def::hor_size * ac073_def::ver_size;
    ret = ret ?: spi_bus_initialize(_spi_periph, &spi_bus_cfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t spi_dev_cfg = {};
    spi_dev_cfg.spics_io_num = GPIO_NUM_NC; // We do CS manually, cuz SPI dev can only send 32KB once per DMA transaction
    spi_dev_cfg.mode = 0; // CPOL & CPHA = 0 - from demo code
    spi_dev_cfg.flags = 0;
    spi_dev_cfg.clock_speed_hz = SPI_MASTER_FREQ_10M; // From demo code, no further timing info
    spi_dev_cfg.queue_size = 2;
    ret = ret ?: spi_bus_add_device(_spi_periph, &spi_dev_cfg, &spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI master setup failed");
        return ret;
    }

    ret = ret ?: send_sequence(init_seq, sizeof(init_seq) / sizeof(ac073_def::seq));

    return ret;
}

esp_err_t ac073tc1::send_data(const uint8_t *buf, size_t len)
{
    if (unlikely(buf == nullptr || len < 1)) {
        return ESP_ERR_INVALID_ARG;
    }

    auto ret = gpio_set_level(dc, 1); // Data = 1;
    spi_transaction_t spi_tract = {};
    spi_tract.tx_buffer = buf;
    spi_tract.length = len * 8;
    spi_tract.rxlength = 0;

    ret = ret ?: spi_device_transmit(spi_dev, &spi_tract);
    return ret;
}

esp_err_t ac073tc1::send_cmd(uint8_t cmd)
{
    auto ret = gpio_set_level(dc, 0); // Command = 0;
    spi_transaction_t spi_tract = {};
    spi_tract.tx_data[0] = cmd;
    spi_tract.length = 8;
    spi_tract.rxlength = 0;
    spi_tract.flags = SPI_TRANS_USE_TXDATA;

    ret = ret ?: spi_device_transmit(spi_dev, &spi_tract);
    return ret;
}

esp_err_t ac073tc1::send_sequence(const ac073_def::seq *seqs, size_t cnt)
{
    esp_err_t ret = ESP_OK;
    for (size_t idx = 0; idx < cnt; idx += 1) {
        ret = send_cmd(seqs[idx].cmd);
        if (seqs[idx].len > 1) {
            ret = ret ?: send_data(seqs[idx].data, seqs[idx].len);
        }

        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ret;
}
