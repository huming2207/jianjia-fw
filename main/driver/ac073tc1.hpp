#pragma once

#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>

namespace ac073_def
{
    static const constexpr size_t hor_size = 800;
    static const constexpr size_t ver_size = 480;

    struct seq
    {
        uint8_t cmd;
        uint8_t data[6];
        uint8_t len;
    };

    enum regs : uint8_t
    {
        PSR         = 0x00,
        PWRR        = 0x01,
        POF         = 0x02,
        POFS        = 0x03,
        PON         = 0x04,
        BTST1       = 0x05,
        BTST2       = 0x06,
        DSLP        = 0x07,
        BTST3       = 0x08,
        DTM         = 0x10,
        DRF         = 0x12,
        PLL         = 0x30,
        CDI         = 0x50,
        TCON        = 0x60,
        TRES        = 0x61,
        REV         = 0x70,
        VDCS        = 0x82,
        T_VDCS      = 0x84,
        PWS         = 0xE3,
    };
}

class ac073tc1
{
public:
    static ac073tc1 *instance()
    {
        static ac073tc1 _instance;
        return &_instance;
    }

    ac073tc1(ac073tc1 const &) = delete;
    void operator=(ac073tc1 const &) = delete;

public:
    esp_err_t init(gpio_num_t _sda, gpio_num_t _scl, gpio_num_t _cs, gpio_num_t _dc, gpio_num_t _rst, gpio_num_t _busy, spi_host_device_t _spi_periph = SPI3_HOST);

private:
    ac073tc1() = default;
    esp_err_t send_data(const uint8_t *buf, size_t len);
    esp_err_t send_cmd(uint8_t cmd);
    esp_err_t send_sequence(const ac073_def::seq *seqs, size_t cnt);

private:
    spi_device_handle_t spi_dev = nullptr;
    gpio_num_t cs = GPIO_NUM_NC;
    gpio_num_t dc = GPIO_NUM_NC;

private:
    static const constexpr char TAG[] = "ac073tc1";
    static const constexpr ac073_def::seq init_seq[] = {
            {0xAA, {0x49, 0x55, 0x20, 0x08, 0x09, 0x18}, 6},
            {ac073_def::PWRR, {0x3F, 0x00, 0x32, 0x2A, 0x0E, 0x2A}, 6},
            {ac073_def::PSR, {0x5F, 0x69}, 2},
            {ac073_def::POFS, {0x00, 0x54, 0x00, 0x44}, 4},
            {ac073_def::BTST1, {0x40, 0x1F, 0x1F, 0x2C}, 4},
            {ac073_def::BTST2, {0x6F, 0x1F, 0x16, 0x25}, 4},
            {ac073_def::BTST3, {0x6F, 0x1F, 0x1F, 0x22}, 4},
            {0x13, {0x00, 0x04}, 2},
            {ac073_def::PLL, {0x02}, 1},
            {0x41, {0x00}, 1},
            {ac073_def::CDI, {0x3F}, 1},
            {ac073_def::TCON, {0x02, 0x00}, 2},
            {ac073_def::TRES, {0x03, 0x20, 0x01, 0xE0}, 4},
            {ac073_def::VDCS, {0x1E}, 1},
            {ac073_def::T_VDCS, {0x00}, 1},
            {0x86, {0x00}, 1},
            {ac073_def::PWS, {0x2F}, 1},
            {0xE0, {0x00}, 1},
            {0xE6, {0x00}, 1},
            {0x04, {}, 0}, // Power ON, and wait here!
    };
};
