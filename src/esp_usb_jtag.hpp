// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2024 EMARD
 */

#ifndef SRC_ESPUSBJTAG_HPP_
#define SRC_ESPUSBJTAG_HPP_

#include <libusb.h>

#include "jtagInterface.hpp"

/*!
 * \file esp_usb_jtag.hpp
 * \class esp_usb_jtag
 * \brief ESP32C3, ESP32C6 ESP32S2, ESP32S3 hardware USB JTAG
 * \author EMARD
 */

class esp_usb_jtag : public JtagInterface {
 public:
    esp_usb_jtag(uint32_t clkHZ, int8_t verbose, int vid, int pid);
    virtual ~esp_usb_jtag();

    int setClkFreq(uint32_t clkHZ) override;

    /* TMS */
    int writeTMS(const uint8_t *tms, uint32_t len, bool flush_buffer, const uint8_t tdi = 1) override;
    /* TDI */
    int writeTDI(const uint8_t *tx, uint8_t *rx, uint32_t len, bool end) override;
    /* clk */
    int toggleClk(uint8_t tms, uint8_t tdo, uint32_t clk_len) override;

    /*!
     * \brief return internal buffer size (in byte).
     * \return _buffer_size divided by 2 (two byte for clk) and divided by 8 (one
     * state == one byte)
     */
    int get_buffer_size() override { return 0;}

    bool isFull() override { return false;}

    int flush() override;

 private:
    int8_t _verbose;

    // int sendBitBang(uint8_t mask, uint8_t val, uint8_t *read, bool last);
    bool getVersion();
    
    void drain_in();
    int setio(int srst, int tms, int tdi, int tck);
    int gettdo();

    libusb_device_handle *dev_handle;
    libusb_context *usb_ctx;
    uint8_t _tdi;
    uint8_t _tms;
    uint8_t _version;
    uint32_t _base_speed_khz;
    uint8_t _div_min, _div_max;
};
#endif  // SRC_ESPUSBJTAG_HPP_
