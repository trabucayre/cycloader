// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2024 EMARD
 */

#include <libusb.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <cassert>

#include "esp_usb_jtag.hpp"
#include "display.hpp"

using namespace std;

#define ESPUSBJTAG_VID 0x303A
#define ESPUSBJTAG_PID 0x1001

#define ESPUSBJTAG_INTF		  0
#define ESPUSBJTAG_WRITE_EP    0x01
#define ESPUSBJTAG_READ_EP     0x82

#define ESPUSBJTAG_TIMEOUT     1000

enum esp_usb_jtag_cmd {
	CMD_STOP =  0x00,
	CMD_INFO =  0x01,
	CMD_FREQ =  0x02,
	CMD_XFER =  0x03,
	CMD_SETSIG = 0x04,
	CMD_GETSIG = 0x05,
	CMD_CLK =    0x06
};

// Modifiers applicable only to esp_usb_jtag_2
enum CommandModifier {
  EXTEND_LENGTH = 0x40,
  NO_READ       = 0x80
};

struct version_specific
{
	uint8_t no_read;  // command modifier for xfer no read
	uint16_t max_bits;  // max bit count that can be transferred
};

static version_specific v_options[4] ={{0, 240}, {0, 240}, {NO_READ, 496},
									{NO_READ, 4000}};


enum espUSBJtagSig {
	SIG_TCK =   (1 << 1),
	SIG_TDI =   (1 << 2),
	SIG_TDO =   (1 << 3),
	SIG_TMS =   (1 << 4)
};

esp_usb_jtag::esp_usb_jtag(uint32_t clkHZ, int8_t verbose, int vid = ESPUSBJTAG_VID, int pid = ESPUSBJTAG_PID):
			_verbose(verbose),
			dev_handle(NULL), usb_ctx(NULL), _tdi(0), _tms(0)
{
	int ret;

	if (libusb_init(&usb_ctx) < 0) {
		cerr << "libusb init failed" << endl;
		throw std::exception();
	}

	dev_handle = libusb_open_device_with_vid_pid(usb_ctx,
					ESPUSBJTAG_VID, ESPUSBJTAG_PID);
	if (!dev_handle) {
		cerr << "fails to open device" << endl;
		libusb_exit(usb_ctx);
		throw std::exception();
	}

	ret = libusb_claim_interface(dev_handle, ESPUSBJTAG_INTF);
	if (ret) {
		cerr << "libusb error while claiming esp_usb_jtag interface" << endl;
		libusb_close(dev_handle);
		libusb_exit(usb_ctx);
		throw std::exception();
	}

	_version = 0;
	if (!getVersion())
		throw std::runtime_error("Fail to get version");

	if (setClkFreq(clkHZ) < 0) {
		cerr << "Fail to set frequency" << endl;
		throw std::exception();
	}
}

esp_usb_jtag::~esp_usb_jtag()
{
	if (dev_handle)
		libusb_close(dev_handle);
	if (usb_ctx)
		libusb_exit(usb_ctx);
}

bool esp_usb_jtag::getVersion()
{
	int actual_length;
	int ret;
	uint8_t buf[] = {CMD_INFO,
					CMD_STOP};
	uint8_t rx_buf[64];
	ret = libusb_bulk_transfer(dev_handle, ESPUSBJTAG_WRITE_EP,
					buf, 2, &actual_length, ESPUSBJTAG_TIMEOUT);
	if (ret < 0) {
		cerr << "getVersion: usb bulk write failed " << ret << endl;
		return false;
	}
	do {
		ret = libusb_bulk_transfer(dev_handle, ESPUSBJTAG_READ_EP,
						rx_buf, 64, &actual_length, ESPUSBJTAG_TIMEOUT);
		if (ret < 0) {
			cerr << "getVersion: read: usb bulk read failed " << ret << endl;
			return false;
		}
	} while (actual_length == 0);
	if (!strncmp("DJTAG1\n", (char*)rx_buf, 7)) {
		_version = 1;
	} else if (!strncmp("DJTAG2\n", (char*)rx_buf, 7)) {
		_version = 2;
	} else if (!strncmp("DJTAG3\n", (char*)rx_buf, 7)) {
		_version = 3;
	} else 	{
		cerr << "espUSBJtag version unknown" << endl;
		_version = 0;
	}

	return true;
}



int esp_usb_jtag::setClkFreq(uint32_t clkHZ)
{
	int actual_length;
	int ret, req_freq = clkHZ;

	if (clkHZ > 40000000) {
		printWarn("esp_usb_jtag probe limited to 40000kHz");
		clkHZ = 40000000;
	}

	_clkHZ = clkHZ;

	printInfo("Jtag frequency : requested " + std::to_string(req_freq) +
			"Hz -> real " + std::to_string(clkHZ) + "Hz");

	uint8_t buf[] = {CMD_FREQ,
					static_cast<uint8_t>(0xff & ((clkHZ / 1000) >> 8)),
					static_cast<uint8_t>(0xff & ((clkHZ / 1000)     )),
					CMD_STOP};
	ret = libusb_bulk_transfer(dev_handle, ESPUSBJTAG_WRITE_EP,
					buf, 4, &actual_length, ESPUSBJTAG_TIMEOUT);
	if (ret < 0) {
		cerr << "setClkFreq: usb bulk write failed " << ret << endl;
		return -EXIT_FAILURE;
	}

	return clkHZ;
}

int esp_usb_jtag::writeTMS(const uint8_t *tms, uint32_t len,
		__attribute__((unused)) bool flush_buffer,
		__attribute__((unused)) const uint8_t tdi)
{
	int actual_length;

	if (len == 0)
		return 0;

	uint8_t mask = SIG_TCK | SIG_TMS;
	uint8_t buf[64];
	u_int buffer_idx = 0;
	for (uint32_t i = 0; i < len; i++)
	{
		uint8_t val = (tms[i >> 3] & (1 << (i & 0x07))) ? SIG_TMS : 0;
		buf[buffer_idx++] = CMD_SETSIG;
		buf[buffer_idx++] = mask;
		buf[buffer_idx++] = val;
		buf[buffer_idx++] = CMD_SETSIG;
		buf[buffer_idx++] = mask;
		buf[buffer_idx++] = val | SIG_TCK;
		if ((buffer_idx + 9) >= sizeof(buf) || (i == len - 1)) {
			// flush the buffer
			if (i == len - 1) {
				// insert tck falling edge
				buf[buffer_idx++] = CMD_SETSIG;
				buf[buffer_idx++] = mask;
				buf[buffer_idx++] = val;
			}
			buf[buffer_idx++] = CMD_STOP;
			int ret = libusb_bulk_transfer(dev_handle, ESPUSBJTAG_WRITE_EP,
										   buf, buffer_idx, &actual_length,
										   ESPUSBJTAG_TIMEOUT);
			if (ret < 0)
			{
				cerr << "writeTMS: usb bulk write failed " << ret << endl;
				return -EXIT_FAILURE;
			}
			buffer_idx = 0;
		}
	}
	return len;
}

int esp_usb_jtag::toggleClk(uint8_t tms, uint8_t tdi, uint32_t clk_len)
{
	int actual_length;
	uint8_t buf[] = {CMD_CLK,
				static_cast<uint8_t>(((tms) ? SIG_TMS : 0) | ((tdi) ? SIG_TDI : 0)),
				0,
				CMD_STOP};
	while (clk_len > 0) {
		buf[2] = (clk_len > 64) ? 64 : (uint8_t)clk_len;

		int ret = libusb_bulk_transfer(dev_handle, ESPUSBJTAG_WRITE_EP,
				buf, 4, &actual_length, ESPUSBJTAG_TIMEOUT);
		if (ret < 0) {
			cerr << "toggleClk: usb bulk write failed " << ret << endl;
			return -EXIT_FAILURE;
		}
		clk_len -= buf[2];
	}

	return EXIT_SUCCESS;
}

int esp_usb_jtag::flush()
{
	return 0;
}

int esp_usb_jtag::writeTDI(const uint8_t *tx, uint8_t *rx, uint32_t len, bool end)
{
	int actual_length;
	uint32_t real_bit_len = len - (end ? 1 : 0);
	uint32_t kRealByteLen = (len + 7) / 8;

	uint8_t tx_cpy[kRealByteLen];
	uint8_t tx_buf[512], rx_buf[512];
	uint8_t *tx_ptr, *rx_ptr = rx;

	if (tx)
		memcpy(tx_cpy, tx, kRealByteLen);
	else
		memset(tx_cpy, 0, kRealByteLen);
	tx_ptr = tx_cpy;

	tx_buf[0] = CMD_XFER | (rx ? 0 : v_options[_version].no_read);
	uint16_t max_bit_transfer_length = v_options[_version].max_bits;
	// need to cut the bits on byte size.
	assert(max_bit_transfer_length % 8 == 0);
	while (real_bit_len != 0) {
		uint16_t bit_to_send = (real_bit_len > max_bit_transfer_length) ?
			max_bit_transfer_length : real_bit_len;
		size_t byte_to_send = (bit_to_send + 7) / 8;
		size_t header_offset = 0;
		if (_version == 3) {
			tx_buf[1] = (bit_to_send >> 8) & 0xFF;
			tx_buf[2] = bit_to_send & 0xFF;
			header_offset = 3;
		} else if (bit_to_send > 255) {
			tx_buf[0] |= EXTEND_LENGTH;
			tx_buf[1] = bit_to_send - 256;
			header_offset = 2;
		}else {
			tx_buf[0] &= ~EXTEND_LENGTH;
			tx_buf[1] = bit_to_send;
			header_offset = 2;
		}
		memset(tx_buf + header_offset, 0, byte_to_send);
		for (int i = 0; i < bit_to_send; i++)
			if (tx_ptr[i >> 3] & (1 << (i & 0x07)))
				tx_buf[header_offset + (i >> 3)] |= (0x80 >> (i & 0x07));

		actual_length = 0;
		int ret = libusb_bulk_transfer(dev_handle, ESPUSBJTAG_WRITE_EP,
				(unsigned char *)tx_buf, (byte_to_send + header_offset),
				&actual_length, ESPUSBJTAG_TIMEOUT);
		if ((ret < 0) || (actual_length != (int)(byte_to_send + header_offset))) {
			cerr << "writeTDI: fill: usb bulk write failed " << ret <<
				"actual length: " << actual_length << endl;
			return EXIT_FAILURE;
		}
		// cerr << actual_length << ", " << bit_to_send << endl;

		if (rx || (_version <= 1)) {
			int transfer_length = (bit_to_send > 255) ? byte_to_send :32;
			do {
				ret = libusb_bulk_transfer(dev_handle, ESPUSBJTAG_READ_EP,
					rx_buf, transfer_length, &actual_length, ESPUSBJTAG_TIMEOUT);
				if (ret < 0) {
					cerr << "writeTDI: read: usb bulk read failed " << ret << endl;
					return EXIT_FAILURE;
				}
			} while (actual_length == 0);
			assert((size_t)actual_length >= byte_to_send);
		}

		if (rx) {
			for (int i = 0; i < bit_to_send; i++)
				rx_ptr[i >> 3] = (rx_ptr[i >> 3] >> 1) |
						(((rx_buf[i >> 3] << (i&0x07)) & 0x80));
			rx_ptr += byte_to_send;
		}

		real_bit_len -= bit_to_send;
		tx_ptr += byte_to_send;
	}

	/* this step exist only with [D|I]R_SHIFT */
	if (end) {
		int pos = len-1;
		uint8_t sig;
		unsigned char last_bit =
				(tx_cpy[pos >> 3] & (1 << (pos & 0x07))) ? SIG_TDI: 0;

		uint8_t mask = SIG_TMS | SIG_TDI;
		uint8_t val = SIG_TMS | (last_bit);

		if (rx)
		{
			mask |= SIG_TCK;
			uint8_t buf[] = {
				CMD_SETSIG,
				static_cast<uint8_t>(mask),
				static_cast<uint8_t>(val),
				CMD_SETSIG,
				static_cast<uint8_t>(mask),
				static_cast<uint8_t>(val | SIG_TCK),
				CMD_GETSIG,  // <---Read instruction
				CMD_STOP,
			};
			if (libusb_bulk_transfer(dev_handle, ESPUSBJTAG_WRITE_EP,
									 buf, sizeof(buf), &actual_length,
									 ESPUSBJTAG_TIMEOUT) < 0)
			{
				cerr << "writeTDI: last bit error: usb bulk write failed 1" << endl;
				return -EXIT_FAILURE;
			}
			do
			{
				if (libusb_bulk_transfer(dev_handle, ESPUSBJTAG_READ_EP,
											&sig, 1, &actual_length,
											ESPUSBJTAG_TIMEOUT) < 0)
				{
					cerr << "writeTDI: last bit error: usb bulk read failed" << endl;
					return -EXIT_FAILURE;
				}
			} while (actual_length == 0);
			rx[pos >> 3] >>= 1;
			if (sig & SIG_TDO)
			{
				rx[pos >> 3] |= (1 << (pos & 0x07));
			}
			buf[2] &= ~SIG_TCK;
			buf[3] = CMD_STOP;
			if (libusb_bulk_transfer(dev_handle, ESPUSBJTAG_WRITE_EP,
									 buf, 4, &actual_length,
									 ESPUSBJTAG_TIMEOUT) < 0)
			{
				cerr << "writeTDI: last bit error: usb bulk write failed 2" << endl;
				return -EXIT_FAILURE;
			}

		} else {
			if (toggleClk(SIG_TMS, last_bit, 1)) {
				cerr << "writeTDI: last bit error" << endl;
				return -EXIT_FAILURE;
			}
		}
	}
	return EXIT_SUCCESS;
}
