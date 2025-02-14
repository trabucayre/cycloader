// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2019 Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 */

#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>
#include <stdexcept>

#ifdef USE_UDEV
#include <libudev.h>
#endif
#include <libusb.h>

#include "display.hpp"
#include "ftdipp_mpsse.hpp"

using namespace std;

//#define DEBUG 1
#define display(...) \
	do { if (_verbose) fprintf(stdout, __VA_ARGS__);}while(0)

FTDIpp_MPSSE::FTDIpp_MPSSE(const cable_t &cable, const string &dev,
				const std::string &serial, uint32_t clkHZ, int8_t verbose):
				_verbose(verbose), _cable(cable.config), _vid(0),
				_pid(0), _index(0),
				_bus(cable.bus_addr), _addr(cable.device_addr),
				_bitmode(BITMODE_RESET),
				_interface(cable.config.interface),
				_clkHZ(clkHZ), _buffer_size(2*32768), _num(0)
{
	libusb_error ret;
	char err[256];

	strcpy(_product, "");
	if (!dev.empty()) {
		if (!search_with_dev(dev)) {
			cerr << "No cable found" << endl;
			throw std::runtime_error("No cable found");
		}
	} else {
		_vid = cable.vid;
		_pid = cable.pid;
		if (cable.config.index == -1)
			_index = 0;
		else
			_index = cable.config.index;
	}

	open_device(serial, 115200);
	_buffer_size = _ftdi->max_packet_size;

	_buffer = (unsigned char *)malloc(sizeof(unsigned char) * _buffer_size);
	if (!_buffer) {
		printError("_buffer malloc failed");
		throw std::runtime_error("_buffer malloc failed");
	}

	/* search for iProduct -> need to have
	 * ftdi->usb_dev (libusb_device_handler) -> libusb_device ->
	 * libusb_device_descriptor
	 */
	struct libusb_device * usb_dev = libusb_get_device(_ftdi->usb_dev);
	if (!usb_dev)
		throw std::runtime_error("can't get USB device");

	struct libusb_device_descriptor usb_desc;
	ret = (libusb_error)libusb_get_device_descriptor(usb_dev, &usb_desc);
	if (ret != 0) {
		snprintf(err, sizeof(err), "unable to get device descriptor: %d %s %s",
			ret, libusb_error_name(ret), libusb_strerror(ret));
		throw std::runtime_error(err);
	}

	ret = (libusb_error)libusb_get_string_descriptor_ascii(_ftdi->usb_dev,
		usb_desc.iProduct, _iproduct, 200);
	/* when FTDI device has no iProduct, libusb return an error
	 * but there is no distinction between
	 * real error and empty field
	 */
	if (ret < 0) {
		snprintf(err, sizeof(err),
			"Can't read iProduct field from FTDI: "
			"considered as empty string");
		printWarn(err);
		memset(_iproduct,'\0', 200);
	}

	ret = (libusb_error)libusb_get_string_descriptor_ascii(_ftdi->usb_dev,
		usb_desc.iManufacturer, _imanufacturer, 200);
	/* when FTDI device has no iManufacturer, libusb returns an error
	 * but there is no distinction between real error and empty field
	 */
	if (ret < 0) {
		snprintf(err, sizeof(err),
			"Can't read iManufacturer field from FTDI: "
			"considered as empty string");
		printWarn(err);
		memset(_imanufacturer,'\0', 200);
	}

	ret = (libusb_error)libusb_get_string_descriptor_ascii(_ftdi->usb_dev,
		usb_desc.iSerialNumber, _iserialnumber, 200);
	/* when FTDI device has no iSerialNumber, libusb returns an error
	 * but there is no distinction between real error and empty field
	 */
	if (ret < 0) {
		snprintf(err, sizeof(err),
			"Can't read iSerialNumber field from FTDI: "
			"considered as empty string");
		printWarn(err);
		memset(_iserialnumber,'\0', 200);
	}
}

FTDIpp_MPSSE::~FTDIpp_MPSSE()
{
	char err[256];
	int ret;

	if (_bitmode == BITMODE_MPSSE) {
		if (_cable.status_pin != -1) {
			gpio_set(1 << _cable.status_pin);
		}
	}

	if ((ret = ftdi_set_bitmode(_ftdi, 0, BITMODE_RESET)) < 0) {
		snprintf(err, sizeof(err), "unable to config pins : %d %s",
			ret, ftdi_get_error_string(_ftdi));
		printError(err);
		free(_buffer);
		return;
	}

	if ((ret = ftdi_usb_reset(_ftdi)) < 0) {
		snprintf(err, sizeof(err), "unable to reset device : %d %s",
			ret, ftdi_get_error_string(_ftdi));
		printError(err);
		free(_buffer);
		return;
	}

	if (close_device() == EXIT_FAILURE)
		printError("unable to close device");
	free(_buffer);
}

void FTDIpp_MPSSE::open_device(const std::string &serial, unsigned int baudrate)
{
	int ret;

	display("try to open %x %x %d %d\n", _vid, _pid, _bus, _addr);

	_ftdi = ftdi_new();
	if (_ftdi == NULL) {
		cout << "open_device: failed to initialize ftdi" << endl;
		throw std::runtime_error("open_device: failed to initialize ftdi");
	}
#if (ATTACH_KERNEL && (FTDI_VERSION >= 105))
	_ftdi->module_detach_mode = AUTO_DETACH_REATACH_SIO_MODULE;
#endif

	if ((ret = ftdi_set_interface(_ftdi, (ftdi_interface)_interface)) < 0) {
		char err[256];
		snprintf(err, sizeof(err), "unable to set interface : %d %s",
			ret, ftdi_get_error_string(_ftdi));
		throw std::runtime_error(err);
	}

	if (_bus == 0 || _addr == 0)
		ret = ftdi_usb_open_desc_index(_ftdi, _vid, _pid, NULL, serial.empty() ? NULL : serial.c_str(), _index);
	else
#if (FTDI_VERSION < 104)
		ret = ftdi_usb_open_desc(_ftdi, _vid, _pid, _product, NULL);
#else
		ret = ftdi_usb_open_bus_addr(_ftdi, _bus, _addr);
#endif
	if (ret < 0) {
		char description[256];
		if (_bus == 0 || _addr == 0)
			memset(description, '\0', 256);
		else
#if (FTDI_VERSION < 104)
			snprintf(description, sizeof(description), "");
#else
			snprintf(description, sizeof(description), " (USB bus %d addr %d)",
				 _bus, _addr);
#endif
		fprintf(stderr, "unable to open ftdi device: %d (%s)%s\n",
			ret, ftdi_get_error_string(_ftdi), description);
		ftdi_free(_ftdi);
		throw std::runtime_error("unable to open ftdi device");
	}
	if (ftdi_set_baudrate(_ftdi, baudrate) < 0) {
		fprintf(stderr, "baudrate error\n");
		close_device();
		throw std::runtime_error("baudrate error");
	}
}

/* cf. ftdi.c same function */
void FTDIpp_MPSSE::ftdi_usb_close_internal()
{
	libusb_close(_ftdi->usb_dev);
	_ftdi->usb_dev = NULL;
}

int FTDIpp_MPSSE::close_device()
{
	int ret;
	if (_ftdi == NULL)
		return EXIT_FAILURE;

	/* purge FTDI */
#if (FTDI_VERSION < 105)
	ret = ftdi_usb_purge_rx_buffer(_ftdi);
	ret |= ftdi_usb_purge_tx_buffer(_ftdi);
	if (ret != 0) {
		printError("unable to purge FTDI buffers");
		return EXIT_FAILURE;
	}

	/*
	 * repompe de la fonction et des suivantes
	 */
	 if (_ftdi->usb_dev != NULL) {
		int rtn = libusb_release_interface(_ftdi->usb_dev, _ftdi->interface);
		if (rtn < 0) {
			fprintf(stderr, "release interface failed %d\n", rtn);
			return EXIT_FAILURE;
		}
#ifdef ATTACH_KERNEL
		/* libusb_attach_kernel_driver is only available on Linux. */
		if (_ftdi->module_detach_mode == AUTO_DETACH_SIO_MODULE) {
			rtn = libusb_attach_kernel_driver(_ftdi->usb_dev, _ftdi->interface);
			if( rtn != 0 && rtn != LIBUSB_ERROR_NOT_FOUND)
				fprintf(stderr, "detach error %d\n", rtn);
		}
#endif
	}
	ftdi_usb_close_internal();
#else
	if ((ret = ftdi_tciflush(_ftdi)) < 0) {
		printError("unable to purge read buffers");
		return EXIT_FAILURE;
	}
	if ((ret = ftdi_tcoflush(_ftdi)) < 0) {
		printError("unable to purge write buffers");
		return EXIT_FAILURE;
	}
	if ((ret = ftdi_usb_close(_ftdi)) < 0) {
		printError("unable to close device");
		return EXIT_FAILURE;
	}
#endif

	ftdi_free(_ftdi);
	return EXIT_SUCCESS;
}



int FTDIpp_MPSSE::init(unsigned char latency, unsigned char bitmask_mode,
				unsigned char mode)
{
	int ret;
	unsigned char buf_cmd[6] = { SET_BITS_LOW, 0, 0,
		SET_BITS_HIGH, 0, 0
	};

	if ((ret = ftdi_usb_reset(_ftdi)) < 0) {
		printError("FTDI reset error with code " +
				std::to_string(ret) + " (" +
				string(ftdi_get_error_string(_ftdi)) + ")");
		return ret;
	}

	if ((ret = ftdi_set_bitmode(_ftdi, 0x00, BITMODE_RESET)) < 0) {
		printError("FTDI bitmode reset error with code " +
				std::to_string(ret) + " (" +
				string(ftdi_get_error_string(_ftdi)) + ")");
		return ret;
	}

#if (FTDI_VERSION < 105)
	if ((ret = ftdi_usb_purge_buffers(_ftdi)) < 0) {
#else
	if ((ret = ftdi_tcioflush(_ftdi)) < 0) {
#endif
		printError("FTDI flush buffer error with code " +
				std::to_string(ret) + " (" +
				string(ftdi_get_error_string(_ftdi)) + ")");
		return ret;
	}
	if ((ret = ftdi_set_latency_timer(_ftdi, latency)) < 0) {
		printError("FTDI set latency timer error with code " +
				std::to_string(ret) + " (" +
				string(ftdi_get_error_string(_ftdi)) + ")");
		return ret;
	}
	/* enable mode */
	if ((ret = ftdi_set_bitmode(_ftdi, bitmask_mode, mode)) < 0) {
		printError("FTDI bitmode config error with code " +
				std::to_string(ret) + " (" +
				string(ftdi_get_error_string(_ftdi)) + ")");
		return ret;
	}
	if (mode == BITMODE_MPSSE) {
		unsigned char buf1[5];
		if ((ret = ftdi_read_data(_ftdi, buf1, 5)) < 0) {
			printError("fail to read data " +
					string(ftdi_get_error_string(_ftdi)));
			return -1;
		}

		if (setClkFreq(_clkHZ) < 0)
			return -1;

		if (_cable.status_pin != -1) {
			if (_cable.status_pin <= 7) {
				_cable.bit_low_dir |= 1 << _cable.status_pin;
				_cable.bit_low_val &= ~(1 << _cable.status_pin);
			} else {
				_cable.bit_high_dir |= 1 << (_cable.status_pin - 8);
				_cable.bit_high_val &= ~(1 << (_cable.status_pin - 8));
			}
		}

		int to_wr = 3;

		buf_cmd[1] = _cable.bit_low_val;  // 0xe8;
		buf_cmd[2] = _cable.bit_low_dir;  // 0xeb;

		if (_ftdi->type != TYPE_4232H) {
			buf_cmd[4] = _cable.bit_high_val;  // 0x00;
			buf_cmd[5] = _cable.bit_high_dir;  // 0x60;
			to_wr = 6;
		}
		if ((ret = mpsse_store(buf_cmd, to_wr)) < 0) {
			printError("fail to store buffer " +
					string(ftdi_get_error_string(_ftdi)));
			return -1;
		}
		if (mpsse_write() < 0) {
			printError("fail to write buffer " +
					string(ftdi_get_error_string(_ftdi)));
			return -1;
		}
	}

	if (ftdi_read_data_set_chunksize(_ftdi, _buffer_size) < 0) {
		printError("fail to set read chunk size: " +
				string(ftdi_get_error_string(_ftdi)));
		return -1;
	}
	if (ftdi_write_data_set_chunksize(_ftdi, _buffer_size) < 0) {
		printError("fail to set write chunk size: " +
				string(ftdi_get_error_string(_ftdi)));
		return -1;
	}

	_bitmode = mode;
	return 0;
}

int FTDIpp_MPSSE::setClkFreq(uint32_t clkHZ)
{
	int ret;
	uint8_t buffer[4] = { TCK_DIVISOR, 0x00, 0x00};
	uint32_t base_freq;
	float real_freq = 0;
	uint16_t presc;

	_clkHZ = clkHZ;

	/* FT2232C has no divide by 5 instruction
	 * and default freq is 12MHz
	 */
	if (_ftdi->type == TYPE_2232C) {
		base_freq = 12000000;
	} else {
		base_freq = 60000000;
		if ((ret = mpsse_store(DIS_DIV_5)) < 0)
			return ret;
	}

	if (_clkHZ > base_freq / 2) {
		printWarn("Jtag probe limited to %d MHz" + std::to_string(base_freq / 2));
		_clkHZ = base_freq / 2;
	}

	presc = ((base_freq /_clkHZ) - 1) / 2;
	real_freq = base_freq / ((1 + presc) * 2);
	if (real_freq > _clkHZ)
		++presc;
	real_freq = base_freq / ((1+presc)*2);

	/* just to have a better display */
	char __buf[16];
	int __buf_valid_bytes;
	if (clkHZ >= 1e6)
		__buf_valid_bytes = snprintf(__buf, 9, "%2.2fMHz", clkHZ / 1e6);
	else if (clkHZ >= 1e3)
		__buf_valid_bytes = snprintf(__buf, 10, "%3.2fKHz", clkHZ / 1e3);
	else
		__buf_valid_bytes = snprintf(__buf, 10, "%3u.00Hz", clkHZ);
	string clkHZ_str(__buf, __buf_valid_bytes);
	clkHZ_str.resize(10, ' ');
	if (real_freq >= 1e6)
		__buf_valid_bytes = snprintf(__buf, 9, "%2.2fMHz", real_freq / 1e6);
	else if (real_freq >= 1e3)
		__buf_valid_bytes = snprintf(__buf, 10, "%3.2fKHz", real_freq / 1e3);
	else
		__buf_valid_bytes = snprintf(__buf, 10, "%3.2fHz", real_freq);
	string real_freq_str(__buf, __buf_valid_bytes);
	real_freq_str.resize(10, ' ');


	printInfo("Jtag frequency : requested " + clkHZ_str +
			" -> real " + real_freq_str);
	display("presc : %d input freq : %u requested freq : %u real freq : %f\n",
			presc, base_freq, _clkHZ, real_freq);

	/*printf("base freq %d div by 5 %c presc %d\n", base_freq, (use_divide_by_5)?'1':'0',
			presc); */


	buffer[1] = presc & 0xff;
	buffer[2] = (presc >> 8) & 0xff;

	if ((ret = mpsse_store(buffer, 3)) < 0)
		return ret;
	if ((ret = mpsse_write()) < 0) {
		fprintf(stderr, "Error: write for frequency return %d\n", ret);
		return ret;
	}
	if ((ret = ftdi_read_data(_ftdi, buffer, 4)) < 0) {
		printError("selfClkFreq: fail to read: " +
				string(ftdi_get_error_string(_ftdi)));
		return ret;
	}

#if (FTDI_VERSION < 105)
	ftdi_usb_purge_buffers(_ftdi);
#else
	if ((ret = ftdi_tcioflush(_ftdi)) < 0) {
		printError("selfClkFreq: fail to flush buffers: " +
				string(ftdi_get_error_string(_ftdi)));
		return ret;
	}

#endif

	_clkHZ = real_freq;

	return real_freq;
}

int FTDIpp_MPSSE::mpsse_store(unsigned char c)
{
	return mpsse_store(&c, 1);
}

int FTDIpp_MPSSE::mpsse_store(unsigned char *buff, int len)
{
	unsigned char *ptr = buff;
	int store_size, ret;
	/* check if _buffer as space to store all */
	if (_num + len > _buffer_size) {
		/* flush buffer if already full */
		if (_num == _buffer_size) {
			if ((ret = mpsse_write()) < 0) {
				printError("mpsse_store: fails to first flush " +
						std::to_string(ret) + " " +
						string(ftdi_get_error_string(_ftdi)));
				return ret;
			}
		}
		/* loop until loop < _buffer_size */
		while (_num + len > _buffer_size) {
			/* we now have len enough to fill
			 * buffer -> just complete buffer
			 */
			store_size = _buffer_size - _num;
			memcpy(_buffer + _num, ptr, store_size);
			_num += store_size;
			if ((ret = mpsse_write()) < 0) {
				printError("mpsse_store: fails to first flush " +
						std::to_string(ret) + " " +
						string(ftdi_get_error_string(_ftdi)));
				return ret;
			}
			ptr += store_size;
			len -= store_size;
		}
	}
#ifdef DEBUG
	display("%s %d %d\n", __func__, _num, len);
#endif
	if (len > 0) {
		memcpy(_buffer + _num, ptr, len);
		_num += len;
	}
	return 0;
}

int FTDIpp_MPSSE::mpsse_write()
{
	int ret;
	if (_num == 0)
		return 0;

#ifdef DEBUG
	display("%s %d\n", __func__, _num);
#endif

	if ((ret = ftdi_write_data(_ftdi, _buffer, _num)) != _num) {
		printError("mpsse_write: fail to write with error " +
				std::to_string(ret) + " (" +
				string(ftdi_get_error_string(_ftdi)) + ")");
		return ret;
	}

	_num = 0;
	return ret;
}

int FTDIpp_MPSSE::mpsse_read(unsigned char *rx_buff, int len)
{
	int n, ret;
	int num_read = 0;
	unsigned char *p = rx_buff;

	/* force buffer transmission before read */
	if ((ret = mpsse_store(SEND_IMMEDIATE)) < 0) {
		printError("mpsse_read: fail to store with error: " +
				std::to_string(ret) + " (" +
				string(ftdi_get_error_string(_ftdi)) + ")");
		return ret;
	}

	if ((ret = mpsse_write()) < 0) {
		printError("mpsse_read: fail to flush buffer with error: " +
				std::to_string(ret) + " (" +
				string(ftdi_get_error_string(_ftdi)) + ")");
		return ret;
	}

	do {
		n = ftdi_read_data(_ftdi, p, len);
		if (n < 0) {
			fprintf(stderr, "Error: ftdi_read_data in %s", __func__);
			return -1;
		}
#ifdef DEBUG
		if (_verbose) {
			display("%s %d\n", __func__, n);
			for (int i = 0; i < n; i++)
				display("\t%s %x\n", __func__, p[i]);
		}
#endif

		len -= n;
		p += n;
		num_read += n;
	} while (len > 0);
	return num_read;
}

/**
 * Read GPIO (xCBUSy + xDBUSy) bank
 * @return pins state
 */
uint16_t FTDIpp_MPSSE::gpio_get()
{
	uint8_t tx[2] = {GET_BITS_LOW, GET_BITS_HIGH};
	uint8_t rx[2];
	if (mpsse_store(tx, 2) < 0)
		return 0;
	if (mpsse_read(rx, 2) < 0)
		return 0;

	return (rx[1] << 8) | rx[0];
}

/**
 * Read low (xCBUSy) or high (xDBUSy) pins.
 * @param[in] low_pins: if true read low, read high otherwise
 * @return pins state
 */
uint8_t FTDIpp_MPSSE::gpio_get(bool low_pins)
{
	uint8_t rx;

	/* select between high and low pins */
	if (mpsse_store((low_pins) ? GET_BITS_LOW : GET_BITS_HIGH) < 0)
		return 0;
	if (mpsse_read(&rx, 1) < 0)
		return 0;
	return rx;
}

/**
 * Set one or more pins of the full bank (CBUS + DBUS).
 * @param[in] pins bitmask
 * @return false when error, true otherwise
 */
bool FTDIpp_MPSSE::gpio_set(uint16_t gpios)
{
	if (gpios & 0x00ff) {
		_cable.bit_low_val |= (0xff & gpios);
		if (!__gpio_write(true))
			return false;
	}
	if (gpios & 0xff00) {
		_cable.bit_high_val |= (0xff & (gpios >> 8));
		if (!__gpio_write(false))
			return false;
	}
	return (mpsse_write() >= 0);
}

/**
 * Set one or more pins of the given half bank (CBUS or BDUS).
 * @param[in] pin bitmask
 * @param[in] low/high half bank
 * @return false when error, true otherwise
 */
bool FTDIpp_MPSSE::gpio_set(uint8_t gpios, bool low_pins)
{
	if (low_pins)
		_cable.bit_low_val |= gpios;
	else
		_cable.bit_high_val |= gpios;
	if (!__gpio_write(low_pins))
		return false;
	return (mpsse_write() >= 0);
}

/**
 * Clear one or more pins of the full bank (CBUS + DBUS).
 * @param[in] pins bitmask
 * @return false when error, true otherwise
 */
bool FTDIpp_MPSSE::gpio_clear(uint16_t gpios)
{
	if (gpios & 0x00ff) {
		_cable.bit_low_val &= ~(0xff & gpios);
		if (!__gpio_write(true))
			return false;
	}
	if (gpios & 0xff00) {
		_cable.bit_high_val &= ~(0xff & (gpios >> 8));
		if (!__gpio_write(false))
			return false;
	}
	return (mpsse_write() >= 0);
}

/**
 * Clear one or more pins of the given half bank (CBUS or BDUS).
 * @param[in] pin bitmask
 * @param[in] low/high half bank
 * @return false when error, true otherwise
 */
bool FTDIpp_MPSSE::gpio_clear(uint8_t gpios, bool low_pins)
{
	if (low_pins)
		_cable.bit_low_val &= ~(gpios);
	else
		_cable.bit_high_val &= ~(gpios);

	if (!__gpio_write(low_pins))
		return false;
	return (mpsse_write() >= 0);
}

/**
 * Full bank write
 * @param[in] GPIOs bitmask
 * @return false when error, true otherwise
 */
bool FTDIpp_MPSSE::gpio_write(uint16_t gpio)
{
	_cable.bit_low_val = (0xff & gpio);
	_cable.bit_high_val = (0xff & (gpio >> 8));
	if (!__gpio_write(true))
		return false;
	if (!__gpio_write(false))
		return false;
	return (mpsse_write() >= 0);
}

/**
 * Half bank write
 * @param[in] gpio: pins values
 * @param[in] low_pins: high/low half bank
 * @return false when error, true otherwise
 */
bool FTDIpp_MPSSE::gpio_write(uint8_t gpio, bool low_pins)
{
	if (low_pins)
		_cable.bit_low_val = gpio;
	else
		_cable.bit_high_val = gpio;

	if (__gpio_write(low_pins))
		return false;
	return (mpsse_write() >= 0);
}

/**
 * update half bank pins direction (no write is done at this time)
 * @param[in] dir: pins direction (1 out, 0 in) for low or high bank
 * @param[in] low_pins: high/low half bank
 */
void FTDIpp_MPSSE::gpio_set_dir(uint8_t dir, bool low_pins)
{
	if (low_pins)
		_cable.bit_low_dir = dir;
	else
		_cable.bit_high_dir = dir;
}

/**
 * update full bank pins direction (no write is done at this time)
 * @param[in] dir: pins direction (1 out, 0 in) for low or high bank
 */
void FTDIpp_MPSSE::gpio_set_dir(uint16_t dir)
{
	_cable.bit_low_dir = dir & 0xff;
	_cable.bit_high_dir = (dir >> 8) & 0xff;
}

/**
 * configure low or high pins as input
 * @param[in] gpio: pins bitmask
 * @param[int] low_pins: select between CBUS or DBUS
 */
void FTDIpp_MPSSE::gpio_set_input(uint8_t gpio, bool low_pins)
{
	if (low_pins)
		_cable.bit_low_dir &= ~gpio;
	else
		_cable.bit_high_dir &= ~gpio;
}

/**
 * configure pins as input
 * @param[in] gpio: pins bitmask
 */
void FTDIpp_MPSSE::gpio_set_input(uint16_t gpio)
{
	if (gpio & 0x00ff)
		_cable.bit_low_dir &= ~(gpio & 0x00ff);
	if (gpio & 0xff00)
		_cable.bit_high_dir &= ~((gpio >> 8) & 0x00ff);
}

/**
 * configure low or high pins as output
 * @param[in] gpio: pins bitmask
 * @param[int] low_pins: select between CBUS or DBUS
 */
void FTDIpp_MPSSE::gpio_set_output(uint8_t gpio, bool low_pins)
{
	if (low_pins)
		_cable.bit_low_dir |= gpio;
	else
		_cable.bit_high_dir |= gpio;
}

/**
 * configure pins as output
 * @param[in] gpio: pins bitmask
 */
void FTDIpp_MPSSE::gpio_set_output(uint16_t gpio)
{
	if (gpio & 0x00ff)
		_cable.bit_low_dir |= (gpio & 0x00ff);
	if (gpio & 0xff00)
		_cable.bit_high_dir |= ((gpio >> 8) & 0x00ff);
}

/**
 * private method to write ftdi half bank GPIOs (pins state are in _cable)
 * @param[in] low or high half bank
 */
bool FTDIpp_MPSSE::__gpio_write(bool low_pins)
{
	uint8_t tx[3];
	tx[0] = ((low_pins) ? SET_BITS_LOW       : SET_BITS_HIGH);
	tx[1] = ((low_pins) ? _cable.bit_low_val : _cable.bit_high_val);
	tx[2] = ((low_pins) ? _cable.bit_low_dir : _cable.bit_high_dir);
	return (mpsse_store(tx, 3) >= 0);
}

#ifdef USE_UDEV
unsigned int FTDIpp_MPSSE::udevstufftoint(const char *udevstring, int base)
{
	char *endp;
	int ret;
	errno = 0;

	if (udevstring == NULL)
		return (-1);

	ret = (unsigned int)strtol(udevstring, &endp, base);
	if (errno) {
		fprintf(stderr,
			"udevstufftoint: Unable to parse number Error : %s (%d)\n",
			strerror(errno), errno);
		return (-2);
	}
	if (endp == optarg) {
		fprintf(stderr, "udevstufftoint: No digits were found\n");
		return (-3);
	}
	return (ret);
}

bool FTDIpp_MPSSE::search_with_dev(const string &device)
{
	struct udev *udev;
	struct udev_device *dev, *usbdeviceparent;
	char devtype;

	struct stat statinfo;
	if (stat(device.c_str(), &statinfo) < 0) {
		printf("unable to stat file\n");
		return false;
	}

	/* get device type */
	switch (statinfo.st_mode & S_IFMT) {
	case S_IFBLK:
		devtype = 'b';
		break;
	case S_IFCHR:
		devtype = 'c';
		break;
	default:
		printf("not char or block device\n");
		return false;
	}

	/* Create the udev object */
	udev = udev_new();
	if (!udev) {
		printf("Can't create udev\n");
		return false;
	}

	dev = udev_device_new_from_devnum(udev, devtype, statinfo.st_rdev);

	if (dev == NULL) {
		printf("no dev\n");
		udev_device_unref(dev);
		udev_unref(udev);
		return false;
	}

	/* Get closest usb device parent (we need VIP/PID)  */
	usbdeviceparent =
		udev_device_get_parent_with_subsystem_devtype(dev, "usb",
							  "usb_device");
	if (!usbdeviceparent) {
		printf
			("Unable to find parent usb device! Is this actually an USB device ?\n");
		udev_device_unref(dev);
		udev_unref(udev);
		return false;
	}

	_bus = static_cast<uint8_t>(udevstufftoint(udev_device_get_sysattr_value(
				usbdeviceparent, "busnum"), 10));
	_addr = static_cast<uint8_t>(udevstufftoint(udev_device_get_sysattr_value(
				usbdeviceparent, "devnum"), 10));
	snprintf(_product, 64, "%s", udev_device_get_sysattr_value(usbdeviceparent, "product"));
	_vid = udevstufftoint(
		udev_device_get_sysattr_value(usbdeviceparent, "idVendor"), 16);
	_pid = udevstufftoint(udev_device_get_sysattr_value(
		usbdeviceparent, "idProduct"), 16);

	display("vid %x pid %x bus %d addr %d product name : %s\n", _vid, _pid, _bus, _addr, _product);

	return true;
}
#else
unsigned int FTDIpp_MPSSE::udevstufftoint(const char *udevstring, int base)
{
	(void)udevstring;
	(void)base;
	return 0;
}
bool FTDIpp_MPSSE::search_with_dev(const string &device)
{
	(void)device;
	return false;
}
#endif
