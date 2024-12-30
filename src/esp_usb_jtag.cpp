// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2024 EMARD
 */

/*
Holy Crap, it's protocol documentation, and it's even vendor-provided!

A device that speaks this protocol has two endpoints intended for JTAG debugging: one
OUT for the host to send encoded commands to, one IN from which the host can read any read
TDO bits. The device will also respond to vendor-defined interface requests on ep0.

The main communication method is over the IN/OUT endpoints. The commands that are expected
on the OUT endpoint are one nibble wide and are processed high-nibble-first, low-nibble-second,
and in the order the bytes come in. Commands are defined as follows:

    bit     3   2    1    0
CMD_CLK   [ 0   cap  tdi  tms  ]
CMD_RST   [ 1   0    0    srst ]
CMD_FLUSH [ 1   0    1    0    ]
CMD_RSV   [ 1   0    1    1    ]
CMD_REP   [ 1   1    R1   R0   ]

CMD_CLK sets the TDI and TMS lines to the value of `tdi` and `tms` and lowers, then raises, TCK. If
`cap` is 1, the value of TDO is captured and can be retrieved over the IN endpoint. The bytes read from
the IN endpoint specifically are these bits, with the lowest it in every byte captured first and the
bytes returned in the order the data in them was captured. The durations of TCK being high / low can
be set using the VEND_JTAG_SETDIV vendor-specific interface request.

CMD_RST controls the SRST line; as soon as the command is processed, the SRST line will be set
to the value of `srst`.

CMD_FLUSH flushes the IN endpoint; zeroes will be added to the amount of bits in the endpoint until
the payload is a multiple of bytes, and the data is offered to the host. If the IN endpoint has
no data, this effectively becomes a no-op; the endpoint won't send any 0-byte payloads.

CMD_RSV is reserved for future use.

CMD_REP repeats the last command that is not CMD_REP. The amount of times a CMD_REP command will
re-execute this command is (r1*2+r0)<<(2*n), where n is the amount of previous repeat commands executed
since the command to be repeated.

An example for CMD_REP: Say the host queues:
1. CMD_CLK - This will execute one CMD_CLK.
2. CMD_REP with r1=0 and r0=1 - This will execute 1. another (0*2+1)<<(2*0)=1 time.
3. CMD_REP with r1=1 and r0=0 - This will execute 1. another (1*2+0)<<(2*1)=4 times.
4. CMD_REP with r1=0 and r0=1 - This will execute 1. another (0*2+1)<<(2*2)=8 time.
5. CMD_FLUSH - This will flush the IN pipeline.
6. CMD_CLK - This will execute one CMD_CLK
7. CMD_REP with r1=1 and r0=0 - This will execute 6. another (1*2+0)<<(2*0)=2 times.
8. CMD_FLUSH - This will flush the IN pipeline.

Note that the net effect of the repetitions is that command 1 is executed (1+1+4+8=) 14 times and
command 6 is executed (1+2=) 3 times.

Note that the device only has a fairly limited amount of endpoint RAM. It's probably best to keep
an eye on the amount of bytes that are supposed to be in the IN endpoint and grab those before stuffing
more commands into the OUT endpoint: the OUT endpoint will not accept any more commands (writes will
time out) when the IN endpoint buffers are all filled up.

The device also supports some vendor-specific interface requests. These requests are sent as control
transfers on endpoint 0 to the JTAG endpoint. Note that these commands bypass the data in the OUT
endpoint; if timing is important, it's important that this endpoint is empty. This can be done by
e.g sending one CMD_CLK capturing TDI, then one CMD_FLUSH, then waiting until the bit appears on the
IN endpoint.

bmRequestType bRequest         wValue   wIndex    wLength Data
01000000b     VEND_JTAG_SETDIV [divide] interface 0       None
01000000b     VEND_JTAG_SETIO  [iobits] interface 0       None
11000000b     VEND_JTAG_GETTDO  0       interface 1       [iostate]
10000000b     GET_DESCRIPTOR(6) 0x2000  0         256     [jtag cap desc]

VEND_JTAG_SETDIV indirectly controls the speed of the TCK clock. The value written here is the length
of a TCK cycle, in ticks of the adapters base clock. Both the base clock value as well as the
minimum and maximum divider can be read from the jtag capabilities descriptor, as explained
below. Note that this should not be set to a value outside of the range described there,
otherwise results are undefined.

VEND_JTAG_SETIO can be controlled to directly set the IO pins. The format of [iobits] normally is
{11'b0, srst, trst, tck, tms, tdi}
Note that the first 11 0 bits are reserved for future use, current hardware ignores them.

VEND_JTAG_GETTDO returns one byte, of which bit 0 indicates the current state of the TDO input.
Note that other bits are reserved for future use and should be ignored.

To describe the capabilities of the JTAG adapter, a specific descriptor (0x20) can be retrieved.
The format of the descriptor documented below. The descriptor works in the same fashion as USB
descriptors: a header indicating the version and total length followed by descriptors with a
specific type and size. Forward compatibility is guaranteed as software can skip over an unknown
descriptor.

this description is a copy from openocd/src/jtag/drivers/esp_usb_jtag.c
*/


#include <libusb.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

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

#define ESPUSBJTAG_INTF		  2
#define ESPUSBJTAG_WRITE_EP    0x02
#define ESPUSBJTAG_READ_EP     0x83

#define ESPUSBJTAG_TIMEOUT_MS   500

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

/* begin copy from openocd */

#define JTAG_PROTO_CAPS_VER 1	/* Version field. At the moment, only version 1 is defined. */
struct jtag_proto_caps_hdr {
	uint8_t proto_ver;	/* Protocol version. Expects JTAG_PROTO_CAPS_VER for now. */
	uint8_t length;	/* of this plus any following descriptors */
} __attribute__((packed));

/* start of the descriptor headers */
#define JTAG_BUILTIN_DESCR_START_OFF            0	/* Devices with builtin usb jtag */
/*
* ESP USB Bridge https://github.com/espressif/esp-usb-bridge uses string descriptor.
* Skip 1 byte length and 1 byte descriptor type
*/
#define JTAG_EUB_DESCR_START_OFF                2	/* ESP USB Bridge */

/*
Note: At the moment, there is only a speed_caps version indicating the base speed of the JTAG
hardware is derived from the APB bus speed of the SoC. If later on, there are standalone
converters using the protocol, we should define e.g. JTAG_PROTO_CAPS_SPEED_FIXED_TYPE to distinguish
between the two.

Note: If the JTAG device has larger buffers than endpoint-size-plus-a-bit, we should have some kind
of caps header to assume this. If no such caps exist, assume a minimum (in) buffer of endpoint size + 4.
*/

struct jtag_gen_hdr {
	uint8_t type;
	uint8_t length;
} __attribute__((packed));

struct jtag_proto_caps_speed_apb {
	uint8_t type;					/* Type, always JTAG_PROTO_CAPS_SPEED_APB_TYPE */
	uint8_t length;					/* Length of this */
	uint8_t apb_speed_10khz[2];		/* ABP bus speed, in 10KHz increments. Base speed is half this. */
	uint8_t div_min[2];				/* minimum divisor (to base speed), inclusive */
	uint8_t div_max[2];				/* maximum divisor (to base speed), inclusive */
} __attribute__((packed));

#define JTAG_PROTO_CAPS_DATA_LEN                255
#define JTAG_PROTO_CAPS_SPEED_APB_TYPE          1

#define VEND_DESCR_BUILTIN_JTAG_CAPS            0x2000

#define VEND_JTAG_SETDIV        0
#define VEND_JTAG_SETIO         1
#define VEND_JTAG_GETTDO        2
#define VEND_JTAG_SET_CHIPID    3

#define BIT(x) (1<<x)

#define VEND_JTAG_SETIO_TDI     BIT(0)
#define VEND_JTAG_SETIO_TMS     BIT(1)
#define VEND_JTAG_SETIO_TCK     BIT(2)
#define VEND_JTAG_SETIO_TRST    BIT(3)
#define VEND_JTAG_SETIO_SRST    BIT(4)

#define CMD_CLK(cap, tdi, tms) ((cap ? BIT(2) : 0) | (tms ? BIT(1) : 0) | (tdi ? BIT(0) : 0))
#define CMD_RST(srst)   (0x8 | (srst ? BIT(0) : 0))
#define CMD_FLUSH       0xA
#define CMD_RSVD        0xB
#define CMD_REP(r)      (0xC + ((r) & 3))

/* The internal repeats register is 10 bits, which means we can have 5 repeat commands in a
 *row at max. This translates to ('b1111111111+1=)1024 reps max. */
#define CMD_REP_MAX_REPS 1024

/* Currently we only support one USB device. */
#define USB_CONFIGURATION 0

/* Buffer size; is equal to the endpoint size. In bytes
 * TODO for future adapters: read from device configuration? */
#define OUT_EP_SZ 64
/* Out data can be buffered for longer without issues (as long as the in buffer does not overflow),
 * so we'll use an out buffer that is much larger than the out ep size. */
#define OUT_BUF_SZ (OUT_EP_SZ * 32)
/* The in buffer cannot be larger than the device can offer, though. */
#define IN_BUF_SZ 64

/* Because a series of out commands can lead to a multitude of IN_BUF_SZ-sized in packets
 *to be read, we have multiple buffers to store those before the bitq interface reads them out. */
#define IN_BUF_CT 8

/*
 * comment from libusb:
 * As per the USB 3.0 specs, the current maximum limit for the depth is 7.
 */
#define MAX_USB_PORTS   7

#define ESP_USB_INTERFACE       1

/* Private data */
struct esp_usb_jtag_s {
	struct libusb_device_handle *usb_device;
	uint32_t base_speed_khz;
	uint16_t div_min;
	uint16_t div_max;
	uint8_t out_buf[OUT_BUF_SZ];
	unsigned int out_buf_pos_nibbles;			/* write position in out_buf */

	uint8_t in_buf[IN_BUF_CT][IN_BUF_SZ];
	unsigned int in_buf_size_bits[IN_BUF_CT];	/* size in bits of the data stored in an in_buf */
	unsigned int cur_in_buf_rd, cur_in_buf_wr;	/* read/write index */
	unsigned int in_buf_pos_bits;	/* which bit in the in buf needs to be returned to bitq next */

	unsigned int read_ep;
	unsigned int write_ep;

	unsigned int prev_cmd;		/* previous command, stored here for RLEing. */
	int prev_cmd_repct;			/* Amount of repetitions of that command we have seen until now */

	/* This is the total number of in bits we need to read, including in unsent commands */
	unsigned int pending_in_bits;
	// FILE *logfile;			/* If non-NULL, we log communication traces here. */

	unsigned int hw_in_fifo_len;
	char *serial[256 + 1];	/* device serial */

	// struct bitq_interface bitq_interface;
};

/* For now, we only use one static private struct. Technically, we can re-work this, but I don't think
 * OpenOCD supports multiple JTAG adapters anyway. */
static struct esp_usb_jtag_s esp_usb_jtag_priv;
static struct esp_usb_jtag_s *priv = &esp_usb_jtag_priv;
static const char *esp_usb_jtag_serial;

static uint16_t esp_usb_vid = ESPUSBJTAG_VID;
static uint16_t esp_usb_pid = ESPUSBJTAG_PID;
static uint16_t esp_usb_jtag_caps = 0x2000; /* capabilites descriptor ID, different esp32 chip may need different value */
static uint16_t esp_usb_target_chip_id = 0; /* not applicable for FPGA, they have chip id 32-bit wide */

/* end copy from openocd */


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
		cerr << "fails to open esp_usb_jtag device vid:pid 0x" << std::hex << vid << ":0x" << std::hex << endl;
		libusb_exit(usb_ctx);
		throw std::exception();
	}

	ret = libusb_claim_interface(dev_handle, ESPUSBJTAG_INTF);
	if (ret) {
		cerr << "libusb error while claiming esp_usb_jtag interface of device vid:pid 0x" << std::hex << vid << ":0x" << std::hex << pid << endl;
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


	/* TODO: This is not proper way to get caps data. Two requests can be done.
	 * 1- With the minimum size required to get to know the total length of that struct,
	 * 2- Then exactly the length of that struct. */
	uint8_t jtag_caps_desc[JTAG_PROTO_CAPS_DATA_LEN];
	int jtag_caps_read_len = libusb_control_transfer(dev_handle,
	/*type*/	LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_DEVICE,
	/*brequest*/	LIBUSB_REQUEST_GET_DESCRIPTOR,
	/*wvalue*/	esp_usb_jtag_caps,
	/*interface*/	0,
	/*data*/	(unsigned char *)jtag_caps_desc,
	/*length*/	JTAG_PROTO_CAPS_DATA_LEN,
	/*timeout ms*/	ESPUSBJTAG_TIMEOUT_MS);
	if (jtag_caps_read_len <= 0) {
		cerr << "esp_usb_jtag: could not retrieve jtag_caps descriptor! len=" << jtag_caps_read_len << endl;
		// goto out;
	}
	for(int i = 0; i < jtag_caps_read_len; i++)
	  cerr << " 0x" << std::hex << (int)(jtag_caps_desc[i]);
	cerr << endl;

	int p = esp_usb_jtag_caps ==
		VEND_DESCR_BUILTIN_JTAG_CAPS ? JTAG_BUILTIN_DESCR_START_OFF : JTAG_EUB_DESCR_START_OFF;

	if (p + sizeof(struct jtag_proto_caps_hdr) > (unsigned int)jtag_caps_read_len) {
		cerr << "esp_usb_jtag: not enough data to get header" << endl;
		// goto out;
	}

	struct jtag_proto_caps_hdr *hdr = (struct jtag_proto_caps_hdr *)&jtag_caps_desc[p];
	if (hdr->proto_ver != JTAG_PROTO_CAPS_VER) {
		cerr << "esp_usb_jtag: unknown jtag_caps descriptor version 0x" << std::hex
		     << hdr->proto_ver << endl;
		// goto out;
	}
	if (hdr->length > jtag_caps_read_len) {
		cerr << "esp_usb_jtag: header length (" << hdr->length
		     << ") bigger then max read bytes (" << jtag_caps_read_len
		     << ")" << endl;
		// goto out;
	}

	/* TODO: grab from (future) descriptor if we ever have a device with larger IN buffers */
	// priv->hw_in_fifo_len = 4;

	cerr << "ok" << endl;
	_version = 1; // currently only protocol version 1 exists

	/* inform bridge board about the connected target chip for the specific operations
	 * it is also safe to send this info to chips that have builtin usb jtag */
	libusb_control_transfer(dev_handle,
	/*type*/	LIBUSB_REQUEST_TYPE_VENDOR,
	/*brequest*/	VEND_JTAG_SET_CHIPID,
	/*wvalue*/	esp_usb_target_chip_id,
	/*interface*/	0,
	/*data*/	NULL,
	/*length*/	0,
	/*timeout ms*/	ESPUSBJTAG_TIMEOUT_MS);

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
	#if 0
	ret = libusb_bulk_transfer(dev_handle, ESPUSBJTAG_WRITE_EP,
					buf, 4, &actual_length, ESPUSBJTAG_TIMEOUT_MS);
	#endif

        #if 0
        // TODO make something like this
	ret = libusb_control_transfer(dev_handle,
	/*type*/	LIBUSB_REQUEST_TYPE_VENDOR,
	/*brequest*/	VEND_JTAG_SET_CHIPID,
	/*wvalue*/	esp_usb_target_chip_id,
	/*windex*/	0,
	/*data*/	NULL,
	/*length*/	0,
	/*timeout ms*/	ESPUSBJTAG_TIMEOUT_MS);
        #endif

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
	uint8_t buf[OUT_BUF_SZ];
	int transferred_length; // never used

	if (len == 0)
		return 0;

	uint8_t prev_high_nibble = CMD_FLUSH << 4; // for odd length 1st command is flush = nop
	uint32_t buffer_idx = 0; // reset
	uint8_t is_high_nibble = 1 & ~len;
	// for even len: start with is_high_nibble = 1
	// for odd len:  start with is_high_nibble = 0
	//               1st (high nibble) is flush = nop
	//               2nd (low nibble) is data
	// last byte in buf will have data in both nibbles, no flush
	// exec order: high-nibble-first, low-nibble-second
	for (uint32_t i = 0; i < len; i++)
	{
	    uint8_t tms_bit = (tms[i >> 3] >> (i & 7)) & 1; // get i'th bit from tms
	    uint8_t cmd = CMD_CLK(0, 0, tms_bit);
	    if(is_high_nibble)
            {   // 1st (high nibble) = data
                buf[buffer_idx] = prev_high_nibble = cmd << 4;
                is_high_nibble = 0;
            }
            else // low nibble
	    {   // 2nd (low nibble) = data, keep prev high nibble
                buf[buffer_idx] = prev_high_nibble | cmd;
                buffer_idx++; // byte complete, advance to the next byte in buf
                is_high_nibble = 1;
	    }
	    if (buffer_idx >= sizeof(buf) /*buf full*/ || i == len - 1 /*last*/)
	    {
		int ret = libusb_bulk_transfer(dev_handle,
			/*endpoint*/    ESPUSBJTAG_WRITE_EP,
			/*data*/        buf,
			/*length*/      buffer_idx,
			/*transf.len*/  &transferred_length,
			/*timeout ms*/  ESPUSBJTAG_TIMEOUT_MS);
		if (ret != 0)
		{
                    cerr << "writeTMS: usb bulk write failed " << ret << endl;
                    return -EXIT_FAILURE;
		}
                buffer_idx = 0; // reset
            }
	}
	return len;
}

int esp_usb_jtag::toggleClk(uint8_t tms, uint8_t tdi, uint32_t len)
{
    uint8_t buf[OUT_BUF_SZ];
    int transferred_length; // never used

    if (len == 0)
	return 0;

    uint8_t prev_high_nibble = CMD_FLUSH << 4; // for odd length 1st command is flush = nop
    uint32_t buffer_idx = 0; // reset
    uint8_t is_high_nibble = 1 & ~len;
    // for even len: start with is_high_nibble = 1
    // for odd len:  start with is_high_nibble = 0
    //               1st (high nibble) is flush = nop
    //               2nd (low nibble) is data
    // last byte in buf will have data in both nibbles, no flush
    // exec order: high-nibble-first, low-nibble-second
    uint8_t cmd = CMD_CLK(0, tdi, tms);
    for (uint32_t i = 0; i < len; i++)
    {
        // TODO: repeat clocking with CMD_REP
        if(is_high_nibble)
        {   // 1st (high nibble) = cmd
            buf[buffer_idx] = prev_high_nibble;
            is_high_nibble = 0;
        }
        else // low nibble
	{   // 2nd (low nibble) = cmd, keep prev high nibble
            buf[buffer_idx] = prev_high_nibble | cmd;
            buffer_idx++; // byte complete, advance to the next byte in buf
            is_high_nibble = 1;
	}

	if (buffer_idx >= sizeof(buf) /*buf full*/ || i == len - 1 /*last*/)
	{
            int ret = libusb_bulk_transfer(dev_handle,
	    /*endpoint*/                   ESPUSBJTAG_WRITE_EP,
            /*data*/                       buf,
            /*length*/                     buffer_idx,
            /*transferred length*/         &transferred_length,
            /*timeout ms*/                 ESPUSBJTAG_TIMEOUT_MS);
            if (ret != 0)
            {
                cerr << "toggleClk: usb bulk write failed " << ret << endl;
                return -EXIT_FAILURE;
            }
            buffer_idx = 0; // reset
        }
    }
    return EXIT_SUCCESS;
}

int esp_usb_jtag::flush()
{
    // TODO OUT CMD_FLUSH
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
				&actual_length, ESPUSBJTAG_TIMEOUT_MS);
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
					rx_buf, transfer_length, &actual_length, ESPUSBJTAG_TIMEOUT_MS);
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
									 ESPUSBJTAG_TIMEOUT_MS) < 0)
			{
				cerr << "writeTDI: last bit error: usb bulk write failed 1" << endl;
				return -EXIT_FAILURE;
			}
			do
			{
				if (libusb_bulk_transfer(dev_handle, ESPUSBJTAG_READ_EP,
											&sig, 1, &actual_length,
											ESPUSBJTAG_TIMEOUT_MS) < 0)
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
									 ESPUSBJTAG_TIMEOUT_MS) < 0)
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
