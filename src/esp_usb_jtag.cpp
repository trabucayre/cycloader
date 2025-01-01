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
CMD_CLK   [ 0   cap  tms  tdi  ]
CMD_RST   [ 1   0    0    srst ]
CMD_FLUSH [ 1   0    1    0    ]
CMD_RSV   [ 1   0    1    1    ]
CMD_REP   [ 1   1    R1   R0   ]

CMD_CLK sets the TDI and TMS lines to the value of `tdi` and `tms` and lowers, then raises, TCK. If
`cap` is 1, the value of TDO is captured and can be retrieved over the IN endpoint. The bytes read from
the IN endpoint specifically are these bits, with the lowest bit in every byte captured first and the
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

#define ESPUSBJTAG_TIMEOUT_MS  1000

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

#define CMD_CLK(cap, tdi, tms) (((1&cap)<<2) | ((1&tms)<<1) | (1&tdi))
#define CMD_RST(srst)   (0x8 | (1&srst))
#define CMD_FLUSH       0xA
#define CMD_RSVD        0xB
#define CMD_REP(r)      (0xC + ((r) & 3))

/* The internal repeats register is 10 bits, which means we can have 5 repeat commands in a
 *row at max. This translates to ('b1111111111+1=)1024 reps max. */
#define CMD_REP_MAX_REPS 1024

/* Currently we only support one USB device. */
// #define USB_CONFIGURATION 0

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
	
	_base_speed_khz = UINT32_MAX;
	_div_min = 1;
	_div_max = 1;

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

	p += sizeof(struct jtag_proto_caps_hdr);
	while (p + sizeof(struct jtag_gen_hdr) < hdr->length) {
		struct jtag_gen_hdr *dhdr = (struct jtag_gen_hdr *)&jtag_caps_desc[p];
		if (dhdr->type == JTAG_PROTO_CAPS_SPEED_APB_TYPE) {
			if (p + sizeof(struct jtag_proto_caps_speed_apb) < hdr->length) {
				cerr << "esp_usb_jtag: not enough data to get caps speed" << endl;
				return false;
			}
			struct jtag_proto_caps_speed_apb *spcap = (struct jtag_proto_caps_speed_apb *)dhdr;
			/* base speed always is half APB speed */
                        _base_speed_khz = (spcap->apb_speed_10khz[0] + 256 * spcap->apb_speed_10khz[1]) * 10 / 2;
			_div_min = spcap->div_min[0] + 256 * spcap->div_min[1];
			_div_max = spcap->div_max[0] + 256 * spcap->div_max[1];
			/* TODO: mark in priv that this is apb-derived and as such may change if apb
			 * ever changes? */
		} else {
			cerr << "esp_usb_jtag: unknown caps type 0x" << dhdr->type << endl;;
		}
		p += dhdr->length;
	}
	if (priv->base_speed_khz == UINT32_MAX) {
		cerr << "esp_usb_jtag: No speed caps found... using sane-ish defaults." << endl;
		_base_speed_khz = 1000;
	}
	cerr << "esp_usb_jtag: Device found. Base speed " << std::dec << _base_speed_khz << " KHz, div range " << (int)_div_min << " to " << (int)_div_max << endl;

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
    int ret = 0, req_freq = clkHZ;

    uint32_t base_speed_Hz = _base_speed_khz * 1000; // TODO read base speed from caps

    if (clkHZ > base_speed_Hz) {
	printWarn("esp_usb_jtag probe limited to %d kHz", _base_speed_khz);
	clkHZ = base_speed_Hz;
    }

    uint16_t divisor = base_speed_Hz / clkHZ;

    _clkHZ = clkHZ;

    printInfo("Jtag frequency : requested " + std::to_string(req_freq) +
              "Hz -> real " + std::to_string(clkHZ) + "Hz divisor=" + std::to_string(divisor));

    ret = libusb_control_transfer(dev_handle,
    /*type*/            LIBUSB_REQUEST_TYPE_VENDOR,
    /*brequest*/        VEND_JTAG_SETDIV,
    /*wvalue*/          divisor,
    /*windex*/          0,
    /*data*/            NULL,
    /*length*/          0,
    /*timeout ms*/      ESPUSBJTAG_TIMEOUT_MS);

    if (ret != 0) {
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

    if(flush_buffer)
        flush();

    if (len == 0)
	return 0;

    for(int i = 0; i < (len+7)>>3; i++)
      cerr << " " << std::hex << (int)tms[i];
    cerr << endl;

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
        uint8_t tms_bit = (tms[i>>3] >> (i&7)) & 1; // get i'th bit from tms
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
            /*endpoint*/                   ESPUSBJTAG_WRITE_EP,
            /*data*/                       buf,
            /*length*/                     buffer_idx,
            /*transf.len*/                 &transferred_length,
            /*timeout ms*/                 ESPUSBJTAG_TIMEOUT_MS);
            if (ret != 0)
            {
                cerr << "writeTMS: usb bulk write failed " << ret << endl;
                return -EXIT_FAILURE;
            }
            cerr << "tms" << endl;
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
            cerr << "clk" << endl;
            buffer_idx = 0; // reset
        }
    }
    return EXIT_SUCCESS;
}

int esp_usb_jtag::setio(int srst, int tms, int tdi, int tck)
{
    uint16_t wvalue = ((1&srst)<<3) | ((1&tck)<<2) | ((1&tms)<<1) | (1&tdi);
    int ret = libusb_control_transfer(dev_handle,
    /*type*/            LIBUSB_REQUEST_TYPE_VENDOR,
    /*brequest*/        VEND_JTAG_SETIO,
    /*wvalue*/          wvalue,
    /*interface*/       0,
    /*data*/            NULL,
    /*length*/          0,
    /*timeout ms*/	ESPUSBJTAG_TIMEOUT_MS);

    if (ret != 0)
    {
        cerr << "setio: control write failed " << ret << endl;
        return -EXIT_FAILURE;
    }
    cerr << "setio 0x" << std::hex << wvalue << endl;
    return 0;
}

int esp_usb_jtag::flush()
{
    uint8_t buf[1] = { (CMD_FLUSH<<3) | CMD_FLUSH };
    int transferred_length; // never used
    int ret = libusb_bulk_transfer(dev_handle,
    /*endpoint*/                   ESPUSBJTAG_WRITE_EP,
    /*data*/                       buf,
    /*length*/                     sizeof(buf),
    /*transferred length*/         &transferred_length,
    /*timeout ms*/                 ESPUSBJTAG_TIMEOUT_MS);
    if (ret != 0)
    {
        cerr << "flush: usb bulk write failed " << ret << endl;
        return -EXIT_FAILURE;
    }
    cerr << "flush" << endl;
    return 0;
}

void esp_usb_jtag::drain_in()
{
  uint8_t dummy_rx[64];
  int transferred_length = 1;
  while(transferred_length > 0)
  {
    transferred_length = 0;
    libusb_bulk_transfer(dev_handle,
            /*endpoint*/                   ESPUSBJTAG_READ_EP,
            /*data*/                       dummy_rx,
            /*length*/                     sizeof(dummy_rx),
            /*transferred length*/         &transferred_length,
            /*timeout ms*/                 ESPUSBJTAG_TIMEOUT_MS);
  }
  cerr << "drain_in" << endl;
}

// TODO
// [ ] odd len
// [ ] end (DR_SHIFT, IR_SHIFT)
int esp_usb_jtag::writeTDI(const uint8_t *tx, uint8_t *rx, uint32_t len, bool end)
{
    int ret;
    uint8_t tx_buf[OUT_EP_SZ];
    memset(rx, 0, len>>3);
    #if 0
    // for debug force IDCODE 0x12345678 returned
    if(len >= 4) memcpy(rx, "\x78\x56\x34\x12", 4);
    return EXIT_SUCCESS;
    #endif
    uint32_t real_bit_len = len - (end ? 1 : 0);
    // uint32_t kRealByteLen = (len + 7) / 8;
    int transferred_length;

    if (len == 0)
	return 0;

    cerr << "real_bit_len=0x" << real_bit_len << endl;

    // drain_in();
    uint8_t prev_high_nibble = CMD_FLUSH << 4; // for odd length 1st command is flush = nop
    uint32_t tx_buffer_idx = 0; // reset
    uint8_t is_high_nibble = 1 & ~real_bit_len;
    // for even len: start with is_high_nibble = 1
    // for odd len:  start with is_high_nibble = 0
    //               1st (high nibble) is flush = nop
    //               2nd (low nibble) is data
    // last byte in buf will have data in both nibbles, no flush
    // exec order: high-nibble-first, low-nibble-second
    cerr << "is high nibble=" << (int)is_high_nibble << endl;
    int bits_in_tx_buf = 0;
    for(int i = 0; i < (real_bit_len+7)>>3; i++)
      cerr << " " << std::hex << (int)tx[i];
    cerr << endl;
    cerr << "tdi_bits ";
    for (uint32_t i = 0; i < real_bit_len; i++)
    {
        uint8_t tdi_bit = (tx[i>>3] >> (i&7)) & 1; // get i'th bit from rx
        cerr << (int)tdi_bit;
        uint8_t cmd = CMD_CLK(/*tdo*/1, /*tdi*/tdi_bit, /*tms*/0); // with TDO capture
        if(is_high_nibble)
        {   // 1st (high nibble) = data
            tx_buf[tx_buffer_idx] = prev_high_nibble = cmd << 4;
            is_high_nibble = 0;
        }
        else // low nibble
        {   // 2nd (low nibble) = data, keep prev high nibble
            tx_buf[tx_buffer_idx++] = prev_high_nibble | cmd;
            is_high_nibble = 1;
        }
        bits_in_tx_buf++;
        if (tx_buffer_idx >= sizeof(tx_buf) /*buf full*/ || i >= real_bit_len - 1 /*last*/)
        {
            cerr << endl << "writeTDI: write_ep len bytes=0x" << tx_buffer_idx << endl;
            for(int j = 0; j < tx_buffer_idx; j++)
                cerr << " " << std::hex << (int)tx_buf[j];
            cerr << endl;
            ret = libusb_bulk_transfer(dev_handle,
            /*endpoint*/                   ESPUSBJTAG_WRITE_EP,
            /*data*/                       tx_buf,
            /*length*/                     tx_buffer_idx,
            /*transferred length*/         &transferred_length,
            /*timeout ms*/                 ESPUSBJTAG_TIMEOUT_MS);
            if (ret != 0)
            {
                cerr << "writeTDI: usb bulk write failed " << ret << endl;
                return -EXIT_FAILURE;
            }
            cerr << "writeTDI write 0x" << tx_buffer_idx << " bytes" << endl;
            flush(); // must flush before reading
            // TODO support odd len for TDO
            // currently only even len TDO works correctly
            // for odd len first command sent is CMD_FUSH
            // so TDI rx_buf will be missing 1 bit
            uint16_t read_bit_len = tx_buffer_idx<<1;
            // read_bit_len = bits_in_tx_buf;
            uint16_t read_byte_len = (read_bit_len+7)>>3;
            // cerr << "read_bit_len=" << (int)read_bit_len << " read_byte_len=" << (int)read_byte_len << endl;
            // read_byte_len = 1;
            int received_bytes = 0;
            while(received_bytes < read_byte_len)
            {
                ret = libusb_bulk_transfer(dev_handle,
                /*endpoint*/                   ESPUSBJTAG_READ_EP,
                /*data*/                       &(rx[(i>>3)+received_bytes]),
                /*length*/                     read_byte_len-received_bytes,
                /*transferred length*/         &transferred_length,
                /*timeout ms*/                 ESPUSBJTAG_TIMEOUT_MS);
                if (ret != 0)
                {
                    cerr << "writeTDI: usb bulk read failed " << ret << endl;
                    // return -EXIT_FAILURE;
                    break;
                }
                cerr << "writeTDI read" << endl;
                if (read_byte_len != transferred_length)
                {
                    cerr << "writeTDI: usb bulk read expected=" << read_byte_len << " received=" << transferred_length << endl;
                    // return -EXIT_FAILURE;
                    break;
                }
                received_bytes += transferred_length;
            }
            tx_buffer_idx = 0; // reset
            bits_in_tx_buf = 0;
        }
    }

    #if 0
    if(end)
    {
        // TODO support end (DR_SHIFT, IR_SHIFT)
    }
    #endif
    return EXIT_SUCCESS;
}
