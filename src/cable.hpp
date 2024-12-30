// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2019 Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 */

#ifndef SRC_CABLE_HPP_
#define SRC_CABLE_HPP_

#include <map>
#include <string>
#include <cstdint>

/*!
 * \brief define type of communication
 */
enum communication_type {
	MODE_ANLOGICCABLE = 0, /*! JTAG probe from Anlogic */
	MODE_CH552_JTAG,       /*! ch552_jtag firmware */
	MODE_FTDI_BITBANG,     /*! used with ft232RL/ft231x */
	MODE_FTDI_SERIAL,      /*! ft2232, ft232H */
	MODE_JLINK,            /*! ft2232, ft232H */
	MODE_DIRTYJTAG,        /*! JTAG probe firmware for STM32F1 */
	MODE_USBBLASTER,       /*! JTAG probe firmware for USBBLASTER */
	MODE_CMSISDAP,         /*! CMSIS-DAP JTAG probe */
	MODE_DFU,              /*! DFU based probe */
	MODE_XVC_CLIENT,       /*! Xilinx Virtual Cable client */
	MODE_LIBGPIOD_BITBANG, /*! Bitbang gpio pins */
	MODE_JETSONNANO_BITBANG, /*! Bitbang gpio pins */
	MODE_REMOTEBITBANG,    /*! Remote Bitbang mode */
	MODE_CH347,            /*! CH347 JTAG mode */
	MODE_GWU2X,            /*! Gowin GWU2X JTAG mode */
	MODE_ESP,              /*! esp32c3, esp32s3 */
};

/*!
 * \brief FTDI specific configuration structure
 */
typedef struct {
	int interface;    /*! FTDI interface (A, B, C, D) */
	int bit_low_val;  /*! xDBUS 0-7 default value */
	int bit_low_dir;  /*! xDBUS 0-7 default direction (0: in, 1: out) */
	int bit_high_val; /*! xCBUS 0-7 default value */
	int bit_high_dir; /*! xCBUS 0-7 default direction (0: in, 1: out) */
	int index;
	int status_pin;
} mpsse_bit_config;

/*!
 * \brief FTDI interface ID
 */
enum ftdi_if {
    FTDI_INTF_A = 1,
    FTDI_INTF_B = 2,
    FTDI_INTF_C = 3,
    FTDI_INTF_D = 4
};

/*!
 * \brief cable characteristics
 */
struct cable_t {
	communication_type type; /*! see enum communication_type */
	int vid;                 /*! Vendor ID */
	int pid;                 /*! Product ID */
	uint8_t bus_addr;        /*! bus number (must be set to 0: user defined */
	uint8_t device_addr;     /*! device number (must be set 0: user defined */
	mpsse_bit_config config; /*! FTDI specific configurations */
};

/* FTDI serial (MPSSE) configuration */
#define FTDI_SER(_vid, _pid, _intf, _blv, _bld, _bhv, _bhd) \
	{MODE_FTDI_SERIAL, _vid, _pid, 0, 0, {_intf, _blv, _bld, _bhv, _bhd, 0, -1}}
/* FTDI bitbang configuration */
#define FTDI_BB(_vid, _pid, _intf, _blv, _bld, _bhv, _bhd) \
	{MODE_FTDI_BITBANG, _vid, _pid, 0, 0, {_intf, _blv, _bld, _bhv, _bhd, 0, -1}}
/* CMSIS DAP configuration */
#define CMSIS_CL(_vid, _pid) \
	{MODE_CMSISDAP, _vid, _pid, 0, 0, {}}
/* Others cable configuration */
#define CABLE_DEF(_type, _vid, _pid) \
	{_type, _vid, _pid, 0, 0, {}}
#define CABLE_DEF_FULL(_type, _vid, _pid, _blv, _bld, _bhv, _bhd) \
	{_type, _vid, _pid, 0, 0, {0, _blv, _bld, _bhv, _bhd, 0, -1}}

static std::map <std::string, cable_t> cable_list = {
	// last 4 bytes are ADBUS7-0 value, ADBUS7-0 direction, ACBUS7-0 value, ACBUS7-0 direction
	// some cables requires explicit values on some of the I/Os
	{"anlogicCable",       CABLE_DEF(MODE_ANLOGICCABLE, 0x0547, 0x1002)},
	{"arm-usb-ocd-h",      FTDI_SER(0x15ba, 0x002b, FTDI_INTF_A, 0x08, 0x1B, 0x09, 0x0B)},
	{"bus_blaster",        FTDI_SER(0x0403, 0x6010, FTDI_INTF_A, 0x08, 0x1B, 0x08, 0x0B)},
	{"bus_blaster_b",      FTDI_SER(0x0403, 0x6010, FTDI_INTF_B, 0x08, 0x0B, 0x08, 0x0B)},
	{"ch552_jtag",         FTDI_SER(0x0403, 0x6010, FTDI_INTF_A, 0x08, 0x0B, 0x08, 0x0B)},
	{"ch347_jtag",         CABLE_DEF(MODE_CH347, 0x1a86, 0x55dd                        )},
	{"cmsisdap",           CMSIS_CL(0x0d28, 0x0204                                     )},
	{"gatemate_pgm",       FTDI_SER(0x0403, 0x6014, FTDI_INTF_A, 0x10, 0x9B, 0x14, 0x17)},
	{"gatemate_evb_jtag",  FTDI_SER(0x0403, 0x6010, FTDI_INTF_A, 0x10, 0x1B, 0x00, 0x01)},
	{"gatemate_evb_spi",   FTDI_SER(0x0403, 0x6010, FTDI_INTF_B, 0x00, 0x1B, 0x00, 0x01)},
	{"dfu",                CABLE_DEF(MODE_DFU, 0, 0                                    )},
	{"digilent",           FTDI_SER(0x0403, 0x6010, FTDI_INTF_A, 0xe8, 0xeb, 0x00, 0x60)},
	{"digilent_b",         FTDI_SER(0x0403, 0x6010, FTDI_INTF_B, 0xe8, 0xeb, 0x00, 0x60)},
	{"digilent_hs2",       FTDI_SER(0x0403, 0x6014, FTDI_INTF_A, 0xe8, 0xeb, 0x00, 0x60)},
	{"digilent_hs3",       FTDI_SER(0x0403, 0x6014, FTDI_INTF_A, 0x88, 0x8B, 0x20, 0x30)},
	{"digilent_ad",        FTDI_SER(0x0403, 0x6014, FTDI_INTF_A, 0x08, 0x0B, 0x80, 0x80)},
	{"dirtyJtag",          CABLE_DEF(MODE_DIRTYJTAG, 0x1209, 0xC0CA                    )},
	{"esp32s3",            CABLE_DEF(MODE_ESP, 0x303a, 0x1001                          )},
	{"efinix_spi_ft4232",  FTDI_SER(0x0403, 0x6011, FTDI_INTF_A, 0x08, 0x8B, 0x00, 0x00)},
	{"efinix_jtag_ft4232", FTDI_SER(0x0403, 0x6011, FTDI_INTF_B, 0x08, 0x8B, 0x00, 0x00)},
	{"efinix_spi_ft2232",  FTDI_SER(0x0403, 0x6010, FTDI_INTF_A, 0x08, 0x8B, 0x00, 0x00)},
	{"efinix_jtag_ft2232", FTDI_SER(0x0403, 0x6010, FTDI_INTF_B, 0x08, 0x8B, 0x00, 0x00)},
	{"ft2232",             FTDI_SER(0x0403, 0x6010, FTDI_INTF_A, 0x08, 0x0B, 0x08, 0x0B)},
	{"ft2232_b",           FTDI_SER(0x0403, 0x6010, FTDI_INTF_B, 0x08, 0x0B, 0x00, 0x00)},
	{"ft231X",             FTDI_BB (0x0403, 0x6015, FTDI_INTF_A, 0x00, 0x00, 0x00, 0x00)},
	{"ft232",              FTDI_SER(0x0403, 0x6014, FTDI_INTF_A, 0x08, 0x0B, 0x08, 0x0B)},
	{"ft232RL",            FTDI_BB( 0x0403, 0x6001, FTDI_INTF_A, 0x08, 0x0B, 0x08, 0x0B)},
	{"ft4232",             FTDI_SER(0x0403, 0x6011, FTDI_INTF_A, 0x08, 0x0B, 0x08, 0x0B)},
	{"ft4232hp",           FTDI_SER(0x0403, 0x6043, FTDI_INTF_A, 0x08, 0x0B, 0x00, 0x00)},
	{"ft4232hp_b",         FTDI_SER(0x0403, 0x6043, FTDI_INTF_B, 0x08, 0x0B, 0x00, 0x00)},
	{"gwu2x",              CABLE_DEF_FULL(MODE_GWU2X, 0x33AA, 0x0120, 0x02, 0x07, 0x0, 0x0)},
	{"ecpix5-debug",       FTDI_SER(0x0403, 0x6010, FTDI_INTF_A, 0xF8, 0xFB, 0xFF, 0xFF)},
	{"jlink",              CABLE_DEF(MODE_JLINK, 0x1366, 0x0105                        )},
	{"jlink_base",         CABLE_DEF(MODE_JLINK, 0x1366, 0x0101                        )},
	{"jtrace_pro",         CABLE_DEF(MODE_JLINK, 0x1366, 0x1020                        )},
	{"jtag-smt2-nc",       FTDI_SER(0x0403, 0x6014, FTDI_INTF_A, 0xe8, 0xeb, 0x00, 0x60)},
	{"lpc-link2",          CMSIS_CL(0x1fc9, 0x0090                                     )},
	{"numato",			   FTDI_SER(0x2a19, 0x1009, FTDI_INTF_B, 0x08, 0x4b, 0x00, 0x00)},
	{"orbtrace",           CMSIS_CL(0x1209, 0x3443                                     )},
	{"papilio",            FTDI_SER(0x0403, 0x6010, FTDI_INTF_A, 0x08, 0x0B, 0x09, 0x0B)},
	{"steppenprobe",       FTDI_SER(0x0403, 0x6010, FTDI_INTF_A, 0x58, 0xFB, 0x00, 0x99)},
	{"tigard",             FTDI_SER(0x0403, 0x6010, FTDI_INTF_B, 0x08, 0x3B, 0x00, 0x00)},
	{"usb-blaster",        CABLE_DEF(MODE_USBBLASTER, 0x09Fb, 0x6001                   )},
	{"usb-blasterII",      CABLE_DEF(MODE_USBBLASTER, 0x09Fb, 0x6810                   )},
	{"xvc-client",         CABLE_DEF(MODE_XVC_CLIENT, 0x0000, 0x0000                   )},
#ifdef ENABLE_LIBGPIOD
	{"libgpiod",           CABLE_DEF(MODE_LIBGPIOD_BITBANG, 0, 0x0000                  )},
#endif
#ifdef ENABLE_JETSONNANOGPIO
	{"jetson-nano-gpio",   {MODE_JETSONNANO_BITBANG, {}}},
#endif
#ifdef ENABLE_REMOTEBITBANG
	{"remote-bitbang",     CABLE_DEF(MODE_REMOTEBITBANG, 0x0000, 0x0000                )},
#endif
};

#endif  // SRC_CABLE_HPP_
