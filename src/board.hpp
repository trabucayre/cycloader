// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2019 Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 */

#ifndef BOARD_HPP
#define BOARD_HPP

#include <map>

#include "cable.hpp"

/* AN_232R-01_Bit_Bang_Mode_Available_For_FT232R_and_Ft245R */
enum {
	FT232RL_TXD = 0,
	FT232RL_RXD = 1,
	FT232RL_RTS = 2,
	FT232RL_CTS = 3,
	FT232RL_DTR = 4,
	FT232RL_DSR = 5,
	FT232RL_DCD = 6,
	FT232RL_RI  = 7
};

/* AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes */
enum {
	DBUS0 = (1 <<  0),
	DBUS1 = (1 <<  1),
	DBUS2 = (1 <<  2),
	DBUS3 = (1 <<  3),
	DBUS4 = (1 <<  4),
	DBUS5 = (1 <<  5),
	DBUS6 = (1 <<  6),
	DBUS7 = (1 <<  7),
	CBUS0 = (1 <<  8),
	CBUS1 = (1 <<  9),
	CBUS2 = (1 << 10),
	CBUS3 = (1 << 11),
	CBUS4 = (1 << 12),
	CBUS5 = (1 << 13),
	CBUS6 = (1 << 14),
	CBUS7 = (1 << 15)
};

/*!
 * \brief for bitbang mode this structure provide value for each JTAG signals
 */
typedef struct {
	uint8_t tms_pin; /*! TMS pin value */
	uint8_t tck_pin; /*! TCK pin value */
	uint8_t tdi_pin; /*! TDI pin value */
	uint8_t tdo_pin; /*! TDO pin value */
} jtag_pins_conf_t;

typedef struct {
	uint16_t cs_pin;    /*! CS pin value */
	uint16_t sck_pin;   /*! SCK pin value */
	uint16_t miso_pin;  /*! MISO pin value */
	uint16_t mosi_pin;  /*! MOSI pin value */
	uint16_t holdn_pin; /*! HOLDN pin value */
	uint16_t wpn_pin;   /*! WPN pin value */
} spi_pins_conf_t;

enum {
	COMM_JTAG = (1 << 0),
	COMM_SPI  = (1 << 1),
	COMM_DFU  = (1 << 2),
};

/*!
 * \brief a board has a target cable and optionally a pin configuration
 * (bitbang mode)
 */
typedef struct {
	std::string manufacturer;
	std::string cable_name; /*! provide name of one entry in cable_list */
	std::string fpga_part;  /*! provide full fpga model name with package */
	uint16_t reset_pin;      /*! reset pin value */
	uint16_t done_pin;       /*! done pin value */
	uint16_t oe_pin;         /*! output enable pin value */
	uint16_t mode;           /*! communication type (JTAG or SPI) */
	jtag_pins_conf_t jtag_pins_config; /*! for bitbang, provide struct with pins value */
	spi_pins_conf_t spi_pins_config; /*! for SPI, provide struct with pins value */
	uint32_t default_freq;   /* Default clock speed: 0 = use cable default */
	uint16_t vid;             /* optional VID: used only with DFU */
	uint16_t pid;             /* optional VID: used only with DFU */
	int16_t altsetting;       /* optional altsetting: used only with DFU */
} target_board_t;

#define CABLE_DEFAULT 0
#define CABLE_MHZ(_m) ((_m) * 1000000)

#define JTAG_BOARD(_name, _fpga_part, _cable, _rst, _done, _freq) \
	{_name, {"", _cable, _fpga_part, _rst, _done, 0, COMM_JTAG, {}, {}, _freq, 0, 0, -1}}
#define JTAG_BITBANG_BOARD(_name, _fpga_part, _cable, _rst, _done, _tms, _tck, _tdi, _tdo, _freq) \
	{_name, {"", _cable, _fpga_part, _rst, _done, 0, COMM_JTAG, { _tms, _tck, _tdi, _tdo }, {}, \
	_freq, 0, 0, -1}}
#define SPI_BOARD(_name, _manufacturer, _cable, _rst, _done, _oe, _cs, _sck, _si, _so, _holdn, _wpn, _freq) \
	{_name, {_manufacturer, _cable, "", _rst, _done, _oe, COMM_SPI, {}, \
		{_cs, _sck, _so, _si, _holdn, _wpn}, _freq, 0, 0, -1}}
#define DFU_BOARD(_name, _fpga_part, _cable, _vid, _pid, _alt) \
	{_name, {"", _cable, _fpga_part, 0, 0, 0, COMM_DFU, {}, {}, 0, _vid, _pid, _alt}}

static std::map <std::string, target_board_t> board_list = {
	JTAG_BOARD("ac701",           "xc7a200tfbg676", "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("acornCle215",     "xc7a200tsbg484", "",         0, 0, CABLE_DEFAULT),
	JTAG_BOARD("analogMax",       "",               "ft2232",   0, 0, CABLE_DEFAULT),
	JTAG_BOARD("litex-acorn-baseboard-mini", "xc7a200tsbg484", "", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("alchitry_au",     "xc7a35tftg256",  "ft2232",   0, 0, CABLE_DEFAULT),
	JTAG_BOARD("alchitry_au_plus","xc7a100tftg256",  "ft2232",   0, 0, CABLE_DEFAULT),
	JTAG_BOARD("alinx_ax516",     "xc6slx16csg324", "",         0, 0, CABLE_DEFAULT),
	JTAG_BOARD("alinx_ax7101",    "xc7a100tfgg484", "",         0, 0, CABLE_DEFAULT),
	JTAG_BOARD("alinx_ax7102",    "xc7a100tfgg484", "",         0, 0, CABLE_DEFAULT),
	JTAG_BOARD("antmicro_ddr4_tester", "xc7k160tffg676", "ft4232", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("antmicro_ddr5_tester", "xc7k160tffg676", "ft4232", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("antmicro_lpddr4_tester", "xc7k70tfbg484", "ft4232", 0, 0, CABLE_DEFAULT),
	/* left for backward compatibility, use right name instead */
	JTAG_BOARD("arty",            "xc7a35tcsg324",  "digilent", 0, 0, CABLE_MHZ(10)),
	JTAG_BOARD("arty_a7_35t",     "xc7a35tcsg324",  "digilent", 0, 0, CABLE_MHZ(10)),
	JTAG_BOARD("arty_a7_100t",    "xc7a100tcsg324", "digilent", 0, 0, CABLE_MHZ(10)),
	JTAG_BOARD("arty_s7_25",      "xc7s25csga324",  "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("arty_s7_50",      "xc7s50csga324",  "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("arty_z7_10",      "xc7z010clg400",  "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("arty_z7_20",      "xc7z020clg400",  "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("axu2cga",         "xczu2cg",        "",         0, 0, CABLE_DEFAULT),
	JTAG_BOARD("basys3",          "xc7a35tcpg236",  "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("c5g",             "", "usb-blaster",0, 0, CABLE_DEFAULT),
	JTAG_BOARD("cmod_s7",         "xc7s25csga225",  "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("cmoda7_15t",      "xc7a15tcpg236",  "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("cmoda7_35t",      "xc7a35tcpg236",  "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("colorlight",      "", "",           0, 0, CABLE_DEFAULT),
	JTAG_BOARD("colorlight-i5",   "", "cmsisdap", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("colorlight-i9",   "", "cmsisdap", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("colorlight-i9+",  "xc7a50tfgg484", "",        0, 0, CABLE_DEFAULT),
	JTAG_BOARD("crosslinknx_evn",       "", "ft2232", 0, 0, CABLE_MHZ(1)),
	JTAG_BOARD("certusnx_versa_evn",    "", "ft2232", 0, 0, CABLE_MHZ(1)),
	JTAG_BOARD("certuspronx_evn",       "", "ft2232", 0, 0, CABLE_MHZ(1)),
	JTAG_BOARD("certuspronx_versa_evn", "", "ft2232", 0, 0, CABLE_MHZ(1)),
	JTAG_BOARD("cyc1000",         "10cl025256", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("cyc5000",         "5ce215", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("c10lp-refkit",    "10cl055484", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("de0",             "", "usb-blaster",0, 0, CABLE_DEFAULT),
	JTAG_BOARD("de0nano",         "ep4ce2217", "usb-blaster",0, 0, CABLE_DEFAULT),
	JTAG_BOARD("de0nanoSoc",      "", "usb-blasterII",0, 0, CABLE_DEFAULT),
	JTAG_BOARD("de10lite",        "", "usb-blaster",0, 0, CABLE_DEFAULT),
	JTAG_BOARD("de10nano",        "", "usb-blasterII",0, 0, CABLE_DEFAULT),
	JTAG_BOARD("de1Soc",          "5CSEMA5", "usb-blasterII",0, 0, CABLE_DEFAULT),
	JTAG_BOARD("deca",            "10M50DA", "usb-blasterII",0, 0, CABLE_DEFAULT),
	JTAG_BOARD("dragonL",         "xc6slx25tcsg324", "",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("ecp5_evn",        "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("ecpix5",          "", "ecpix5-debug", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("ecpix5_r03",      "", "ft4232",       0, 0, CABLE_DEFAULT),
	SPI_BOARD("fireant",              "efinix", "ft232",
			DBUS4, DBUS5, 0, DBUS3, DBUS0, DBUS1, DBUS2, DBUS6, 0, CABLE_DEFAULT),
	DFU_BOARD("fomu",             "", "dfu", 0x1209, 0x5bf0, 0),
	SPI_BOARD("ft2232_spi",      "none", "ft2232",
			DBUS7, DBUS6, 0,
			DBUS4, DBUS0, DBUS1, DBUS2,
			0, 0, CABLE_DEFAULT),
	SPI_BOARD("gatemate_pgm_spi",   "colognechip", "gatemate_pgm",
			DBUS4, DBUS5, CBUS0, DBUS3, DBUS0, DBUS1, DBUS2, 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("gatemate_evb_jtag", "", "gatemate_evb_jtag", 0, 0, CABLE_DEFAULT),
	SPI_BOARD("gatemate_evb_spi",   "colognechip", "gatemate_evb_spi",
			DBUS4, DBUS5, CBUS0, DBUS3, DBUS0, DBUS1, DBUS2, 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("genesys2",        "xc7k325tffg900", "digilent_b", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("gr740-mini",      "", "ft4232hp_b",       0, 0, CABLE_MHZ(1)),
	JTAG_BOARD("hseda-xc6slx16",  "xc6slx16ftg256", "",           0, 0, CABLE_DEFAULT),
	/* most ice40 boards uses the same pinout */
	SPI_BOARD("ice40_generic",    "lattice", "ft2232",
			DBUS7, DBUS6, 0,
			DBUS4, DBUS0, DBUS1, DBUS2,
			0, 0, CABLE_DEFAULT),
	DFU_BOARD("icebreaker-bitsy", "", "dfu", 0x1d50, 0x6146, 0),
	JTAG_BOARD("kc705",           "", "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("kcu105",          "", "jtag-smt2-nc", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("kcu116",          "xcku5p-ffvb676", "jtag-smt2-nc", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("LD-SCHOKO",       "LFE5U-45F-6CABGA256", "", 0, 0, CABLE_MHZ(6)),
	DFU_BOARD("LD-SCHOKO-DFU",        "", "dfu", 0x16d0, 0x116d, 0),
	JTAG_BOARD("LD-KONFEKT",      "LFE5U-12F-6BG256C", "", 0, 0, CABLE_MHZ(6)),
	DFU_BOARD("LD-KONFEKT-DFU",       "", "dfu", 0x16d0, 0x116d, 0),
	JTAG_BOARD("licheeTang",      "", "anlogicCable", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("lilygo-t-fpga",   "", "gwu2x",      0, 0, CABLE_DEFAULT),
	/* left for backward compatibility, use tec0117 instead */
	JTAG_BOARD("littleBee",       "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("machXO2EVN",      "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("machXO3SK",       "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("machXO3EVN",      "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("mimas_a7",		  "xc7a50tfgg484",	"numato", 0, 0, CABLE_MHZ(30)),
	JTAG_BOARD("minispartan6",    "", "ft2232",    0, 0, CABLE_DEFAULT),
	JTAG_BOARD("nexys_a7_50",     "xc7a50tcsg324",  "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("nexys_a7_100",    "xc7a100tcsg324", "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("nexysVideo",      "xc7a200tsbg484", "digilent_b", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("olimex_gatemateevb", "GM1A1", "dirtyJtag", 0, 0, CABLE_DEFAULT),
	DFU_BOARD("orangeCrab",       "", "dfu", 0x1209, 0x5af0, 0),
	DFU_BOARD("orbtrace_dfu",     "", "dfu", 0x1209, 0x3442, 1),
	JTAG_BOARD("papilio_one",     "xc3s500evq100", "papilio", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("pipistrello",     "xc6slx45csg324", "ft2232", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("pynq_z1",         "xc7z020clg400",  "ft2232", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("pynq_z2",         "xc7z020clg400",  "ft2232", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("qmtechCyclone10", "10cl016484", "",  0, 0, CABLE_DEFAULT),
	JTAG_BOARD("qmtechCycloneIVGX", "ep4cgx15027", "",  0, 0, CABLE_DEFAULT),
	JTAG_BOARD("qmtechCycloneIV", "ep4ce1523", "",  0, 0, CABLE_DEFAULT),
	JTAG_BOARD("qmtechCycloneV",  "5ce223", "",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("qmtechCycloneV_5ce523",  "5ce523", "",	0,0, CABLE_DEFAULT),
	JTAG_BOARD("qmtechKintex7",   "xc7k325tffg676", "", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("redpitaya14",     "xc7z010clg400", "", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("runber",          "", "ft232",      0, 0, CABLE_DEFAULT),
	JTAG_BOARD("spartanEdgeAccelBoard", "", "",0, 0, CABLE_DEFAULT),
	JTAG_BOARD("spec45",          "xc6slx45tfgg484",  "", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("spec150",         "xc6slx150tfgg484", "", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("stlv7325",        "xc7k325tffg676", "ft4232", 0, 0, CABLE_MHZ(3)),
	JTAG_BOARD("tangnano",        "", "ch552_jtag", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("tangnano1k",      "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("tangnano4k",      "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("tangnano9k",      "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("tangnano20k",     "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("tangprimer20k",   "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("tangprimer25k",   "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("tangmega138k",    "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("te0712_8",      "xc7a200tfbg484", "",     0, 0, CABLE_MHZ(15)),
	JTAG_BOARD("tec0117",         "", "ft2232",     0, 0, CABLE_DEFAULT),
	JTAG_BOARD("tec0330",         "xc7vx330tffg1157", "",     0, 0, CABLE_MHZ(15)),
	SPI_BOARD("titanium_ti60_f225","efinix", "efinix_spi_ft4232",
			DBUS4, DBUS5, DBUS7, DBUS3, DBUS0, DBUS1, DBUS2, DBUS6, 0, CABLE_DEFAULT),
	JTAG_BOARD("titanium_ti60_f225_jtag", "ti60f225","efinix_jtag_ft4232",  0, 0, CABLE_DEFAULT),
	JTAG_BOARD("trion_t20_bga256_jtag",   "t20f256", "efinix_jtag_ft2232",  0, 0, CABLE_DEFAULT),
	SPI_BOARD("trion_t120_bga576","efinix", "efinix_spi_ft2232",
			DBUS4, DBUS5, DBUS7, DBUS3, DBUS0, DBUS1, DBUS2, DBUS6, 0, CABLE_DEFAULT),
	JTAG_BOARD("trion_t120_bga576_jtag", "",        "ft2232_b",     0, 0, CABLE_DEFAULT),
	JTAG_BITBANG_BOARD("ulx2s",   "", "ft232RL", 0, 0,
			FT232RL_RI, FT232RL_DSR, FT232RL_CTS, FT232RL_DCD, CABLE_DEFAULT),
	JTAG_BITBANG_BOARD("ulx3s",   "", "ft231X",  0, 0,
			FT232RL_DCD, FT232RL_DSR, FT232RL_RI, FT232RL_CTS, CABLE_DEFAULT),
	DFU_BOARD("ulx3s_dfu",        "", "dfu", 0x1d50, 0x614b, 0),
	JTAG_BOARD("ulx3s_esp",       "", "esp32s3", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("usrpx300",        "xc7k325tffg900", "digilent", 0, 0, CABLE_MHZ(15)),
	JTAG_BOARD("usrpx310",        "xc7k410tffg900", "digilent", 0, 0, CABLE_MHZ(15)),
	JTAG_BOARD("vec_v6",          "xc6vlx130tff784", "ft2232", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("vc709", 		  "xc7vx690tffg1761", "digilent", 0, 0, CABLE_MHZ(15)),
	JTAG_BOARD("vcu108",          "xcvu095-ffva2104", "jtag-smt2-nc", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("vcu118",          "xcvu9p-flga2104", "jtag-smt2-nc", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("vcu128",          "xcvu37p-fsvh2892", "ft4232", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("vcu1525",         "xcvu9p-fsgd2104",  "ft4232", 0, 0, CABLE_MHZ(15)),
	JTAG_BOARD("xem8320",         "xcau25p-2ffvb676", ""       , 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("xyloni_jtag",     "t8f81", "efinix_jtag_ft4232", 0, 0, CABLE_DEFAULT),
	SPI_BOARD("xyloni_spi",       "efinix", "efinix_spi_ft4232",
			DBUS4, DBUS5, DBUS7, DBUS3, DBUS0, DBUS1, DBUS2, DBUS6, 0, CABLE_DEFAULT),
	JTAG_BOARD("xtrx",            "xc7a50tcpg236", ""            , 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("zc702",           "xc7z020clg484", "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("zc706",           "xc7z045ffg900", "jtag-smt2-nc", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("zcu102",          "xczu9egffvb1156", "jtag-smt2-nc", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("zcu106",          "xczu7evffvc1156", "jtag-smt2-nc", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("zedboard",        "xc7z020clg484", "digilent_hs2", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("zybo_z7_10",      "xc7z010clg400",  "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("zybo_z7_20",      "xc7z020clg400",  "digilent", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("mini_itx",        "xc7z100ffg900", "jtag-smt2-nc", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("vmm3",            "xc7s50csga324", "ft2232", 0, 0, CABLE_DEFAULT),
	JTAG_BOARD("step-max10_v1",   "10m02scm153c8g", "usb-blaster",0, 0, CABLE_DEFAULT),
	JTAG_BOARD("step-mxo2_v2",    "lcmxo2-4000hc-4mg132cc", "ft232",0, 0, CABLE_DEFAULT)
};

#endif
