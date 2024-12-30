// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (C) 2020 Gwenhael Goavec-Merou <gwenhael.goavec-merou@trabucayre.com>
 */

#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <unistd.h>
#include <vector>
#include <string>

#include "anlogicCable.hpp"
#include "ch552_jtag.hpp"
#include "display.hpp"
#include "jtag.hpp"
#include "ftdiJtagBitbang.hpp"
#include "ftdiJtagMPSSE.hpp"
#ifdef ENABLE_GOWIN_GWU2X
#include "gwu2x_jtag.hpp"
#endif
#ifdef ENABLE_LIBGPIOD
#include "libgpiodJtagBitbang.hpp"
#endif
#ifdef ENABLE_JETSONNANOGPIO
#include "jetsonNanoJtagBitbang.hpp"
#endif
#include "jlink.hpp"
#ifdef ENABLE_CMSISDAP
#include "cmsisDAP.hpp"
#endif
#include "dirtyJtag.hpp"
#include "esp_usb_jtag.hpp"
#include "ch347jtag.hpp"
#include "part.hpp"
#ifdef ENABLE_REMOTEBITBANG
#include "remoteBitbang_client.hpp"
#endif
#include "usbBlaster.hpp"
#ifdef ENABLE_XVC
#include "xvc_client.hpp"
#endif

using namespace std;

#define DEBUG 0

#if DEBUG
#define display(...) \
	do { if (_verbose) fprintf(stdout, __VA_ARGS__);}while(0)
#else
#define display(...) do {}while(0)
#endif

/*
 * FT232 JTAG PINS MAPPING:
 * AD0 -> TCK
 * AD1 -> TDI
 * AD2 -> TDO
 * AD3 -> TMS
 */

/* Rmq:
 * pour TMS: l'envoi de n necessite de mettre n-1 comme longueur
 *           mais le bit n+1 est utilise pour l'etat suivant le dernier
 *           front. Donc il faut envoyer 6bits ([5:0]) pertinents pour
 *           utiliser le bit 6 comme etat apres la commande,
 *           le bit 7 corresponds a l'etat de TDI (donc si on fait 7 cycles
 *           l'etat de TDI va donner l'etat de TMS...)
 * transfert/lecture: le dernier bit de IR ou DR doit etre envoye en
 *           meme temps que le TMS qui fait sortir de l'etat donc il faut
 *           pour n bits a transferer :
 *           - envoyer 8bits * (n/8)-1
 *           - envoyer les 7 bits du dernier octet;
 *           - envoyer le dernier avec 0x4B ou 0x6B
 */

Jtag::Jtag(const cable_t &cable, const jtag_pins_conf_t *pin_conf,
			const string &dev,
			const string &serial, uint32_t clkHZ, int8_t verbose,
			const string &ip_adr, int port,
			const bool invert_read_edge, const string &firmware_path,
			const std::map<uint32_t, misc_device> &user_misc_devs):
			_verbose(verbose > 1),
			_state(RUN_TEST_IDLE),
			_tms_buffer_size(128), _num_tms(0),
			_board_name("nope"), _user_misc_devs(user_misc_devs),
			device_index(0), _dr_bits_before(0), _dr_bits_after(0),
			_ir_bits_before(0), _ir_bits_after(0), _curr_tdi(1)
{
	switch (cable.type) {
	case MODE_ANLOGICCABLE:
		_jtag = new AnlogicCable(clkHZ);
		break;
	case MODE_FTDI_BITBANG:
		if (pin_conf == NULL)
			throw std::exception();
		_jtag = new FtdiJtagBitBang(cable, pin_conf, dev, serial, clkHZ, verbose);
		break;
	case MODE_FTDI_SERIAL:
		_jtag = new FtdiJtagMPSSE(cable, dev, serial, clkHZ,
				invert_read_edge, verbose);
		break;
	case MODE_CH552_JTAG:
		_jtag = new CH552_jtag(cable, dev, serial, clkHZ, verbose);
		break;
	case MODE_CH347:
		_jtag = new CH347Jtag(clkHZ, verbose, cable.vid, cable.pid, cable.bus_addr, cable.device_addr);
		break;
	case MODE_DIRTYJTAG:
		_jtag = new DirtyJtag(clkHZ, verbose);
		break;
#ifdef ENABLE_GOWIN_GWU2X
	case MODE_GWU2X:
		_jtag = new GowinGWU2x((cable_t *)&cable, clkHZ, verbose);
		break;
#else
		std::cerr << "Jtag: support for Gowin GWU2X was not enabled at compile time" << std::endl;
		throw std::exception();
#endif
	case MODE_JLINK:
		_jtag = new Jlink(clkHZ, verbose, cable.vid, cable.pid);
		break;
	case MODE_ESP:
		_jtag = new esp_usb_jtag(clkHZ, verbose, 0x303a, 0x1001);
		break;
	case MODE_USBBLASTER:
		_jtag = new UsbBlaster(cable, firmware_path, verbose);
		break;
	case MODE_CMSISDAP:
#ifdef ENABLE_CMSISDAP
		_jtag = new CmsisDAP(cable, cable.config.index, verbose);
		break;
#else
		std::cerr << "Jtag: support for cmsisdap was not enabled at compile time" << std::endl;
		throw std::exception();
#endif
	case MODE_XVC_CLIENT:
#ifdef ENABLE_XVC
		_jtag = new XVC_client(ip_adr, port, clkHZ, verbose);
		break;
#else
		std::cerr << "Jtag: support for xvc-client was not enabled at compile time" << std::endl;
		throw std::exception();
#endif
#ifdef ENABLE_LIBGPIOD
	case MODE_LIBGPIOD_BITBANG:
		_jtag = new LibgpiodJtagBitbang(pin_conf, dev, clkHZ, verbose);
		break;
#endif
#ifdef ENABLE_JETSONNANOGPIO
	case MODE_JETSONNANO_BITBANG:
		_jtag = new JetsonNanoJtagBitbang(pin_conf, dev, clkHZ, verbose);
		break;
#endif
#ifdef ENABLE_REMOTEBITBANG
	case MODE_REMOTEBITBANG:
		_jtag = new RemoteBitbang_client(ip_adr, port, verbose);
		break;
#endif
	default:
		std::cerr << "Jtag: unknown cable type" << std::endl;
		throw std::exception();
	}

	_tms_buffer = (unsigned char *)malloc(sizeof(unsigned char) * _tms_buffer_size);
	if (_tms_buffer == nullptr)
		throw std::runtime_error("Error: memory allocation failed");
	memset(_tms_buffer, 0, _tms_buffer_size);

	detectChain(32);
}

Jtag::~Jtag()
{
	free(_tms_buffer);
	delete _jtag;
}
int Jtag::detectChain(unsigned max_dev)
{
	char message[256];
	uint8_t rx_buff[4];
	/* WA for CH552/tangNano: write is always mandatory */
	const uint8_t tx_buff[4] = {0xff, 0xff, 0xff, 0xff};
	uint32_t tmp;

	/* cleanup */
	_devices_list.clear();
	_irlength_list.clear();
	_ir_bits_before = _ir_bits_after = _dr_bits_before = _dr_bits_after = 0;
	go_test_logic_reset();
	set_state(SHIFT_DR);

	if (_verbose)
		printInfo("Raw IDCODE:");

	for (unsigned i = 0; i < max_dev; ++i) {
		read_write(tx_buff, rx_buff, 32, 0);
		tmp = 0;
		for (int ii = 0; ii < 4; ++ii)
			tmp |= (rx_buff[ii] << (8 * ii));

		if (_verbose) {
			snprintf(message, sizeof(message), "- %d -> 0x%08x", i, tmp);
			printInfo(message);
		}

		if (tmp == 0) {
			throw std::runtime_error("TDO is stuck at 0");
		}
		if (tmp == 0xffffffff) {
			if (_verbose) {
				snprintf(message, sizeof(message), "Fetched TDI, end-of-chain");
				printInfo(message);
			}
			break;
		}

		/* search IDCODE in fpga_list and misc_dev_list
		 * since most device have idcode with high nibble masked
		 * we start to search sub IDCODE
		 * if IDCODE has no match: try the same with version unmasked
		 */
		bool found = false;
		/* ckeck highest nibble to prevent confusion between Cologne Chip
		 * GateMate and Efinix Trion T4/T8 devices
		 */
		if (tmp == 0x20000001)
			found = search_and_insert_device_with_idcode(tmp);
		if (!found) /* not specific case -> search for full */
			found = search_and_insert_device_with_idcode(tmp);
		if (!found) /* if full idcode not found -> search for masked */
			found = search_and_insert_device_with_idcode(tmp & 0x0fffffff);

		if (!found) {
			uint16_t mfg = IDCODE2MANUFACTURERID(tmp);
			uint8_t part = IDCODE2PART(tmp);
			uint8_t vers = IDCODE2VERS(tmp);

			char error[1024];
			snprintf(error, sizeof(error),
				 "Unknown device with IDCODE: 0x%08x"
				 " (manufacturer: 0x%03x (%s),"
				 " part: 0x%02x vers: 0x%x", tmp,
				 mfg, list_manufacturer[mfg].c_str(), part, vers);
			throw std::runtime_error(error);
		}
	}
	set_state(TEST_LOGIC_RESET);
	flushTMS(true);
	return _devices_list.size();
}

bool Jtag::search_and_insert_device_with_idcode(uint32_t idcode)
{
	int irlength = -1;
	auto dev = fpga_list.find(idcode);
	if (dev != fpga_list.end())
		irlength = dev->second.irlength;
	if (irlength == -1) {
		auto misc = misc_dev_list.find(idcode);
		if (misc != misc_dev_list.end())
			irlength = misc->second.irlength;
	}
	if (irlength == -1) {
		auto misc = this->_user_misc_devs.find(idcode);
		if (misc != this->_user_misc_devs.end())
			irlength = misc->second.irlength;
	}
	if (irlength == -1)
		return false;

	return insert_first(idcode, irlength);
}

bool Jtag::insert_first(uint32_t device_id, uint16_t irlength)
{
	_devices_list.insert(_devices_list.begin(), device_id);
	_irlength_list.insert(_irlength_list.begin(), irlength);

	return true;
}

int Jtag::device_select(unsigned index)
{
	if (index > _devices_list.size())
		return -1;
	device_index = index;
	/* get number of devices, in the JTAG chain,
	 * before the selected one
	 */
	_dr_bits_before = _devices_list.size() - device_index - 1;
	/* get number of devices in the JTAG chain
	 * after the selected one
	 */
	_dr_bits_after = device_index;
	_dr_bits = vector<uint8_t>((std::max(_dr_bits_after, _dr_bits_before) + 7)/8, 0);

	/* when the device is not alone and not
	 * the first a serie of bypass must be
	 * send to complete send ir sequence
	 */
	_ir_bits_after = 0;
	for (int i = 0; i < device_index; ++i)
		_ir_bits_after += _irlength_list[i];

	/* send serie of bypass instructions
	 * final size depends on number of device
	 * before targeted and irlength of each one
	 */
	_ir_bits_before = 0;
	for (unsigned i = device_index + 1; i < _devices_list.size(); ++i)
		_ir_bits_before += _irlength_list[i];
	_ir_bits = vector<uint8_t>((std::max(_ir_bits_before, _ir_bits_after) + 7) / 8, 0xff); // BYPASS command is all-ones

	return device_index;
}

void Jtag::setTMS(unsigned char tms)
{
	display("%s %x %d %d\n", __func__, tms, _num_tms, (_num_tms >> 3));
	if (_num_tms+1 == _tms_buffer_size * 8)
		flushTMS(false);
	if (tms != 0)
		_tms_buffer[_num_tms>>3] |= (0x1) << (_num_tms & 0x7);
	_num_tms++;
}

/* reconstruct byte sent to TMS pins
 * - use up to 6 bits
 * -since next bit after length is use to
 *  fix TMS state after sent we copy last bit
 *  to bit after next
 * -bit 7 is TDI state for each clk cycles
 */

int Jtag::flushTMS(bool flush_buffer)
{
	int ret = 0;
	if (_num_tms != 0) {
		display("%s: %d %x\n", __func__, _num_tms, _tms_buffer[0]);

		ret = _jtag->writeTMS(_tms_buffer, _num_tms, flush_buffer, _curr_tdi);

		/* reset buffer and number of bits */
		memset(_tms_buffer, 0, _tms_buffer_size);
		_num_tms = 0;
	} else if (flush_buffer) {
		_jtag->flush();
	}
	return ret;
}

void Jtag::go_test_logic_reset()
{
	/* independently to current state 5 clk with TMS high is enough */
	for (int i = 0; i < 6; i++)
		setTMS(0x01);
	flushTMS(false);
	_state = TEST_LOGIC_RESET;
}

int Jtag::read_write(const uint8_t *tdi, unsigned char *tdo, int len, char last)
{
	flushTMS(false);
	_jtag->writeTDI(tdi, tdo, len, last);
	if (last == 1)
		_state = (_state == SHIFT_DR) ? EXIT1_DR : EXIT1_IR;
	return 0;
}

void Jtag::toggleClk(int nb)
{
	unsigned char c = (TEST_LOGIC_RESET == _state) ? 1 : 0;
	flushTMS(false);
	if (_jtag->toggleClk(c, 0, nb) >= 0)
		return;
	throw std::exception();
	return;
}

int Jtag::shiftDR(const uint8_t *tdi, unsigned char *tdo, int drlen, tapState_t end_state)
{
	/* if current state not shift DR
	 * move to this state
	 */
	if (_state != SHIFT_DR) {
		set_state(SHIFT_DR);
		flushTMS(false);  // force transmit tms state

		if (_dr_bits_before)
			read_write(_dr_bits.data(), NULL, _dr_bits_before, false);
	}

	/* write tdi (and read tdo) to the selected device
	 * end (ie TMS high) is used only when current device
	 * is the last of the chain and a state change must
	 * be done
	 */
	read_write(tdi, tdo, drlen, _dr_bits_after == 0 && end_state != SHIFT_DR);

	/* if it's asked to move in FSM */
	if (end_state != SHIFT_DR) {
		/* if current device is not the last */
		if (_dr_bits_after)
			read_write(_dr_bits.data(), NULL, _dr_bits_after, true);  // its the last force
								   // tms high with last bit


		/* move to end_state */
		set_state(end_state);
	}
	return 0;
}

int Jtag::shiftIR(unsigned char tdi, int irlen, tapState_t end_state)
{
	if (irlen > 8) {
		cerr << "Error: this method this direct char don't support more than 1 byte" << endl;
		return -1;
	}
	return shiftIR(&tdi, NULL, irlen, end_state);
}

int Jtag::shiftIR(unsigned char *tdi, unsigned char *tdo, int irlen, tapState_t end_state)
{
	display("%s: avant shiftIR\n", __func__);

	/* if not in SHIFT IR move to this state */
	if (_state != SHIFT_IR) {
		set_state(SHIFT_IR);
		if (_ir_bits_before)
			read_write(_ir_bits.data(), NULL, _ir_bits_before, false);
	}

	display("%s: envoi ircode\n", __func__);

	/* write tdi (and read tdo) to the selected device
	 * end (ie TMS high) is used only when current device
	 * is the last of the chain and a state change must
	 * be done
	 */
	read_write(tdi, tdo, irlen, _ir_bits_after == 0 && end_state != SHIFT_IR);

	/* it's asked to move out of SHIFT IR state */
	if (end_state != SHIFT_IR) {
		/* again if devices after fill '1' */
		if (_ir_bits_after > 0)
			read_write(_ir_bits.data(), NULL, _ir_bits_after, true);
		/* move to the requested state */
		set_state(end_state);
	}

	return 0;
}

void Jtag::set_state(tapState_t newState, const uint8_t tdi)
{
	_curr_tdi = tdi;
	unsigned char tms = 0;
	while (newState != _state) {
		display("_state : %16s(%02d) -> %s(%02d) ",
			getStateName((tapState_t)_state),
			_state,
			getStateName((tapState_t)newState), newState);
		switch (_state) {
		case TEST_LOGIC_RESET:
			if (newState == TEST_LOGIC_RESET) {
				tms = 1;
			} else {
				tms = 0;
				_state = RUN_TEST_IDLE;
			}
			break;
		case RUN_TEST_IDLE:
			if (newState == RUN_TEST_IDLE) {
				tms = 0;
			} else {
				tms = 1;
				_state = SELECT_DR_SCAN;
			}
			break;
		case SELECT_DR_SCAN:
			switch (newState) {
			case CAPTURE_DR:
			case SHIFT_DR:
			case EXIT1_DR:
			case PAUSE_DR:
			case EXIT2_DR:
			case UPDATE_DR:
				tms = 0;
				_state = CAPTURE_DR;
				break;
			default:
				tms = 1;
				_state = SELECT_IR_SCAN;
			}
			break;
		case SELECT_IR_SCAN:
			switch (newState) {
			case CAPTURE_IR:
			case SHIFT_IR:
			case EXIT1_IR:
			case PAUSE_IR:
			case EXIT2_IR:
			case UPDATE_IR:
				tms = 0;
				_state = CAPTURE_IR;
				break;
			default:
				tms = 1;
				_state = TEST_LOGIC_RESET;
			}
			break;
			/* DR column */
		case CAPTURE_DR:
			if (newState == SHIFT_DR) {
				tms = 0;
				_state = SHIFT_DR;
			} else {
				tms = 1;
				_state = EXIT1_DR;
			}
			break;
		case SHIFT_DR:
			if (newState == SHIFT_DR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT1_DR;
			}
			break;
		case EXIT1_DR:
			switch (newState) {
			case PAUSE_DR:
			case EXIT2_DR:
			case SHIFT_DR:
			case EXIT1_DR:
				tms = 0;
				_state = PAUSE_DR;
				break;
			default:
				tms = 1;
				_state = UPDATE_DR;
			}
			break;
		case PAUSE_DR:
			if (newState == PAUSE_DR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT2_DR;
			}
			break;
		case EXIT2_DR:
			switch (newState) {
			case SHIFT_DR:
			case EXIT1_DR:
			case PAUSE_DR:
				tms = 0;
				_state = SHIFT_DR;
				break;
			default:
				tms = 1;
				_state = UPDATE_DR;
			}
			break;
		case UPDATE_DR:
		case UPDATE_IR:
			if (newState == RUN_TEST_IDLE) {
				tms = 0;
				_state = RUN_TEST_IDLE;
			} else {
				tms = 1;
				_state = SELECT_DR_SCAN;
			}
			break;
			/* IR column */
		case CAPTURE_IR:
			if (newState == SHIFT_IR) {
				tms = 0;
				_state = SHIFT_IR;
			} else {
				tms = 1;
				_state = EXIT1_IR;
			}
			break;
		case SHIFT_IR:
			if (newState == SHIFT_IR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT1_IR;
			}
			break;
		case EXIT1_IR:
			switch (newState) {
			case PAUSE_IR:
			case EXIT2_IR:
			case SHIFT_IR:
			case EXIT1_IR:
				tms = 0;
				_state = PAUSE_IR;
				break;
			default:
				tms = 1;
				_state = UPDATE_IR;
			}
			break;
		case PAUSE_IR:
			if (newState == PAUSE_IR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT2_IR;
			}
			break;
		case EXIT2_IR:
			switch (newState) {
			case SHIFT_IR:
			case EXIT1_IR:
			case PAUSE_IR:
				tms = 0;
				_state = SHIFT_IR;
				break;
			default:
				tms = 1;
				_state = UPDATE_IR;
			}
			break;
		case UNKNOWN:;
			// UNKNOWN should not be valid...
			throw std::exception();
		}

		setTMS(tms);
		display("%d %d %d %x\n", tms, _num_tms-1, _state,
			_tms_buffer[(_num_tms-1) / 8]);
	}
	/* force write buffer */
	flushTMS(false);
}

const char *Jtag::getStateName(tapState_t s)
{
	switch (s) {
	case TEST_LOGIC_RESET:
		return "TEST_LOGIC_RESET";
	case RUN_TEST_IDLE:
		return "RUN_TEST_IDLE";
	case SELECT_DR_SCAN:
		return "SELECT_DR_SCAN";
	case CAPTURE_DR:
		return "CAPTURE_DR";
	case SHIFT_DR:
		return "SHIFT_DR";
	case EXIT1_DR:
		return "EXIT1_DR";
	case PAUSE_DR:
		return "PAUSE_DR";
	case EXIT2_DR:
		return "EXIT2_DR";
	case UPDATE_DR:
		return "UPDATE_DR";
	case SELECT_IR_SCAN:
		return "SELECT_IR_SCAN";
	case CAPTURE_IR:
		return "CAPTURE_IR";
	case SHIFT_IR:
		return "SHIFT_IR";
	case EXIT1_IR:
		return "EXIT1_IR";
	case PAUSE_IR:
		return "PAUSE_IR";
	case EXIT2_IR:
		return "EXIT2_IR";
	case UPDATE_IR:
		return "UPDATE_IR";
	default:
		return "Unknown";
	}
}
