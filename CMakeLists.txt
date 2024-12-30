cmake_minimum_required(VERSION 3.5)

# set the project name
project(openFPGALoader VERSION "0.13.1" LANGUAGES CXX)
add_definitions(-DVERSION=\"v${PROJECT_VERSION}\")

option(ENABLE_OPTIM "Enable build with -O3 optimization level" ON)
option(BUILD_STATIC "Whether or not to build with static libraries" OFF)
if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(ENABLE_UDEV OFF)
else()
	option(ENABLE_UDEV "use udev to search JTAG adapter from /dev/xx" ON)
endif()
option(ENABLE_CMSISDAP "enable cmsis DAP interface (requires hidapi)" ON)
option(ENABLE_GOWIN_GWU2X "enable Gowin GWU2X interface" ON)
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	option(ENABLE_LIBGPIOD "enable libgpiod bitbang driver (requires libgpiod)" ON)
	option(ENABLE_REMOTEBITBANG "enable remote bitbang driver" ON)
else()
	set(ENABLE_LIBGPIOD OFF)
	set(ENABLE_REMOTEBITBANG OFF)
endif()
option(USE_PKGCONFIG "Use pkgconfig to find libraries" ON)
option(LINK_CMAKE_THREADS "Use CMake find_package to link the threading library" OFF)
set(BLASTERII_PATH "" CACHE STRING "usbBlasterII firmware directory")
set(ISE_PATH "/opt/Xilinx/14.7" CACHE STRING "ise root directory (default: /opt/Xilinx/14.7)")

## specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra ${CMAKE_CXX_FLAGS_DEBUG}")
if (ENABLE_OPTIM)
	set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif()

if (BUILD_STATIC)
	set(CMAKE_EXE_LINKER_FLAGS "-static-libstdc++ -static ${CMAKE_EXE_LINKER_FLAGS}")
	set(BUILD_SHARED_LIBS OFF)
endif()

include(GNUInstallDirs)
# By default: DATA_DIR="/usr/local/share"
add_definitions(-DDATA_DIR=\"${CMAKE_INSTALL_FULL_DATAROOTDIR}\")

add_definitions(-DISE_DIR=\"${ISE_PATH}\")
add_definitions(-DBLASTERII_DIR=\"${BLASTERII_PATH}\")

if (USE_PKGCONFIG)
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(LIBFTDI REQUIRED libftdi1)
	pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
	pkg_check_modules(HIDAPI hidapi-libusb)
	# if libusb not found try with hidraw
	if (NOT HIDAPI_FOUND)
		pkg_check_modules(HIDAPI hidapi-hidraw)
	endif()
	if (NOT HIDAPI_FOUND)
		pkg_check_modules(HIDAPI hidapi)
	endif()
	# zlib support (gzip)
	pkg_check_modules(ZLIB zlib)
	if (NOT ZLIB_FOUND)
		# try zlib-ng
		pkg_check_modules(ZLIB zlib-ng)
		if (ZLIB_FOUND)
			add_definitions(-DHAS_ZLIBNG)
		else()
			message(FATAL_ERROR "Could find ZLIB")
		endif()
	endif(NOT ZLIB_FOUND)

	if (ENABLE_UDEV)
		pkg_check_modules(LIBUDEV libudev)
		if (LIBUDEV_FOUND)
			add_definitions(-DUSE_UDEV)
		else()
			message("libudev not found, disabling udev support and -D parameter")
			set(ENABLE_UDEV OFF)
		endif()
	endif()

	if (ENABLE_LIBGPIOD)
		pkg_check_modules(LIBGPIOD libgpiod)
		if (NOT LIBGPIOD_FOUND)
			message("libgpiod not found, disabling gpiod support")
			set(ENABLE_LIBGPIOD OFF)
		endif()
	endif()
endif()

set(OPENFPGALOADER_SOURCE
	src/anlogic.cpp
	src/anlogicBitParser.cpp
	src/anlogicCable.cpp
	src/ch552_jtag.cpp
	src/common.cpp
	src/dfu.cpp
	src/dfuFileParser.cpp
	src/dirtyJtag.cpp
	src/ch347jtag.cpp
	src/efinix.cpp
	src/efinixHexParser.cpp
	src/fx2_ll.cpp
	src/ice40.cpp
	src/ihexParser.cpp
	src/pofParser.cpp
	src/rawParser.cpp
	src/spiFlash.cpp
	src/spiInterface.cpp
	src/usbBlaster.cpp
	src/epcq.cpp
	src/svf_jtag.cpp
	src/jedParser.cpp
	src/feaparser.cpp
	src/display.cpp
	src/jtag.cpp
	src/ftdiJtagBitbang.cpp
	src/ftdiJtagMPSSE.cpp
	src/configBitstreamParser.cpp
	src/ftdipp_mpsse.cpp
	src/main.cpp
	src/latticeBitParser.cpp
	src/libusb_ll.cpp
	src/gowin.cpp
	src/device.cpp
	src/jlink.cpp
	src/lattice.cpp
	src/progressBar.cpp
	src/fsparser.cpp
	src/mcsParser.cpp
	src/ftdispi.cpp
	src/altera.cpp
	src/bitparser.cpp
	src/xilinx.cpp
	src/xilinxMapParser.cpp
	src/colognechip.cpp
	src/colognechipCfgParser.cpp
	src/esp_usb_jtag.cpp
)

set(OPENFPGALOADER_HEADERS
	src/altera.hpp
	src/anlogic.hpp
	src/anlogicBitParser.hpp
	src/anlogicCable.hpp
	src/ch552_jtag.hpp
	src/common.hpp
	src/cxxopts.hpp
	src/dfu.hpp
	src/dfuFileParser.hpp
	src/dirtyJtag.hpp
	src/ch347jtag.hpp
	src/efinix.hpp
	src/efinixHexParser.hpp
	src/fx2_ll.hpp
	src/ice40.hpp
	src/ihexParser.hpp
	src/pofParser.hpp
	src/progressBar.hpp
	src/rawParser.hpp
	src/usbBlaster.hpp
	src/bitparser.hpp
	src/ftdiJtagBitbang.hpp
	src/ftdiJtagMPSSE.hpp
	src/jlink.hpp
	src/jtag.hpp
	src/jtagInterface.hpp
	src/libusb_ll.hpp
	src/fsparser.hpp
	src/part.hpp
	src/board.hpp
	src/jedParser.hpp
	src/feaparser.hpp
	src/display.hpp
	src/mcsParser.hpp
	src/ftdipp_mpsse.hpp
	src/spiFlash.hpp
	src/spiFlashdb.hpp
	src/epcq.hpp
	src/spiInterface.hpp
	src/svf_jtag.hpp
	src/configBitstreamParser.hpp
	src/device.hpp
	src/gowin.hpp
	src/cable.hpp
	src/ftdispi.hpp
	src/lattice.hpp
	src/latticeBitParser.hpp
	src/xilinx.hpp
	src/xilinxMapParser.hpp
	src/colognechip.hpp
	src/colognechipCfgParser.hpp
	src/esp_usb_jtag.hpp
)

link_directories(
	${LIBUSB_LIBRARY_DIRS}
	${LIBFTDI_LIBRARY_DIRS}
)

if (ENABLE_LIBGPIOD)
	link_directories(${LIBGPIOD_LIBRARY_DIRS})
endif()

if (ENABLE_CMSISDAP AND HIDAPI_FOUND)
	link_directories(${HIDAPI_LIBRARY_DIRS})
endif()

add_executable(openFPGALoader
	${OPENFPGALOADER_SOURCE}
	${OPENFPGALOADER_HEADERS}
)

include_directories(
	${LIBUSB_INCLUDE_DIRS}
	${LIBFTDI_INCLUDE_DIRS}
)

target_link_libraries(openFPGALoader
	${LIBUSB_LIBRARIES}
	${LIBFTDI_LIBRARIES}
)

# Gowin GWU2X JTAG interface
if(ENABLE_GOWIN_GWU2X)
	target_sources(openFPGALoader PRIVATE src/gwu2x_jtag.cpp)
	list (APPEND OPENFPGALOADER_HEADERS src/gwu2x_jtag.hpp)
	add_definitions(-DENABLE_GOWIN_GWU2X=1)
	message("Gowin GWU2X support enabled")
else()
	message("Gowin GWU2X support disabled")
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	# winsock provides ntohs
	target_link_libraries(openFPGALoader ws2_32)

	target_sources(openFPGALoader PRIVATE src/pathHelper.cpp)
	list(APPEND OPENFPGALOADER_HEADERS src/pathHelper.hpp)
endif()

# libusb_attach_kernel_driver is only available on Linux.
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	add_definitions(-DATTACH_KERNEL)
endif()

if (ENABLE_UDEV)
	include_directories(${LIBUDEV_INCLUDE_DIRS})
	target_link_libraries(openFPGALoader ${LIBUDEV_LIBRARIES})
endif()

if (ENABLE_LIBGPIOD)
	include_directories(${LIBGPIOD_INCLUDE_DIRS})
	target_link_libraries(openFPGALoader ${LIBGPIOD_LIBRARIES})
	add_definitions(-DENABLE_LIBGPIOD=1)
	target_sources(openFPGALoader PRIVATE src/libgpiodJtagBitbang.cpp)
	list (APPEND OPENFPGALOADER_HEADERS src/libgpiodJtagBitbang.hpp)
	if (LIBGPIOD_VERSION VERSION_GREATER_EQUAL 2)
		message("libgpiod v2 support enabled")
		add_definitions(-DGPIOD_APIV2)
	else()
		message("libgpiod v1 support enabled")
	endif()
endif(ENABLE_LIBGPIOD)

if (ENABLE_JETSONNANOGPIO)
	add_definitions(-DENABLE_JETSONNANOGPIO=1)
	target_sources(openFPGALoader PRIVATE src/jetsonNanoJtagBitbang.cpp)
	list (APPEND OPENFPGALOADER_HEADERS src/jetsonNanoJtagBitbang.hpp)
	message("Jetson Nano GPIO support enabled")
endif(ENABLE_JETSONNANOGPIO)

if (ENABLE_UDEV OR ENABLE_LIBGPIOD OR ENABLE_JETSONNANOGPIO)
	add_definitions(-DUSE_DEVICE_ARG)
endif(ENABLE_UDEV OR ENABLE_LIBGPIOD OR ENABLE_JETSONNANOGPIO)

if (BUILD_STATIC)
	set_target_properties(openFPGALoader PROPERTIES LINK_SEARCH_END_STATIC 1)
endif()

if (ENABLE_CMSISDAP)
	if (HIDAPI_FOUND)
		include_directories(${HIDAPI_INCLUDE_DIRS})
		target_link_libraries(openFPGALoader ${HIDAPI_LIBRARIES})
		add_definitions(-DENABLE_CMSISDAP=1)
		target_sources(openFPGALoader PRIVATE src/cmsisDAP.cpp)
		list (APPEND OPENFPGALOADER_HEADERS src/cmsisDAP.hpp)
		message("cmsis_dap support enabled")
	else()
		message("hidapi-libusb not found: cmsis_dap support disabled")
	endif()
endif(ENABLE_CMSISDAP)

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	add_definitions(-DENABLE_XVC=1)
	target_sources(openFPGALoader PRIVATE src/xvc_client.cpp src/xvc_server.cpp)
	list (APPEND OPENFPGALOADER_HEADERS src/xvc_client.hpp src/xvc_server.hpp)
	set(CMAKE_EXE_LINKER_FLAGS "-pthread ${CMAKE_EXE_LINKER_FLAGS}")
	message("Xilinx Virtual Server support enabled")
else()
		message("Xilinx Virtual Server support disabled")
endif()

if (ENABLE_REMOTEBITBANG)
	add_definitions(-DENABLE_REMOTEBITBANG=1)
	target_sources(openFPGALoader PRIVATE src/remoteBitbang_client.cpp)
	list (APPEND OPENFPGALOADER_HEADERS src/remoteBitbang_client.hpp)
	message("Remote bitbang client support enabled")
else()
	message("Remote bitbang client support disabled")
endif()

if (ZLIB_FOUND)
	include_directories(${ZLIB_INCLUDE_DIRS})
	target_link_libraries(openFPGALoader ${ZLIB_LIBRARIES})
	add_definitions(-DHAS_ZLIB=1)
else()
	message("zlib library not found: can't flash intel/altera devices")
endif()

if (LINK_CMAKE_THREADS)
	find_package(Threads REQUIRED)
	target_link_libraries(openFPGALoader Threads::Threads)
endif()

# libftdi < 1.4 as no usb_addr
# libftdi >= 1.5 as purge_buffer obsolete
string(REPLACE "." ";" VERSION_LIST ${LIBFTDI_VERSION})
list(GET VERSION_LIST 0 LIBFTDI_VERSION_MAJOR)
list(GET VERSION_LIST 1 LIBFTDI_VERSION_MINOR)
math(EXPR FTDI_VAL "${LIBFTDI_VERSION_MAJOR} * 100 + ${LIBFTDI_VERSION_MINOR}")

add_definitions(-DFTDI_VERSION=${FTDI_VAL})

install(TARGETS openFPGALoader DESTINATION bin)

file(GLOB GZ_FILES spiOverJtag/spiOverJtag_*.*.gz)

# Compress rbf and bit files present into repository
# TODO: test compat with Windows and MacOS
list(INSERT CMAKE_MODULE_PATH 0 ${PROJECT_SOURCE_DIR}/cmake/Modules)
include(FindGZIP)

if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows" AND GZIP_PRG)
	set(SPIOVERJTAG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/spiOverJtag")
	file(GLOB BITS_FILES RELATIVE ${SPIOVERJTAG_DIR} spiOverJtag/spiOverJtag_*.bit)
	file(GLOB RBF_FILES RELATIVE ${SPIOVERJTAG_DIR} spiOverJtag/spiOverJtag_*.rbf)

	STRING(REGEX REPLACE ".bit" ".bit.gz" BIT_GZ_FILES "${BITS_FILES}")
	STRING(REGEX REPLACE ".rbf" ".rbf.gz" RBF_GZ_FILES "${RBF_FILES}")

	FOREACH(bit ${BITS_FILES} ${RBF_FILES})
		ADD_CUSTOM_COMMAND(OUTPUT ${bit}.gz
			COMMAND ${GZIP_PRG} -9 -c ${bit} > ${CMAKE_CURRENT_BINARY_DIR}/${bit}.gz
			DEPENDS ${SPIOVERJTAG_DIR}/${bit}
			WORKING_DIRECTORY ${SPIOVERJTAG_DIR}
			COMMENT "Building ${bit}.gz")
		list(APPEND GZ_FILES ${CMAKE_CURRENT_BINARY_DIR}/${bit}.gz)
	ENDFOREACH(bit)

	ADD_CUSTOM_TARGET(bit ALL DEPENDS ${BIT_GZ_FILES} ${RBF_GZ_FILES})
else()
	file(GLOB BITS_FILES spiOverJtag/spiOverJtag_*.bit)
	file(GLOB RBF_FILES spiOverJtag/spiOverJtag_*.rbf)
	install(FILES
		${BITS_FILES}
		${RBF_FILES}
		DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/openFPGALoader
	)
endif()

install(FILES
	${GZ_FILES}
	DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/openFPGALoader
)
