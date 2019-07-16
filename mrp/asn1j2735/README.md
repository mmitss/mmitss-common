# About
This directory contains C++11 source code which provide library API functions for UPER (Unaligned Packed Encoding Rules) encoding and decoding of the following SAE J2735 messages:
- Basic Safety Message (BSM)
- Signal Phase and Timing Message (SPaT)
- MAP message (MAP)
- Signal Request Message (SRM)
- Signal Status Message (SSM), and
- RTCM position corrections (RTCM)

The standard of SAE J2735 message sets is [version SAE J2735_201603](http://www.sae.org/standardsdev/dsrc/).

The library API functions can be used on a Linux-like processor such as *MMITSS Roadside Processor (MRP)*, as well as on a *roadside unit (RSU)* and/or an *on-board unit (OBU)*. To use the API functions on a *RSU* or an *OBU*, this directory and the directory of `mrp/asn1` need to be built with RSU/OBU vendor's SDK toolchain.

The encoded message frame can be validated using [USDOT Connected Vehicle Message Validator](https://webapp2.connectedvcs.com/validator/).

# Operating System
The SAE J2735 Message Library functions have been tested in
- A PC with Ubuntu 18.04; and
- An Savari OBU with SDK-5.10.1.7.

# Use of UPER Encoding and Decoding API Functions
- Two API functions are provided:
	- `encode_msgFrame` for UPER message encoding; and
	- `decode_msgFrame` for UPER message decoding.
- Definitions of `encode_msgFrame` and `decode_msgFrame` API functions are defined in `AsnJ2735Lib.h`.

## UPER Message Encoding
- `size_t encode_msgFrame(const Frame_element_t& dsrcFrameIn, uint8_t* buf, size_t size)`
- Input
	- User to fill `dsrcFrameIn` data structure (defined in `dsrcFrame.h`); and
	- User to allocate `buf` with length `size`
- Output
	- The encoded byte array are placed and returned in `buf`; and
	- The number of bytes of the encoded byte array (which should be less than `size`. Set `size = 2000` will be sufficient for all messages).

## UPER Message Decoding
- `size_t decode_msgFrame(const uint8_t* buf, size_t size, Frame_element_t& dsrcFrameOut)`
- Input
	- User to provide encoded byte array in `buf` with length `size`; and
	- User to allocate `dsrcFrameOut`
- Output
	- The decoded messages are filled and returned in `dsrcFrameOut`; and
	- The number of bytes in `buf` that have been used for decoding.

## Description of Frame_element_t Data Structure
The definition of internal data structures for BSM, SRM, MAP, SPaT, RTCM, and SSM are defined in `dsrcBSM.h`, `dsrcSRM.h`, `dsrcMapData.h`, `dsrcSPAT.h`, `dsrcRTCM.h`, and `dsrcSSM.h`, respectively. To have an uniform API for UPER encoding and decoding, the data structure `Frame_element_t` is used to include the 6 aforementioned DSRC messages.

	struct Frame_element_t
	{
		uint16_t dsrcMsgId;
		MapData_element_t mapData;
		SPAT_element_t    spat;
		BSM_element_t     bsm;
		RTCM_element_t    rtcm;
		SRM_element_t     srm;
		SSM_element_t     ssm;
		void reset(void)
		{
			dsrcMsgId = MsgEnum::DSRCmsgID_unknown;
			mapData.reset();
			spat.reset();
			bsm.reset();
			rtcm.reset();
			srm.reset();
			ssm.reset();
		};
	};

When encoding a message, using BSM as an example, a user
- declare `uint8_t buf[2000]` to hold the encoded byte array
- declare `struct Frame_element_t dsrcFrameIn`
- fill `dsrcFrameIn.bsm` with user data
- set `dsrcFrameIn.dsrcMsgId = MsgEnum::DSRCmsgID_bsm`
- call `encode_msgFrame`
- `buf` is then filled with the encoded byte array

When decoding a message, using BSM as an example, a user
- declare `struct Frame_element_t dsrcFrameOut`
- call `decode_msgFrame` with the encoded byte array as input
- `dsrcFrameOut` is then filled with the decoded message structure, the type of decoded message is specified by `dsrcFrameOut.dsrcMsgId`.
- DSRC Message ID is defined in `msgEnum.h`.
- Help API functions for printing the encoded message are included as `print_xxxdata`, defined in `AsnJ2735Lib.h`

# Build and Install
- This directory is included in the top level (directory `mrp`) Makefile and does not need to build manually.
- To build this directory manually, run `make clean; make all`.
- After the compilation process, a dynamic shared library (`libdsrc.so`) is created in the `mrp/lib` subdirectory.
- User's applications link the `libdsrc.so` to access the UPER encoding and decoding library API functions.
