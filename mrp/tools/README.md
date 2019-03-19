# About
This directory contains an example program (i.e., `testDecoder`) for testing UPER encoding and decoding using the SAE J2735 Message Library.

# Build and Install

## On Ubuntu 18.04 PC
- Run `make clean; make all; make install`.
- After the compilation process, the `testDecoder` binary executable is created in the `mrp/bin` subdirectory.

## On Ubuntu 18.04 Virtual Machine with Savri SDK-5.10.1.7 
- Run `make -f MakeV2X clean; make -f MakeV2X all; make -f MakeV2X install`.
- After the compilation process, the `testDecoder` binary executable is created in the `savari/bin` subdirectory.

# Usage
	./testDecoder -s <BSM|RSM|SSM|SPaT>
	
# Outputs
Output data files created by `testDecoder` are in `output` subdirectory:
- Directory `mrp` contain outputs on a Ubuntu 18.04 PC; and
- Directory `savari` contain outputs on an Savari OBU.
	