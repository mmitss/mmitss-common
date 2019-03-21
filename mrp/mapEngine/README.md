# About
This directory contains C++11 source code which provide library API functions for MAP Engine, including:
- Read intersection geographic description file (*.nmap*) and store MAP structure in memory (`readNmap`);
- Read pre-encoded intersection MAP paylod file (*.payload*) and store MAP structure in memory (`readPayload`);
- Add new intersection MAP to MAP structure in memory (used on an OBU)(`checkNmapUpdate` and `addIntersection`);
- Save MAP structure in memory to *.nmap* file (used on an OBU) (`saveNmap`);
- Locate a vehicle on MAP (determining the active MAP and lane of travel) (`locateVehicleInMap`);
- Update intersection location awareness (`updateLocationAware`); and
- Calculate distance to the stop-bar (`getPtDist2D`).

The library API functions can be used on a Linux-like processor such as *MMITSS Roadside Processor (MRP)*, as well as on a *roadside unit (RSU)* and/or an *on-board unit (OBU)*. To use the API functions on a *RSU* or an *OBU*, this directory needs to be built with RSU/OBU vendor's SDK toolchain.

# Intersection Geographic Description File (*.nmap*)
The over-the-air MAP message is encoded based on an *.nmap* file that stores the geographic description of an intersection. The SAE J2735 Message Library in directory `mrp/asn1j2735` is utilized to encode the MAP payload.
- The specification of the *.nmap* text file is described in [nmap file format](Format_of_nmap.pdf).
- Sample *.nmap* and *.payload* files are contained in the `mrp/tools/nmap` subdirectory:
	- [namp of Page Mill Rd at El Camino Real, Palo Alto, CA](../tools/nmap/ecr-page-mill.nmap).
	- [Encoded MAP payload Page Mill Rd at El Camino Real, Palo Alto, CA](../tools/nmap/ecr-page-mill.map.payload).

# Build and Install
- This directory is included in the top level (directory `mrp`) Makefile and does not need to build manually.
- To build this directory manually, run `make clean; make all`.
- After the compilation process, a dynamic shared library (`liblocAware.so`) is created in the `mrp/lib` subdirectory.
- User's applications link the `liblocAware.so` to access the MAP Engine API functions.
