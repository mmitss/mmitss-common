# Makefile for 'mrp' directory

include $(MRP_COMMON_MK_DEFS)

.PHONY: all asn libs tools install

all: asn libs tools

asn:
	(cd $(ASN1_DIR); make clean; make all)

libs:
	(cd $(J2735_DIR); make clean; make all)

tools:
	(cd $(TOOLS_DIR); make clean; make all)

install:
	(cd $(TOOLS_DIR); make install)

