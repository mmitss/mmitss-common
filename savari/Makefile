# Makefile for 'savari' directory

include $(SAVARI_COMMON_MK_DEFS)

.PHONY: all asn libs tools install

all: asn libs tools

asn:
	(cd $(ASN1_DIR); make -f MakeV2X clean; make -f MakeV2X all)

libs:
	(cd $(J2735_DIR); make -f MakeV2X clean; make -f MakeV2X all)

tools:
	(cd $(TOOLS_DIR); make -f MakeV2X clean; make -f MakeV2X all)

install:
	(cd $(TOOLS_DIR); make -f MakeV2X install)
