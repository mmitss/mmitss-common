# Makefile for 'asn1' directory

include $(MRP_COMMON_MK_DEFS)

ASN_CFLAGS := -Wno-missing-field-initializers
LIBNAME    := libasn.so
VERSION    := 0
SONAME     := $(LIBNAME).1
SOFLAGS    := -shared -fPIC -Wl,-soname,$(SONAME)
TARGET     := $(LIB_DIR)/$(SONAME).$(VERSION)
OBJS       := $(patsubst $(SOURCE_DIR)/%.c,$(OBJ_DIR)/%.o,$(wildcard $(SOURCE_DIR)/*.c))

all: $(OBJ_DIR) $(LIB_DIR) $(OBJS) $(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(LIB_DIR):
	mkdir -p $(LIB_DIR)

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/%.c
	$(MRP_CC) $(MRP_CFLAGS) $(ASN_CFLAGS) -c -o $@ $<

$(TARGET): $(OBJS)
	($(MRP_CC) $(SOFLAGS) -o $(TARGET) $(OBJS))
	(mkdir -p $(MRP_SO_DIR); cp $(TARGET) $(MRP_SO_DIR))
	(cd $(MRP_SO_DIR); rm -f $(SONAME); ln -s $(SONAME).$(VERSION) $(SONAME))
	(cd $(MRP_SO_DIR); rm -f $(LIBNAME); ln -s $(SONAME).$(VERSION) $(LIBNAME))

clean:
	rm -f $(OBJS) $(TARGET)
