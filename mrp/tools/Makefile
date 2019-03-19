# Makefile for 'tools' directory

include $(MRP_COMMON_MK_DEFS)

OBJS   := $(patsubst $(SOURCE_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(wildcard $(SOURCE_DIR)/*.cpp))
TARGET := $(OBJ_DIR)/testDecoder
ADDINC := -I$(J2735_DIR)/$(HEADER_DIR) -I/usr/include/cppconn
LINKSO := -Wl,-rpath,$(MRP_SO_DIR) -L$(MRP_SO_DIR) -ldsrc -lasn

all: $(OBJ_DIR) $(OBJS) $(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/%.cpp
	$(MRP_C++) $(MRP_C++FLAGS) $(ADDINC) -c -o $@ $<

$(OBJ_DIR)/testDecoder: $(OBJ_DIR)/testDecoder.o
	$(MRP_C++) $(MRP_C++FLAGS) -o $(OBJ_DIR)/testDecoder $(OBJ_DIR)/testDecoder.o $(LINKSO)

install:
	(mkdir -p $(MRP_EXEC_DIR))
	(cp $(TARGET) $(MRP_EXEC_DIR))
	(cd $(MRP_EXEC_DIR); chmod +x *)

clean:
	rm -f $(OBJS) $(TARGET)