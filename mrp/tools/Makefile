# Makefile for 'tools' directory

include $(MRP_COMMON_MK_DEFS)

OBJS   := $(patsubst $(SOURCE_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(wildcard $(SOURCE_DIR)/*.cpp))
TARGET := $(OBJ_DIR)/testDecoder $(OBJ_DIR)/testMapData
ADDINC := -I$(J2735_DIR)/$(HEADER_DIR) -I$(MAPENGINE_DIR)/$(HEADER_DIR)
LINKSO := -Wl,-rpath,$(MRP_SO_DIR) -L$(MRP_SO_DIR) -llocAware -ldsrc -lasn

all: $(OBJ_DIR) $(OBJS) $(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: $(SOURCE_DIR)/%.cpp
	$(MRP_C++) $(MRP_C++FLAGS) $(ADDINC) -c -o $@ $<

$(OBJ_DIR)/testDecoder: $(OBJ_DIR)/testDecoder.o
	$(MRP_C++) $(MRP_C++FLAGS) -o $(OBJ_DIR)/testDecoder $(OBJ_DIR)/testDecoder.o $(LINKSO)

$(OBJ_DIR)/testMapData: $(OBJ_DIR)/testMapData.o
	$(MRP_C++) $(MRP_C++FLAGS) -o $(OBJ_DIR)/testMapData $(OBJ_DIR)/testMapData.o $(LINKSO)

install:
	(mkdir -p $(MRP_EXEC_DIR))
	(cp $(TARGET) $(MRP_EXEC_DIR))
	(cd $(MRP_EXEC_DIR); chmod +x *)

clean:
	rm -f $(OBJS) $(TARGET)