CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -pedantic -Iinclude
BUILD_DIR := build

MODEL_SRC := src/i2c_model.c
TEST_SRC := tests/test_i2c_model.c
DEMO_SRC := src/demo.c

TEST_BIN := $(BUILD_DIR)/test_i2c_model
DEMO_BIN := $(BUILD_DIR)/demo

.PHONY: all test demo clean

all: $(TEST_BIN) $(DEMO_BIN)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TEST_BIN): $(MODEL_SRC) $(TEST_SRC) include/i2c_model.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(MODEL_SRC) $(TEST_SRC) -o $(TEST_BIN)

$(DEMO_BIN): $(MODEL_SRC) $(DEMO_SRC) include/i2c_model.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(MODEL_SRC) $(DEMO_SRC) -o $(DEMO_BIN)

test: $(TEST_BIN)
	./$(TEST_BIN)

demo: $(DEMO_BIN)
	./$(DEMO_BIN)

clean:
	rm -rf $(BUILD_DIR)
