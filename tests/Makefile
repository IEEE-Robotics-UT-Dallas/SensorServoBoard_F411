# Makefile — Host-side unit tests for SensorServoBoard_F411
# Usage:  make test_all          (build & run everything)
#         make test_shared_data  (build & run shared_data tests)
#         make test_servo_control
#         make test_sensor_drivers
#         make clean

CC      := gcc
CFLAGS  := -Wall -Wextra -g -DTEST_HOST -DMICRO_ROS_ENABLED -DRMW_UXRCE_TRANSPORT_CUSTOM \
           -Wno-unused-parameter

# Directories
TESTS_DIR    := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
PROJECT_ROOT := $(TESTS_DIR)/..
PROJECT_SRC  := $(PROJECT_ROOT)/CubeIDE\ Project/SensorServorBoard/Core/Src
PROJECT_INC  := $(PROJECT_ROOT)/CubeIDE\ Project/SensorServorBoard/Core/Inc
UNITY_DIR    := $(TESTS_DIR)/unity
CMOCK_SRC    := $(TESTS_DIR)/cmock_repo/src
CMOCK_LIB    := $(TESTS_DIR)/cmock_repo/lib
HOST_DIR     := $(TESTS_DIR)/host
MOCK_DIR     := $(TESTS_DIR)/mocks
BUILD_DIR    := $(TESTS_DIR)/build

# Include paths — host/ comes FIRST to intercept stm32f4xx_hal.h, main.h, cmsis_os.h
INCLUDES := -I$(HOST_DIR) \
            -I$(UNITY_DIR) \
            -I$(CMOCK_SRC) \
            -I$(MOCK_DIR) \
            -I$(PROJECT_ROOT)/CubeIDE\ Project/SensorServorBoard/Core/Inc

# Common sources
UNITY_SRC    := $(UNITY_DIR)/unity.c
CMOCK_C_SRC  := $(CMOCK_SRC)/cmock.c
HAL_STUBS    := $(HOST_DIR)/hal_stubs.c

# CMock generation
CMOCK_RB     := $(CMOCK_LIB)/cmock.rb
CMOCK_CFG    := $(TESTS_DIR)/cmock_config.yml
MOCK_HEADER  := $(HOST_DIR)/mock_hal_i2c.h
MOCK_GEN_C   := $(MOCK_DIR)/mock_mock_hal_i2c.c
MOCK_GEN_H   := $(MOCK_DIR)/mock_mock_hal_i2c.h

.PHONY: test_all test_shared_data test_servo_control test_sensor_drivers clean mocks

# ---- Aggregate target ----
test_all: test_shared_data test_servo_control test_sensor_drivers test_memory_manager test_microros_allocators test_microros_time

# ---- Generate CMock mocks ----
$(MOCK_GEN_C) $(MOCK_GEN_H): $(MOCK_HEADER) $(CMOCK_CFG)
	@echo "=== Generating CMock mocks ==="
	cd $(TESTS_DIR) && ruby $(CMOCK_RB) -o$(CMOCK_CFG) $(MOCK_HEADER)

mocks: $(MOCK_GEN_C)

# ---- Build directory ----
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ---- test_shared_data (no mocks) ----
test_shared_data: $(BUILD_DIR)
	@echo "=== Building test_shared_data ==="
	$(CC) $(CFLAGS) $(INCLUDES) -o $(BUILD_DIR)/test_shared_data \
		$(HOST_DIR)/test_shared_data.c \
		"$(PROJECT_ROOT)/CubeIDE Project/SensorServorBoard/Core/Src/shared_data.c" \
		$(UNITY_SRC) \
		$(HAL_STUBS)
	@echo "=== Running test_shared_data ==="
	$(BUILD_DIR)/test_shared_data

# ---- test_servo_control (stub-tracked, no CMock) ----
test_servo_control: $(BUILD_DIR)
	@echo "=== Building test_servo_control ==="
	$(CC) $(CFLAGS) $(INCLUDES) -o $(BUILD_DIR)/test_servo_control \
		$(HOST_DIR)/test_servo_control.c \
		"$(PROJECT_ROOT)/CubeIDE Project/SensorServorBoard/Core/Src/servo_control.c" \
		$(UNITY_SRC) \
		$(HAL_STUBS)
	@echo "=== Running test_servo_control ==="
	$(BUILD_DIR)/test_servo_control

# ---- test_sensor_drivers (CMock mocks) ----
test_sensor_drivers: $(MOCK_GEN_C) $(BUILD_DIR)
	@echo "=== Building test_sensor_drivers ==="
	$(CC) $(CFLAGS) $(INCLUDES) -o $(BUILD_DIR)/test_sensor_drivers \
		$(HOST_DIR)/test_sensor_drivers.c \
		"$(PROJECT_ROOT)/CubeIDE Project/SensorServorBoard/Core/Src/sensor_drivers.c" \
		$(MOCK_GEN_C) \
		$(UNITY_SRC) \
		$(CMOCK_C_SRC) \
		$(HAL_STUBS) \
		-lm
	@echo "=== Running test_sensor_drivers ==="
	$(BUILD_DIR)/test_sensor_drivers

# ---- Clean ----
clean:
	rm -rf $(BUILD_DIR)
	rm -f $(MOCK_DIR)/mock_mock_hal_i2c.c $(MOCK_DIR)/mock_mock_hal_i2c.h

# ---- test_memory_manager (no mocks) ----
test_memory_manager: $(BUILD_DIR)
	@echo "=== Building test_memory_manager ==="
	$(CC) $(CFLAGS) $(INCLUDES) -o $(BUILD_DIR)/test_memory_manager \
		$(HOST_DIR)/test_memory_manager.c \
		"$(PROJECT_ROOT)/CubeIDE Project/SensorServorBoard/Core/Src/custom_memory_manager.c" \
		$(UNITY_SRC) \
		$(HAL_STUBS)
	@echo "=== Running test_memory_manager ==="
	$(BUILD_DIR)/test_memory_manager

# ---- test_microros_allocators (needs custom_memory_manager too) ----
test_microros_allocators: $(BUILD_DIR)
	@echo "=== Building test_microros_allocators ==="
	$(CC) $(CFLAGS) $(INCLUDES) -o $(BUILD_DIR)/test_microros_allocators \
		$(HOST_DIR)/test_microros_allocators.c \
		"$(PROJECT_ROOT)/CubeIDE Project/SensorServorBoard/Core/Src/microros_allocators.c" \
		"$(PROJECT_ROOT)/CubeIDE Project/SensorServorBoard/Core/Src/custom_memory_manager.c" \
		$(UNITY_SRC) \
		$(HAL_STUBS)
	@echo "=== Running test_microros_allocators ==="
	$(BUILD_DIR)/test_microros_allocators

# ---- test_microros_time (rename clock_gettime to avoid libc conflict) ----
test_microros_time: $(BUILD_DIR)
	@echo "=== Building test_microros_time ==="
	$(CC) $(CFLAGS) -Dclock_gettime=microros_clock_gettime $(INCLUDES) \
		-o $(BUILD_DIR)/test_microros_time \
		$(HOST_DIR)/test_microros_time.c \
		"$(PROJECT_ROOT)/CubeIDE Project/SensorServorBoard/Core/Src/microros_time.c" \
		$(UNITY_SRC) \
		$(HAL_STUBS)
	@echo "=== Running test_microros_time ==="
	$(BUILD_DIR)/test_microros_time
