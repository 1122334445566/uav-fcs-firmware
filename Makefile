# Lightweight build for hosts without CMake. Override CC / CFLAGS as needed:
#   make test
#   make sil
#   make CC="zig cc" all
CC      ?= cc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra
INCLUDE  = -Iinclude -Iplatform/posix -Itests
BUILD    = build

ifeq ($(OS),Windows_NT)
LDLIBS ?=
EXE     = .exe
else
LDLIBS ?= -lm
EXE     =
endif

CORE = src/math/fcs_math.c \
       src/estimation/attitude.c \
       src/control/controller.c \
       src/control/mixer.c \
       src/core/hal.c \
       src/core/params.c \
       src/core/failsafe.c \
       src/core/state_machine.c \
       src/core/telemetry.c \
       src/core/fcs.c

SIL  = platform/posix/sim_dynamics.c \
       platform/posix/hal_sim.c \
       platform/posix/main_sil.c

TESTS = tests/test_main.c tests/test_math.c tests/test_attitude.c \
        tests/test_control.c tests/test_mixer.c tests/test_safety.c \
        tests/test_integration.c

.PHONY: all test sil run clean

all: $(BUILD)/unit_tests$(EXE) $(BUILD)/sil$(EXE)

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/unit_tests$(EXE): $(CORE) $(TESTS) | $(BUILD)
	$(CC) $(CFLAGS) $(INCLUDE) $^ -o $@ $(LDLIBS)

$(BUILD)/sil$(EXE): $(CORE) $(SIL) | $(BUILD)
	$(CC) $(CFLAGS) $(INCLUDE) $^ -o $@ $(LDLIBS)

test: $(BUILD)/unit_tests$(EXE)
	./$(BUILD)/unit_tests$(EXE)

sil: $(BUILD)/sil$(EXE)
	./$(BUILD)/sil$(EXE) --scenario hover    --out $(BUILD)/hover.csv
	./$(BUILD)/sil$(EXE) --scenario waypoint --out $(BUILD)/waypoint.csv
	./$(BUILD)/sil$(EXE) --scenario failsafe --out $(BUILD)/failsafe.csv

run: test sil

clean:
	rm -rf $(BUILD)
