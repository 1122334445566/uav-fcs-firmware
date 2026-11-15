#!/usr/bin/env bash
# Zero-dependency build/test driver.
#   ./scripts/build.sh            build unit tests + SIL
#   ./scripts/build.sh test       build and run unit tests
#   ./scripts/build.sh sil        build and run all SIL scenarios
#   ./scripts/build.sh all        everything
#   ./scripts/build.sh clean
#
# Compiler discovery: $CC -> cc -> gcc -> clang -> zig cc (pip install ziglang).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
BUILD="$ROOT/build"
mkdir -p "$BUILD"

find_compiler() {
    if [ -n "${CC:-}" ]; then
        # shellcheck disable=SC2206
        CC_CMD=( $CC ); return
    fi
    for c in cc gcc clang; do
        if command -v "$c" >/dev/null 2>&1; then CC_CMD=( "$c" ); return; fi
    done
    # ziglang ships a full C compiler; call zig.exe directly because the
    # `python -m ziglang` wrapper can mis-resolve ZIG_LIB_DIR on Windows.
    local zig
    zig="$(python -c 'import ziglang,os,sys; print(os.path.join(os.path.dirname(ziglang.__file__), "zig.exe" if os.name=="nt" else "zig"))' 2>/dev/null || true)"
    if [ -n "$zig" ] && [ -x "$zig" ]; then
        CC_CMD=( "$zig" cc ); return
    fi
    echo "No C compiler found. Install gcc/clang, or: pip install ziglang" >&2
    exit 2
}

find_compiler
CFLAGS=( -std=c99 -O2 -Wall -Wextra -Iinclude -Iplatform/posix -Itests )
CORE=(
    src/math/fcs_math.c src/estimation/attitude.c
    src/control/controller.c src/control/mixer.c
    src/core/hal.c src/core/params.c src/core/failsafe.c
    src/core/state_machine.c src/core/telemetry.c src/core/fcs.c
)
TESTS=( tests/test_main.c tests/test_math.c tests/test_attitude.c
        tests/test_control.c tests/test_mixer.c tests/test_safety.c
        tests/test_integration.c )
SIL=( platform/posix/sim_dynamics.c platform/posix/hal_sim.c
      platform/posix/main_sil.c )

case "${1:-all}" in
    clean)
        rm -rf "$BUILD"; echo "cleaned"; exit 0 ;;
    test)
        "${CC_CMD[@]}" "${CFLAGS[@]}" "${CORE[@]}" "${TESTS[@]}" -o "$BUILD/unit_tests"
        "$BUILD/unit_tests"
        ;;
    sil)
        "${CC_CMD[@]}" "${CFLAGS[@]}" "${CORE[@]}" "${SIL[@]}" -o "$BUILD/sil"
        for s in hover waypoint failsafe; do
            "$BUILD/sil" --scenario "$s" --out "$BUILD/$s.csv" | tail -4
        done
        ;;
    all)
        "${CC_CMD[@]}" "${CFLAGS[@]}" "${CORE[@]}" "${TESTS[@]}" -o "$BUILD/unit_tests"
        "${CC_CMD[@]}" "${CFLAGS[@]}" "${CORE[@]}" "${SIL[@]}" -o "$BUILD/sil"
        "$BUILD/unit_tests"
        for s in hover waypoint failsafe; do
            "$BUILD/sil" --scenario "$s" --out "$BUILD/$s.csv" | tail -4
        done
        ;;
    *)
        echo "usage: $0 [all|test|sil|clean]" >&2; exit 2 ;;
esac
