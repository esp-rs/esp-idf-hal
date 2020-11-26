#!/usr/bin/env bash

set -e

if [ -z "$IDF_PATH" ]; then
    IDF_PATH=".espressif/esp-idf"
    if [ ! -d "$IDF_PATH" ]; then
        mkdir .espressif
        cd .espressif
        git clone -b v4.1 --recursive https://github.com/espressif/esp-idf.git
        cd ..
    fi
fi

: "${TARGET:=xtensa-esp32-none-elf}"

: "${XTENSA_GCC_TOOLCHAIN:=$(dirname $(dirname $(which xtensa-esp32-elf-gcc)))}"
: "${SYSROOT:=$XTENSA_GCC_TOOLCHAIN/xtensa-esp32-elf}"

: "${BINDGEN:=bindgen}"
: "${LIBCLANG_PATH:=$(dirname $(which clang))/../lib}"

CLANG_FLAGS="\
    --sysroot=$SYSROOT \
    -I"$(pwd)" \
    -D__bindgen \
    --target=$TARGET \
    -x c"

CLANG_FLAGS="${CLANG_FLAGS} -I$SYSROOT/sys-include"

# For now: avoid including stuff which resides in .../esp32*/... subdirectories, as these contain definitions which vary from chip to chip
# For example, the interrupt IDs have different numbers in esp32 vs esp32s2
COMPS=$IDF_PATH/components
for INC in $(ls -d "$COMPS"/**/*/include); do
    if [ $INC != *"/esp32"* ]; then
        CLANG_FLAGS="${CLANG_FLAGS} -I$INC"
    fi
done
for INC in $(ls -d "$COMPS"/*/include); do
    if [ $INC != *"/esp32"* ]; then
        CLANG_FLAGS="${CLANG_FLAGS} -I$INC"
    fi
done

generate_bindings()
{
    # --no-rustfmt-bindings because we run rustfmt separately with regular rust
    LIBCLANG_PATH="$LIBCLANG_PATH" \
    "$BINDGEN" \
        --use-core \
        --no-layout-tests \
        --no-rustfmt-bindings \
        --ctypes-prefix c_types \
        $BINDGEN_FLAGS \
        --output src/bindings.rs \
        src/bindings.h \
        -- $CLANG_FLAGS

    rustup run stable rustfmt src/bindings.rs
}

generate_bindings "$@"
