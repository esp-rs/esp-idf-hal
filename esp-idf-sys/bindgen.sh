#!/usr/bin/env bash

set -e

. ./setenv.sh

TARGET=xtensa-esp32-none-elf
XTENSA_GCC_TOOLCHAIN="$(dirname $(dirname $(which xtensa-esp32-elf-gcc)))"
COMPS=$IDF_PATH/components

#: "${SYSROOT:=$(xtensa-esp32-elf-gcc --print-sysroot)}"
: "${SYSROOT:=$XTENSA_GCC_TOOLCHAIN/xtensa-esp32-elf}"
: "${BINDGEN:=bindgen}"
#: "${LIBCLANG_PATH:=../llvm-project/llvm/build/lib}"
: "${LIBCLANG_PATH:=$(dirname $(which clang))/../lib}"

CLANG_FLAGS="\
    --sysroot=$SYSROOT \
    -I"$(pwd)" \
    -D__bindgen \
    --target=$TARGET \
    -x c"

CLANG_FLAGS="${CLANG_FLAGS} -I$SYSROOT/sys-include"

for INC in $(ls -d "$COMPS"/**/*/include); do
    CLANG_FLAGS="${CLANG_FLAGS} -I$INC"
done
for INC in $(ls -d "$COMPS"/*/include); do
    CLANG_FLAGS="${CLANG_FLAGS} -I$INC"
done

generate_bindings()
{
    readonly crate="$1"

    cd "$crate"

    # --no-rustfmt-bindings because we run rustfmt separately with regular rust
    LIBCLANG_PATH="$LIBCLANG_PATH" \
    "$BINDGEN" \
        --use-core \
        --no-layout-tests \
        --no-rustfmt-bindings \
        $BINDGEN_FLAGS \
        --output esp-idf-sys/src/bindings.rs \
        esp-idf-sys/src/bindings.h \
        -- $CLANG_FLAGS

    rustup run stable rustfmt esp-idf-sys/src/bindings.rs
}

generate_bindings "$@"
