#!/usr/bin/env bash

set -e

. ./setenv.sh

COMPS=$IDF_PATH/components
: "${SYSROOT:=$(xtensa-esp32-elf-gcc --print-sysroot)}"
TARGET=xtensa-esp32-none-elf

: "${BINDGEN:=bindgen}"
: "${LIBCLANG_PATH:=../llvm-project/llvm/build/lib}"
CLANG_FLAGS="\
    --sysroot=$SYSROOT \
    -I"$(pwd)" \
    -D__bindgen \
    --target=$TARGET \
    -x c"

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

    LIBCLANG_PATH="$LIBCLANG_PATH" \
    "$BINDGEN" \
        --use-core \
        --no-layout-tests \
        $BINDGEN_FLAGS \
        --output src/bindings.rs \
        src/bindings.h \
        -- $CLANG_FLAGS

    rustup run stable rustfmt src/bindings.rs
}

generate_bindings "$@"
