#!/usr/bin/env bash

set -e

if [ -z "$IDF_VERSION" ]; then
    IDF_VERSION="v4.1"
fi

if [ -z "$ESP8266_RTOS_VERSION" ]; then
    ESP8266_RTOS_VERSION="v3.3"
fi

if [ -z "$IDF_PATH" ]; then
    IDF_PATH=".espressif/esp-idf"
    if [ ! -d "$IDF_PATH" ]; then
        mkdir -p .espressif
        cd .espressif
        git clone -b $IDF_VERSION --recursive https://github.com/espressif/esp-idf.git
        cd ..
    fi
fi

if [ -z "$ESP8266_RTOS_PATH" ]; then
    ESP8266_RTOS_PATH=".espressif/ESP8266_RTOS_SDK"
    if [ ! -d "$ESP8266_RTOS_PATH" ]; then
        mkdir -p .espressif
        cd .espressif
        git clone -b $ESP8266_RTOS_VERSION --recursive https://github.com/espressif/ESP8266_RTOS_SDK.git
        cd ..
    fi
fi

generate_bindings()
{
    IDF_COMPS=$1/components
    IDF_INCLUDES=""

    if [ $2 == "esp8266" ]; then
        # The only thing that works for the ESP8266 SDK is to list every directory as include directory
        for INC in $(find "$IDF_COMPS" -type d | grep -v $6); do
            IDF_INCLUDES="${IDF_INCLUDES} -I$INC"
        done
    else
        for INC in $(ls -d "$IDF_COMPS"/*/include | grep -v $6); do
            IDF_INCLUDES="${IDF_INCLUDES} -I$INC"
        done
        for INC in $(ls -d "$IDF_COMPS"/**/*/include | grep -v $6); do
            IDF_INCLUDES="${IDF_INCLUDES} -I$INC"
        done
    fi

    BINDGEN="bindgen"
    LIBCLANG_PATH="$(dirname $(which clang))/../lib"

    XTENSA_GCC_TOOLCHAIN="$(dirname $(dirname $(which $4)))"
    SYSROOT="$XTENSA_GCC_TOOLCHAIN/$5"

    CLANG_FLAGS="\
        -D__bindgen \
        -x c \
        --target=$3 \
        --sysroot=$SYSROOT \
        -I$SYSROOT/sys-include \
        -I$(pwd)/src/idf-target/$2 \
        $IDF_INCLUDES"

    # --no-rustfmt-bindings because we run rustfmt separately with regular rust
    LIBCLANG_PATH="$LIBCLANG_PATH" \
    "$BINDGEN" \
        --use-core \
        --no-layout-tests \
        --no-rustfmt-bindings \
        --ctypes-prefix c_types \
        $BINDGEN_FLAGS \
        --output src/bindings_$2.rs \
        src/idf-target/$2/bindings.h \
        -- $CLANG_FLAGS

    rustup run stable rustfmt "src/bindings_$2.rs"
}

generate_bindings $IDF_PATH "esp32" "xtensa-esp32-none-elf" "xtensa-esp32-elf-gcc" "xtensa-esp32-elf" "/esp32s2beta/"
generate_bindings $ESP8266_RTOS_PATH "esp8266" "xtensa-esp8266-none-elf" "xtensa-lx106-elf-gcc" "xtensa-lx106-elf" "/esp32/\|/esp32s2beta/"

# Future
#generate_bindings $IDF_PATH "esp32s2" "xtensa-esp32s2-none-elf" "xtensa-esp32s2-elf-gcc" "xtensa-esp32s2-elf" "/esp32/\|/esp32s3/\|/esp32c3/"
#generate_bindings $IDF_PATH "esp32s3" "xtensa-esp32s3-none-elf" "xtensa-esp32s3-elf-gcc" "xtensa-esp32s3-elf" "/esp32/\|/esp32s2/\|/esp32c3/"
#generate_bindings $IDF_PATH "esp32c3" "riscv32-esp32c3-none-elf" "riscv32-esp32c3-elf-gcc" "riscv32-esp32c3-elf" "/esp32/\|/esp32s3/\|/esp32s3/"
