#!/bin/bash

source setenv.sh

TARGET_DIR=target/xtensa-none-elf/release

# export V=1
make -j6 app

cargo build --release #--verbose

$IDF_PATH/components/esptool_py/esptool/esptool.py \
	--chip esp32 \
	elf2image \
	--flash_mode "dio" \
	--flash_freq "40m" \
	--flash_size "2MB" \
	-o $TARGET_DIR/esp32-hello.bin \
	$TARGET_DIR/esp32-hello
    
