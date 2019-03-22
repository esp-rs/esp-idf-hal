#!/bin/bash

# problem:- the core crate is not using the passed rustflags, temp work around copy the output of -v, and add the correct emit flags for the core crate

source setenv.sh

TARGET_DIR=target/xtensa-none-elf/release

# export V=1
make -j6 app

rustup run xtensa-1.34 \
    cargo xbuild --release #--verbose

$IDF_PATH/components/esptool_py/esptool/esptool.py \
	--chip esp32 \
	elf2image \
	--flash_mode "dio" \
	--flash_freq "40m" \
	--flash_size "2MB" \
	-o $TARGET_DIR/esp32-hello.bin \
	$TARGET_DIR/esp32-hello
    
