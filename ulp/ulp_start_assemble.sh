#!/bin/bash

set -euxo pipefail

# remove existing blob because otherwise this will append object file to the old blob
rm -f ulp_start.a

riscv32-esp-elf-gcc -Desp_ulp -ggdb3 -fdebug-prefix-map=$(pwd)=/ulp_start -c -mabi=ilp32 -march=rv32imc ulp_start.S -o ulp_start.o
riscv32-esp-elf-ar crs libulp_start.a ulp_start.o

rm ulp_start.o
