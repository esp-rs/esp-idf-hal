# remove existing blob because otherwise this will append object file to the old blob
Remove-Item -Force riscv_ulp_start.a

riscv32-esp-elf-gcc -Desp_ulp -ggdb3 -fdebug-prefix-map=$(pwd)=/riscv_ulp_start -c -mabi=ilp32 -march=rv32imc riscv_ulp_start.S -o riscv_ulp_start.o
riscv32-esp-elf-ar crs libriscv_ulp_start.a riscv_ulp_start.o

Remove-Item riscv_ulp_start.o
