# (c) 2021-2024 Philipp Ruppel

TOOLCHAIN=/opt/riscv32i/bin/riscv32-unknown-elf-

IDBASE=1966080

.SECONDARY:

all:

clean:
	rm -r build

clean_firmware:
	rm -rf build/*firmware*
	rm -rf build/*system*

build/%_firmware.elf: $(wildcard firmware/*.h) $(wildcard firmware/*.hpp) $(wildcard firmware/*.cpp)
	mkdir -p build
	${TOOLCHAIN}c++ \
		-march=rv32i \
		-Os -mtune=size \
		-nostartfiles \
		--std=c++11 \
		-fno-exceptions \
		-ffreestanding \
		-Wl,-Bstatic,-T,firmware/link.lds,--strip-debug \
		-o build/$*_firmware.elf \
		firmware/start.s \
		firmware/main_$*.cpp

build/%_firmware.asm: build/%_firmware.elf
	${TOOLCHAIN}objdump -D -mriscv build/$*_firmware.elf > build/$*_firmware.asm

build/%_firmware.bin: build/%_firmware.elf
	${TOOLCHAIN}objcopy -O binary build/$*_firmware.elf build/$*_firmware.bin

build/%_firmware.mem: build/%_firmware.bin
	python3 3rdparty/picorv32/firmware/makehex.py build/$*_firmware.bin 4096 > build/$*_firmware.mem

build/random.mem: 
	mkdir -p build
	ecpbram -g build/random.mem -s 1 -w 32 -d 4096

build/%_hardware.json: build/random.mem $(wildcard hardware/*.v) $(wildcard hardware/*.sv)
	mkdir -p build


	yosys -p "read_verilog -noautowire hardware/top_$*.v" -p "synth_ecp5 -top top -json build/$*_hardware.json"

build/%_hardware.config: build/%_hardware.json hardware/top_%.lpf hardware/top_%.py
	nextpnr-ecp5 --pre-pack hardware/top_$*.py --timing-allow-fail --speed 6 --freq 200 --lpf hardware/top_$*.lpf --json build/$*_hardware.json --12k --package CABGA256 --textcfg build/$*_hardware.config

build/%_system.config: build/%_firmware.mem build/%_hardware.config build/random.mem 
	ecpbram -f build/random.mem -t build/$*_firmware.mem -i build/$*_hardware.config -o build/$*_system.config

build/%_system.bin: build/%_system.config
	ecppack build/$*_system.config build/$*_system.bin --freq 19.4 --compress
	
stamp_%:
	whiptail --defaultno --yesno "Flashing safety stamp. Are you sure the connected device is a '$*'?" 8 50 && (echo "$*" | md5sum | head -c 32 | ecpprog -o ${IDBASE} -X /dev/stdin)
	ecpprog -R 32 -o ${IDBASE} /dev/stdout | head -c 32
	@echo ""
	
run_%: build/%_system.bin
	echo "testing flash stamp"
	python3 args_equal.py `ecpprog -R 32 -o ${IDBASE} /dev/stdout | head -c 32` `echo "$*" | md5sum | head -c 32` && ecpprog -S build/$*_system.bin

rerun_%:
	echo "testing flash stamp"
	python3 args_equal.py `ecpprog -R 32 -o ${IDBASE} /dev/stdout | head -c 32` `echo "$*" | md5sum | head -c 32` && ecpprog -S build/$*_system.bin

flash_%: build/%_system.bin
	echo "testing flash stamp"
	python3 args_equal.py `ecpprog -R 32 -o ${IDBASE} /dev/stdout | head -c 32` `echo "$*" | md5sum | head -c 32` && ecpprog -a build/$*_system.bin

%_tb:
	mkdir -p build
	iverilog hardware/$@.v -o build/$@ -s top -I./hardware
	build/$@

%_vl:
	mkdir -p build
	verilator --cc --exe hardware/$@.sv hardware/$@.cpp -I./hardware verilated_vcd_c.cpp --trace -CFLAGS "`pkg-config --cflags opencv4`" -LDFLAGS "`pkg-config --libs opencv4`"
	make -C obj_dir -f V$@.mk
	obj_dir/V$@
