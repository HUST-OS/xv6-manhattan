# platform	:= k210
platform	:= qemu
mode 		:= debug
# mode		:= release

K := kernel
U := xv6-user
T := target
BUILD_DIR := build

# config toolchain
# TOOLPREFIX 	:= riscv64-unknown-elf-
TOOLPREFIX	:= riscv64-linux-gnu-
CC 		:= $(TOOLPREFIX)gcc
AS		:= $(TOOLPREFIX)gas
LD		:= $(TOOLPREFIX)ld
OBJCOPY	:= $(TOOLPREFIX)objcopy
OBJDUMP	:= $(TOOLPREFIX)objdump

QEMU	:= qemu-system-riscv64

# flags
CFLAGS = -Wall -Werror -O -fno-omit-frame-pointer -ggdb -g
CFLAGS += -MD
CFLAGS += -mcmodel=medany
CFLAGS += -ffreestanding -fno-common -nostdlib -mno-relax
CFLAGS += -Iinclude/
CFLAGS += $(shell $(CC) -fno-stack-protector -E -x c /dev/null >/dev/null 2>&1 && echo -fno-stack-protector)

ifeq ($(mode), debug) 
CFLAGS += -DDEBUG 
CFLAGS += $(addprefix "-D__DEBUG_",$(module))
endif 

ifeq ($(platform), qemu)
CFLAGS += -D QEMU
endif

LDFLAGS = -z max-page-size=4096

ifeq ($(platform), k210)
linker = ./linker/k210.ld
endif

ifeq ($(platform), qemu)
linker = ./linker/qemu.ld
endif

# RustSBI
ifeq ($(platform), k210)
	RUSTSBI := ./sbi/sbi-k210
else
	RUSTSBI	:= ./sbi/sbi-qemu
endif

# QEMU 
CPUS := 2

QEMUOPTS = -machine virt -kernel $T/kernel -m 8M -nographic

# use multi-core 
QEMUOPTS += -smp $(CPUS)

QEMUOPTS += -bios $(RUSTSBI)

# import virtual disk image
QEMUOPTS += -drive file=fs.img,if=none,format=raw,id=x0 
QEMUOPTS += -device virtio-blk-device,drive=x0,bus=virtio-mmio-bus.0

k210-serialport := /dev/ttyUSB0

# entry file 
ifeq ($(platform), k210) 
SRC := $K/entry_k210.S
else 
SRC := $K/entry_qemu.S
endif 

SRC	+= \
	$K/printf.c \
	$K/sprintf.c \
	$K/console.c \
	$K/exec.c \
	$K/intr.c \
	$K/logo.c \
	$K/main.c \
	$K/timer.c \
	$K/uname.c \
	$K/fs/bio.c \
	$K/fs/blkdev.c \
	$K/fs/fat32/cluster.c \
	$K/fs/fat32/dirent.c \
	$K/fs/fat32/fat.c \
	$K/fs/fat32/fat32.c \
	$K/fs/file.c \
	$K/fs/fs.c \
	$K/fs/mount.c \
	$K/fs/pipe.c \
	$K/fs/poll.c \
	$K/fs/rootfs.c \
	$K/mesg/signal.c \
	$K/mm/kmalloc.c \
	$K/mm/mmap.c \
	$K/mm/pm.c \
	$K/mm/usrmm.c \
	$K/mm/vm.c \
	$K/sched/proc.c \
	$K/sched/swtch.S \
	$K/sync/sleeplock.c \
	$K/sync/spinlock.c \
	$K/syscall/syscall.c \
	$K/syscall/sysfile.c \
	$K/syscall/sysproc.c \
	$K/syscall/syssignal.c \
	$K/syscall/systime.c \
	$K/syscall/sysuname.c \
	$K/trap/kernelvec.S \
	$K/trap/trampoline.S \
	$K/trap/trap.c \
	$K/utils/list.c \
	$K/utils/rbtree.c \
	$K/utils/string.c \
	$K/hal/plic.c \
	$K/hal/disk.c

ifeq ($(platform), k210) 
SRC += \
	$K/hal/spi.c \
	$K/hal/gpiohs.c \
	$K/hal/fpioa.c \
	$K/hal/sdcard.c \
	$K/hal/dmac.c \
	$K/hal/sysctl.c \
	$K/utils/utils.c
else 
SRC += \
	$K/hal/virtio_disk.c
endif 

# linker script 
ifeq ($(platform), k210) 
linker = ./linker/k210.ld
else 
linker = ./linker/qemu.ld
endif 

# object files 
OBJ := $(basename $(SRC))
OBJ := $(addsuffix .o, $(OBJ))


# Generate binary file to burn onto k210
all: $T/kernel $(RUSTSBI) 
ifeq ($(platform), k210) 
	@$(OBJCOPY) $T/kernel --strip-all -O binary $T/kernel.bin
	@$(OBJCOPY) $(RUSTSBI) --strip-all -O binary $T/k210.bin
	@dd if=$T/kernel.bin of=$T/k210.bin bs=128k seek=1
	cp $T/k210.bin ./k210.bin
endif 

# Compile Kernel
$T/kernel: $(OBJ) 
	@if [ ! -d "$T" ]; then mkdir $T; fi
	@$(LD) $(LDFLAGS) -T $(linker) -o $@ $^
	@$(OBJDUMP) -S $@ >$T/kernel.asm

# Compile RustSBI 
#! This may not work now
$(RUSTSBI): 
ifeq ($(mode), release)
	@cd ./sbi/$(addprefix rust, $@) && cargo build --release
else 
	@cd ./sbi/$(addprefix rust, $@) && cargo build 
endif 

run: all
ifeq ($(platform), k210) 
	@sudo chmod 777 $(k210-serialport)
	@python3 ./tools/kflash.py -p $(k210-serialport) -b 1500000 -t ./k210.bin
else 
	$(QEMU) $(QEMUOPTS)
endif 


# Prevent deletion of intermediate files, e.g. cat.o, after first build, so
# that disk image changes after first build are persistent until clean.  More
# details:
# http://www.gnu.org/software/make/manual/html_node/Chained-Rules.html
.PRECIOUS: %.o

dst=/mnt

# Make fs image
fs:
	@if [ ! -f "fs.img" ]; then \
		echo "making fs image..."; \
		dd if=/dev/zero of=fs.img bs=512k count=512; \
		mkfs.vfat -F 32 fs.img; fi
	@sudo mount fs.img $(dst)
	@make sdcard dst=$(dst)
	@sudo umount $(dst)

# Write sdcard mounted at $(dst)
sdcard:
	@if [ ! -d "$(dst)/bin" ]; then sudo mkdir $(dst)/bin; fi
	@for file in $$( ls $U/_* ); do \
		sudo cp $$file $(dst)/bin/$${file#$U/_}; done
	@sudo cp $U/_init $(dst)/init
	@sudo cp $U/_sh $(dst)/sh
	@sudo cp $U/shrc $(dst)/shrc
	@sudo cp $U/_echo $(dst)/echo
	@sudo cp README $(dst)/README

.PHONY: clean run all fs sdcard

clean: 
	@rm -rf $(OBJ) $(addsuffix .d, $(basename $(OBJ))) \
		target \
		k210.bin \
		$U/*.d $U/*.o