#
# Makefile for the linux kernel.
#

# Object file lists.

subdir-ccflags-y := -Werror
subdir-ccflags-y += -I$(ANDROID_BUILD_TOP)/$(RDA_TARGET_DEVICE_DIR)/include

obj-y	:= devices.o gpio.o regulator.o regulator-devices.o rda-debug.o

obj-$(CONFIG_MACH_RDA8810)	+= board-rda8810.o io-rda8810.o

obj-$(CONFIG_CLK_RDA)	+= clk_rda.o clk_ops_rda.o

obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o
