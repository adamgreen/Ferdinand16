PROJECT         := Heading
DEVICES         := LPC1768
GCC4MBED_DIR    := ../gcc4mbed
NO_FLOAT_SCANF  := 1
NO_FLOAT_PRINTF := 0
GCC4MBED_TYPE   := Release
MBED_LIBS       := dsp

include $(GCC4MBED_DIR)/build/gcc4mbed.mk

main.cpp : files.h

FILES_H_DEPENDENTS := $(wildcard ../files/*)

files.h : files.bin
	$(Q) ../osx/bin2h $< $@ g_fileSystemData

files.bin : ../files/ $(FILES_H_DEPENDENTS)
	$(Q) ../osx/fsbld ../files $@

