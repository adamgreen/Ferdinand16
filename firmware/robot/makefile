PROJECT         := Ferdinand16
DEVICES         := LPC1768
GCC4MBED_DIR    := ../../gcc4mbed
NO_FLOAT_SCANF  := 1
NO_FLOAT_PRINTF := 0
MBED_LIBS       := dsp
USER_LIBS       := ../../libs/MotorController \
                   ../../libs/DmaSerial \
                   ../../libs/DebugSerial \
                   ../../libs/Interlocked \
                   ../../libs/GPDMA \
                   ../../libs/Sparkfun9DoFSensorStick \
                   ../../libs/ConfigFile \
                   ../../libs/FlashFileSystem

# Allow firmware to be debugged wirelessly with MRI via esp-link.
GCC4MBED_TYPE     := Checked
# MRI_BREAK_ON_INIT := 0
MRI_UART          := MRI_UART_MBED_P13_P14 MRI_UART_BAUD=115200

include $(GCC4MBED_DIR)/build/gcc4mbed.mk

main.cpp : files.h

FILES_H_DEPENDENTS := $(wildcard ../../files/*)

files.h : files.bin
	$(Q) ../../osx/bin2h $< $@ g_fileSystemData

files.bin : ../../files/ $(FILES_H_DEPENDENTS)
	$(Q) ../../osx/fsbld ../../files $@

