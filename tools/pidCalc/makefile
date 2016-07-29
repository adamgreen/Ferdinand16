.PHONY: clean all

all: pidCalc

clean:
	rm pidCalc >/dev/null 2>/dev/null & exit 0

pidCalc : main.cpp
	clang $^ -o $@
	pidCalc ../../docs/pid.csv
