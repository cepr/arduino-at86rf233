all: arduino-at86rf233
LDLIBS:=-lmraa
arduino-at86rf233: arduino-at86rf233.cpp at86rf2xx.cpp at86rf2xx-getset.cpp at86rf2xx-internal.cpp
