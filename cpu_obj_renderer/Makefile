CC ?= cc
CFLAGS ?= -O3 -std=c11 -Wall -Wextra -pedantic
LDFLAGS ?= -lm

SRC := src/main.c
BIN := objrt

.PHONY: all clean run-example

all: $(BIN)

$(BIN): $(SRC)
	$(CC) $(CFLAGS) -o $@ $(SRC) $(LDFLAGS)

run-example: $(BIN)
	./$(BIN) examples/mirror_room.obj output.ppm 640 360 --samples 4 --bounces 3

clean:
	rm -f $(BIN) output.ppm
