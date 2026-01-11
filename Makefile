# Makefile for mini_quadlib

CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99
LDFLAGS = -lm

# Source files
LIB_SRC = mini_quadlib.c
LIB_OBJ = $(LIB_SRC:.c=.o)
EXAMPLE_SRC = example.c
EXAMPLE_OBJ = $(EXAMPLE_SRC:.c=.o)
TEST_SRC = test.c
TEST_OBJ = $(TEST_SRC:.c=.o)

# Output files
LIB = libmini_quadlib.a
EXAMPLE = example
TEST = test

.PHONY: all clean run runtests

all: $(LIB) $(EXAMPLE) $(TEST)

# Build static library
$(LIB): $(LIB_OBJ)
	ar rcs $@ $^
	@echo "Library built: $@"

# Build example program
$(EXAMPLE): $(EXAMPLE_OBJ) $(LIB)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "Example built: $@"

# Build test program
$(TEST): $(TEST_OBJ) $(LIB)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "Test built: $@"

# Compile object files
%.o: %.c mini_quadlib.h
	$(CC) $(CFLAGS) -c $< -o $@

# Run the example
run: $(EXAMPLE)
	./$(EXAMPLE)

# Run the tests
runtests: $(TEST)
	./$(TEST)

clean:
	rm -f $(LIB_OBJ) $(EXAMPLE_OBJ) $(TEST_OBJ) $(LIB) $(EXAMPLE) $(TEST)
	@echo "Cleaned build artifacts"

# Install header (optional)
install:
	@echo "To install, copy mini_quadlib.h and $(LIB) to your include and lib directories"
