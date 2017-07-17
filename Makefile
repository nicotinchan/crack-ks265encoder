CC       = gcc
CFLAGS   = -Wall -Warray-bounds -g -funwind-tables -Wno-write-strings -Wno-unused-result -Werror
INC_PATH = -I./ -I./include/
LIB_PATH = -L./lib/
LIBS     = -lm -lpthread
SRCS := $(filter-out hook.c, $(wildcard *.c))
OBJS := $(patsubst %c,%o,$(SRCS))

%.o: %.c
	$(CC) -c $(CFLAGS) $(INC_PATH) $< -o $@

all: demo hook.so

demo: $(OBJS)
	$(CC) $(CFLAGS) $(INC_PATH) $(LIB_PATH) $^ $(LIBS) -o $@ $(LIBS)

hook.so: hook.c
	$(CC) -fPIC -shared -o $@ $^ -ldl

clean:
	rm -rf demo *.o *.so
