# Makefile cho ectest

CC = gcc
CFLAGS = -I/opt/etherlab/include -O2
LDFLAGS = -L/opt/etherlab/lib -lethercat -Wl,--rpath -Wl,/opt/etherlab/lib -lm
TARGET = build/main
# SRC = main_test_omron.c
# SRC = templates/main_omron_2_servo.c
# SRC = templates/ex_remove_fault_2.c
# SRC = templates/ex_multi_axes.c
SRC = templates/ex_4_servo_f.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(SRC) -o $(TARGET) $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(TARGET)