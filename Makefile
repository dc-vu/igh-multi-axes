# Makefile cho ectest

CC = gcc
CFLAGS = -I/opt/etherlab/include -Ilib -O2
LDFLAGS = -L/opt/etherlab/lib -lethercat -Wl,--rpath -Wl,/opt/etherlab/lib -lm

TARGET = build/ex_6_servo_state_machine_position

# SRC = main_test_omron.c
# SRC = templates/main_omron_2_servo.c
# SRC = templates/ex_remove_fault_2.c
# SRC = templates/ex_multi_axes.c
# SRC = templates/ex_6_servo.c
# SRC = templates/check_state.c
# SRC = templates/ex_servo_torque.c
# SRC = templates/ex_6_servo_torque.c
# SRC = src/ec_print.c templates/ex_getInfo.c 
# SRC = src/ec_print.c templates/ex_6_servo_torque_v6.c
# SRC = src/ec_print.c templates/ex_6_servo_torque.c
# SRC = src/ec_print.c templates/ex_IO.c
# SRC = src/ec_print.c templates/ex_6_servo_state_machine_v3.c

SRC = src/ec_print.c templates/ex_6_servo_state_machine_position.c
# SRC = src/ec_print.c templates/servo_vel_ctrl.c


all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(SRC) -o $(TARGET) $(CFLAGS) $(LDFLAGS)

clean:
	rm -f $(TARGET)