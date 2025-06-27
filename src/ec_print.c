#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include "ec_print.h"
#include <stdbool.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

void ec_clear_console(void) {
#ifdef _WIN32
    system("cls");
#else
    int ret = system("clear");
    (void)ret;  // avoid unused-result warning
#endif
}

void ec_print_line(void) {
    printf("+-------------------+-------------+-------------+-------------+-------------+-------------+-------------+\n");
}
void ec_print_header(void) {
    ec_print_line();
    printf("|   PDO             |   Servo 1   |   Servo 2   |   Servo 3   |   Servo 4   |   Servo 5   |   Servo 6   |\n");
    ec_print_line();
}

void ec_print_row_hex(const char* pdo_name,
                  int d1, int d2, int d3,
                  int d4, int d5, int d6)
{
    printf("| %-18s|", pdo_name);
    printf("     0x%04X  |", d1);
    printf("     0x%04X  |", d2);
    printf("     0x%04X  |", d3);
    printf("     0x%04X  |", d4);
    printf("     0x%04X  |", d5);
    printf("     0x%04X  |\n", d6);
}

void ec_print_row_float(const char* pdo_name,
                        float f1, float f2, float f3,
                        float f4, float f5, float f6)
{
    printf("| %-18s|", pdo_name);
    printf(" %10.5f  |", f1);
    printf(" %10.5f  |", f2);
    printf(" %10.5f  |", f3);
    printf(" %10.5f  |", f4);
    printf(" %10.5f  |", f5);
    printf(" %10.5f  |\n", f6);
}

void ec_print_row_bool(const char* pdo_name,
                       bool b1, bool b2, bool b3,
                       bool b4, bool b5, bool b6)
{
    printf("| %-18s|", pdo_name);
    printf(" %10s  |", b1 ? "true" : "false");
    printf(" %10s  |", b2 ? "true" : "false");
    printf(" %10s  |", b3 ? "true" : "false");
    printf(" %10s  |", b4 ? "true" : "false");
    printf(" %10s  |", b5 ? "true" : "false");
    printf(" %10s  |\n", b6 ? "true" : "false");
}

void ec_print_uint32_binary(uint32_t value) {
    for (int i = 31; i >= 0; i--) {
        printf("%d", (value >> i) & 1);
        if (i % 4 == 0 && i != 0) {
            printf(" ");  // Add space after every 4 bits, except after the last group
        }
    }
    printf("\n");
}



void ec_print_state_code(StateCode state_code)
{
    const char* state_str;

    switch (state_code)
    {
        case TURN_DOWN:
            state_str = "TURN_DOWN";
            break;
        case GO_HOME:
            state_str = "GO_HOME";
            break;
        case OPERATE:
            state_str = "OPERATE";
            break;
        default:
            state_str = "UNKNOWN_STATE";
            break;
    }

    printf("State code of operation: %s\n", state_str);
}