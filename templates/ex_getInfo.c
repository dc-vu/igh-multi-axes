#include <stdlib.h>
#include <time.h>
#include "ec_print.h"

int getRandomHexByte() {
    return rand() % 256;
}

int main() {
    srand(time(NULL));

    while (1) {
        ec_clear_console();

        ec_print_header();

        ec_print_row("0x6061",
            getRandomHexByte(), getRandomHexByte(), getRandomHexByte(),
            getRandomHexByte(), getRandomHexByte(), getRandomHexByte());

        ec_print_row("0x6062",
            getRandomHexByte(), getRandomHexByte(), getRandomHexByte(),
            getRandomHexByte(), getRandomHexByte(), getRandomHexByte());

        ec_print_line();

#ifdef _WIN32
        Sleep(1000);
#else
        usleep(1000000);
#endif
    }

    return 0;
}
