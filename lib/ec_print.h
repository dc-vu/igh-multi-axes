#ifndef EC_PRINT_H
#define EC_PRINT_H
#include <stdbool.h>


typedef enum
{
    TURN_DOWN,
    GO_HOME,
    OPERATE
} StateCode;


void ec_print_line(void);

void ec_print_header(void);

void ec_print_row_hex(const char* pdo_name,
                  int d1, int d2, int d3,
                  int d4, int d5, int d6);

void ec_print_row_float(const char* pdo_name,
                        float f1, float f2, float f3,
                        float f4, float f5, float f6);

void ec_print_row_bool(const char* pdo_name,
                       bool b1, bool b2, bool b3,
                       bool b4, bool b5, bool b6);

void ec_print_uint32_binary(uint32_t value);

void ec_clear_console(void);


void ec_print_state_code(StateCode state_code);

#endif // EC_PRINT_H
