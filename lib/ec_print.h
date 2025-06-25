#ifndef EC_PRINT_H
#define EC_PRINT_H

void ec_print_line(void);

void ec_print_header(void);

void ec_print_row_hex(const char* pdo_name,
                  int d1, int d2, int d3,
                  int d4, int d5, int d6);

void ec_print_row_float(const char* pdo_name,
                        float f1, float f2, float f3,
                        float f4, float f5, float f6);

void ec_clear_console(void);

#endif // EC_PRINT_H
