#ifndef SAFE_PRINT_H
#define SAFE_PRINT_H

#include <stdio.h>
#include "pico/mutex.h"

extern mutex_t printf_mutex;

#define safe_printf(...) do { \
    mutex_enter_blocking(&printf_mutex); \
    printf(__VA_ARGS__); \
    mutex_exit(&printf_mutex); \
} while(0)

#endif