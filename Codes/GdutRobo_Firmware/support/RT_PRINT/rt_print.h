#ifndef __RT_PRINT_H
#define __RT_PRINT_H

#include "sys.h"

void rt_kprintf(const char *fmt, ...);
void rt_hw_console_output(const char *str);

#endif
