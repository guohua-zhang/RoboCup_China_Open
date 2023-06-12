#include "my_print.h"
#include <stdarg.h>
#include <stdio.h>

void my_printf(char *str,...)  
{  
    int n;    //¼ÇÂ¼·µ»ØÖµ  
    va_list list;  
    va_start(list,str);  
    n=vprintf(str,list);  
    va_end(list);  
} 

