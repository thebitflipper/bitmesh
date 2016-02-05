#ifndef __uart_h_
#define __uart_h_
#include <stdio.h>
int uart_putchar(char c, FILE *stream);
int uart_getchar(FILE *stream);

void uart_init(void);

/* http://www.ermicro.com/blog/?p=325 */
FILE uart_output;
FILE uart_input;

#endif
